import os
import sys

# Biz "robotlar güvenli ROS2 MoveIt2 kodu yazar" davranışı kazandırmak istiyoruz
alpaca_prompt = """Below is an instruction that describes a task. Write a response that appropriately completes the request.

### Instruction:
{instruction}

### Response:
{response}"""

def formatting_prompts_func(examples, eos_token="<|endoftext|>"):
    instructions = examples.get("instruction", [])
    responses    = examples.get("response", [])
    texts = []
    for instruction, response in zip(instructions, responses):
        text = alpaca_prompt.format(instruction=instruction, response=response) + eos_token
        texts.append(text)
    return { "text" : texts }

def main():
    print("Loading ML libraries...")
    import torch
    from datasets import load_dataset
    from trl import SFTTrainer
    from transformers import TrainingArguments, AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
    from peft import LoraConfig, get_peft_model, prepare_model_for_kbit_training

    # RAM/VRAM sorunları yüzünden PyTorch allocator ayarını yapalım
    os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"

    # Ayarlar: 6GB VRAM için çok daha agresif optimize edilmiştir (OOM'yi engellemek için)
    max_seq_length = 512 # 1024'ten 512'ye düşürüldü, maksimum VRAM tasarrufu

    # Model seçimi (3B, 6GB karta tam uyar)
    model_name = "Qwen/Qwen2.5-Coder-3B-Instruct"

    # 1. 4-bit Quantization (VRAM kurtarıcı)
    print("4-bit konfigürasyon hazırlanıyor...")
    bnb_config = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_use_double_quant=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_compute_dtype=torch.bfloat16 if torch.cuda.is_bf16_supported() else torch.float16
    )

    # 2. Modeli ve Tokenizer'ı Yükle
    print("Model yükleniyor... Bu işlem birkaç dakika sürebilir.")
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    tokenizer.pad_token = tokenizer.eos_token # Qwen için pad token ayarı

    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        quantization_config=bnb_config,
        device_map="auto"
    )
    
    # K-bit training için hazırlık (Gradient Checkpointing dahil edilir)
    model = prepare_model_for_kbit_training(model)

    # 3. Peft/LoRA Ayarları (VRAM Kısıtlaması nedeniyle sadece çekirdek modüller)
    print("LoRA adaptörleri ekleniyor...")
    lora_config = LoraConfig(
        r=8, # 16'dan 8'e düşürüldü
        lora_alpha=16,
        # MLP parçaları (gate/up/down) çıkarıldı, sadece attention kısımları hedeflendi
        target_modules=["q_proj", "k_proj", "v_proj", "o_proj"],
        lora_dropout=0.05,
        bias="none",
        task_type="CAUSAL_LM"
    )
    
    model = get_peft_model(model, lora_config)
    model.print_trainable_parameters()

    EOS_TOKEN = tokenizer.eos_token
    # Wrapper function for dataset mapping
    def dataset_formatting_wrapper(examples):
        return formatting_prompts_func(examples, EOS_TOKEN)


    import json
    from datasets import Dataset

    print("Veri seti yükleniyor...")
    dataset_path = "ros2_dataset.jsonl"
    if not os.path.exists(dataset_path):
        print(f"HATA: {dataset_path} bulunamadı!")
        sys.exit(1)

    raw_data = []
    with open(dataset_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            try:
                item = json.loads(line)
                raw_data.append(item)
            except json.JSONDecodeError as e:
                print(f"Uyarı: {i}. satır bozuk olduğu için atlandı! Hata: {e}")

    print(f"Toplam {len(raw_data)} geçerli veri yüklendi.")
    dataset = Dataset.from_list(raw_data)
    dataset = dataset.map(dataset_formatting_wrapper, batched = True,)

    # 4. Eğitim (Training) Ayarları
    print("Eğitim başlıyor...")
    
    is_bf16 = torch.cuda.is_bf16_supported()
    
    trainer = SFTTrainer(
        model = model,
        tokenizer = tokenizer,
        train_dataset = dataset,
        dataset_text_field = "text",
        max_seq_length = max_seq_length,
        dataset_num_proc = 2,
        packing = False, # VRAM korumak için False
        args = TrainingArguments(
            per_device_train_batch_size = 1, # 6GB VRAM için 1 batch uygundur (OOM'yi engeller)
            gradient_accumulation_steps = 8, # 1 * 8 = 8 effective batch size
            warmup_steps = 10,
            num_train_epochs = 1, # Gerçek bir eğitim için 1 epoch (1 tur dataset bitirme) yeterlidir
            learning_rate = 2e-4,
            fp16 = not is_bf16,
            bf16 = is_bf16,
            logging_steps = 5,
            optim = "adamw_8bit", # Optimize edilmiş 8bit VRAM dostu optimizer
            weight_decay = 0.01,
            lr_scheduler_type = "linear",
            seed = 3407,
            output_dir = "outputs",
            report_to = "none",
            gradient_checkpointing=True, # VRAM korumak için kritik
        ),
    )

    # Gerekli VRAM istatistiklerini hesapla
    gpu_stats = torch.cuda.get_device_properties(0)
    start_gpu_memory = round(torch.cuda.max_memory_reserved() / 1024 / 1024 / 1024, 3)
    max_memory = round(gpu_stats.total_memory / 1024 / 1024 / 1024, 3)
    print(f"GPU = {gpu_stats.name}. Max memory = {max_memory} GB.")
    print(f"Başlangıçta rezerve edilen VRAM: {start_gpu_memory} GB")

    # Eğitimi başlat
    trainer_stats = trainer.train()

    # 5. Modeli Kaydetme (SADECE LoRA Adaptörleri Kaydedilir)
    print("Eğitim bitti! Model kaydediliyor (LoRA ağırlıkları)...")
    model.save_pretrained("qwen2.5_coder_3b_ros2_lora")
    tokenizer.save_pretrained("qwen2.5_coder_3b_ros2_lora")
    
    print("\n[OK] Model ağırlıkları başarıyla kaydedildi!")
    print("Ollama'da kullanmak için llama.cpp kütüphanesi ile GGUF'a çevirme işlemi yapılabilir.")

if __name__ == "__main__":
    main()
