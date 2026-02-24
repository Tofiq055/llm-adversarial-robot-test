# Local Fine-Tuning Setup Instructions (RTX 3060 - 6GB VRAM)

Bu döküman, A4 projesi kapsamında `Qwen2.5-Coder` veya `Llama-3` modellerinin 6GB VRAM ile lokal bilgisayarda eğitilmesi için oluşturulmuştur.

### 1. Ortam Hazırlığı (Ubuntu / WSL)
Eğitim sürecini Docker içinden ziyade doğrudan host makinede (veya Conda/Venv) yapmamız daha performanslıdır.

```bash
# Conda önerilir
conda create --name unsloth_env python=3.10
conda activate unsloth_env
```

### 2. Pytorch Kurulumu (CUDA 12.1 önerilir)
```bash
conda install pytorch-cuda=12.1 pytorch cudatoolkit xformers -c pytorch -c nvidia -c xformers
```

### 3. Unsloth Kurulumu
```bash
pip install "unsloth[colab-new] @ git+https://github.com/unslothai/unsloth.git"
pip install --no-deps trl peft accelerate bitsandbytes
```

### 4. Hugging Face Login
```bash
huggingface-cli login
# Buraya Hugging Face sayfasından (Settings -> Access Tokens) alacağınız "Write" yetkili token'ı yapıştırın.
```
