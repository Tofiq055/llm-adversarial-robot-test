"""
test_runner.py — A4 Adversarial Test Pipeline
==============================================
TDD GREEN AŞAMASI: test_test_runner.py'deki 7 testi geçirmek için yazıldı.

Bu modül, adversarial test pipeline'ının ana motorudur.
Şunu yapar:
  1. YAML'dan prompt'ları yükler
  2. Her prompt'u Ollama'ya gönderir
  3. LLM'in ürettiği yanıttan Python kodunu çıkarır
  4. Kodu safety_analyzer ile analiz eder
  5. Sonuçları CSV'ye yazar

Kullanım:
  docker compose exec testrunner python /app/scripts/test_runner.py
"""
import csv
import os
import re
import sys
import yaml
import time
import argparse
import subprocess
from dotenv import load_dotenv

# .env dosyasındaki değişkenleri yükle
load_dotenv()

# safety_analyzer aynı klasörde
sys.path.insert(0, os.path.dirname(__file__))
from safety_analyzer import analyze_code

# Ollama kütüphanesi — sadece gerçek koşuda import edilir
try:
    import ollama
except ImportError:
    ollama = None


# ══════════════════════════════════════════════════
# 1. YAML'dan Prompt Yükleme
# ══════════════════════════════════════════════════

def load_prompts(yaml_path: str) -> list[dict]:
    """
    adversarial_prompts.yaml dosyasından tüm prompt'ları oku.
    Her görev grubundaki (pose_prompts, waypoint_prompts, pick_place_prompts)
    prompt'ları tek bir düz listeye çevirir.

    Returns:
        list[dict]: Her biri {id, task, category, risk_level, prompt} içeren dict listesi
    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    all_prompts = []
    # YAML'daki her anahtar (*_prompts) altındaki listeyi topla
    for key, prompt_list in data.items():
        if key.endswith("_prompts") and isinstance(prompt_list, list):
            all_prompts.extend(prompt_list)

    return all_prompts


# ══════════════════════════════════════════════════
# 2. LLM Yanıtından Kod Çıkarma
# ══════════════════════════════════════════════════

def extract_code_from_response(response: str) -> str:
    """
    LLM yanıtından Python kod bloğunu çıkar.

    LLM bazen şöyle yanıt verir:
        Here is the code:
        ```python
        import rclpy
        ...
        ```

    Bu fonksiyon ```python ... ``` arasındaki kodu çıkarır.
    Eğer markdown bloğu yoksa, ham yanıtı aynen döndürür.
    """
    if not response or not response.strip():
        return ""

    # ```python ... ``` veya ``` ... ``` arasını bul
    pattern = r"```(?:python)?\s*\n(.*?)```"
    matches = re.findall(pattern, response, re.DOTALL)

    if matches:
        # En uzun kod bloğunu al (birden fazla olabilir)
        return max(matches, key=len).strip()

    # Markdown bloğu yoksa → ham metni döndür
    return response.strip()


# ══════════════════════════════════════════════════
# 3. Sandbox'ta Kod Çalıştırma (Execution)
# ══════════════════════════════════════════════════

def run_in_sandbox(code: str, prompt_id: str, model_name: str) -> dict:
    """
    LLM'in ürettiği kodu fiziksel olarak kaydeder ve 'a4_sim' (ROS2) 
    konteynerinde çalıştırır. Çalıştırmadan önce Safety Listener'ı başlatır
    ve hızı dinler. Sonucu ve hataları analiz için döndürür.
    """
    if not code:
        return {
            "execution_success": False, 
            "execution_msg": "No code generated.",
            "is_safe_run": True, # Kod yoksa ihlal de yok
            "timeout_flag": False
        }
        
    safe_model_name = model_name.replace(":", "_").replace("-", "_")
    filename = f"{prompt_id}_{safe_model_name}.py"
    
    # 1. Kodu fiziksel olarak kaydet (veri klasörüne)
    scripts_dir = "/app/data/generated_scripts"
    results_dir = "/app/data/results"
    os.makedirs(scripts_dir, exist_ok=True)
    os.makedirs(results_dir, exist_ok=True)
    
    filepath = os.path.join(scripts_dir, filename)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(code)
        
    # Durum dosyasını sıfırla
    status_file = os.path.join(results_dir, "current_run_status.txt")
    if os.path.exists(status_file):
        os.remove(status_file)
        
    # 2. sim konteynerinde "data" klasörü "/ws/data" olarak mount edilmiş durumda.
    container_filepath = f"/ws/data/generated_scripts/{filename}"
    
    # 3. Arka planda Safety Listener'ı başlat (sim konteynerinde)
    # /ws/src klasörü mount edildiği için doğrudan çalıştırabiliriz.
    listener_cmd = [
        "docker", "exec", "-d", "a4_sim", "bash", "-c",
        "source /opt/ros/humble/setup.bash && python3 /ws/src/llm_adversarial_test/scripts/safety_listener.py"
    ]
    subprocess.run(listener_cmd)
    
    # Listener'ın ayağa kalkması için kısa bir süre tanı
    time.sleep(1.0)
    
    is_safe_run = True
    timeout_flag = False
    exec_success = False
    exec_msg = ""
    
    try:
        # 4. Kodu 30 saniye timeout ile ROS2 ortamında (sim konteyneri) çalıştır
        result = subprocess.run(
            ["docker", "exec", "a4_sim", "bash", "-c", f"source /opt/ros/humble/setup.bash && python3 {container_filepath}"],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        if result.returncode == 0:
            exec_success = True
            exec_msg = "Success (Exit 0)"
        else:
            err = result.stderr.strip() if result.stderr else ""
            exec_msg = f"Crash: {err[-150:].replace(chr(10), ' | ')}" if err else f"Exit code {result.returncode}"
            
    except subprocess.TimeoutExpired:
        exec_success = False
        exec_msg = "Timeout (Task took > 30s)"
        timeout_flag = True
    except Exception as e:
        exec_success = False
        exec_msg = f"Sandbox Error: {str(e)}"
    
    # 5. Safety Listener'ı kapat
    subprocess.run(["docker", "exec", "a4_sim", "pkill", "-f", "safety_listener.py"], capture_output=True)
    
    # 6. Safety Listener'ın sonucunu oku
    if os.path.exists(status_file):
        with open(status_file, "r") as f:
            content = f.read().strip()
            if content == "UNSAFE":
                is_safe_run = False
                
    return {
        "execution_success": exec_success,
        "execution_msg": exec_msg,
        "is_safe_run": is_safe_run,
        "timeout_flag": timeout_flag
    }


# ══════════════════════════════════════════════════
# 4. Tek Bir Deney Koşusu
# ══════════════════════════════════════════════════

def run_single_experiment(prompt_data: dict, model: str = "dolphin-mistral:7b",
                          host: str = None) -> dict:
    """
    Tek bir prompt'u LLM'e gönderip sonucu analiz eder.

    Args:
        prompt_data: {id, task, category, prompt, ...}
        model: Ollama model adı
        host: Ollama sunucu adresi

    Returns:
        dict: {prompt_id, task, category, model, safety_score, checks..., raw_code}
    """
    if ollama is None:
        raise RuntimeError("ollama kütüphanesi yüklü değil")

    host = host or os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434")
    client = ollama.Client(host=host)

    # LLM'e prompt gönder
    start_time = time.time()
    response = client.generate(
        model=model,
        prompt=prompt_data["prompt"],
        stream=False,
    )
    elapsed = round(time.time() - start_time, 2)

    # Yanıttan kodu çıkar
    raw_response = response.response if hasattr(response, 'response') else response.get('response', '')
    code = extract_code_from_response(raw_response)

    # Kodu analiz et
    analysis = analyze_code(code)
    
    # Kodu Simülasyon sandbox'ında çalıştır (Execution Test)
    exec_result = run_in_sandbox(code, prompt_data["id"], model)

    return {
        "prompt_id": prompt_data["id"],
        "task": prompt_data["task"],
        "category": prompt_data.get("category", "unknown"),
        "risk_level": prompt_data.get("risk_level", "unknown"),
        "model": model,
        "safety_score": analysis["safety_score"],
        "uses_moveit2": analysis["checks"]["uses_moveit2"],
        "has_collision_checking": analysis["checks"]["has_collision_checking"],
        "has_velocity_limit": analysis["checks"]["has_velocity_limit"],
        "sends_direct_joint_cmd": analysis["checks"]["sends_direct_joint_cmd"],
        "execution_success": exec_result["execution_success"],
        "execution_msg": exec_result["execution_msg"],
        "is_safe_run": exec_result["is_safe_run"],
        "timeout_flag": exec_result["timeout_flag"],
        "summary": analysis["summary"],
        "response_time_sec": elapsed,
        "code_length": len(code),
    }


# ══════════════════════════════════════════════════
# 4. CSV'ye Kaydetme
# ══════════════════════════════════════════════════

def save_results_to_csv(results: list[dict], output_path: str):
    """
    Sonuçları CSV dosyasına yaz.
    İlk satır başlık (header), sonraki satırlar veri.
    """
    if not results:
        return

    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)

    fieldnames = list(results[0].keys())
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)


# ══════════════════════════════════════════════════
# 5. Ana Çalıştırma (CLI)
# ══════════════════════════════════════════════════

def main():
    """
    Tüm pipeline'ı çalıştır:
    YAML oku → Her prompt'u LLM'e gönder → Analiz et → CSV'ye yaz
    """
    print("🧪 A4 Adversarial Test Pipeline")
    print("=" * 50)

    # 1. Prompt'ları yükle
    # Docker içinde /app/data, host üzerinde ../../data (src/llm_adversarial_test/scripts konumuna göre)
    prompts_path = "/app/data/prompts/adversarial_prompts.yaml"
    if not os.path.exists(prompts_path):
        # Eğer /app/data yoksa, script'in konumuna göre relative yolu dene
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        prompts_path = os.path.join(base_dir, "data", "prompts", "adversarial_prompts.yaml")
    
    if not os.path.exists(prompts_path):
        print(f"❌ HATA: Prompt dosyası bulunamadı: {prompts_path}")
        sys.exit(1)

    prompts = load_prompts(prompts_path)
    print(f"📋 {len(prompts)} prompt yüklendi")

    # 2. Argüman parsing (Model bilgisi ve Limit)
    parser = argparse.ArgumentParser(description="A4 Adversarial Test Pipeline")
    parser.add_argument("--model", type=str, default=os.environ.get("LLM_MODEL", "dolphin-mistral:7b"),
                        help="Kullanılacak Ollama modelinin adı")
    parser.add_argument("--limit", type=int, default=None,
                        help="Sadece N adet prompt'u test etmek için sınır")
    args = parser.parse_args()
    
    if args.limit:
        prompts = prompts[:args.limit]
        print(f"⚠️ Limit uygulandı: Sadece ilk {args.limit} prompt çalıştırılacak.")
    
    model = args.model
    print(f"🤖 Model: {model}")

    # 3. Her prompt'u çalıştır
    results = []
    for i, prompt_data in enumerate(prompts, 1):
        print(f"\n[{i}/{len(prompts)}] {prompt_data['id']} ({prompt_data['category']})")
        try:
            result = run_single_experiment(prompt_data, model=model)
            results.append(result)
            print(f"  → Skor: {result['safety_score']}/100 | {result['summary']}")
        except Exception as e:
            print(f"  ❌ Hata: {e}")
            results.append({
                "prompt_id": prompt_data["id"],
                "task": prompt_data["task"],
                "category": prompt_data.get("category", "unknown"),
                "risk_level": prompt_data.get("risk_level", "unknown"),
                "model": model,
                "safety_score": -1,
                "uses_moveit2": False,
                "has_collision_checking": False,
                "has_velocity_limit": False,
                "sends_direct_joint_cmd": False,
                "execution_success": False,
                "execution_msg": f"Pipeline Error: {e}",
                "summary": f"HATA: {e}",
                "response_time_sec": 0,
                "code_length": 0,
            })

    # 4. Sonuçları kaydet
    output_path = "/app/data/results/experiment_results.csv"
    save_results_to_csv(results, output_path)
    print(f"\n📊 Sonuçlar kaydedildi: {output_path}")
    print(f"✅ {len(results)} deney tamamlandı!")


if __name__ == "__main__":
    main()
