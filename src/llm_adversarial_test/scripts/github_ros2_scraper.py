"""
github_ros2_scraper.py — A4 ROS2 Fine-Tuning Data Collector
===========================================================
Bu script, Github API'sini kullanarak içinde 'rclpy', 'moveit', 'ur5e' gibi
ROS2 bileşenleri geçen Python scriptlerini tarar, indirir ve LLM eğitimi
(Fine-Tuning) için kullanılabilecek 'Instruction-Response' formatında bir
JSONL (JSON Lines) dosyasına dönüştürür.

Kullanım için GITHUB_TOKEN ortam değişkeni gereklidir.
"""

import os
import json
import requests
import time
from typing import List, Dict

# GitHub API rate limitlerini aşmamak için kişisel token
# Export GITHUB_TOKEN="ghp_xxxxxx..."
GITHUB_TOKEN = os.environ.get("GITHUB_TOKEN", "")

HEADERS = {
    "Accept": "application/vnd.github.v3+json",
}

if GITHUB_TOKEN:
    HEADERS["Authorization"] = f"token {GITHUB_TOKEN}"

# Arama sorgusu: Python dosyaları, içinde rclpy ve moveit geçmeli
SEARCH_QUERY = "rclpy moveit language:python"

def search_github_files(query: str, max_results: int = 100) -> List[Dict]:
    """GitHub API üzerinden belirtilen sorguya uygun dosyaları arar."""
    print(f"🔍 GitHub'da aranıyor: '{query}'")
    url = f"https://api.github.com/search/code?q={query}&per_page=100"
    
    response = requests.get(url, headers=HEADERS)
    if response.status_code == 403:
        print("❌ Hata: API Limitine ulaşıldı veya Token geçersiz!")
        if not GITHUB_TOKEN:
            print("   Lütfen GITHUB_TOKEN ortam değişkenini ayarlayın.")
        return []
    elif response.status_code != 200:
        print(f"❌ Hata: {response.status_code} - {response.text}")
        return []
        
    data = response.json()
    items = data.get("items", [])
    print(f"✅ {len(items)} dosya bulundu.")
    return items[:max_results]

def download_file_content(download_url: str) -> str:
    """GitHub dosyasının raw içeriğini indirir."""
    response = requests.get(download_url, headers=HEADERS)
    if response.status_code == 200:
        return response.text
    return ""

def generate_instruction_for_code(code: str, filename: str) -> str:
    """
    Geçici olarak statik, ileride Gemini/Claude ile dinamik üretilecek
    Instruction (Soru) kısmını oluşturur.
    """
    return f"Write a ROS2 Python node named {filename} that uses rclpy and MoveIt2 for robotic manipulation."

def main():
    if not GITHUB_TOKEN:
        print("⚠️ DİKKAT: GITHUB_TOKEN olmadan api limitiniz saatlik 60 istek ile sınırlıdır.")
        
    output_file = "ros2_dataset.jsonl"
    results = search_github_files(SEARCH_QUERY, max_results=50)
    
    dataset = []
    
    print("⏳ Dosyalar indiriliyor ve kontrol ediliyor...")
    for i, item in enumerate(results):
        print(f"[{i+1}/{len(results)}] İndiriliyor: {item['name']}")
        
        # Raw indirme linkini bul
        # Repositorie URL'sinden raw URL'sini parse edelim
        repo_name = item['repository']['full_name']
        file_path = item['path']
        
        # API üzerinden içeriği çek (base64)
        content_url = item['url']
        content_resp = requests.get(content_url, headers=HEADERS)
        
        if content_resp.status_code == 200:
            import base64
            content_data = content_resp.json()
            if 'content' in content_data:
                code_content = base64.b64decode(content_data['content']).decode('utf-8', errors='ignore')
                
                # Kod çok kısaysa (10 satırdan az) salla
                if len(code_content.splitlines()) < 10:
                    continue
                    
                instruction = generate_instruction_for_code(code_content, item['name'])
                
                # JSONL Formatı: {"instruction": "...", "response": "..."}
                dataset.append({
                    "instruction": instruction,
                    "response": code_content
                })
        
        # API limitine takılmamak için minik bekleme
        time.sleep(1)

    # JSONL'ye yaz
    if dataset:
        with open(output_file, 'w', encoding='utf-8') as f:
            for entry in dataset:
                f.write(json.dumps(entry, ensure_ascii=False) + '\\n')
        print(f"\\n🎉 Başarılı! {len(dataset)} adet örnek {output_file} dosyasına kaydedildi.")
        print("🧠 Bu veri seti Qwen2.5-Coder veya Llama modelini eğitmek için hazırdır!")
    else:
        print("\\n⚠️ Hiç veri toplanamadı.")

if __name__ == "__main__":
    main()
