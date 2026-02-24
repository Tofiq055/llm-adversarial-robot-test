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
from dotenv import load_dotenv

# .env dosyasını yükle (Aynı dizinde veya proje kök dizininde arar)
load_dotenv()

# GitHub API rate limitlerini aşmamak için kişisel token
# .env veya terminal üzerinden GITHUB_TOKEN aranır
GITHUB_TOKEN = os.environ.get("GITHUB_TOKEN", "")

HEADERS = {
    "Accept": "application/vnd.github.v3+json",
}

if GITHUB_TOKEN:
    HEADERS["Authorization"] = f"token {GITHUB_TOKEN}"

# Arama sorgusu: Python dosyaları, içinde rclpy ve moveit geçmeli
# Not: GitHub Code Search API aramada 'stars' filtresini desteklemez.
# Kalite kontrolü script içinde ayrı API isteği ile yapılacaktır.
SEARCH_QUERY = "rclpy moveit language:python"

MIN_STARS = 5

def search_github_files(query: str, max_results: int = 100) -> List[Dict]:
    """GitHub API üzerinden belirtilen sorguya uygun dosyaları arar."""
    print(f"🔍 GitHub'da aranıyor: '{query}'")
    items = []
    page = 1
    
    while len(items) < max_results:
        url = f"https://api.github.com/search/code?q={query}&per_page=100&page={page}"
        response = requests.get(url, headers=HEADERS)
        if response.status_code == 403:
            print("❌ Hata: API Limitine ulaşıldı veya Token geçersiz!")
            if not GITHUB_TOKEN:
                print("   Lütfen GITHUB_TOKEN ortam değişkenini ayarlayın.")
            break
        elif response.status_code != 200:
            print(f"❌ Hata: {response.status_code} - {response.text}")
            break
            
        data = response.json()
        new_items = data.get("items", [])
        if not new_items:
            break
            
        items.extend(new_items)
        print(f"✅ Sayfa {page}: {len(new_items)} dosya bulundu. Toplam: {len(items)}")
        
        if len(new_items) < 100:
            break # Son sayfa veya limit
            
        page += 1
        time.sleep(2) # search api rate limitlerini yormamak için kısa bekleme
        
    return items[:max_results]

def download_file_content(download_url: str) -> str:
    """GitHub dosyasının raw içeriğini indirir."""
    response = requests.get(download_url, headers=HEADERS)
    if response.status_code == 200:
        return response.text
    return ""

def check_repo_stars(repo_full_name: str, headers: Dict) -> int:
    """GitHub API üzerinden deponun güncel yıldız sayısını döndürür."""
    repo_url = f"https://api.github.com/repos/{repo_full_name}"
    resp = requests.get(repo_url, headers=headers)
    if resp.status_code == 200:
        return resp.json().get("stargazers_count", 0)
    return 0

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
    results = search_github_files(SEARCH_QUERY, max_results=300)
    
    dataset = []
    
    print("⏳ Dosyalar indiriliyor ve kontrol ediliyor...")
    for i, item in enumerate(results):
        print(f"[{i+1}/{len(results)}] İndiriliyor: {item['name']}")
        
        # Raw indirme linkini bul
        # Repositorie URL'sinden raw URL'sini parse edelim
        repo_name = item['repository']['full_name']
        file_path = item['path']
        
        # Repo yıldız sayısını kontrol et
        stars = check_repo_stars(repo_name, HEADERS)
        if stars < MIN_STARS:
            print(f"      [{stars} yıldız] {MIN_STARS} yıldızdan düşük = Atlanıyor.")
            continue
        
        print(f"      ⭐ Repo kalitesi onaylandı: {stars} yıldız")
        
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

    # JSONL'ye yaz (Eski veriler kaybolmasın diye 'a' append modu kullanılır)
    if dataset:
        with open(output_file, 'a', encoding='utf-8') as f:
            for entry in dataset:
                f.write(json.dumps(entry, ensure_ascii=False) + '\\n')
        print(f"\\n🎉 Başarılı! {len(dataset)} adet yeni örnek {output_file} dosyasına EKLENDİ (Eskiler korundu).")
        print("🧠 Bu veri seti Qwen2.5-Coder veya Llama modelini eğitmek için hazırdır!")
    else:
        print("\\n⚠️ Hiç veri toplanamadı.")

if __name__ == "__main__":
    main()
