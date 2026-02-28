#!/usr/bin/env python3
"""
A4 Projesi — GitHub'dan ROS2/MoveIt2 Python Kodları Scraper
============================================================
GitHub Code Search API kullanarak ROS2 robotik Python scriptlerini çeker.
Tekrar eden dosyaları SHA hash ile filtreler.

Kullanım:
    python3 scrape_github_ros2.py

Çıktı:
    ros2_dataset_v2.jsonl  (instruction + response formatında)
"""

import os
import json
import time
import hashlib
import base64
import re
import sys
from pathlib import Path

try:
    import requests
except ImportError:
    print("❌ 'requests' kütüphanesi bulunamadı. Yükleniyor...")
    os.system(f"{sys.executable} -m pip install requests")
    import requests

from dotenv import load_dotenv

# ═══════════════════ CONFIG ═══════════════════

load_dotenv()
GITHUB_TOKEN = os.getenv("GITHUB_TOKEN", "")
if not GITHUB_TOKEN:
    print("❌ GITHUB_TOKEN .env dosyasında bulunamadı!")
    sys.exit(1)

HEADERS = {
    "Authorization": f"Bearer {GITHUB_TOKEN}",
    "Accept": "application/vnd.github.v3+json",
    "X-GitHub-Api-Version": "2022-11-28",
}

OUTPUT_FILE = "ros2_dataset_v2.jsonl"
MIN_FILE_SIZE = 300       # byte — çok kısa snippet'leri atla
MAX_FILE_SIZE = 80_000    # byte — çok uzun dosyaları atla
MAX_RESULTS_PER_QUERY = 30  # GitHub API sayfa limiti (max 100, ama rate limit)
API_DELAY = 12            # GitHub code search: 10 req/min, güvenli aralık

# ═══════════════════ KEYWORD GROUPS ═══════════════════

SEARCH_QUERIES = [
    # ── GRUP 1: Güvenli Kodlar (MoveIt2 ile planlama) ──
    {
        "query": "rclpy MoveIt2 move_to_pose language:python",
        "category": "safe",
        "desc": "MoveIt2 ile güvenli poz hedefi",
    },
    {
        "query": "rclpy moveit plan_kinematic_path language:python",
        "category": "safe",
        "desc": "MoveIt2 ile kinematik yol planlama",
    },
    {
        "query": "max_velocity_scaling_factor rclpy language:python",
        "category": "safe",
        "desc": "Hız sınırlaması uygulayan ROS2 kodu",
    },
    {
        "query": "avoid_collisions MoveIt rclpy language:python",
        "category": "safe",
        "desc": "Çarpışma kontrolü içeren MoveIt kodu",
    },
    {
        "query": "MoveItPy planning_component language:python",
        "category": "safe",
        "desc": "MoveIt2 Python API ile planlama",
    },
    {
        "query": "pymoveit2 move_to_configuration language:python",
        "category": "safe",
        "desc": "pymoveit2 ile güvenli konfigürasyon hareketi",
    },
    {
        "query": "collision_checking moveit rclpy language:python",
        "category": "safe",
        "desc": "Çarpışma kontrolü ile MoveIt kullanımı",
    },

    # ── GRUP 2: Güvensiz Kalıplar (Güvenlik önlemi eksik) ──
    {
        "query": "JointTrajectory publish rclpy language:python",
        "category": "unsafe_pattern",
        "desc": "MoveIt2 kullanmadan doğrudan joint trajectory yayınlama",
    },
    {
        "query": "FollowJointTrajectory send_goal rclpy language:python",
        "category": "unsafe_pattern",
        "desc": "Planlama olmadan doğrudan joint action komutu",
    },
    {
        "query": "JointTrajectoryPoint velocities rclpy language:python",
        "category": "unsafe_pattern",
        "desc": "Joint hız değerleri ile doğrudan hareket komutu",
    },
    {
        "query": "joint_trajectory_controller rclpy language:python",
        "category": "unsafe_pattern",
        "desc": "ros2_control controller'a doğrudan trajectory yazma",
    },
    {
        "query": "JointTrajectory points append rclpy language:python",
        "category": "unsafe_pattern",
        "desc": "Plansız trajectory oluşturma",
    },
    {
        "query": "set_joint_value_target rclpy language:python",
        "category": "unsafe_pattern",
        "desc": "Doğrudan joint değeri hedefleme",
    },

    # ── GRUP 3: UR Robot Spesifik ──
    {
        "query": "ur5e joint_states rclpy language:python",
        "category": "ur_specific",
        "desc": "UR5e joint durumu okuma ve kontrol",
    },
    {
        "query": "ur_robot_driver rclpy language:python",
        "category": "ur_specific",
        "desc": "UR robot driver ile haberleşme",
    },
    {
        "query": "universal_robots rclpy moveit language:python",
        "category": "ur_specific",
        "desc": "Universal Robots MoveIt entegrasyonu",
    },
    {
        "query": "ur_moveit_config rclpy language:python",
        "category": "ur_specific",
        "desc": "UR MoveIt konfigürasyon kullanımı",
    },
]

# ═══════════════════ DEDUPLICATION ═══════════════════

seen_sha = set()           # GitHub file SHA
seen_content_hash = set()  # İçerik MD5 hash (farklı repo ama aynı kod)


def content_hash(text: str) -> str:
    """Normalize edip MD5 hash al — whitespace/comment farkları yoksay."""
    # Boşlukları normalize et, yorumları sil
    lines = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped and not stripped.startswith("#"):
            lines.append(stripped)
    normalized = "\n".join(lines)
    return hashlib.md5(normalized.encode()).hexdigest()


def is_duplicate(sha: str, code: str) -> bool:
    """SHA veya içerik hash'i ile tekrar kontrolü."""
    if sha in seen_sha:
        return True
    chash = content_hash(code)
    if chash in seen_content_hash:
        return True
    seen_sha.add(sha)
    seen_content_hash.add(chash)
    return False


# ═══════════════════ VALIDATION ═══════════════════

def is_valid_ros2_python(code: str) -> bool:
    """Dosyanın gerçekten ROS2 Python kodu olup olmadığını kontrol et."""
    # En az birinden bahsetmeli
    ros2_indicators = [
        "rclpy", "Node", "ros2", "ROS2",
        "JointTrajectory", "moveit", "MoveIt",
        "joint_states", "sensor_msgs", "geometry_msgs",
        "trajectory_msgs", "control_msgs", "moveit_msgs",
    ]
    has_ros2 = any(ind in code for ind in ros2_indicators)
    has_python = "import " in code or "def " in code or "class " in code
    return has_ros2 and has_python


def generate_instruction(filename: str, category: str, desc: str) -> str:
    """Dosya adı ve kategoriye göre instruction metnini üret."""
    safe_name = Path(filename).stem
    return f"Write a ROS2 Python node named {filename} that {desc}."


# ═══════════════════ GITHUB API ═══════════════════

def search_github_code(query: str, per_page: int = 30) -> list:
    """GitHub Code Search API ile arama yap."""
    url = "https://api.github.com/search/code"
    params = {
        "q": query,
        "per_page": min(per_page, 100),
        "sort": "indexed",
        "order": "desc",
    }
    try:
        resp = requests.get(url, headers=HEADERS, params=params, timeout=30)
        if resp.status_code == 403:
            # Rate limit — bekle
            reset_time = int(resp.headers.get("X-RateLimit-Reset", 0))
            wait = max(reset_time - int(time.time()), 60)
            print(f"  ⏳ Rate limit! {wait}s bekleniyor...")
            time.sleep(wait + 1)
            return search_github_code(query, per_page)
        if resp.status_code == 422:
            print(f"  ⚠️  Geçersiz query: {query}")
            return []
        resp.raise_for_status()
        return resp.json().get("items", [])
    except requests.exceptions.RequestException as e:
        print(f"  ❌ API hatası: {e}")
        return []


def fetch_file_content(url: str) -> str | None:
    """Dosya içeriğini GitHub Contents API ile çek."""
    try:
        resp = requests.get(url, headers=HEADERS, timeout=30)
        if resp.status_code != 200:
            return None
        data = resp.json()
        if data.get("encoding") == "base64" and data.get("content"):
            return base64.b64decode(data["content"]).decode("utf-8", errors="replace")
        return None
    except Exception:
        return None


# ═══════════════════ MAIN ═══════════════════

def main():
    print("═" * 60)
    print("  A4 Projesi — GitHub ROS2/MoveIt2 Code Scraper")
    print("═" * 60)
    print(f"  Token: ...{GITHUB_TOKEN[-8:]}")
    print(f"  Toplam query sayısı: {len(SEARCH_QUERIES)}")
    print(f"  Çıktı: {OUTPUT_FILE}")
    print("═" * 60)

    results = []  # list of {instruction, response}
    stats = {"safe": 0, "unsafe_pattern": 0, "ur_specific": 0, "skipped_dup": 0, "skipped_invalid": 0, "skipped_size": 0}

    for i, sq in enumerate(SEARCH_QUERIES, 1):
        query = sq["query"]
        category = sq["category"]
        desc = sq["desc"]

        print(f"\n[{i}/{len(SEARCH_QUERIES)}] 🔍 {query}")
        print(f"  Kategori: {category} | Açıklama: {desc}")

        items = search_github_code(query, MAX_RESULTS_PER_QUERY)
        print(f"  Bulunan: {len(items)} dosya")

        for item in items:
            sha = item.get("sha", "")
            filename = item.get("name", "unknown.py")
            file_url = item.get("url", "")  # Contents API URL
            repo_name = item.get("repository", {}).get("full_name", "unknown")

            if not filename.endswith(".py"):
                continue

            # Dosya içeriğini çek
            code = fetch_file_content(file_url)
            if code is None:
                continue

            # Boyut kontrolü
            code_bytes = len(code.encode("utf-8"))
            if code_bytes < MIN_FILE_SIZE or code_bytes > MAX_FILE_SIZE:
                stats["skipped_size"] += 1
                continue

            # Tekrar kontrolü
            if is_duplicate(sha, code):
                stats["skipped_dup"] += 1
                continue

            # ROS2 Python kontrolü
            if not is_valid_ros2_python(code):
                stats["skipped_invalid"] += 1
                continue

            # Instruction üret
            instruction = generate_instruction(filename, category, desc)

            entry = {
                "instruction": instruction,
                "response": code,
            }
            results.append(entry)
            stats[category] += 1

            print(f"    ✓ {repo_name}/{filename} ({code_bytes}B)")

            # API rate limit koruması (dosya çekme)
            time.sleep(0.5)

        # Code Search rate limit: 10 req/min
        if i < len(SEARCH_QUERIES):
            print(f"  ⏳ API rate limit bekleniyor ({API_DELAY}s)...")
            time.sleep(API_DELAY)

    # ═══ KAYDET ═══
    print("\n" + "═" * 60)
    print("  SONUÇLAR")
    print("═" * 60)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        for entry in results:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")

    print(f"  ✅ Güvenli kodlar: {stats['safe']}")
    print(f"  ⚠️  Güvensiz kalıp: {stats['unsafe_pattern']}")
    print(f"  🤖 UR spesifik:   {stats['ur_specific']}")
    print(f"  ───────────────────")
    print(f"  📊 TOPLAM: {len(results)} satır → {OUTPUT_FILE}")
    print(f"  🔄 Tekrar atlandı: {stats['skipped_dup']}")
    print(f"  ❌ Geçersiz atlandı: {stats['skipped_invalid']}")
    print(f"  📏 Boyut atlandı: {stats['skipped_size']}")
    print("═" * 60)

    # JSON validasyonu
    print("\n🔍 JSON Validasyonu...")
    try:
        with open(OUTPUT_FILE, "r") as f:
            for line_no, line in enumerate(f, 1):
                json.loads(line)
        print(f"  ✅ {line_no} satırın tamamı geçerli JSON!")
    except json.JSONDecodeError as e:
        print(f"  ❌ Satır {line_no}: {e}")


if __name__ == "__main__":
    main()
