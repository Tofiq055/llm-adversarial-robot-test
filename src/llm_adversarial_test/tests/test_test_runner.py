"""
TDD ADIM 2 — RED AŞAMASI (test_runner için)
=============================================
Bu testler, test_runner modülünün şu işlevlerini kontrol eder:
  1. YAML'dan prompt'ları okuma
  2. LLM'e prompt gönderip kod alma
  3. Üretilen kodu safety_analyzer ile analiz etme
  4. Sonuçları CSV'ye kaydetme
"""
import pytest
import sys
import os
import csv
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from scripts.test_runner import (
    load_prompts,
    extract_code_from_response,
    run_single_experiment,
    save_results_to_csv,
)


# ─────────────────────────────────────────────────
# TEST 1: YAML'dan prompt yükleme
# ─────────────────────────────────────────────────
class TestLoadPrompts:
    """YAML dosyasından prompt'ları doğru okuyabilmeli"""

    def test_loads_all_prompts(self):
        """3 görev × 5 varyant = 15 prompt yüklenmeli"""
        prompts = load_prompts("/app/data/prompts/adversarial_prompts.yaml")
        assert len(prompts) >= 15, f"En az 15 prompt olmalı, {len(prompts)} bulundu"

    def test_prompt_has_required_fields(self):
        """Her prompt'ta id, task, category, prompt alanları olmalı"""
        prompts = load_prompts("/app/data/prompts/adversarial_prompts.yaml")
        first = prompts[0]
        assert "id" in first
        assert "task" in first
        assert "category" in first
        assert "prompt" in first


# ─────────────────────────────────────────────────
# TEST 2: LLM yanıtından kod çıkarma
# ─────────────────────────────────────────────────
class TestExtractCode:
    """LLM yanıtından Python kod bloğunu ayıklayabilmeli"""

    def test_extracts_code_from_markdown(self):
        """```python ... ``` blokları arasındaki kodu çıkarmalı"""
        response = '''Here is the code:

```python
import rclpy
print("hello")
```

This code prints hello.'''
        code = extract_code_from_response(response)
        assert "import rclpy" in code
        assert "print" in code

    def test_extracts_plain_code(self):
        """Markdown işaretlemesi olmadan da kodu almalı"""
        response = '''import rclpy
from rclpy.node import Node
node = Node("test")'''
        code = extract_code_from_response(response)
        assert "import rclpy" in code

    def test_empty_response(self):
        """Boş yanıt → boş string dönmeli"""
        code = extract_code_from_response("")
        assert code == ""


# ─────────────────────────────────────────────────
# TEST 3: CSV kaydetme
# ─────────────────────────────────────────────────
class TestSaveResults:
    """Sonuçları CSV dosyasına kaydedebilmeli"""

    def test_csv_created_with_headers(self):
        """CSV dosyası başlık satırıyla oluşturulmalı"""
        results = [
            {
                "prompt_id": "pose_baseline",
                "task": "task_pose",
                "category": "baseline",
                "safety_score": 80,
                "uses_moveit2": True,
                "has_collision_checking": True,
                "has_velocity_limit": True,
                "model": "dolphin-mistral:7b",
            }
        ]
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv',
                                         delete=False) as f:
            tmpfile = f.name

        save_results_to_csv(results, tmpfile)

        with open(tmpfile, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            assert len(rows) == 1
            assert rows[0]["prompt_id"] == "pose_baseline"
            assert rows[0]["safety_score"] == "80"

        os.unlink(tmpfile)
