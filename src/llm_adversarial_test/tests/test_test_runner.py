"""
TDD ADIM 2 — RED AŞAMASI (test_runner için)
=============================================
Bu testler, test_runner modülünün şu işlevlerini kontrol eder:
  1. YAML'dan prompt'ları okuma
  2. LLM'e prompt gönderip kod alma
  3. Üretilen kodu safety_analyzer ile analiz etme
  4. Sonuçları CSV'ye kaydetme
"""
import unittest
import sys
import os
import csv
import tempfile
from unittest.mock import patch, MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from scripts.test_runner import (
    load_prompts,
    extract_code_from_response,
    run_single_experiment,
    save_results_to_csv,
    run_in_sandbox,
)


# ─────────────────────────────────────────────────
# TEST 1: YAML'dan prompt yükleme
# ─────────────────────────────────────────────────
class TestLoadPrompts(unittest.TestCase):
    """YAML dosyasından prompt'ları doğru okuyabilmeli"""

    def setUp(self):
        # Proje kök dizinini test dosyasının konumundan bul
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        self.yaml_path = os.path.join(base_dir, "data", "prompts", "adversarial_prompts.yaml")

    def test_loads_all_prompts(self):
        """3 görev × 5 varyant = 15 prompt yüklenmeli"""
        prompts = load_prompts(self.yaml_path)
        self.assertGreaterEqual(len(prompts), 15, f"En az 15 prompt olmalı, {len(prompts)} bulundu")

    def test_prompt_has_required_fields(self):
        """Her prompt'ta id, task, category, prompt alanları olmalı"""
        prompts = load_prompts(self.yaml_path)
        first = prompts[0]
        self.assertIn("id", first)
        self.assertIn("task", first)
        self.assertIn("category", first)
        self.assertIn("prompt", first)


# ─────────────────────────────────────────────────
# TEST 2: LLM yanıtından kod çıkarma
# ─────────────────────────────────────────────────
class TestExtractCode(unittest.TestCase):
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
        self.assertIn("import rclpy", code)
        self.assertIn("print", code)

    def test_extracts_plain_code(self):
        """Markdown işaretlemesi olmadan da kodu almalı"""
        response = '''import rclpy
from rclpy.node import Node
node = Node("test")'''
        code = extract_code_from_response(response)
        self.assertIn("import rclpy", code)

    def test_empty_response(self):
        """Boş yanıt → boş string dönmeli"""
        code = extract_code_from_response("")
        self.assertEqual(code, "")


# ─────────────────────────────────────────────────
# TEST 3: CSV kaydetme
# ─────────────────────────────────────────────────
class TestSaveResults(unittest.TestCase):
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
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["prompt_id"], "pose_baseline")
            self.assertEqual(rows[0]["safety_score"], "80")

        os.unlink(tmpfile)

# ─────────────────────────────────────────────────
# TEST 4: Sandbox çalıştırma testi (Execution)
# ─────────────────────────────────────────────────
class TestSandboxExecution(unittest.TestCase):
    """Sandbox içinde LLM kodunun düzgün çalışıp çalışmadığını (execution) test eder"""
    
    @patch('scripts.test_runner.subprocess.run')
    @patch('scripts.test_runner.os.makedirs')
    @patch('builtins.open', new_callable=unittest.mock.mock_open)
    def test_run_in_sandbox_success(self, mock_open, mock_makedirs, mock_run):
        """Çalışan kod sonucu Success (Exit 0) dönmeli"""
        mock_result = MagicMock()
        mock_result.returncode = 0
        mock_run.return_value = mock_result
        
        result = run_in_sandbox("print('hello')", "test_id", "test_model")
        
        self.assertTrue(result["execution_success"])
        self.assertIn("Success", result["execution_msg"])
        mock_run.assert_called_once()
        
    @patch('scripts.test_runner.subprocess.run')
    @patch('scripts.test_runner.os.makedirs')
    @patch('builtins.open', new_callable=unittest.mock.mock_open)
    def test_run_in_sandbox_crash(self, mock_open, mock_makedirs, mock_run):
        """Hata veren kod sonucu Crash dönmeli"""
        mock_result = MagicMock()
        mock_result.returncode = 1
        mock_result.stderr = "Traceback SyntaxError: invalid syntax"
        mock_run.return_value = mock_result
        
        result = run_in_sandbox("prnt('hello')", "test_id", "test_model")
        
        self.assertFalse(result["execution_success"])
        self.assertIn("Crash", result["execution_msg"])
        self.assertIn("SyntaxError", result["execution_msg"])

if __name__ == '__main__':
    unittest.main()
