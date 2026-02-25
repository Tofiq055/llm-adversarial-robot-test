import unittest
import sys
import os

# finetune_qwen.py scriptini import edebilmek için yolu ekleyelim
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))

try:
    from finetune_qwen import formatting_prompts_func
except ImportError as e:
    print(f"ImportError: {e}")
    formatting_prompts_func = None

class TestFinetuneScript(unittest.TestCase):
    def setUp(self):
        if formatting_prompts_func is None:
            self.skipTest("finetune_qwen.py not found or not importable")

    def test_formatting_prompts_func_single_example(self):
        """Tek bir örneğin (instruction & response) doğru Alpaca formatına çevrilmesini test eder."""
        example = {
            "instruction": ["Write a generic ROS2 MoveIt2 code"],
            "response": ["import rclpy\n# MoveIt2 Code..."]
        }
        
        result = formatting_prompts_func(example, eos_token="<EOS>")
        
        self.assertIn("text", result)
        self.assertEqual(len(result["text"]), 1)
        
        formatted_text = result["text"][0]
        self.assertIn("Below is an instruction that describes a task.", formatted_text)
        self.assertIn("### Instruction:\nWrite a generic ROS2 MoveIt2 code", formatted_text)
        self.assertIn("### Response:\nimport rclpy\n# MoveIt2 Code...", formatted_text)
        self.assertTrue(formatted_text.endswith("<EOS>"))

    def test_formatting_prompts_func_batch(self):
        """Birden fazla örneğin (batch) doğru çevrilip listeye çevrilmesini test eder."""
        example = {
            "instruction": ["Task 1", "Task 2"],
            "response": ["Response 1", "Response 2"]
        }
        
        result = formatting_prompts_func(example, eos_token="<EOS>")
        
        self.assertIn("text", result)
        self.assertEqual(len(result["text"]), 2)
        
        self.assertIn("Task 1", result["text"][0])
        self.assertIn("Task 2", result["text"][1])
        self.assertTrue(result["text"][0].endswith("<EOS>"))
        self.assertTrue(result["text"][1].endswith("<EOS>"))

    def test_formatting_prompts_func_empty(self):
        """Boş bir giriş listesi durumunda gracefully çalışmasını test eder."""
        example = {"instruction": [], "response": []}
        result = formatting_prompts_func(example, eos_token="<EOS>")
        self.assertEqual(result["text"], [])

if __name__ == '__main__':
    unittest.main()
