import unittest
from unittest.mock import patch, MagicMock
import os
import sys

# Scraper'in bulunduğu dizini path'e ekle
# Scraper'in bulunduğu dizini path'e ekle
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../scripts')))
from github_ros2_scraper import search_github_files, generate_instruction_for_code, check_repo_stars

class TestGithubScraper(unittest.TestCase):

    @patch('github_ros2_scraper.requests.get')
    def test_search_github_files_success(self, mock_get):
        # API'den gelecek sanal (mock) cevabı ayarla
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {
            "items": [
                {"name": "test_node.py", "path": "src/test_node.py", "url": "http://api.github.com/123", "repository": {"full_name": "user/repo"}}
            ]
        }
        mock_get.return_value = mock_response

        # Fonksiyonu çalıştır (Gerçek internete gitmeyecek, mock cevabı alacak)
        results = search_github_files("rclpy moveit", max_results=1)
        
        # Sonuçları kontrol et
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["name"], "test_node.py")

    @patch('github_ros2_scraper.requests.get')
    def test_search_github_files_rate_limit(self, mock_get):
        # API Rate Limit (403) hatası simülasyonu
        mock_response = MagicMock()
        mock_response.status_code = 403
        mock_get.return_value = mock_response

        results = search_github_files("rclpy moveit", max_results=1)
        
        # Hata durumunda boş liste dönmeli
        self.assertEqual(results, [])

    def test_generate_instruction(self):
        # Basit string formatting testi
        code = "import rclpy"
        filename = "ur5e_controller.py"
        expected = "Write a ROS2 Python node named ur5e_controller.py that uses rclpy and MoveIt2 for robotic manipulation."
        
        result = generate_instruction_for_code(code, filename)
        self.assertEqual(result, expected)

    @patch('github_ros2_scraper.requests.get')
    def test_check_repo_stars(self, mock_get):
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {"stargazers_count": 42}
        mock_get.return_value = mock_response

        stars = check_repo_stars("user/repo", {})
        self.assertEqual(stars, 42)
        mock_get.assert_called_with("https://api.github.com/repos/user/repo", headers={})

if __name__ == '__main__':
    unittest.main()
