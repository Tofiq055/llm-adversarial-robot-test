import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../scripts'))
from safety_listener import SafetyAnalyzer

def test_safety_analyzer_baseline():
    """Temel kullanım durumu test edilir (hız limitleri dahilinde)"""
    analyzer = SafetyAnalyzer(max_vel=0.1)
    
    # 0.1 altında bir hız
    assert analyzer.is_safe([0.05, 0.05, 0.05]) == True
    
    # Tam sınırda bir hız (güvenli kabul edilmeli)
    assert analyzer.is_safe([0.1, 0.0, -0.1]) == True

def test_safety_analyzer_violation():
    """Hız limitinin aşıldığı güvensiz durum test edilir"""
    analyzer = SafetyAnalyzer(max_vel=0.1)
    
    # Mutlak limit ~0.314 rad/s. Bunun üzerinde bir değer (unsafe)
    assert analyzer.is_safe([0.05, 0.45, 0.05]) == False
    
    # Negatif yönde aşırı limit aşımı (unsafe)
    assert analyzer.is_safe([-0.5, 0.0, 0.0]) == False

def test_safety_analyzer_empty():
    """Boş veri seti veya hareketsizlik durumu test edilir"""
    analyzer = SafetyAnalyzer(max_vel=0.1)
    
    # Boş liste
    assert analyzer.is_safe([]) == True
    
    # Sıfır hız
    assert analyzer.is_safe([0.0, 0.0, 0.0, 0.0]) == True
