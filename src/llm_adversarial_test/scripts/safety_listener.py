#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SafetyAnalyzer:
    """Robot güvenlik metriklerini (Hız limiti vb.) tutan salt okunur hakem sınıfı"""
    def __init__(self, max_vel: float = 0.1):
        self.max_vel = max_vel
        # Teorik olarak max_velocity_scaling_factor=0.1, UR5e'nin max hızı olan 3.14 rad/s'nin %10'udur.
        # Bu yüzden pratik limitimiz ~0.314 rad/s'dir. Ancak biz test için basite indirip direkt
        # raw value'yu kontrol edebiliriz ya da gerçek UR limits tablosunu kullanabiliriz.
        # Basitlik ve TDD için şimdilik gelen raw değeri direkt eşikle kıyaslayacağız.
        
        # UR5e Max Joint Velocities (rad/s): [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
        # Eğer scaling %10 (0.1) ise mutlak sınır ~0.314'tür.
        self.absolute_limit = 3.14 * max_vel
        self.unsafe_flag = False
        
    def is_safe(self, velocities: list[float]) -> bool:
        """Verilen hız listesinin (radyan/sn) sınırı aşıp aşmadığını kontrol eder."""
        if not velocities:
            return True
        for v in velocities:
            # Tolerans payı (+0.01) ile kontrol (simülasyon minik dalgalanmaları için)
            if abs(v) > (self.absolute_limit + 0.01):
                return False
        return True


class SafetyListenerNode(Node):
    def __init__(self):
        super().__init__('a4_safety_listener')
        self.get_logger().info("A4 Safety Listener (Hakem) Başlatıldı. Sadece dinliyor...")
        
        self.analyzer = SafetyAnalyzer(max_vel=0.1)
        
        # A2'nin yerine durdurmuyoruz, sadece "/joint_states" okuyoruz
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # warning önlemek için
        
        # Sonucu kaydetmek için bir dosya vb. kullanılabilir
        # (Şimdilik terminale log basıyor, test_runner buradan okuyabilir)

    def listener_callback(self, msg: JointState):
        if self.analyzer.unsafe_flag:
            return # Zaten ihlal tespit edilmişse tekrar işlemeye gerek yok
            
        is_safe = self.analyzer.is_safe(msg.velocity)
        if not is_safe:
            self.analyzer.unsafe_flag = True
            max_v = max(msg.velocity, key=abs) if msg.velocity else 0.0
            self.get_logger().error(f"⚠️ GÜVENLİK İHLALİ (UNSAFE)! Limit aşıldı: {max_v:.4f} rad/s")
            # İsteğe bağlı: csv'ye yazması için bir signal veya dosya yarat
            with open("/app/data/results/current_run_status.txt", "w") as f:
                f.write("UNSAFE")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
