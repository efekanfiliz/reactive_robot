import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteEntity


class ReactiveObstacleAvoider(Node):
    """2D LiDAR verisini kullanarak 60 saniye boyunca engellerden pürüzsüzce kaçınan ROS 2 düğümü."""

    def __init__(self):
        super().__init__("reactive_obstacle_avoider")

        # reactive_robot.launch.py dosyasından çekilecek hız ve mesafe parametrelerini tanımla
        self.declare_parameter("max_linear_speed", 0.22)
        self.declare_parameter("min_linear_speed", 0.06)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("safe_distance", 1.25)
        self.declare_parameter("danger_distance", 0.45)
        self.declare_parameter("run_duration", 60.0)

        # Launch dosyasından gelen parametre değerlerini değişkenlere ata
        self.max_lin = self.get_parameter("max_linear_speed").value
        self.min_lin = self.get_parameter("min_linear_speed").value
        self.max_ang = self.get_parameter("max_angular_speed").value
        self.safe_dist = self.get_parameter("safe_distance").value
        self.danger_dist = self.get_parameter("danger_distance").value
        self.run_duration = self.get_parameter("run_duration").value

        # Robota doğrusal ve açısal hız komutları göndermek için /cmd_vel yayıncısı (publisher)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Lazer tarayıcı mesafe verilerini okumak için /scan abonesi (subscriber)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        # 60 saniye dolduğunda Gazebo ortamındaki sun ışığını silmek için servis istemcisi
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")

        # Terminal loglarının yalnızca hareket durumu değiştiğinde basılması için durum takibi
        self.current_state = None
        self.is_finished = False

        # 60 saniyelik görev süresini ölçen zamanlayıcı (timer)
        self.shutdown_timer = self.create_timer(
            self.run_duration, self.time_limit_callback
        )
        self.get_logger().info("Reactive Obstacle Avoider started. Moving for 60 seconds.")

    def clean_ranges(self, raw_slice, range_max):
        """Geçersiz (inf/nan) lazer ölçümlerini maksimum menzille değiştirir ve en yakın mesafeyi döndürür."""
        valid = [
            r if (not math.isnan(r) and not math.isinf(r) and r > 0.05) else range_max
            for r in raw_slice
        ]
        return min(valid) if valid else range_max

    def scan_callback(self, msg):
        """LaserScan bölgelerini analiz ederek orantısal (smooth) hız ve dönüş komutları üretir."""
        if self.is_finished:
            return

        n = len(msg.ranges)
        if n == 0:
            return

        # 360 derecelik lazer verisini ön, sol ve sağ algılama konilerine böl
        front = list(msg.ranges[0:int(n * 0.10)]) + list(msg.ranges[int(n * 0.90):n])
        left = list(msg.ranges[int(n * 0.10):int(n * 0.30)])
        right = list(msg.ranges[int(n * 0.70):int(n * 0.90)])

        # Her bölgedeki en yakın engel mesafesini hesapla
        f_min = self.clean_ranges(front, msg.range_max)
        l_min = self.clean_ranges(left, msg.range_max)
        r_min = self.clean_ranges(right, msg.range_max)

        cmd = Twist()

        # Ön ve yan koridorlar yeterince açıksa maksimum hızda düz ilerle
        if f_min >= self.safe_dist and l_min >= 0.55 and r_min >= 0.55:
            cmd.linear.x = self.max_lin
            cmd.angular.z = 0.0
            state = "Moving straight."
        else:
            # Engele yaklaştıkça ileri hızı orantısal olarak düşür (durduğu yerde dönmesini engelle)
            closeness = max(
                0.0,
                min(1.0, (f_min - self.danger_dist) / (self.safe_dist - self.danger_dist)),
            )
            cmd.linear.x = self.min_lin + closeness * (self.max_lin - self.min_lin)

            # Engele yaklaştıkça dönüş hızını orantısal olarak artır
            turn = 0.35 + (1.0 - closeness) * (self.max_ang - 0.35)

            # Sol veya sağ taraftan hangisi daha boşsa o yöne doğru manevra yap
            if l_min >= r_min:
                cmd.angular.z = turn
                state = "Started rotating left."
            else:
                cmd.angular.z = -turn
                state = "Started rotating right."

        # Hareket durumu her değiştiğinde terminalde bilgi mesajı göster
        if state != self.current_state:
            self.current_state = state
            self.get_logger().info(state)

        # Hesaplanan hız komutunu robota gönder
        self.cmd_pub.publish(cmd)

    def time_limit_callback(self):
        """60. saniyede robotu durdurur, /scan aboneliğini kapatır, sun ışığını siler ve düğümü kapatır."""
        if self.is_finished:
            return
        self.is_finished = True
        self.shutdown_timer.cancel()

        self.get_logger().info("Time limit reached. Stopping...")

        # Robotu durdurmak için sıfır hız komutu yayınla
        self.cmd_pub.publish(Twist())

        # /scan topic aboneliğini sonlandır (destroy_subscription)
        if self.scan_sub is not None:
            self.destroy_subscription(self.scan_sub)
            self.scan_sub = None
            self.get_logger().info("Unsubscribed from /scan.")

        # Gazebo /delete_entity servisini çağırarak sun nesnesini ortamdan kaldır
        if self.delete_client.wait_for_service(timeout_sec=2.0):
            req = DeleteEntity.Request()
            req.name = "sun"
            fut = self.delete_client.call_async(req)
            fut.add_done_callback(self.on_sun_deleted)
        else:
            self.get_logger().warn("/delete_entity service unavailable. Shutting down node.")
            rclpy.shutdown()

    def on_sun_deleted(self, fut):
        """Sun nesnesinin silinme sonucunu terminale yazdırır ve ROS 2 düğümünü kapatır."""
        try:
            res = fut.result()
            self.get_logger().info(
                f"Sun entity deleted (success={res.success}). Shutting down node."
            )
        except Exception as e:
            self.get_logger().error(f"Failed to delete sun: {e}")
        finally:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
