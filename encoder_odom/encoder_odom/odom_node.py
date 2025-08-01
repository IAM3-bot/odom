import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import serial
import time
import math
import re

class EncoderOdomPublisher(Node):
    def __init__(self):
        super().__init__('encoder_odom_publisher')

        # 시리얼 통신 설정
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200)
            time.sleep(2)
            self.get_logger().info("Serial connection established on /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise e

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ✅ 실측 기반 보정 계수: 1 tick 당 이동 거리 (m)
        # 예: 1m 이동 시 173 ticks → 1 / 173 ≈ 0.0057803
        self.m_per_tick = 1.0 / 173

        self.last_tick = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 50Hz 주기 타이머
        self.create_timer(0.02, self.update_odom)

    def update_odom(self):
        try:
            raw = self.ser.readline()
            tick_str = raw.decode("utf-8", errors="ignore").strip()

            # 숫자 추출 (음수 포함)
            numbers = re.findall(r'-?\d+', tick_str)
            if not numbers:
                self.get_logger().warn(f"Ignored non-numeric tick data: '{tick_str}'")
                return

            current_tick = int(numbers[0])
            self.get_logger().info(f"tick_str = '{tick_str}', parsed = {current_tick}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse tick: {e}")
            return

        if self.last_tick is None:
            self.last_tick = current_tick
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        delta_tick = current_tick - self.last_tick
        self.last_tick = current_tick

        # ✅ 오버플로우 보정 (16비트 unsigned → signed 대응용, 필요 시 사용)
        if delta_tick < -32768:
            delta_tick += 65536
        elif delta_tick > 32768:
            delta_tick -= 65536

        # ✅ 거리 계산: 실측 기반 보정값 사용
        distance = delta_tick * self.m_per_tick

        dx = distance * math.cos(self.theta)
        dy = distance * math.sin(self.theta)
        self.x += dx
        self.y += dy

        # 오도메트리 메시지 생성
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = distance / dt if dt > 0 else 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

