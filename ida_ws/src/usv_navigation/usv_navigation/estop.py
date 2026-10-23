#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RC CH9 -> Jetson GPIO + ROS yazılım E-Stop köprüsü.

- CH9 > threshold: /emergency_stop=True ve GPIO HIGH
- CH9 <= threshold: /emergency_stop=False ve GPIO LOW
- RC verisi bayatlarsa yazılım tarafı fail-safe olarak emergency_stop=True olur.
"""

import rclpy
from mavros_msgs.msg import RCIn
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool

import Jetson.GPIO as GPIO


GPIO_PIN = 7
PWM_THRESHOLD = 1600


class RCTriggerNode(Node):
    def __init__(self):
        super().__init__('rc_trigger_node')

        self.declare_parameter('gpio_pin', GPIO_PIN)
        self.declare_parameter('pwm_threshold', PWM_THRESHOLD)
        self.declare_parameter('rc_timeout_sec', 1.0)

        self.gpio_pin = int(self.get_parameter('gpio_pin').value)
        self.pwm_threshold = int(
            self.get_parameter('pwm_threshold').value
        )
        self.rc_timeout_sec = float(
            self.get_parameter('rc_timeout_sec').value
        )

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gpio_pin, GPIO.OUT, initial=GPIO.LOW)

        self.last_state = None
        self.last_rc_time = None
        self.rc_timeout_active = True

        estop_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.estop_pub = self.create_publisher(
            Bool, '/emergency_stop', estop_qos
        )

        self.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.2, self.watchdog_cb)

        # İlk geçerli RC paketi gelene kadar yazılım otonomisini durdur.
        self.estop_pub.publish(Bool(data=True))

        self.get_logger().info(
            f'MAVROS RC Trigger aktif | Pin={self.gpio_pin} | '
            f'CH9 threshold={self.pwm_threshold} | '
            f'RC timeout={self.rc_timeout_sec:.1f}s'
        )

    def _publish_estop(self, state):
        self.estop_pub.publish(Bool(data=bool(state)))

    def rc_callback(self, msg: RCIn):
        if len(msg.channels) < 9:
            self.get_logger().warn(
                '⚠️ RC paketinde CH9 yok; yazılım E-Stop aktif tutuluyor.',
                throttle_duration_sec=2.0,
            )
            self._publish_estop(True)
            return

        self.last_rc_time = self.get_clock().now()
        self.rc_timeout_active = False

        ch9_val = int(msg.channels[8])
        is_active = ch9_val > self.pwm_threshold

        # Her callback'te ROS durumunu tazele.
        self._publish_estop(is_active)

        if is_active != self.last_state:
            self.last_state = is_active

            if is_active:
                GPIO.output(self.gpio_pin, GPIO.HIGH)
                self.get_logger().warn(
                    f'🛑 [E-STOP AKTİF] CH9={ch9_val} -> '
                    f'Pin {self.gpio_pin} HIGH'
                )
            else:
                GPIO.output(self.gpio_pin, GPIO.LOW)
                self.get_logger().info(
                    f'✅ [E-STOP PASİF] CH9={ch9_val} -> '
                    f'Pin {self.gpio_pin} LOW'
                )

    def watchdog_cb(self):
        if self.last_rc_time is None:
            self._publish_estop(True)
            return

        age = (
            self.get_clock().now() - self.last_rc_time
        ).nanoseconds / 1e9

        if age > self.rc_timeout_sec:
            self._publish_estop(True)
            if not self.rc_timeout_active:
                self.rc_timeout_active = True
                self.get_logger().error(
                    f'❌ RC VERİSİ {age:.2f}s BAYAT. '
                    'Yazılım E-Stop fail-safe AKTİF.'
                )

    def destroy_node(self):
        try:
            # Mevcut donanım semantiğini değiştirmemek için GPIO çıkışı kapanışta LOW.
            GPIO.output(self.gpio_pin, GPIO.LOW)
            GPIO.cleanup()
        except Exception as e:
            print(f'GPIO cleanup hatası: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RCTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
