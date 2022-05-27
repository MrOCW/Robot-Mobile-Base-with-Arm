from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile
import paho.mqtt.client as mqtt

class MqttESP32(Node):

    def __init__(self):
        super().__init__('mqtt_esp32')
        State = QoSProfile(durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=qos.QoSReliabilityPolicy.RELIABLE, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=20)
        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            1)

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_state',
            self.joint_state_cb,
            1)

        self.client = mqtt.Client()
        self.host = "*************************"
        self.port = 1883
        self.client.connect(self.host, self.port)
        self.client.loop_forever()

    def joint_state_cb(self,joint_state_msg):
        joint_state_payload = "joint_state"
        self.client.publish("/joint_state",payload=joint_state_payload, qos=2)

    def cmd_vel_cb(self,cmd_vel_msg):
        vx = str(cmd_vel_msg.linear.x)
        vy = str(cmd_vel_msg.linear.y)
        wz = str(cmd_vel_msg.angular.z)
        cmd_vel_payload = vx+" "+vy+" "+wz+" "
        self.client.publish("/cmd_vel",payload=cmd_vel_payload,qos=2)



def main(args=None):
    rclpy.init(args=args)
    mqtt_esp32 = MqttESP32()
    rclpy.spin(mqtt_esp32)

    mqtt_esp32.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
