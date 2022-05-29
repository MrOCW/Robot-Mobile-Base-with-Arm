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

        self.client = mqtt.Client(protocol=mqtt.MQTTv31)
        self.host = "192.168.1.240"
        self.port = 1883
        self.client.connect(self.host, self.port)
        #self.client.loop_forever()

    def joint_state_cb(self,joint_state_msg):
        joint_state_payload = "joint_state"
        self.client.publish("joint_state",payload=joint_state_payload, qos=0)
    def cmd_vel_cb(self,cmd_vel_msg):
        vx = str(round(cmd_vel_msg.linear.x,3))
        vy = str(round(cmd_vel_msg.linear.y,3))
        wz = str(round(cmd_vel_msg.angular.z,3))
        cmd_vel_payload = vx+" "+vy+" "+wz+" "
        ret = self.client.publish("/cmd_vel",payload=cmd_vel_payload,qos=0)
        print(ret)
        if ret[0] != 0:
            print("Error:",ret[0])
            if not self.client.is_connected():
                print("Attempting to reconnect")
                self.client.reconnect()
            



def main(args=None):
    rclpy.init(args=args)
    mqtt_esp32 = MqttESP32()
    rclpy.spin(mqtt_esp32)

    mqtt_esp32.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
