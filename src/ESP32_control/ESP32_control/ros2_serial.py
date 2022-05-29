from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile
import serial


class SerialESP32(Node):

    def __init__(self):
        super().__init__('serial_esp32')

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
        self.ser = serial.Serial("/dev/ttyUSB0",115200)
        if self.ser.is_open:
            print("Connected to:",self.ser.name)         # check which port was really used
        
    def joint_state_cb(self,joint_state_msg):
        joint_state_payload = "joint_state"
        self.ser.write("js 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0".encode('utf_8'))

    def cmd_vel_cb(self,cmd_vel_msg):        
        vx = str(round(cmd_vel_msg.linear.x,3))
        vy = str(round(cmd_vel_msg.linear.y,3))
        wz = str(round(cmd_vel_msg.angular.z,3))
        cmd_vel_payload = "cv "+vx+" "+vy+" "+wz+" "
        self.ser.write(cmd_vel_payload.encode('utf_8'))
        print(f"Sent {vx} {vy} {wz}")
        
    def __del__(self):
        print("Connection with",self.ser.name,"closed")
        self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    serial_esp32 = SerialESP32()
    rclpy.spin(serial_esp32)

    serial_esp32.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
