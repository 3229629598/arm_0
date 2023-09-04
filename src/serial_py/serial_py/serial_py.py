import rclpy
from rclpy.node import Node
import serial
from serial import SerialException
from threading import Thread
from std_msgs.msg import UInt8MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from .serial_config import *

class serial_node(Node):
    def __init__(self):
        super().__init__("serial_node")

        self.get_logger().info("Serial_node is running.")
        self.rx_data=UInt8MultiArray()
        #创建允许无限制并行执行回调的回调组
        self._callback_group = ReentrantCallbackGroup()
        #串口初始化
        self.ser = serial.Serial()
        self.ser.port = port_name 
        self.ser.baudrate = port_baudrate
        self.ser.bytesize = port_bytesize
        self.ser.stopbits = port_stopbits
        self.ser.parity = port_parity
        self.ser.timeout = port_timeout       
        #为send_data创建订阅者
        self.create_subscription(
            msg_type=UInt8MultiArray,
            topic="/tx_data",
            callback=self.tx_callback,
            qos_profile=QoSProfile(depth=10),
            callback_group=self._callback_group,
        )
        #为receive_data创建发布者
        self.rx_pub = self.create_publisher(
            msg_type=UInt8MultiArray,
            topic="/rx_data",
            qos_profile=QoSProfile(depth=10),
        )

    def port_open(self):
        while True:
            try:
                self.ser.open()
                self.get_logger().info("port open success")
                break
            except SerialException:
                None

    def port_restart(self):
        try:
            self.ser.close()
            self.ser.open()
            self.get_logger().info("serial reset success.")
        except:
            None

    def tx_callback(self,tx_data:UInt8MultiArray):
        try:
            self.ser.write(tx_data.data)
        except:
            self.port_restart()

    def rx_callback(self):
        while True:
            try:
                count=self.ser.inWaiting()
                if count>0:
                    self.rx_data.data=self.ser.read(count)
                    self.rx_pub.publish(self.rx_data)
                else:
                    None
            except SerialException:
                None
            except:
                self.port_restart()

def main(args=None):
    rclpy.init(args=args)
    node = serial_node()
    node.port_open()

    #多线程接收数据
    executor_thread = Thread(target=node.rx_callback, daemon=True, args=())
    executor_thread.start()

    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)
