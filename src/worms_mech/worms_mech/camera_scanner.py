import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
import os
import time

from std_msgs.msg import String
    
import time
import lgpio


import pandas as pd
import subprocess


def get_mac_address():
    
        mac_address = subprocess.check_output(f"cat /sys/class/net/wlan0/address", shell=True).decode().strip()
        
        if mac_address:
            return mac_address
            
        print("Error getting MAC address: No suitable interface found")
        return None

def find_robot_info(mac_address, spreadsheet_path):
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['MAC Address'] == mac_address, ['Species', 'Motor1_Direction', 'Motor2_Direction', 'Motor3_Direction']]
    if not match.empty:
        return match.iloc[0]
    else:
        return None

class QRScannerNode(Node):

    def __init__(self):

        super().__init__('camera_scanner')
       
        # Construct the path to the CSV file
        spreadsheet_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/database.csv')

        mac_address = get_mac_address()

        worm_info = find_robot_info(mac_address, spreadsheet_path)

        # Check if worm_info is not None
        if worm_info is not None:
            worm_id = worm_info['Species']
        else: 
            print("No WORM Detected")

        self.LED = 21
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.LED)
        
        self.get_logger().info("GPIO INITIALIZED")

        self.worm_heartbeat_topic = f'{worm_id}_heartbeat'

        self.subscription = self.create_subscription(
            String,
            self.worm_heartbeat_topic,
            self.heartbeat_callback,
            10)


        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.qr_scanned = False
        self.motor_state = "Disabled"
        self.last_head = None

        self.get_logger().info("Reading Video Feed")
        self.cap = cv2.VideoCapture(0)  # Adjust '0' if necessary to match your camera
        self.get_logger().info("Initialized Video Capture")

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Open the file and read its contents
        path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/head.txt')


    def heartbeat_callback(self, msg):
        self.motor_state = msg.data
        self.get_logger().info("MOTORS ENABLED. STOPPING SEARCH")
        

    def timer_callback(self):

        if(self.motor_state == "Disabled"):

            with open(path, 'r') as file:
                self.last_head = file.readline().strip()

            self.get_logger().info("Looking for QR Code ʕ•ᴥ•ʔ")
            _, frame = self.cap.read()
            decoded_objects = decode(frame)
            
            for obj in decoded_objects:
                self.get_logger().info(f"QR Code detected: {obj.data.decode('utf-8').upper()}")

                with open(path, "w") as file:
                    file.write(obj.data.decode('utf-8').upper())

                self.qr_scanned = True

                
                if (self.qr_scanned and (obj.data.decode('utf-8').upper() != self.last_head)):
                    self.last_head = obj.data.decode('utf-8').upper()
                    lgpio.gpio_write(self.h, self.LED, 1)
                    time.sleep(0.5)
                    lgpio.gpio_write(self.h, self.LED, 0)
                    break

                elif((obj.data.decode('utf-8').upper() == self.last_head)):
                    self.get_logger().info(f"Still Connected to: {obj.data.decode('utf-8').upper()}. Waiting to Detect New QR")

        else:
            self.get_logger().info(f"Current QR Code on File: {self.last_head}")    

    
    def on_shutdown(self):
        self.get_logger().info("Disabling GPIO...")
        lgpio.gpiochip_close(self.h)
        cv2.destroyAllWindows()
        self.cap.release()
        self.get_logger().info("Capture Released")




def main(args=None):

    rclpy.init(args=args)
    qr_scanner_node = QRScannerNode()
    try:
        rclpy.spin(qr_scanner_node)
    except KeyboardInterrupt:
        pass
    #THIS PART SHUTSDOWN THE BUZZER

    qr_scanner_node.on_shutdown()
    qr_scanner_node.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()