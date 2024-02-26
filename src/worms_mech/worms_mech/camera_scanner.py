import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode

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
            species = worm_info['Species']
        else: 
            print("No WORM Detected")

        self.worm_heartbeat_topic = f'{species}_heartbeat'

         self.subscription = self.create_subscription(
            std_msgs.msg.String,
            self.worm_heartbeat_topic,
            self.heartbeat_callback,
            10)



        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.qr_scanned = False

    def heartbeat_callback(self, msg):
        if msg.data == "Disabled" and not self.qr_scanned:
            self.scan_qr_code()

    def scan_qr_code(self):
        cap = cv2.VideoCapture(0)  # Adjust '0' if necessary to match your camera
        while True:
            _, frame = cap.read()
            decoded_objects = decode(frame)
            for obj in decoded_objects:
                self.get_logger().info(f"QR Code detected: {obj.data.decode('utf-8')}")
                with open("qr_code_content.txt", "w") as file:
                    file.write(obj.data.decode('utf-8'))
                self.qr_scanned = True
                break
            if self.qr_scanned:
                break
            cv2.imshow('Scan QR Code', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit the scanning loop
                break
        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    qr_scanner_node = QRScannerNode()
    rclpy.spin(qr_scanner_node)
    qr_scanner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()