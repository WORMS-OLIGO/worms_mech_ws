import cv2
from pyzbar.pyzbar import decode
import os
import time
import lgpio

class QRScanner:
    def __init__(self):
        self.LED = 21
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.LED)

        print("GPIO INITIALIZED")

        self.cap = cv2.VideoCapture(0)  # Adjust '0' if necessary to match your camera
        print("Initialized Video Capture")

        # Open the file and read its contents
        self.path = os.path.expanduser('src/worms_mech/worms_mech/head.txt')

    def read_last_qr_code(self):
        try:
            with open(self.path, 'r') as file:
                last_head = file.readline().strip()
        except FileNotFoundError:
            last_head = None
        return last_head

    def write_qr_code(self, qr_data):
        with open(self.path, "w") as file:
            file.write(qr_data)

    def scan_qr_codes(self):
        try:
            while True:
                last_head = self.read_last_qr_code()
                print("Last Head Was: " + last_head)
                print("Looking for QR Code ʕ•ᴥ•ʔ")
                _, frame = self.cap.read()
                decoded_objects = decode(frame)

                for obj in decoded_objects:
                    qr_data = obj.data.decode('utf-8').upper()
                    print(f"QR Code detected: {qr_data}")

                    if qr_data != last_head:
                        self.write_qr_code(qr_data)
                        print(f"New QR Code detected and saved: {qr_data}")
                        lgpio.gpio_write(self.h, self.LED, 1)
                        time.sleep(0.5)
                        lgpio.gpio_write(self.h, self.LED, 0)
                        break  # Exit the for loop after handling a new QR code
                    else:
                        print(f"Still Connected to: {qr_data}. Waiting to Detect New QR")

        except KeyboardInterrupt:
            self.on_shutdown()

    def on_shutdown(self):
        print("Disabling GPIO...")
        lgpio.gpiochip_close(self.h)
        cv2.destroyAllWindows()
        self.cap.release()
        print("Capture Released")

if __name__ == '__main__':
    qr_scanner = QRScanner()
    qr_scanner.scan_qr_codes()