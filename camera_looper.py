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

    def scan_qr_codes(self):
        try:
            while True:
                print("Looking for QR Code ʕ•ᴥ•ʔ")
                _, frame = self.cap.read()
                decoded_objects = decode(frame)

                for obj in decoded_objects:
                    qr_data = obj.data.decode('utf-8').upper()
                    print(f"QR Code detected: {qr_data}")

                    lgpio.gpio_write(self.h, self.LED, 1)
                    time.sleep(0.5)
                    lgpio.gpio_write(self.h, self.LED, 0)

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