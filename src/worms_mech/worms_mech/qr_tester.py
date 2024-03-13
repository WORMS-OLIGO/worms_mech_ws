import cv2
from pyzbar.pyzbar import decode

def main():

    # Reading Video Feed
    cap = cv2.VideoCapture(0) # Adjust '0' if necessary to match your camera
    print("Initialized Video Capture")

    last_head = None
    while True:
    # Looking for QR Code
        _, frame = cap.read()
        decoded_objects = decode(frame)
        for obj in decoded_objects:
            print(f"QR Code detected: {obj.data.decode('utf-8').upper()}")

            if last_head is None or obj.data.decode('utf-8').upper() != last_head:
                last_head = obj.data.decode('utf-8').upper()
                break

            else:
                print(f"Still Connected to: {obj.data.decode('utf-8').upper()}. Waiting to Detect New QR")

        # Display the video feed
        cv2.imshow('Video Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Release the video capture
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()