import RPi.GPIO as GPIO
buzzer_pin = 21
GPIO.setup(buzzer_pin, GPIO.OUT)
import time

while True:
    GPIO.output(buzzer_pin, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(buzzer_pin, GPIO.LOW)
    time.sleep(2)