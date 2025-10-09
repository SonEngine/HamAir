import time 
from adafruit_servokit import ServoKit 

kit = ServoKit(channels=16) 
# 사용할 채널 (예: 0번 채널) 
SERVO_CH = 2 
kit = ServoKit(channels=16)

kit.servo[2].set_pulse_width_range(500, 2500)

def test_servo(ch):
    print(f"Testing servo on channel {ch}")
    kit.servo[ch].angle = 0
    kit.servo[ch].angle = 90
    kit.servo[ch].angle = 180


test_servo(2)