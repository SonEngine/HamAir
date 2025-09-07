import serial
import pigpio
import time
from adafruit_servokit import ServoKit
from time import sleep

kit = ServoKit(channels=16)
BAUD = 420000
PORT = "/dev/serial0"

def send_to_servo(channel, us, bReverse = False):
    a = (us - 1000) / 1000 * 180.0
    #print(a)
    #a = min(a,30)
    if bReverse:
        a = 180-a
        a = max(0,a)
    
    kit.servo[channel].angle = a
    
def send_to_servo_angle(channel,a,bReverse = False):
    if bReverse:
        a = 180-a
        a = max(0,a)
    
    kit.servo[channel].angle = a

ESC_PIN = 13    
#SERVO_PIN = 12
ser = serial.Serial(PORT, BAUD, timeout=1)
buf = bytearray()

pi = pigpio.pi()
pi.set_mode(ESC_PIN, pigpio.OUTPUT)

def get_data_servo(raw, name):
    #us = (raw - 172) * (2000 / (1811 - 172)) + 500
    #us = max(500, min(2500, us))
    us = (raw - 172) * (1000 / (1811 - 172)) + 1000
    us = max(1000, min(2000, us))
    print(f"{name} - raw={raw}, us={int(us)}")

    return us

def get_data(raw, name):
    us = (raw - 172) * (1000 / (1811 - 172)) + 1000
    us = max(1000, min(2000, us))
    print(f"{name} - raw={raw}, us={int(us)}")

    return us

def send_us(pin, us):
    pi.set_servo_pulsewidth(pin, us)


def parse_rc_channels(payload):

    if len(payload) < 22:
        return

    bits = int.from_bytes(payload[:22] ,"little")
    channels = []

    for i in range(16):
        val = ((bits >> i*11) & 0x7FF)
        channels.append(val)

    return channels

try:
    while True:
        chunk = ser.read(64)
        if chunk:
            buf.extend(chunk)

        while len(buf) >= 2:
            addr = buf[0]
            length = buf[1]
            frame_len = 2+length

            if length < 3 or length > 64:   
                buf.pop(0)                  
                continue

            if len(buf) < frame_len:
                break

            frame = bytes(buf[:frame_len])
            del buf[:frame_len]

            if len(frame) < 4:
                continue

            typ = frame[2]
            payload = frame[3:-1]

            if typ == 0x16:
                channels = parse_rc_channels(payload)

                if channels:
                    #line = " ".join([f"CH{i+1}={val}" for i, val in enumerate(channels)])
                    #print(line)
                    roll = get_data(channels[0], "Roll")
                    pitch = get_data_servo(channels[1], "Pitch")
                    throttle = get_data(channels[2], "Throttle")
                    yaw = get_data(channels[3], "Yaw")
                    sc = channels[8]

                    print(f"sc: {sc}")
                    send_us(ESC_PIN, throttle)
                    if sc==997:
                        send_to_servo_angle(0, 90+15, False)
                        send_to_servo_angle(1, 90-15, False)
                    elif sc== 1792:
                        send_to_servo_angle(0, 90+30,False)
                        send_to_servo_angle(1, 90-30,False)
                    else:
                        send_to_servo(0, roll, False)
                        send_to_servo(1, roll, False)

                    send_to_servo(2, yaw)
                    send_to_servo(3, pitch)

except KeyboardInterrupt:
    print("Stopped by user")
    pi.set_servo_pulsewidth(ESC_PIN, 0)  # stop PWM
    pi.stop()
