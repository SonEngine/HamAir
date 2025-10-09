import serial
import pigpio
import time
from adafruit_servokit import ServoKit
from time import sleep

kit = ServoKit(channels=16)
BAUD = 420000
PORT = "/dev/serial0"

SC_CH = 8
SA_CH = 6
SD_CH = 9

MODE0 = 191
MODE1 = 997
MODE2 = 1792
ROT_SERVO_IDX = 4
ESC_PIN = 13    
#SERVO_PIN = 12
ser = serial.Serial(PORT, BAUD, timeout=1)
buf = bytearray()
#kit.servo[ROT_SERVO_IDX].set_pulse_width_range(500, 2500)
pi = pigpio.pi()
pi.set_mode(ESC_PIN, pigpio.OUTPUT)

def send_to_yaw(ch, sa, sd):
    center = 90
    a = center
    mode1A = 30
    mode2A = 60
    if sa == MODE1:
        a-=mode1A
        print("sa mode1")

    elif sa == MODE2:
        print("sd mode2")
        a-=mode2A

    if sd == MODE1:
        print("sd mode1")
        a+=mode1A

    elif sd == MODE2:
        a+=mode2A
        print("sd mode2")

    print("angle : {a}")
    kit.servo[ch].angle = a
    
def send_to_dServo(ch, us):
    rot = (us - 1500) / 500
    speed = 20*2
    center = 90
    if abs(rot) < 0.2:
        kit.servo[ch].angle = 90 
    #right
    elif rot > 0:
        kit.servo[ch].angle = center + speed*1.4
    else:
        kit.servo[ch].angle = center - speed
        #kit.continuous_servo[ch].throttle = -speed
       
    
def send_to_servo(channel, us, bReverse = False, alphaAngle = 0, minAlpha = 45):
    # roll : [-1,1]
    roll = (us - 1500) / 500
    minAngle = 90 - minAlpha
    maxAngle = 90 + minAlpha

    #roll의 좌
    midAngle = 90 + alphaAngle
    roll *= minAlpha
    a = roll + midAngle

    if channel == 0:
        a+=alphaAngle

    if bReverse:
        a = 180-a

    a = max(minAngle, min(a, maxAngle))
    kit.servo[channel].angle = a

 
def send_to_servo_(channel, us, bReverse = False, alphaAngle = 0, minAlpha = 45, maxAlpha = 45):
    # roll : [-1,1]
    roll = (us - 1500) / 500
    minAngle = 90 - minAlpha
    maxAngle = 90 + maxAlpha

    #roll의 좌
    midAngle = 90 + alphaAngle
    roll *= minAlpha
    a = roll + midAngle

    if channel == 0:
        a+=alphaAngle

    if bReverse:
        a = 180-a

    a = max(minAngle, min(a, maxAngle))
    kit.servo[channel].angle = a

def send_to_servo_angle(channel, a, us, bReverse = False, alphaAngle = 0, minAlpha = 45):

    # us는 중립이 1500 이니까 0이 중립 [-1,1]
    roll = (us - 1500) / 500

    # 130이 들어오면 최대 15각이 움직이도록
    maxRange = (90+minAlpha) - (90 + abs(a-90))
    roll *= maxRange
    
    # 130 이면 a = 130 + 5 = 135
    a = roll + a + alphaAngle
    
    if bReverse:
        a = 180-a
        #minAngle = 50

    print(f"angle : {a}")
    kit.servo[channel].angle = a


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
    flag = False

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
                    #yaw = get_data(channels[0], "Yaw")
                    pitch = get_data_servo(channels[1], "Pitch")
                    throttle = get_data(channels[2], "Throttle")
                    roll = get_data(channels[3], "Roll")
                    rot = get_data(channels[0], "Rot")
                    
                    sc = channels[SC_CH]
                    #left
                    sa = channels[SA_CH]
                    #right
                    sd = channels[SD_CH]

                    #print("RRRRRR!!")
                    #print(f"sc: {sc}")
                    send_us(ESC_PIN, throttle)

                    a90 = 90
                    if sc==997:
                        a1 = 30
                        send_to_servo_angle(0, a90 + a1, roll, True, 10)
                        send_to_servo_angle(1, a90 - a1, roll, True, -15)

                    elif sc== 1792:
                        a1 = 40
                        send_to_servo_angle(0, a90 + a1, roll, True, 10)
                        send_to_servo_angle(1, a90 - a1, roll, True, -15)

                    else:
                        send_to_servo(0, roll, True, 10)
                        send_to_servo(1, roll, True, -15)
                    
                    #send_to_dServo(4, rot)
                    send_to_servo(4, rot, False)
                    #send_to_servo(2, yaw, False, 0, 70)
                    send_to_servo_(3, pitch, True, 0, 30, 60)
                    send_to_yaw(2, sa, sd)

except KeyboardInterrupt:
    print("Stopped by user")
    pi.set_servo_pulsewidth(ESC_PIN, 0)  # stop PWM
    pi.stop()
