import serial
import time
import keyboard  # pip install keyboard

# Cấu hình cổng COM
PORT = 'COM3'
BAUD_RATE = 115200

# Khởi tạo Serial
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Đợi thiết bị ESP32 khởi động

# Giá trị mặc định
ly = 127  # Trung bình
rx = 127
ry = 127
pot = 100
buttons = [0]*6
js1 = 0
js2 = 0

def send_packet():
    header = 0x4E
    footer = 0x41
    packet = bytearray([
        header,
        ly,
        rx,
        ry,
        pot,
        *buttons,
        js1,
        js2,
        footer
    ])
    ser.write(packet)
    print(f"LY={ly}, RX={rx}")

try:
    while True:
        # Điều khiển LY (tiến/lùi)
        if keyboard.is_pressed('w'):
            ly = 255
        elif keyboard.is_pressed('s'):
            ly = 0
        else:
            ly = 127  # Trung tâm

        # Điều khiển RX (trái/phải)
        if keyboard.is_pressed('d'):
            rx = 0
        elif keyboard.is_pressed('a'):
            rx = 255
        else:
            rx = 127  # Trung tâm

        send_packet()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Dừng chương trình.")
finally:
    ser.close()
ChildProcessError
