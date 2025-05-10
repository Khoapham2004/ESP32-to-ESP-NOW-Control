import cv2
import numpy as np
import time
import serial

# === UART setup ===
UART_PORT = '/dev/ttyUSB0'  # Thay đổi nếu cần
BAUD_RATE = 115200
try:
    ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("✅ UART đã kết nối.")
except serial.SerialException:
    print("⚠️ Không thể mở cổng UART.")
    ser = None

# === Camera setup ===
cam = cv2.VideoCapture(0)
if not cam.isOpened():
    print("⚠️ Không thể mở camera.")
    exit()

# Giảm độ phân giải để xử lý nhanh hơn
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# === Giá trị mặc định ===
ly = 127     # Luôn tiến
pot = 80      # Tốc độ cố định
rx = 127      # Giá trị servo ban đầu
ry = 127
buttons = [0] * 6
js1 = 0
js2 = 0

# === Xử lý ảnh ===
def detect_edges(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    return edges

def find_lane_points(edges, draw=False):
    h, w = edges.shape
    y = int(h * 0.7)
    line = edges[y, :]
    left = right = -1
    lane_width = 590
    center = w // 2

    # Tìm điểm trái
    for x in range(center, 0, -1):
        if line[x] > 0:
            left = x
            break
    # Tìm điểm phải
    for x in range(center + 1, w):
        if line[x] > 0:
            right = x
            break

    # Nếu chỉ phát hiện một bên, giả định khoảng cách cố định
    if left != -1 and right == -1:
        right = left + lane_width
    if right != -1 and left == -1:
        left = right - lane_width

    if draw:
        viz = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        if left != -1:
            cv2.circle(viz, (left, y), 7, (0, 255, 0), -1)
        if right != -1:
            cv2.circle(viz, (right, y), 7, (255, 255, 0), -1)
        return left, right, viz
    else:
        return left, right, None
def calculate_steering(frame):
    h, w = frame.shape[:2]
    edges = detect_edges(frame)
    left, right, viz = find_lane_points(edges, draw=True)
    angle = 0

    if left != -1 and right != -1:
        lane_center = (left + right) // 2
        diff = (w // 2) - lane_center
        angle = diff * 0.01
        cv2.circle(viz, (lane_center, int(h * 0.7)), 7, (0, 255, 0), -1)
        cv2.circle(viz, (w // 2, int(h * 0.7)), 7, (0, 0, 255), -1)

    elif left != -1 and right == -1:
        angle = -3.0
        cv2.putText(viz, "RIGHT BIAS", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    elif right != -1 and left == -1:
        angle = 3.0
        cv2.putText(viz, "LEFT BIAS", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    else:
        angle = 0.0
        cv2.putText(viz, "NO LANE", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # === GIỚI HẠN VỀ 3 TRẠNG THÁI SERVO ===
    if angle > 1.0:
        rx_value = 255     # Quẹo trái
    elif angle < -1.0:
        rx_value = 70   # Quẹo phải
    else:
        rx_value = 127   # Giữ thẳng

    if viz is not None:
        small_viz = cv2.resize(viz, (500, 300))
        cv2.imshow("Lane Detection", small_viz)

    return rx_value
#
# def calculate_steering(frame):
    h, w = frame.shape[:2]
    edges = detect_edges(frame)
    left, right, viz = find_lane_points(edges, draw=True)
    angle = 0

    # Cả hai làn cùng tồn tại
    if left != -1 and right != -1:
        lane_center = (left + right) // 2
        diff = (w // 2) - lane_center
        angle = diff * 0.01
        cv2.circle(viz, (lane_center, int(h * 0.7)), 7, (0, 255, 0), -1)
        cv2.circle(viz, (w // 2, int(h * 0.7)), 7, (0, 0, 255), -1)

    # Chỉ phát hiện làn bên trái → đẩy sang phải
    elif left != -1 and right == -1:
        angle = -3.0
        cv2.putText(viz, "RIGHT BIAS", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # Chỉ phát hiện làn bên phải → đẩy sang trái
    elif right != -1 and left == -1:
        angle = 3.0
        cv2.putText(viz, "LEFT BIAS", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # Không thấy làn → giữ thẳng
    else:
        angle = 0.0
        cv2.putText(viz, "NO LANE", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Giới hạn góc và chuyển đổi sang giá trị servo
    angle = np.clip(angle, -4.5, 4.5)
    servo_degree = int(np.interp(angle, [-4.5, 4.5], [0, 180]))
    rx_value = int(np.interp(servo_degree, [0, 180], [0, 255]))

    # Hiển thị khung lane detection đã resize
    if viz is not None:
        small_viz = cv2.resize(viz, (380, 300))
        cv2.imshow("Lane Detection", small_viz)

    return rx_value
#


# === Gửi gói dữ liệu UART ===
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
    if ser:
        ser.write(packet)
        print(f"📤 Sent | LY={ly}, RX={rx}, POT={pot}")

# === Main loop ===
try:
    while True:
        ret, frame = cam.read()
        if not ret:
            print("⚠️ Không đọc được frame.")
            break

        # Tính toán góc lái mới
        rx = calculate_steering(frame)
        # Gửi gói UART
        send_packet()

        # Hiển thị feed camera gốc đã resize
        small_frame = cv2.resize(frame, (320, 240))
        cv2.imshow("Camera Feed", small_frame)

        # Nhấn 'q' để thoát
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("⛔ Dừng bởi người dùng.")
finally:
    if ser:
        ser.close()
    cam.release()
    cv2.destroyAllWindows()
