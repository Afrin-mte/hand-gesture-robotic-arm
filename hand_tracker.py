"""
Hand Gesture Controlled 2-DOF/3-DOF Robotic Manipulator
---------------------------------------------------------
Captures live webcam video, runs MediaPipe Hands to extract 21 hand
landmarks, and converts the index-fingertip position + thumb-index
pinch distance into servo commands that are streamed over serial to
an ESP32 running `firmware/esp32_servo_controller`.

Mapping used (see project report, Section 3.2):
    theta1 = 180 * x_hat        # base yaw   (normalized screen x -> [0, 180] deg)
    theta2 = 180 * y_hat        # shoulder   (normalized screen y -> [0, 180] deg)
    theta3 = 2   * pinch_mm     # gripper    (0-60 mm pinch -> 0-120 deg)

Author: Group 3, Dept. of Mechatronics Engineering, KUET
"""

import argparse
import math
import time

import cv2
import mediapipe as mp
import serial

# MediaPipe hand-landmark indices used in this project
THUMB_TIP = 4
INDEX_TIP = 8

# Empirically calibrated pixel-to-mm scale for the pinch-distance measurement.
# Recalibrate for your own camera/working distance.
PIXEL_TO_MM = 0.30


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Hand gesture to servo-angle serial bridge")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                         help="Serial port for the ESP32 (e.g. /dev/ttyUSB0, COM3)")
    parser.add_argument("--baud", type=int, default=115200,
                         help="Serial baud rate; must match Serial.begin() in the ESP32 sketch")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--send-interval", type=float, default=0.03,
                         help="Minimum seconds between serial writes (throttles the stream)")
    parser.add_argument("--no-serial", action="store_true",
                         help="Run the vision pipeline only, without opening a serial port")
    return parser.parse_args()


def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    value = max(in_min, min(in_max, value))
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)


def main() -> None:
    args = parse_args()

    esp32 = None
    if not args.no_serial:
        esp32 = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(2)  # allow the ESP32 to reset after the port opens

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}")

    last_send_time = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            frame = cv2.flip(frame, 1)  # mirror for a natural, "looking in a mirror" feel
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(rgb_frame)

            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    h, w, _ = frame.shape

                    x_thumb, y_thumb = (int(hand_landmarks.landmark[THUMB_TIP].x * w),
                                         int(hand_landmarks.landmark[THUMB_TIP].y * h))
                    x_index, y_index = (int(hand_landmarks.landmark[INDEX_TIP].x * w),
                                         int(hand_landmarks.landmark[INDEX_TIP].y * h))

                    cv2.circle(frame, (x_thumb, y_thumb), 10, (0, 0, 255), -1)
                    cv2.circle(frame, (x_index, y_index), 10, (0, 255, 0), -1)
                    cv2.line(frame, (x_thumb, y_thumb), (x_index, y_index), (255, 0, 0), 2)

                    pinch_px = math.hypot(x_index - x_thumb, y_index - y_thumb)
                    pinch_mm = pinch_px * PIXEL_TO_MM

                    cv2.putText(frame, f"Pinch: {pinch_mm:.1f} mm", (10, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                    # Normalized index-fingertip coordinates, x_hat/y_hat in [0, 1]
                    x_hat = float(hand_landmarks.landmark[INDEX_TIP].x)
                    y_hat = float(hand_landmarks.landmark[INDEX_TIP].y)

                    cv2.putText(frame, f"({int(x_hat * w)}, {int(y_hat * h)})",
                                (int(x_hat * w) + 10, int(y_hat * h) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    theta1 = int(map_range(x_hat, 0, 1, 0, 180))      # base yaw
                    theta2 = int(map_range(y_hat, 0, 1, 0, 180))      # shoulder pitch
                    theta3 = int(map_range(min(pinch_mm, 60), 0, 60, 0, 120))  # gripper

                    now = time.time()
                    if now - last_send_time >= args.send_interval:
                        payload = f"{theta1},{theta2},{theta3}\n"
                        if esp32 is not None:
                            esp32.write(payload.encode())
                        print(f"Sent -> {payload.strip()}")
                        last_send_time = now

            cv2.imshow("Hand Gesture Robotic Arm Control", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if esp32 is not None:
            esp32.close()


if __name__ == "__main__":
    main()
