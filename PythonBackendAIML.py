# hehe shaan developer

import cv2 #opencv
import mediapipe as mp #mediapipe google
import numpy as np #numpy, data
import math #math func


class HandTrackerMediaPipe:#besttt class - hand tracker
    def __init__(self): #default func to run
        self.mp_hands = mp.solutions.hands #wanna track hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,#process video, not image
            max_num_hands=1,#need 1 hand
            min_detection_confidence=0.7, #max confidence for decision
            min_tracking_confidence=0.5 # min confidence for decision
        )
        self.mp_draw = mp.solutions.drawing_utils #utility for drawing

        # Camera setup
        self.cap = cv2.VideoCapture(0) #capture video
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) #width of vid
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) #height of vid

        # MediaPipe hand landmarks as per library
        self.finger_tips = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky tips
        self.finger_pips = [3, 6, 10, 14, 18]  # Finger PIP joints
        self.finger_mcps = [2, 5, 9, 13, 17]  # Finger MCP joints

    def calculate_angle(self, point1, point2, point3): #p1, p2, p3 are in form of objects

        # Extract coordinates from objects, store in array to perform subraction like matrices
        p1 = np.array([point1.x, point1.y])
        p2 = np.array([point2.x, point2.y])
        p3 = np.array([point3.x, point3.y])

        # Calculate vectors (direction from one point to other, or the difference in x and y of the 2 points.)
        v1 = p1 - p2
        v2 = p3 - p2

        # Calculate angle using dot product
        cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cosine_angle = np.clip(cosine_angle, -1.0, 1.0)  # Prevent math domain error
        angle = math.acos(cosine_angle)

        # Convert to degrees and map to 0-180 range
        angle_degrees = math.degrees(angle)
        return round(angle_degrees, 1)

    def get_finger_angles(self, landmarks):
        """Calculate precise angles for each finger"""
        finger_angles = {}
        finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']

        for i, finger_name in enumerate(finger_names):
            if finger_name == 'thumb':
                # Thumb: use different joints (CMC, MCP, IP, TIP)
                angle = self.calculate_angle(
                    landmarks[1],  # CMC
                    landmarks[2],  # MCP
                    landmarks[3]  # IP
                )
            else:
                # Other fingers: use MCP, PIP, DIP joints
                mcp_idx = self.finger_mcps[i]
                pip_idx = self.finger_pips[i]
                tip_idx = self.finger_tips[i]

                angle = self.calculate_angle(
                    landmarks[mcp_idx],
                    landmarks[pip_idx],
                    landmarks[tip_idx]
                )

            finger_angles[finger_name] = angle

        return finger_angles

    def get_wrist_angle(self, landmarks):
        """Calculate wrist rotation angle"""
        # Use wrist (0), index MCP (5), and pinky MCP (17)
        wrist_angle = self.calculate_angle(
            landmarks[5],  # Index MCP
            landmarks[0],  # Wrist
            landmarks[17]  # Pinky MCP
        )
        return round(wrist_angle, 1)

    def get_hand_position(self, landmarks, frame_width, frame_height):
        """Get hand position mapped to 0-180 range for robotic arm control"""
        # Use wrist landmark for position reference
        wrist = landmarks[0]

        # Map X position (0 to 180) - Left to Right movement
        x_pos = int(wrist.x * 180)
        x_pos = max(0, min(180, x_pos))

        # Map Y position (0 to 180) - Bottom to Top movement
        # Inverted so 0 = bottom (arm down), 180 = top (arm up)
        y_pos = int((1 - wrist.y) * 180)
        y_pos = max(0, min(180, y_pos))

        return x_pos, y_pos

    def get_gesture_state(self, finger_angles):
        """Determine basic gesture states for robotic control"""
        # Simple gesture recognition based on finger angles
        extended_fingers = []

        # Check which fingers are extended (angle > 160 degrees)
        for finger, angle in finger_angles.items():
            if angle > 160:
                extended_fingers.append(finger)

        finger_count = len(extended_fingers)

        # Determine gesture
        if finger_count == 0:
            gesture = "FIST - STOP"
        elif finger_count == 1 and 'index' in extended_fingers:
            gesture = "POINT - PRECISE MODE"
        elif finger_count == 2 and 'index' in extended_fingers and 'middle' in extended_fingers:
            gesture = "PEACE - MEDIUM SPEED"
        elif finger_count == 5:
            gesture = "OPEN HAND - FAST MODE"
        else:
            gesture = f"{finger_count} FINGERS - NORMAL MODE"

        return gesture, finger_count

    def run(self):
        """Main tracking loop with enhanced robotic arm control data"""
        print("MediaPipe Hand Tracking Started! Press 'q' to quit.")
        print("Move your hand to control the robotic arm.")
        print("Watch console for real-time control data.")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)
            frame_height, frame_width = frame.shape[:2]

            # Convert BGR to RGB for MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Process the frame
            results = self.hands.process(rgb_frame)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Draw hand landmarks and connections
                    self.mp_draw.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                        self.mp_draw.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                        self.mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2)
                    )

                    # Calculate all measurements
                    finger_angles = self.get_finger_angles(hand_landmarks.landmark)
                    wrist_angle = self.get_wrist_angle(hand_landmarks.landmark)
                    x_pos, y_pos = self.get_hand_position(
                        hand_landmarks.landmark, frame_width, frame_height
                    )
                    gesture, finger_count = self.get_gesture_state(finger_angles)

                    # === DISPLAY INFORMATION ON FRAME ===

                    # Hand Position
                    cv2.putText(frame, "=== ROBOTIC ARM CONTROL ===",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    cv2.putText(frame, f"Base Rotation: {x_pos}° (L-R)",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    cv2.putText(frame, f"Arm Height: {y_pos}° (Up-Down)",
                                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    cv2.putText(frame, f"Wrist Rotation: {wrist_angle}°",
                                (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    # Gesture Recognition
                    cv2.putText(frame, f"Gesture: {gesture}",
                                (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                    # Individual Finger Angles
                    cv2.putText(frame, "=== FINGER ANGLES ===",
                                (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    y_offset = 200
                    for finger, angle in finger_angles.items():
                        # Color code: Green if extended, Red if bent
                        color = (0, 255, 0) if angle > 160 else (0, 0, 255)
                        text = f"{finger.capitalize()}: {angle}°"
                        cv2.putText(frame, text, (10, y_offset),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                        y_offset += 20

                    # Robotic Arm Commands (Right side)
                    cv2.putText(frame, "=== ARDUINO COMMANDS ===",
                                (350, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                    commands = [
                        f"servo1.write({x_pos});  // Base",
                        f"servo2.write({y_pos});  // Height",
                        f"servo3.write({wrist_angle});  // Wrist",
                        f"servo4.write({finger_angles['thumb']});  // Gripper",
                        "",
                        f"// Gesture: {gesture}",
                        f"// Speed: {finger_count}/5",
                        "",
                        "// Serial Output:",
                        f"// {x_pos},{y_pos},{wrist_angle},{int(finger_angles['thumb'])}"
                    ]

                    for i, cmd in enumerate(commands):
                        cv2.putText(frame, cmd, (350, 60 + i * 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

                    # === CONSOLE OUTPUT FOR ARDUINO SERIAL ===
                    # Format: X,Y,Wrist,Gripper,Gesture
                    serial_data = f"X:{x_pos},Y:{y_pos},W:{wrist_angle},G:{int(finger_angles['thumb'])},F:{finger_count}"
                    print(serial_data)

                    # Alternative simple format for Arduino parsing
                    simple_format = f"{x_pos},{y_pos},{wrist_angle},{int(finger_angles['thumb'])},{finger_count}"
                    # Uncomment below line if you want simple comma-separated values
                    # print(simple_format)

            else:
                # No hand detected
                cv2.putText(frame, "No Hand Detected", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(frame, "Place hand in camera view", (50, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)

            # Show the frame
            cv2.imshow('MediaPipe Hand Tracking - Robotic Arm Control', frame)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    """Main function to run the hand tracker"""
    try:
        print("Initializing MediaPipe Hand Tracking...")
        tracker = HandTrackerMediaPipe()
        tracker.run()
    except Exception as e:
        print(f"Error: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure you're in the correct conda environment: conda activate mediapipe_env")
        print("2. Install required packages: pip install mediapipe opencv-python numpy")
        print("3. Check if camera is connected and not being used by another application")


if __name__ == "__main__":
    main()