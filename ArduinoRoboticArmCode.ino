/*
 * Robotic Arm Gesture Control - Arduino Code
 * 
 * Hardware Setup:
 * - 5 Servos for fingers (pins 4-8)
 * - 2 Servos for additional axes (pins 9-10)
 * - 2 RMCS-2303 encoder motors for main axes
 * - Serial communication with Python (USB Serial)
 * 
 * Data Format from Python: "X:90,Y:120,W:45,F1:30,F2:60,F3:90,F4:45,F5:75,A1:90,A2:120"
 */

#include <Servo.h>
#include <RMCS2303drive.h>

// ========== SERVO SETUP ==========
Servo fingerServos[5];    // 5 finger servos
Servo wristServo;         // Wrist servo
Servo additionalServo;    // Additional axis servo

// Servo pin assignments
const int FINGER_PINS[5] = {4, 5, 6, 7, 8};  // Fingers: Thumb, Index, Middle, Ring, Pinky
const int WRIST_PIN = 9;
const int ADDITIONAL_PIN = 10;

// ========== RMCS-2303 SETUP ==========
RMCS2303 rmcs;
SoftwareSerial motorSerial(2, 3);  // RX=2, TX=3 for motor communication

// Motor parameters
byte baseMotorID = 11;      // Base rotation motor
byte armMotorID = 12;       // Arm height motor
int speed = 5000;           // Motor speed
int acceleration = 3000;    // Motor acceleration

// ========== POSITION VARIABLES ==========
struct GestureData {
  int baseRotation;     // X-axis (0-180°) -> maps to encoder positions
  int armHeight;        // Y-axis (0-180°) -> maps to encoder positions  
  int wristAngle;       // Wrist rotation (0-180°)
  int fingerAngles[5];  // Finger positions (0-180°)
  int additionalAxis;   // Additional axis (0-180°)
};

GestureData currentGesture;
GestureData targetGesture;

// Position mapping for encoder motors
const long BASE_MIN_POS = -25000;   // Minimum encoder position for base
const long BASE_MAX_POS = 25000;    // Maximum encoder position for base
const long ARM_MIN_POS = -15000;    // Minimum encoder position for arm
const long ARM_MAX_POS = 15000;     // Maximum encoder position for arm

// ========== SETUP FUNCTION ==========
void setup() {
  // Initialize USB Serial for Python communication
  Serial.begin(9600);
  Serial.println("Robotic Arm Gesture Control Started!");
  Serial.println("Waiting for gesture commands...");
  
  // Initialize servos
  for (int i = 0; i < 5; i++) {
    fingerServos[i].attach(FINGER_PINS[i]);
    fingerServos[i].write(90);  // Neutral position
  }
  wristServo.attach(WRIST_PIN);
  wristServo.write(90);
  additionalServo.attach(ADDITIONAL_PIN);
  additionalServo.write(90);
  
  // Initialize RMCS-2303 motors
  rmcs.Serial_selection(1);  // Use software serial
  rmcs.Serial0(9600);        // USB serial baudrate
  rmcs.begin(&motorSerial, 9600);  // Motor serial baudrate
  
  // Configure motor parameters
  setupMotors();
  
  // Initialize gesture data to neutral positions
  initializeGestureData();
  
  Serial.println("Setup complete! Send gesture data in format:");
  Serial.println("X:90,Y:120,W:45,F1:30,F2:60,F3:90,F4:45,F5:75,A1:90");
}

// ========== MAIN LOOP ==========
void loop() {
  // Check for incoming serial data from Python
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.print("Received: ");
      Serial.println(command);
      
      // Parse the gesture command
      if (parseGestureCommand(command)) {
        Serial.println("Command parsed successfully!");
        executeGestureCommand();
      } else {
        Serial.println("Error: Invalid command format!");
      }
    }
  }
  
  // Small delay to prevent overwhelming the system
  delay(50);
}

// ========== MOTOR SETUP FUNCTION ==========
void setupMotors() {
  Serial.println("Configuring RMCS-2303 motors...");
  
  // Motor parameters (refer to datasheet)
  int INP_CONTROL_MODE = 513;  // Position control mode
  int PP_gain = 32;
  int PI_gain = 16;
  int VF_gain = 32;
  int LPR = 334;  // Lines per revolution
  
  // Configure base motor
  rmcs.WRITE_PARAMETER(baseMotorID, INP_CONTROL_MODE, PP_gain, PI_gain, VF_gain, LPR, acceleration, speed);
  delay(100);
  
  // Configure arm motor  
  rmcs.WRITE_PARAMETER(armMotorID, INP_CONTROL_MODE, PP_gain, PI_gain, VF_gain, LPR, acceleration, speed);
  delay(100);
  
  Serial.println("Motors configured!");
}

// ========== INITIALIZE GESTURE DATA ==========
void initializeGestureData() {
  currentGesture.baseRotation = 90;
  currentGesture.armHeight = 90;
  currentGesture.wristAngle = 90;
  currentGesture.additionalAxis = 90;
  
  for (int i = 0; i < 5; i++) {
    currentGesture.fingerAngles[i] = 90;
  }
  
  targetGesture = currentGesture;
}

// ========== PARSE GESTURE COMMAND ==========
bool parseGestureCommand(String command) {
  // Expected format: "X:90,Y:120,W:45,F1:30,F2:60,F3:90,F4:45,F5:75,A1:90"
  
  // Reset target gesture
  targetGesture = currentGesture;
  
  // Split command by commas
  int startIndex = 0;
  int commaIndex = 0;
  
  while (commaIndex != -1) {
    commaIndex = command.indexOf(',', startIndex);
    String segment;
    
    if (commaIndex == -1) {
      segment = command.substring(startIndex);
    } else {
      segment = command.substring(startIndex, commaIndex);
    }
    
    // Parse each segment (e.g., "X:90")
    int colonIndex = segment.indexOf(':');
    if (colonIndex == -1) continue;
    
    String key = segment.substring(0, colonIndex);
    int value = segment.substring(colonIndex + 1).toInt();
    
    // Clamp value to 0-180 range
    value = constrain(value, 0, 180);
    
    // Assign value based on key
    if (key == "X") {
      targetGesture.baseRotation = value;
    } else if (key == "Y") {
      targetGesture.armHeight = value;
    } else if (key == "W") {
      targetGesture.wristAngle = value;
    } else if (key == "F1") {
      targetGesture.fingerAngles[0] = value;  // Thumb
    } else if (key == "F2") {
      targetGesture.fingerAngles[1] = value;  // Index
    } else if (key == "F3") {
      targetGesture.fingerAngles[2] = value;  // Middle
    } else if (key == "F4") {
      targetGesture.fingerAngles[3] = value;  // Ring
    } else if (key == "F5") {
      targetGesture.fingerAngles[4] = value;  // Pinky
    } else if (key == "A1") {
      targetGesture.additionalAxis = value;
    }
    
    startIndex = commaIndex + 1;
  }
  
  return true;  // Successfully parsed
}

// ========== EXECUTE GESTURE COMMAND ==========
void executeGestureCommand() {
  Serial.println("Executing gesture command...");
  
  // Control servos (immediate movement)
  controlServos();
  
  // Control encoder motors (smooth movement)
  controlEncoderMotors();
  
  // Update current gesture
  currentGesture = targetGesture;
  
  Serial.println("Gesture executed!");
  printGestureStatus();
}

// ========== CONTROL SERVOS ==========
void controlServos() {
  // Control finger servos
  for (int i = 0; i < 5; i++) {
    fingerServos[i].write(targetGesture.fingerAngles[i]);
  }
  
  // Control wrist and additional axis servos
  wristServo.write(targetGesture.wristAngle);
  additionalServo.write(targetGesture.additionalAxis);
  
  Serial.print("Servos moved - Wrist:");
  Serial.print(targetGesture.wristAngle);
  Serial.print(", Fingers:");
  for (int i = 0; i < 5; i++) {
    Serial.print(targetGesture.fingerAngles[i]);
    if (i < 4) Serial.print(",");
  }
  Serial.println();
}

// ========== CONTROL ENCODER MOTORS ==========
void controlEncoderMotors() {
  // Map 0-180° to encoder positions
  long basePosition = map(targetGesture.baseRotation, 0, 180, BASE_MIN_POS, BASE_MAX_POS);
  long armPosition = map(targetGesture.armHeight, 0, 180, ARM_MIN_POS, ARM_MAX_POS);
  
  // Send position commands to motors
  rmcs.Absolute_position(baseMotorID, basePosition);
  rmcs.Absolute_position(armMotorID, armPosition);
  
  Serial.print("Motors commanded - Base:");
  Serial.print(basePosition);
  Serial.print(", Arm:");
  Serial.println(armPosition);
  
  // Wait for motors to reach positions (optional monitoring)
  monitorMotorMovement(baseMotorID, basePosition, "Base");
  monitorMotorMovement(armMotorID, armPosition, "Arm");
}

// ========== MONITOR MOTOR MOVEMENT ==========
void monitorMotorMovement(byte motorID, long targetPosition, String motorName) {
  unsigned long startTime = millis();
  const unsigned long timeout = 5000;  // 5 second timeout
  
  while (millis() - startTime < timeout) {
    long currentPosition = rmcs.Position_Feedback(motorID);
    long error = abs(currentPosition - targetPosition);
    
    // Check if close enough to target (within 100 encoder counts)
    if (error < 100) {
      Serial.print(motorName);
      Serial.println(" motor reached target position");
      break;
    }
    
    delay(100);  // Check every 100ms
  }
}

// ========== PRINT GESTURE STATUS ==========
void printGestureStatus() {
  Serial.println("=== Current Gesture Status ===");
  Serial.print("Base Rotation: ");
  Serial.print(currentGesture.baseRotation);
  Serial.println("°");
  
  Serial.print("Arm Height: ");
  Serial.print(currentGesture.armHeight);
  Serial.println("°");
  
  Serial.print("Wrist Angle: ");
  Serial.print(currentGesture.wristAngle);
  Serial.println("°");
  
  Serial.print("Fingers: ");
  String fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
  for (int i = 0; i < 5; i++) {
    Serial.print(fingerNames[i]);
    Serial.print(":");
    Serial.print(currentGesture.fingerAngles[i]);
    Serial.print("° ");
  }
  Serial.println();
  
  Serial.print("Additional Axis: ");
  Serial.print(currentGesture.additionalAxis);
  Serial.println("°");
  Serial.println("==============================");
}

// ========== EMERGENCY STOP FUNCTION ==========
void emergencyStop() {
  Serial.println("EMERGENCY STOP ACTIVATED!");
  
  // Disable motor position control
  rmcs.Disable_Position_Mode(baseMotorID);
  rmcs.Disable_Position_Mode(armMotorID);
  
  // Move servos to safe neutral positions
  for (int i = 0; i < 5; i++) {
    fingerServos[i].write(90);
  }
  wristServo.write(90);
  additionalServo.write(90);
  
  Serial.println("All motors stopped and servos in neutral position");
}

// ========== SERIAL EVENT (OPTIONAL) ==========
void serialEvent() {
  // Handle emergency stop command
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command == "STOP" || command == "EMERGENCY") {
      emergencyStop();
    }
  }
}