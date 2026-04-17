/*
 * Neutron Imaging Lab - 3-Axis Motor Controller
 * Arduino MEGA2560 Firmware
 * 
 * Controls 3 servo motors for sample positioning:
 * - X-Axis (Left-Right)
 * - Y-Axis (Up-Down)
 * - Z-Axis (Rotation)
 * 
 * Communication: Serial @ 115200 baud
 * Protocol: Simple text-based commands
 * 
 * Pin Configuration:
 * - X-Axis Servo: Pin 2 (PWM)
 * - Y-Axis Servo: Pin 3 (PWM)
 * - Z-Axis Servo: Pin 4 (PWM)
 * - X-Endstop: Pin 22 (digital input with pullup)
 * - Y-Endstop: Pin 23 (digital input with pullup)
 * - Z-Endstop: Pin 24 (digital input with pullup)
 * 
 * Commands:
 *   CONNECT          - Handshake with PC
 *   HOME:<axis>      - Home specific axis (X, Y, Z, or ALL)
 *   MOVE:<axis>:<pos> - Move axis to position
 *   STOP:<axis>      - Stop specific axis (X, Y, Z, or ALL)
 *   GETPOS:<axis>    - Get current position
 *   GETSTATUS        - Get all axes status
 *   SETHOME:<axis>:<pos> - Set home position for axis
 * 
 * Response Format:
 *   OK:<command>           - Command executed
 *   ERROR:<message>        - Error occurred
 *   POS:<axis>:<value>     - Position response
 *   STATUS:<axis>:<moving>:<target>:<current>
 */

#include <Servo.h>
#include <elapsedMillis.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define BAUD_RATE 115200
#define SERIAL_TIMEOUT 1000

// Servo pins
#define PIN_SERVO_X 2
#define PIN_SERVO_Y 3
#define PIN_SERVO_Z 4

// Endstop pins
#define PIN_ENDSTOP_X 22
#define PIN_ENDSTOP_Y 23
#define PIN_ENDSTOP_Z 24

// Motor configuration
#define SERVO_MIN_PULSE 1000   // Minimum pulse width (microseconds)
#define SERVO_MAX_PULSE 2000   // Maximum pulse width (microseconds)
#define DEFAULT_SPEED 100       // Default speed in steps/second
#define ACCELERATION 200        // Acceleration factor

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct AxisConfig {
    Servo servo;
    int pin;
    int endstopPin;
    float targetPosition;      // Target position in degrees
    float currentPosition;     // Current position in degrees
    float speed;               // Current speed
    float maxSpeed;            // Maximum speed
    float acceleration;        // Acceleration
    bool isMoving;             // Movement state
    bool isHomed;              // Home state
    int direction;             // Current direction (-1, 0, 1)
    float homeOffset;          // Offset from endstop
};

struct AxisConfig axes[3];

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

String inputBuffer = "";
bool connected = false;
elapsedMillis updateTimer;
const unsigned long UPDATE_INTERVAL = 10;  // ms

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(BAUD_RATE);
    inputBuffer.reserve(64);
    
    initializeAxes();
    
    // Wait for serial connection
    waitForSerial();
    
    sendResponse("OK:READY");
}

void initializeAxes() {
    // X-Axis configuration
    axes[0].pin = PIN_SERVO_X;
    axes[0].endstopPin = PIN_ENDSTOP_X;
    axes[0].targetPosition = 0;
    axes[0].currentPosition = 0;
    axes[0].speed = 0;
    axes[0].maxSpeed = DEFAULT_SPEED;
    axes[0].acceleration = ACCELERATION;
    axes[0].isMoving = false;
    axes[0].isHomed = false;
    axes[0].direction = 0;
    axes[0].homeOffset = 0;
    
    // Y-Axis configuration
    axes[1].pin = PIN_SERVO_Y;
    axes[1].endstopPin = PIN_ENDSTOP_Y;
    axes[1].targetPosition = 0;
    axes[1].currentPosition = 0;
    axes[1].speed = 0;
    axes[1].maxSpeed = DEFAULT_SPEED;
    axes[1].acceleration = ACCELERATION;
    axes[1].isMoving = false;
    axes[1].isHomed = false;
    axes[1].direction = 0;
    axes[1].homeOffset = 0;
    
    // Z-Axis configuration
    axes[2].pin = PIN_SERVO_Z;
    axes[2].endstopPin = PIN_ENDSTOP_Z;
    axes[2].targetPosition = 0;
    axes[2].currentPosition = 0;
    axes[2].speed = 0;
    axes[2].maxSpeed = DEFAULT_SPEED;
    axes[2].acceleration = ACCELERATION;
    axes[2].isMoving = false;
    axes[2].isHomed = false;
    axes[2].direction = 0;
    axes[2].homeOffset = 0;
    
    // Attach servos
    for (int i = 0; i < 3; i++) {
        axes[i].servo.attach(axes[i].pin, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
        axes[i].servo.write(90);  // Neutral position
        pinMode(axes[i].endstopPin, INPUT_PULLUP);
    }
}

void waitForSerial() {
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 5000)) {
        delay(100);
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    processSerial();
    updateMotors();
}

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

void processSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                processCommand(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
}

void processCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "CONNECT") {
        handleConnect();
    }
    else if (cmd.startsWith("HOME:")) {
        handleHome(cmd);
    }
    else if (cmd.startsWith("MOVE:")) {
        handleMove(cmd);
    }
    else if (cmd.startsWith("STOP:")) {
        handleStop(cmd);
    }
    else if (cmd.startsWith("GETPOS:")) {
        handleGetPosition(cmd);
    }
    else if (cmd == "GETSTATUS") {
        handleGetStatus();
    }
    else if (cmd.startsWith("SETHOME:")) {
        handleSetHome(cmd);
    }
    else if (cmd.startsWith("SETSPEED:")) {
        handleSetSpeed(cmd);
    }
    else if (cmd == "PING") {
        sendResponse("OK:PONG");
    }
    else {
        sendResponse("ERROR:UNKNOWN_COMMAND");
    }
}

void sendResponse(String response) {
    Serial.println(response);
}

// ============================================================================
// COMMAND HANDLERS
// ============================================================================

void handleConnect() {
    connected = true;
    sendResponse("OK:CONNECTED");
    delay(10);
    sendResponse("INFO:MEGA2560_MOTOR_CONTROLLER_V1.0");
}

void handleHome(String cmd) {
    String axisStr = cmd.substring(5);
    axisStr.trim();
    
    if (axisStr == "ALL") {
        for (int i = 0; i < 3; i++) {
            homeAxis(i);
        }
        sendResponse("OK:HOME_ALL");
    }
    else {
        int axisIndex = getAxisIndex(axisStr.charAt(0));
        if (axisIndex >= 0) {
            homeAxis(axisIndex);
            sendResponse("OK:HOME:" + axisStr);
        }
        else {
            sendResponse("ERROR:INVALID_AXIS");
        }
    }
}

void homeAxis(int axisIndex) {
    AxisConfig &axis = axes[axisIndex];
    
    // Move towards endstop
    axis.direction = -1;
    axis.isMoving = true;
    axis.speed = axis.maxSpeed * 0.5;
    
    // Move until endstop is triggered
    unsigned long homeTimeout = 30000;  // 30 seconds max
    unsigned long startTime = millis();
    
    while (digitalRead(axis.endstopPin) == HIGH) {
        updateMotors();
        if (millis() - startTime > homeTimeout) {
            stopAxis(axisIndex);
            sendResponse("ERROR:HOME_TIMEOUT:" + String(axisIndex));
            return;
        }
    }
    
    // Stop at endstop
    stopAxis(axisIndex);
    
    // Set position
    axis.currentPosition = axis.homeOffset;
    axis.targetPosition = axis.homeOffset;
    axis.isHomed = true;
}

void handleMove(String cmd) {
    // Format: MOVE:X:100.5
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    
    if (secondColon == -1) {
        sendResponse("ERROR:INVALID_FORMAT");
        return;
    }
    
    String axisStr = cmd.substring(firstColon + 1, secondColon);
    String posStr = cmd.substring(secondColon + 1);
    
    axisStr.trim();
    posStr.trim();
    
    int axisIndex = getAxisIndex(axisStr.charAt(0));
    if (axisIndex < 0) {
        sendResponse("ERROR:INVALID_AXIS");
        return;
    }
    
    float position = posStr.toFloat();
    moveAxisTo(axisIndex, position);
    
    sendResponse("OK:MOVE:" + axisStr + ":" + String(position, 4));
}

void moveAxisTo(int axisIndex, float position) {
    AxisConfig &axis = axes[axisIndex];
    
    axis.targetPosition = constrain(position, -180.0, 180.0);
    axis.isMoving = true;
    
    // Set direction
    if (position > axis.currentPosition) {
        axis.direction = 1;
    }
    else if (position < axis.currentPosition) {
        axis.direction = -1;
    }
    else {
        axis.direction = 0;
        axis.isMoving = false;
    }
}

void handleStop(String cmd) {
    String axisStr = cmd.substring(5);
    axisStr.trim();
    
    if (axisStr == "ALL") {
        for (int i = 0; i < 3; i++) {
            stopAxis(i);
        }
        sendResponse("OK:STOP_ALL");
    }
    else {
        int axisIndex = getAxisIndex(axisStr.charAt(0));
        if (axisIndex >= 0) {
            stopAxis(axisIndex);
            sendResponse("OK:STOP:" + axisStr);
        }
        else {
            sendResponse("ERROR:INVALID_AXIS");
        }
    }
}

void stopAxis(int axisIndex) {
    AxisConfig &axis = axes[axisIndex];
    axis.isMoving = false;
    axis.speed = 0;
    axis.direction = 0;
    axis.servo.write(90);  // Neutral
}

void handleGetPosition(String cmd) {
    String axisStr = cmd.substring(7);
    axisStr.trim();
    
    int axisIndex = getAxisIndex(axisStr.charAt(0));
    if (axisIndex >= 0) {
        String response = "POS:" + axisStr + ":" + String(axes[axisIndex].currentPosition, 4);
        sendResponse(response);
    }
    else {
        sendResponse("ERROR:INVALID_AXIS");
    }
}

void handleGetStatus() {
    String status = "STATUS";
    for (int i = 0; i < 3; i++) {
        status += ":" + String(axes[i].isMoving ? "1" : "0") 
                + ":" + String(axes[i].targetPosition, 2)
                + ":" + String(axes[i].currentPosition, 2);
    }
    sendResponse(status);
}

void handleSetHome(String cmd) {
    // Format: SETHOME:X:0
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    
    if (secondColon == -1) {
        sendResponse("ERROR:INVALID_FORMAT");
        return;
    }
    
    String axisStr = cmd.substring(firstColon + 1, secondColon);
    String offsetStr = cmd.substring(secondColon + 1);
    
    axisStr.trim();
    offsetStr.trim();
    
    int axisIndex = getAxisIndex(axisStr.charAt(0));
    if (axisIndex >= 0) {
        axes[axisIndex].homeOffset = offsetStr.toFloat();
        axes[axisIndex].currentPosition = axes[axisIndex].homeOffset;
        sendResponse("OK:SETHOME:" + axisStr);
    }
    else {
        sendResponse("ERROR:INVALID_AXIS");
    }
}

void handleSetSpeed(String cmd) {
    // Format: SETSPEED:X:100
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    
    if (secondColon == -1) {
        sendResponse("ERROR:INVALID_FORMAT");
        return;
    }
    
    String axisStr = cmd.substring(firstColon + 1, secondColon);
    String speedStr = cmd.substring(secondColon + 1);
    
    axisStr.trim();
    speedStr.trim();
    
    int axisIndex = getAxisIndex(axisStr.charAt(0));
    if (axisIndex >= 0) {
        axes[axisIndex].maxSpeed = speedStr.toFloat();
        sendResponse("OK:SETSPEED:" + axisStr + ":" + speedStr);
    }
    else if (axisStr == "ALL") {
        float speed = speedStr.toFloat();
        for (int i = 0; i < 3; i++) {
            axes[i].maxSpeed = speed;
        }
        sendResponse("OK:SETSPEED:ALL:" + speedStr);
    }
    else {
        sendResponse("ERROR:INVALID_AXIS");
    }
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void updateMotors() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastUpdate < UPDATE_INTERVAL) {
        return;
    }
    
    float dt = (currentTime - lastUpdate) / 1000.0;  // Convert to seconds
    lastUpdate = currentTime;
    
    for (int i = 0; i < 3; i++) {
        updateAxis(i, dt);
    }
}

void updateAxis(int axisIndex, float dt) {
    AxisConfig &axis = axes[axisIndex];
    
    if (!axis.isMoving) {
        axis.servo.write(90);
        return;
    }
    
    // Calculate error
    float error = axis.targetPosition - axis.currentPosition;
    
    // Check if reached target
    if (abs(error) < 0.1) {
        axis.currentPosition = axis.targetPosition;
        axis.isMoving = false;
        axis.speed = 0;
        axis.direction = 0;
        axis.servo.write(90);
        sendResponse("OK:DONE:" + String(axisIndex));
        return;
    }
    
    // Update speed with acceleration
    float targetSpeed = axis.direction * axis.maxSpeed;
    if (axis.speed < targetSpeed) {
        axis.speed = min(axis.speed + axis.acceleration * dt, targetSpeed);
    }
    else if (axis.speed > targetSpeed) {
        axis.speed = max(axis.speed - axis.acceleration * dt, targetSpeed);
    }
    
    // Update position
    float positionDelta = axis.speed * dt;
    axis.currentPosition += positionDelta;
    
    // Calculate servo angle (90 = neutral, 0-180 range)
    int servoAngle = mapFloat(axis.currentPosition, -90.0, 90.0, 0.0, 180.0);
    servoAngle = constrain(servoAngle, 0, 180);
    
    axis.servo.write(servoAngle);
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

int getAxisIndex(char axis) {
    axis = toupper(axis);
    switch (axis) {
        case 'X': return 0;
        case 'Y': return 1;
        case 'Z': return 2;
        default: return -1;
    }
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
