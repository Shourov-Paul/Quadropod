#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Robot geometry in mm (can be adjusted)
const float L_COXA = 50.0;
const float L_FEMUR = 70.0;
const float L_TIBIA = 90.0;

#define NUM_LEGS 4
#define JOINTS_PER_LEG 3
#define MAX_CHANNELS 16

// Current and target angles (in degrees)
float currentAngles[MAX_CHANNELS];
float targetAngles[MAX_CHANNELS];
float servoOffsets[MAX_CHANNELS]; // Loaded from EEPROM

// Maps Leg (0-3) and Joint (0-2) to PCA9685 Channel
// Leg 0: channels 0,1,2
// Leg 1: channels 4,5,6
// Leg 2: channels 8,9,10
// Leg 3: channels 12,13,14
const int servoMap[NUM_LEGS][JOINTS_PER_LEG] = {
  {0, 1, 2},    // Leg 1
  {4, 5, 6},    // Leg 2
  {8, 9, 10},   // Leg 3
  {12, 13, 14}  // Leg 4
};

// Interpolation speed
float moveSpeed = 5.0; // max degrees per update
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 20; // 50Hz = 20ms

void setup() {
  Serial.begin(115200);
  delay(100);

  // Setup I2C on custom pins 21, 22
  Wire.begin(21, 22); 
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  EEPROM.begin(512);
  loadCalibration();

  // Initialize arrays to 90 degrees (neutral position)
  for (int i = 0; i < MAX_CHANNELS; i++) {
    currentAngles[i] = 90.0;
    targetAngles[i] = 90.0;
  }

  // Set physical servos to initial positions defined by the map
  for (int l = 0; l < NUM_LEGS; l++) {
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
      int channel = servoMap[l][j];
      setServoAngleRaw(channel, 90.0);
    }
  }
  
  Serial.println("Quadruped Controller Ready");
}

void loadCalibration() {
  // Check magic bytes to see if EEPROM is initialized
  if (EEPROM.read(0) == 0xAA && EEPROM.read(1) == 0xBB) {
    int addr = 2;
    for (int i = 0; i < MAX_CHANNELS; i++) {
      float offset;
      EEPROM.get(addr, offset);
      // Ensure the offset is somewhat sane (-90 to +90)
      if (isnan(offset) || offset < -90.0 || offset > 90.0) {
        offset = 0.0;
      }
      servoOffsets[i] = offset;
      addr += sizeof(float);
    }
    Serial.println("Loaded Calibration from EEPROM");
  } else {
    // Uninitialized, set to 0
    for (int i = 0; i < MAX_CHANNELS; i++) {
      servoOffsets[i] = 0.0;
    }
    Serial.println("New EEPROM. Using default 0 offsets");
  }
}

void saveCalibration() {
  EEPROM.write(0, 0xAA);
  EEPROM.write(1, 0xBB);
  int addr = 2;
  for (int i = 0; i < MAX_CHANNELS; i++) {
    EEPROM.put(addr, servoOffsets[i]);
    addr += sizeof(float);
  }
  EEPROM.commit();
  Serial.println("OK SAVE");
}

// Map angle (0-180) to PCA9685 pulse
void setServoAngleRaw(int channel, float angle) {
  // Apply individual offset
  float finalAngle = angle + servoOffsets[channel];
  
  // Constrain based on reasonable servo limitations 
  if (finalAngle < 0) finalAngle = 0;
  if (finalAngle > 180) finalAngle = 180;
  
  int pulse = map(finalAngle * 10, 0, 1800, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void updateServosSmoothly() {
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();
    for (int l = 0; l < NUM_LEGS; l++) {
      for (int j = 0; j < JOINTS_PER_LEG; j++) {
        int channel = servoMap[l][j];
        if (abs(currentAngles[channel] - targetAngles[channel]) > 0.1) {
          if (currentAngles[channel] < targetAngles[channel]) {
            currentAngles[channel] += moveSpeed;
            if (currentAngles[channel] > targetAngles[channel]) currentAngles[channel] = targetAngles[channel];
          } else {
            currentAngles[channel] -= moveSpeed;
            if (currentAngles[channel] < targetAngles[channel]) currentAngles[channel] = targetAngles[channel];
          }
          setServoAngleRaw(channel, currentAngles[channel]);
        }
      }
    }
  }
}

void loop() {
  processSerial();
  updateServosSmoothly();
}

void processSerial() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    // Example Commands:
    // L1S1:90           (Leg 1, Servo 1 to 90 deg)
    // CALIB:1:1:5.0     (Leg 1, Servo 1 offset 5.0 deg)
    // MOVE:1:50,0,-50   (IK Move Leg 1 to XYZ)
    // SAVE              (Save currently requested calibration)

    if (cmd == "SAVE") {
      saveCalibration();
    }
    else if (cmd.startsWith("L") && cmd.indexOf("S") > 0 && cmd.indexOf(":") > 0) {
      int sIndex = cmd.indexOf("S");
      int colonIndex = cmd.indexOf(":");
      int legIndex = cmd.substring(1, sIndex).toInt() - 1;
      int jointIndex = cmd.substring(sIndex + 1, colonIndex).toInt() - 1;
      float angle = cmd.substring(colonIndex + 1).toFloat();
      
      if (legIndex >= 0 && legIndex < NUM_LEGS && jointIndex >= 0 && jointIndex < JOINTS_PER_LEG) {
        int channel = servoMap[legIndex][jointIndex];
        
        // Safety bounds
        if(angle < 0) angle = 0;
        if(angle > 180) angle = 180;
        
        targetAngles[channel] = angle;
        Serial.print("OK L"); Serial.print(legIndex+1);
        Serial.print("S"); Serial.print(jointIndex+1);
        Serial.print(":"); Serial.println(angle);
      }
    }
    else if (cmd.startsWith("CALIB:")) {
      int f1 = cmd.indexOf(':', 6);
      int f2 = cmd.indexOf(':', f1 + 1);
      if (f1 > 0 && f2 > 0) {
        int legIndex = cmd.substring(6, f1).toInt() - 1;
        int jointIndex = cmd.substring(f1 + 1, f2).toInt() - 1;
        float offset = cmd.substring(f2 + 1).toFloat();
        
        if (legIndex >= 0 && legIndex < NUM_LEGS && jointIndex >= 0 && jointIndex < JOINTS_PER_LEG) {
          int channel = servoMap[legIndex][jointIndex];
          servoOffsets[channel] = offset;
          
          Serial.print("OK CALIB:"); Serial.print(legIndex+1);
          Serial.print(":"); Serial.print(jointIndex+1);
          Serial.print(":"); Serial.println(offset);
          
          // Apply to hardware immediately for visual feedback
          setServoAngleRaw(channel, currentAngles[channel]); 
        }
      }
    }
    else if (cmd.startsWith("MOVE:")) {
      // MOVE:1:X,Y,Z
      int legParamsStart = cmd.indexOf(':', 5);
      if (legParamsStart > 0) {
        int legIndex = cmd.substring(5, legParamsStart).toInt() - 1;
        String coords = cmd.substring(legParamsStart + 1);
        int comma1 = coords.indexOf(',');
        int comma2 = coords.indexOf(',', comma1 + 1);
        
        if (comma1 > 0 && comma2 > 0) {
          float x = coords.substring(0, comma1).toFloat();
          float y = coords.substring(comma1 + 1, comma2).toFloat();
          float z = coords.substring(comma2 + 1).toFloat();
          
          if (legIndex >= 0 && legIndex < NUM_LEGS) {
            calculateIK(legIndex, x, y, z);
            Serial.print("OK MOVE:"); Serial.print(legIndex+1);
            Serial.print(":"); Serial.print(x); Serial.print(",");
            Serial.print(y); Serial.print(","); Serial.println(z);
          }
        }
      }
    }
    else {
      Serial.println("ERR Unknown Command");
    }
  }
}

void calculateIK(int legIndex, float x, float y, float z) {
  // Simple 3DOF Inverse Kinematics
  // Origin is at Coxa (Hip), x is forward/back relative to leg orientation,
  // y is side-side (away from body), z is up/down (usually negative).

  float L = sqrt(x * x + y * y);
  float gamma = atan2(y, x); // Hip angle
  
  float L1 = L - L_COXA;
  if(L1 < 0) L1 = 0;
  
  float D = sqrt(L1 * L1 + z * z);
  
  // Reach constraint
  if (D > (L_FEMUR + L_TIBIA)) {
    D = L_FEMUR + L_TIBIA - 0.01;
  }

  // Alpha is the internal angle for the shoulder
  float alpha1 = atan2(abs(z), L1); 
  float cosAlpha2 = (L_FEMUR * L_FEMUR + D * D - L_TIBIA * L_TIBIA) / (2 * L_FEMUR * D);
  float alpha2 = acos(constrain(cosAlpha2, -1.0, 1.0));
  float alpha = alpha1 + alpha2; 

  // Beta is internal knee angle
  float cosBeta = (L_FEMUR * L_FEMUR + L_TIBIA * L_TIBIA - D * D) / (2 * L_FEMUR * L_TIBIA);
  float beta_inner = acos(constrain(cosBeta, -1.0, 1.0));
  float beta = PI - beta_inner; 

  // Map to degrees assuming 90 is center
  // These mapping equations will differ based on physical assembly,
  // but provide a unified starting mathematical framework.
  float joint0 = 90.0 + (gamma * 180.0 / PI); 
  float joint1 = 90.0 + (alpha * 180.0 / PI); 
  float joint2 = 90.0 + (beta * 180.0 / PI);  

  int ch0 = servoMap[legIndex][0];
  int ch1 = servoMap[legIndex][1];
  int ch2 = servoMap[legIndex][2];
  
  targetAngles[ch0] = joint0;
  targetAngles[ch1] = joint1; 
  targetAngles[ch2] = joint2; 
}
