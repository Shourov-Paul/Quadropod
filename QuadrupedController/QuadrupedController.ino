#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
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
float homeAngles[MAX_CHANNELS];   // The saved manual pose used as the gait base

// Walking state
bool isWalking = false;
char walkDir = 'F';
int walkSpeed = 5;
unsigned long walkTimer = 0;
int walkStep = 0;

// Gait tuning: angular offsets applied relative to the saved home pose
const float HIP_SWING = 15.0;   // degrees hip swings forward/back per step
const float LIFT_AMOUNT = 20.0; // degrees shoulder lifts up during swing phase
const float KNEE_BEND = 15.0;   // degrees knee bends during lift phase

// Maps Leg (0-3) and Joint (0-2) to PCA9685 Channel
// Leg 0: channels 0,1,2
// Leg 1: channels 4,5,6
// Leg 2: channels 8,9,10
// Leg 3: channels 12,13,14
const int servoMap[NUM_LEGS][JOINTS_PER_LEG] = {
    {0, 1, 2},   // Leg 1
    {4, 5, 6},   // Leg 2
    {8, 9, 10},  // Leg 3
    {12, 13, 14} // Leg 4
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

  // Initialize arrays to 90 degrees (neutral position)
  for (int i = 0; i < MAX_CHANNELS; i++) {
    currentAngles[i] = 90.0;
    targetAngles[i] = 90.0;
    homeAngles[i] = 90.0;
  }

  EEPROM.begin(512);
  loadStateFromEEPROM();

  // Set physical servos to initial positions defined by the EEPROM or defaults
  for (int l = 0; l < NUM_LEGS; l++) {
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
      int channel = servoMap[l][j];
      setServoAngleRaw(channel, currentAngles[channel]);
    }
  }

  Serial.println("Quadruped Controller Ready");
}

void loadStateFromEEPROM() {
  // Check magic bytes to see if EEPROM is initialized
  if (EEPROM.read(0) == 0xAA && EEPROM.read(1) == 0xBB) {
    int addr = 2;
    // 1. Load Calibration Offsets (MAX_CHANNELS * float)
    for (int i = 0; i < MAX_CHANNELS; i++) {
      float offset;
      EEPROM.get(addr, offset);
      if (isnan(offset) || offset < -90.0 || offset > 90.0) {
        offset = 0.0;
      }
      servoOffsets[i] = offset;
      addr += sizeof(float);
    }

    // 2. Load Saved Target Angles (MAX_CHANNELS * float)
    for (int i = 0; i < MAX_CHANNELS; i++) {
      float savedAngle;
      EEPROM.get(addr, savedAngle);
      if (isnan(savedAngle) || savedAngle < 0.0 || savedAngle > 180.0) {
        savedAngle = 90.0; // Default to neutral
      }
      currentAngles[i] = savedAngle;
      targetAngles[i] = savedAngle;
      homeAngles[i] = savedAngle; // Use saved pose as gait home base
      addr += sizeof(float);
    }
    Serial.println("Loaded Calibration and Angles from EEPROM");
  } else {
    // Uninitialized, set default
    for (int i = 0; i < MAX_CHANNELS; i++) {
      servoOffsets[i] = 0.0;
      currentAngles[i] = 90.0;
      targetAngles[i] = 90.0;
      homeAngles[i] = 90.0;
    }
    Serial.println("New EEPROM. Using default 0 offsets and 90 angles");
  }
}

void saveStateToEEPROM() {
  EEPROM.write(0, 0xAA);
  EEPROM.write(1, 0xBB);
  int addr = 2;
  // 1. Save Offsets
  for (int i = 0; i < MAX_CHANNELS; i++) {
    EEPROM.put(addr, servoOffsets[i]);
    addr += sizeof(float);
  }
  // 2. Save Current Configured Angles
  for (int i = 0; i < MAX_CHANNELS; i++) {
    EEPROM.put(addr, targetAngles[i]);
    addr += sizeof(float);
  }
  EEPROM.commit();

  // Also update homeAngles to match whatever was just saved
  for (int i = 0; i < MAX_CHANNELS; i++) {
    homeAngles[i] = targetAngles[i];
  }

  Serial.println("OK SAVE");
}

// Map angle (0-180) to PCA9685 pulse
void setServoAngleRaw(int channel, float angle) {
  // Apply individual offset
  float finalAngle = angle + servoOffsets[channel];

  // Constrain based on reasonable servo limitations
  if (finalAngle < 0)
    finalAngle = 0;
  if (finalAngle > 180)
    finalAngle = 180;

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
            if (currentAngles[channel] > targetAngles[channel])
              currentAngles[channel] = targetAngles[channel];
          } else {
            currentAngles[channel] -= moveSpeed;
            if (currentAngles[channel] < targetAngles[channel])
              currentAngles[channel] = targetAngles[channel];
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
  updateGait();
}

void processSerial() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0)
      return;

    // Example Commands:
    // L1S1:90           (Leg 1, Servo 1 to 90 deg)
    // CALIB:1:1:5.0     (Leg 1, Servo 1 offset 5.0 deg)
    // MOVE:1:50,0,-50   (IK Move Leg 1 to XYZ)
    // WALK:F:5          (Start walking Forward at speed 5)
    // STOP              (Stop walking)
    // SAVE              (Save currently requested calibration AND angles)
    // REPORT            (Send current angles back to WebApp)

    if (cmd == "SAVE") {
      saveStateToEEPROM();
    } else if (cmd == "REPORT") {
      // Send STATE:leg:joint:angle to sync web UI
      for (int l = 0; l < NUM_LEGS; l++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
          int channel = servoMap[l][j];
          Serial.print("STATE:");
          Serial.print(l + 1);
          Serial.print(":");
          Serial.print(j + 1);
          Serial.print(":");
          Serial.println(targetAngles[channel]);
        }
      }
      Serial.println("STATE:DONE");
    } else if (cmd == "STOP") {
      isWalking = false;
      // Return all legs to saved home pose
      for (int i = 0; i < MAX_CHANNELS; i++) {
        targetAngles[i] = homeAngles[i];
      }
      Serial.println("OK STOP");
    } else if (cmd.startsWith("WALK:")) {
      int colonIndex = cmd.lastIndexOf(':');
      if (colonIndex > 5) {
        walkDir = cmd.charAt(5);
        walkSpeed = cmd.substring(colonIndex + 1).toInt();
        isWalking = true;
        walkStep = 0; // reset gait cycle
        Serial.print("OK WALK:");
        Serial.print(walkDir);
        Serial.print(":");
        Serial.println(walkSpeed);
      }
    } else if (cmd.startsWith("L") && cmd.indexOf("S") > 0 &&
               cmd.indexOf(":") > 0) {
      int sIndex = cmd.indexOf("S");
      int colonIndex = cmd.indexOf(":");
      int legIndex = cmd.substring(1, sIndex).toInt() - 1;
      int jointIndex = cmd.substring(sIndex + 1, colonIndex).toInt() - 1;
      float angle = cmd.substring(colonIndex + 1).toFloat();

      if (legIndex >= 0 && legIndex < NUM_LEGS && jointIndex >= 0 &&
          jointIndex < JOINTS_PER_LEG) {
        int channel = servoMap[legIndex][jointIndex];

        // Safety bounds
        if (angle < 0)
          angle = 0;
        if (angle > 180)
          angle = 180;

        targetAngles[channel] = angle;
        Serial.print("OK L");
        Serial.print(legIndex + 1);
        Serial.print("S");
        Serial.print(jointIndex + 1);
        Serial.print(":");
        Serial.println(angle);
      }
    } else if (cmd.startsWith("CALIB:")) {
      int f1 = cmd.indexOf(':', 6);
      int f2 = cmd.indexOf(':', f1 + 1);
      if (f1 > 0 && f2 > 0) {
        int legIndex = cmd.substring(6, f1).toInt() - 1;
        int jointIndex = cmd.substring(f1 + 1, f2).toInt() - 1;
        float offset = cmd.substring(f2 + 1).toFloat();

        if (legIndex >= 0 && legIndex < NUM_LEGS && jointIndex >= 0 &&
            jointIndex < JOINTS_PER_LEG) {
          int channel = servoMap[legIndex][jointIndex];
          servoOffsets[channel] = offset;

          Serial.print("OK CALIB:");
          Serial.print(legIndex + 1);
          Serial.print(":");
          Serial.print(jointIndex + 1);
          Serial.print(":");
          Serial.println(offset);

          // Apply to hardware immediately for visual feedback
          setServoAngleRaw(channel, currentAngles[channel]);
        }
      }
    } else if (cmd.startsWith("MOVE:")) {
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
            Serial.print("OK MOVE:");
            Serial.print(legIndex + 1);
            Serial.print(":");
            Serial.print(x);
            Serial.print(",");
            Serial.print(y);
            Serial.print(",");
            Serial.println(z);
          }
        }
      }
    } else {
      Serial.println("ERR Unknown Command");
    }
  }
}

void calculateIK(int legIndex, float x, float y, float z) {
  float L = sqrt(x * x + y * y);
  float gamma = atan2(y, x);

  float L1 = L - L_COXA;
  if (L1 < 0)
    L1 = 0;

  float D = sqrt(L1 * L1 + z * z);

  if (D > (L_FEMUR + L_TIBIA)) {
    D = L_FEMUR + L_TIBIA - 0.01;
  }

  float alpha1 = atan2(abs(z), L1);
  float cosAlpha2 =
      (L_FEMUR * L_FEMUR + D * D - L_TIBIA * L_TIBIA) / (2 * L_FEMUR * D);
  float alpha2 = acos(constrain(cosAlpha2, -1.0, 1.0));
  float alpha = alpha1 + alpha2;

  float cosBeta =
      (L_FEMUR * L_FEMUR + L_TIBIA * L_TIBIA - D * D) / (2 * L_FEMUR * L_TIBIA);
  float beta_inner = acos(constrain(cosBeta, -1.0, 1.0));
  float beta = PI - beta_inner;

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

// ============================================================
// GAIT ENGINE — works relative to saved homeAngles[]
// ============================================================
// Helper: constrain an angle to 0-180 safely
float safeAngle(float a) {
  if (a < 0)
    return 0;
  if (a > 180)
    return 180;
  return a;
}

// Set a single leg's targets relative to its home angles
// hipOffset: positive = swing forward, negative = swing backward
// shoulderOffset: negative = lift leg up (reduce angle)
// kneeOffset: positive = bend knee more
void setLegRelative(int legIdx, float hipOff, float shoulderOff,
                    float kneeOff) {
  int ch0 = servoMap[legIdx][0]; // hip
  int ch1 = servoMap[legIdx][1]; // shoulder
  int ch2 = servoMap[legIdx][2]; // knee

  targetAngles[ch0] = safeAngle(homeAngles[ch0] + hipOff);
  targetAngles[ch1] = safeAngle(homeAngles[ch1] + shoulderOff);
  targetAngles[ch2] = safeAngle(homeAngles[ch2] + kneeOff);
}

// Return a single leg to its home (saved) position
void setLegHome(int legIdx) { setLegRelative(legIdx, 0, 0, 0); }

void updateGait() {
  if (!isWalking)
    return;

  // Delay between gait steps inversely proportional to speed
  int stepDelay = map(walkSpeed, 1, 10, 800, 100);

  if (millis() - walkTimer >= (unsigned long)stepDelay) {
    walkTimer = millis();

    // Determine direction multipliers
    // For Forward/Back:  hip swings forward(+) and backward(-)
    // For Left/Right:    hip offsets simulate turning
    float hipSwing = HIP_SWING;

    float dirMultiplier = 0;
    if (walkDir == 'F')
      dirMultiplier = 1.0;
    else if (walkDir == 'B')
      dirMultiplier = -1.0;
    else if (walkDir == 'L')
      dirMultiplier = 1.0; // Turn left
    else if (walkDir == 'R')
      dirMultiplier = -1.0; // Turn right

    bool isTurning = (walkDir == 'L' || walkDir == 'R');

    // 8-step creep gait: move one leg at a time
    // Each leg cycle: lift → swing forward → plant → push back
    //
    // For walking straight: diagonal pairs work in opposition
    //   Leg order: 0(FL) → 3(BR) → 1(FR) → 2(BL)
    //
    // For turning: all legs swing in same direction to yaw the body

    switch (walkStep) {
    case 0:
      // --- LIFT Leg 0 (FL) and swing it forward ---
      if (isTurning) {
        setLegRelative(0, hipSwing * dirMultiplier, -LIFT_AMOUNT, KNEE_BEND);
      } else {
        setLegRelative(0, hipSwing * dirMultiplier, -LIFT_AMOUNT, KNEE_BEND);
      }
      break;

    case 1:
      // --- PLANT Leg 0 down in new position ---
      if (isTurning) {
        setLegRelative(0, hipSwing * dirMultiplier, 0, 0);
      } else {
        setLegRelative(0, hipSwing * dirMultiplier, 0, 0);
      }
      break;

    case 2:
      // --- LIFT Leg 3 (BR) and swing it forward ---
      if (isTurning) {
        setLegRelative(3, hipSwing * dirMultiplier, -LIFT_AMOUNT, KNEE_BEND);
      } else {
        setLegRelative(3, hipSwing * dirMultiplier, -LIFT_AMOUNT, KNEE_BEND);
      }
      break;

    case 3:
      // --- PLANT Leg 3 ---
      if (isTurning) {
        setLegRelative(3, hipSwing * dirMultiplier, 0, 0);
      } else {
        setLegRelative(3, hipSwing * dirMultiplier, 0, 0);
      }
      break;

    case 4:
      // --- LIFT Leg 1 (FR) and swing it forward ---
      if (isTurning) {
        setLegRelative(1, hipSwing * dirMultiplier, -LIFT_AMOUNT, KNEE_BEND);
      } else {
        setLegRelative(1, hipSwing * dirMultiplier, -LIFT_AMOUNT, KNEE_BEND);
      }
      break;

    case 5:
      // --- PLANT Leg 1 ---
      if (isTurning) {
        setLegRelative(1, hipSwing * dirMultiplier, 0, 0);
      } else {
        setLegRelative(1, hipSwing * dirMultiplier, 0, 0);
      }
      break;

    case 6:
      // --- LIFT Leg 2 (BL) and swing it forward ---
      if (isTurning) {
        setLegRelative(2, hipSwing * dirMultiplier, -LIFT_AMOUNT, KNEE_BEND);
      } else {
        setLegRelative(2, hipSwing * dirMultiplier, -LIFT_AMOUNT, KNEE_BEND);
      }
      break;

    case 7:
      // --- PLANT Leg 2 and push body: shift all planted legs backward ---
      if (isTurning) {
        setLegRelative(2, hipSwing * dirMultiplier, 0, 0);
      } else {
        setLegRelative(2, hipSwing * dirMultiplier, 0, 0);
        // Push phase: shift the 3 grounded legs backward to propel body forward
        setLegRelative(0, -hipSwing * dirMultiplier * 0.5, 0, 0);
        setLegRelative(3, -hipSwing * dirMultiplier * 0.5, 0, 0);
        setLegRelative(1, -hipSwing * dirMultiplier * 0.5, 0, 0);
      }
      break;
    }

    walkStep++;
    if (walkStep > 7)
      walkStep = 0;
  }
}
