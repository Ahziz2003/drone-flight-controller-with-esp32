/** 
 * Hardware Configuration:
 * - MCU: ESP32
 * - IMU: MPU6050
 *        X → GAUCHE (vers la gauche du drone)
 *        Y → ARRIÈRE (vers l'arrière du drone)
 *        Z → HAUT (vers le ciel)
 * - ESCs: DShot300 protocol
 * 
 * Motor Layout (view from top):
 *        FRONT
 *    FL(25)   FR(26)
 *      CCW     CW
 *        \   /
 *         \ /
 *         / \
 *        /   \
 *      CW     CCW
 *    RL(14)   RR(27)
 *        REAR
 * 
 * Conventions:
 * - Roll positif  = côté DROIT descend (incliné à droite)
 * - Pitch positif = NEZ monte
 * 
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/rmt.h"


// WiFi AP Configuration
const char* AP_SSID = "DroneFC";
const char* AP_PASSWORD = "12345678";
const uint16_t UDP_PORT = 4210;

// I2C Pins for MPU6050
#define I2C_SDA 19
#define I2C_SCL 18

// Motor GPIO Pins - CORRECTED
#define MOTOR_FL 25  // Front Left  - CCW
#define MOTOR_FR 26  // Front Right - CW
#define MOTOR_RL 14  // Rear Left   - CW
#define MOTOR_RR 27  // Rear Right  - CCW

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// Loop timing
#define LOOP_FREQUENCY_HZ 500
#define LOOP_PERIOD_US (1000000 / LOOP_FREQUENCY_HZ)

// Motor limits
#define MOTOR_MIN 48      // DShot minimum (motor spin)
#define MOTOR_MAX 2000    // DShot maximum
#define MOTOR_IDLE 100    // Idle speed when armed

// DShot Configuration
#define DSHOT_T0H_TICKS 10
#define DSHOT_T0L_TICKS 17
#define DSHOT_T1H_TICKS 20
#define DSHOT_T1L_TICKS 7

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// WiFi & UDP
WiFiUDP udp;
IPAddress clientIP;
uint16_t clientPort = 0;
bool clientConnected = false;

// IMU Data
float gyroX = 0, gyroY = 0, gyroZ = 0;       // deg/s
float accelX = 0, accelY = 0, accelZ = 0;    // g

// Attitude
float roll = 0, pitch = 0, yaw = 0;          // degrees
float rollRate = 0, pitchRate = 0, yawRate = 0;  // deg/s

// Quaternion for Madgwick
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Calibration offsets
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
bool imuCalibrated = false;

// PID Gains - Angle loop (outer)
float rollAngleP = 4.0, rollAngleI = 0.0, rollAngleD = 0.0;
float pitchAngleP = 4.0, pitchAngleI = 0.0, pitchAngleD = 0.0;

// PID Gains - Rate loop (inner)
float rollRateP = 0.7, rollRateI = 0.3, rollRateD = 0.01;
float pitchRateP = 0.7, pitchRateI = 0.3, pitchRateD = 0.01;

// PID State - Angle loop
float rollAngleIntegral = 0, rollAnglePrevError = 0;
float pitchAngleIntegral = 0, pitchAnglePrevError = 0;

// PID State - Rate loop
float rollRateIntegral = 0, rollRatePrevError = 0;
float pitchRateIntegral = 0, pitchRatePrevError = 0;

// PID Limits
const float ANGLE_INTEGRAL_LIMIT = 50.0;
const float ANGLE_OUTPUT_LIMIT = 300.0;
const float RATE_INTEGRAL_LIMIT = 100.0;
const float RATE_OUTPUT_LIMIT = 500.0;

// Flight state
bool armed = false;
bool failsafe = false;
uint32_t lastCommandTime = 0;
float throttle = 0.0;
float targetRoll = 0.0;
float targetPitch = 0.0;

// Motor outputs
uint16_t motorFL = 0, motorFR = 0, motorRL = 0, motorRR = 0;

// Timing
uint32_t lastLoopTime = 0;
uint32_t loopCount = 0;
float dt = 0.002f;

// Madgwick filter gain (higher = faster response, more noise)
float madgwickBeta = 0.5f;  // Increased from 0.1 for faster response

// Telemetry divider
#define TELEMETRY_DIVIDER 10

// DShot RMT
rmt_item32_t dshotBuffer[4][16];
const uint8_t motorPins[4] = {MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR};
const rmt_channel_t rmtChannels[4] = {RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3};

// DSHOT FUNCTIONS

void initDShot() {
  for (int i = 0; i < 4; i++) {
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = rmtChannels[i];
    config.gpio_num = (gpio_num_t)motorPins[i];
    config.clk_div = 10;  // 80MHz / 10 = 8MHz
    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;
    
    rmt_config(&config);
    rmt_driver_install(rmtChannels[i], 0, 0);
  }
  
  Serial.println("[DSHOT] Initialized on pins FL:25 FR:26 RL:14 RR:27");
}

uint16_t prepareDshotPacket(uint16_t value) {
  value = constrain(value, 0, 2047);
  uint16_t packet = (value << 1);  // telemetry bit = 0
  uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
  return (packet << 4) | crc;
}

void sendDshotCommand(int motorIndex, uint16_t value) {
  uint16_t packet = prepareDshotPacket(value);
  
  for (int i = 0; i < 16; i++) {
    if (packet & (1 << (15 - i))) {
      dshotBuffer[motorIndex][i].duration0 = DSHOT_T1H_TICKS;
      dshotBuffer[motorIndex][i].level0 = 1;
      dshotBuffer[motorIndex][i].duration1 = DSHOT_T1L_TICKS;
      dshotBuffer[motorIndex][i].level1 = 0;
    } else {
      dshotBuffer[motorIndex][i].duration0 = DSHOT_T0H_TICKS;
      dshotBuffer[motorIndex][i].level0 = 1;
      dshotBuffer[motorIndex][i].duration1 = DSHOT_T0L_TICKS;
      dshotBuffer[motorIndex][i].level1 = 0;
    }
  }
  
  rmt_write_items(rmtChannels[motorIndex], dshotBuffer[motorIndex], 16, false);
}

void sendMotorCommands() {
  sendDshotCommand(0, motorFL);
  sendDshotCommand(1, motorFR);
  sendDshotCommand(2, motorRL);
  sendDshotCommand(3, motorRR);
}

void stopMotors() {
  motorFL = 0;
  motorFR = 0;
  motorRL = 0;
  motorRR = 0;
  sendMotorCommands();
}

// MPU6050 FUNCTIONS

void initMPU6050() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  // Wake up
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  
  // Gyro: ±500 deg/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  // Accel: ±4g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  
  // DLPF: 42Hz
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  
  // Sample rate: 500Hz
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x19);
  Wire.write(0x01);
  Wire.endTransmission();
  
  Serial.println("[IMU] MPU6050 initialized");
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);
  
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // temp
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  
  accelX = (float)ax / 8192.0f - accelOffsetX;
  accelY = (float)ay / 8192.0f - accelOffsetY;
  accelZ = (float)az / 8192.0f - accelOffsetZ;
  
  gyroX = (float)gx / 65.5f - gyroOffsetX;
  gyroY = (float)gy / 65.5f - gyroOffsetY;
  gyroZ = (float)gz / 65.5f - gyroOffsetZ;
}

void calibrateIMU() {
  Serial.println("[IMU] Calibrating... Keep drone STILL and LEVEL!");
  
  float sumGx = 0, sumGy = 0, sumGz = 0;
  float sumAx = 0, sumAy = 0, sumAz = 0;
  const int samples = 1000;
  
  // Reset offsets temporarily
  gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0;
  accelOffsetX = accelOffsetY = accelOffsetZ = 0;
  
  for (int i = 0; i < samples; i++) {
    readMPU6050();
    sumGx += gyroX;
    sumGy += gyroY;
    sumGz += gyroZ;
    sumAx += accelX;
    sumAy += accelY;
    sumAz += accelZ;
    delay(2);
  }
  
  // Gyro offsets - remove bias
  gyroOffsetX = sumGx / samples;
  gyroOffsetY = sumGy / samples;
  gyroOffsetZ = sumGz / samples;
  
  // Accel offsets - X and Y should be 0, Z should be +1g (pointing up)
  accelOffsetX = sumAx / samples;        // Should be ~0
  accelOffsetY = sumAy / samples;        // Should be ~0
  accelOffsetZ = (sumAz / samples) - 1.0f;  // Remove the +1g so calibrated Z reads +1g
  
  imuCalibrated = true;
  
  Serial.printf("[IMU] Calibration done!\n");
  Serial.printf("      Gyro offset: %.2f, %.2f, %.2f deg/s\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("      Accel offset: %.3f, %.3f, %.3f g\n", accelOffsetX, accelOffsetY, accelOffsetZ);
}

// MADGWICK FILTER

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  /**
   * IMPORTANT: Cette fonction attend les données dans le repère NED standard:
   * - X vers l'avant
   * - Y vers la droite  
   * - Z vers le bas
   * 
   * Notre MPU a: X→gauche, Y→bas, Z→avant
   * 
   * Donc on doit transformer AVANT d'appeler cette fonction:
   * - NED_X (avant)  = MPU_Z
   * - NED_Y (droite) = -MPU_X  
   * - NED_Z (bas)    = MPU_Y
   * 
   * Cette transformation est faite dans loop() avant d'appeler cette fonction.
   */
  
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;
  
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
  
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;
    
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    
    qDot1 -= madgwickBeta * s0;
    qDot2 -= madgwickBeta * s1;
    qDot3 -= madgwickBeta * s2;
    qDot4 -= madgwickBeta * s3;
  }
  
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;
  
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void quaternionToEuler() {
  /**
   * Après transformation vers NED, les formules Euler standard s'appliquent:
   * - Roll  = rotation autour de X (avant) → inclinaison gauche/droite
   * - Pitch = rotation autour de Y (droite) → nez monte/descend
   * - Yaw   = rotation autour de Z (bas) → rotation horizontale
   * 
   * Convention de signe:
   * - Roll positif  = côté droit descend
   * - Pitch positif = nez monte
   */
  
  // Roll (phi) - rotation autour de X
  roll = atan2(2.0f * (q0 * q1 + q2 * q3), 
               1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
  
  // Pitch (theta) - rotation autour de Y
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (fabs(sinp) >= 1.0f) {
    pitch = copysign(90.0f, sinp);
  } else {
    pitch = asin(sinp) * 57.29578f;
  }
  
  // Yaw (psi) - rotation autour de Z
  yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 
              1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
}

// PID CONTROL

void resetPID() {
  rollAngleIntegral = 0;
  rollAnglePrevError = 0;
  pitchAngleIntegral = 0;
  pitchAnglePrevError = 0;
  rollRateIntegral = 0;
  rollRatePrevError = 0;
  pitchRateIntegral = 0;
  pitchRatePrevError = 0;
}

float computePID(float error, float &integral, float &prevError, 
                 float kP, float kI, float kD, 
                 float integralLimit, float outputLimit, float dt) {
  // Proportional
  float P = kP * error;
  
  // Integral with anti-windup
  integral += error * dt;
  integral = constrain(integral, -integralLimit, integralLimit);
  float I = kI * integral;
  
  // Derivative
  float derivative = (error - prevError) / dt;
  float D = kD * derivative;
  prevError = error;
  
  // Output
  float output = P + I + D;
  return constrain(output, -outputLimit, outputLimit);
}

void computeCascadedPID(float &rollOutput, float &pitchOutput) {
  // === OUTER LOOP: Angle PID ===
  float rollAngleError = targetRoll - roll;
  float pitchAngleError = targetPitch - pitch;
  
  float targetRollRate = computePID(rollAngleError, rollAngleIntegral, rollAnglePrevError,
                                    rollAngleP, rollAngleI, rollAngleD,
                                    ANGLE_INTEGRAL_LIMIT, ANGLE_OUTPUT_LIMIT, dt);
  
  float targetPitchRate = computePID(pitchAngleError, pitchAngleIntegral, pitchAnglePrevError,
                                     pitchAngleP, pitchAngleI, pitchAngleD,
                                     ANGLE_INTEGRAL_LIMIT, ANGLE_OUTPUT_LIMIT, dt);
  
  // === INNER LOOP: Rate PID ===
  float rollRateError = targetRollRate - rollRate;
  float pitchRateError = targetPitchRate - pitchRate;
  
  rollOutput = computePID(rollRateError, rollRateIntegral, rollRatePrevError,
                          rollRateP, rollRateI, rollRateD,
                          RATE_INTEGRAL_LIMIT, RATE_OUTPUT_LIMIT, dt);
  
  pitchOutput = computePID(pitchRateError, pitchRateIntegral, pitchRatePrevError,
                           pitchRateP, pitchRateI, pitchRateD,
                           RATE_INTEGRAL_LIMIT, RATE_OUTPUT_LIMIT, dt);
}

// MOTOR MIXING

void computeMotorMix(float throttleCmd, float rollCmd, float pitchCmd) {
  /**
   * Motor mixing pour configuration X
   * 
   * Vue du dessus:
   *        FRONT
   *    FL(25)   FR(26)
   *        \   /
   *         \ /
   *         / \
   *        /   \
   *    RL(14)   RR(27)
   *        REAR
   * 
   * MPU: X→gauche, Y→arrière, Z→haut
   * 
   * LOGIQUE DE STABILISATION (soulever le côté qui est en bas):
   * - Drone penché DROITE (droite en bas) → FR(26)/RR(27) augmentent
   * - Drone penché GAUCHE (gauche en bas) → FL(25)/RL(14) augmentent
   * - Drone nez en BAS (avant en bas) → FL(25)/FR(26) augmentent
   * - Drone nez en HAUT (arrière en bas) → RL(14)/RR(27) augmentent
   */
  
  // Si pas de throttle, AUCUN moteur ne tourne
  if (throttleCmd < 0.01f) {
    motorFL = 0;
    motorFR = 0;
    motorRL = 0;
    motorRR = 0;
    return;
  }
  
  float baseThrottle = throttleCmd * (MOTOR_MAX - MOTOR_MIN) + MOTOR_MIN;
  
  // Convention: Roll+ = droite en bas, Pitch+ = nez en haut
  // PID: erreur = target(0) - mesure(+) = négatif → output négatif
  //
  // Roll: drone droite en bas → rollCmd négatif → FR/RR doivent augmenter
  //       Donc FR/RR = T - rollCmd (moins un négatif = plus)
  // Pitch: drone nez en haut → pitchCmd négatif → RL/RR doivent augmenter  
  //       Donc RL/RR = T - pitchCmd (moins un négatif = plus)
  
  float fl = baseThrottle - rollCmd - pitchCmd;
  float fr = baseThrottle + rollCmd - pitchCmd;
  float rl = baseThrottle - rollCmd + pitchCmd;
  float rr = baseThrottle + rollCmd + pitchCmd;
  
  motorFL = constrain((uint16_t)fl, MOTOR_MIN, MOTOR_MAX);
  motorFR = constrain((uint16_t)fr, MOTOR_MIN, MOTOR_MAX);
  motorRL = constrain((uint16_t)rl, MOTOR_MIN, MOTOR_MAX);
  motorRR = constrain((uint16_t)rr, MOTOR_MIN, MOTOR_MAX);
}

// WIFI & UDP

void initWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("[WIFI] AP: %s, IP: %s, Port: %d\n", AP_SSID, IP.toString().c_str(), UDP_PORT);
  
  udp.begin(UDP_PORT);
}

void processUDPCommands() {
  int packetSize = udp.parsePacket();
  if (packetSize == 0) return;
  
  char buffer[256];
  int len = udp.read(buffer, sizeof(buffer) - 1);
  buffer[len] = '\0';
  
  clientIP = udp.remoteIP();
  clientPort = udp.remotePort();
  clientConnected = true;
  lastCommandTime = millis();
  
  String cmd = String(buffer);
  cmd.trim();
  
  // ARM
  if (cmd == "ARM") {
    if (imuCalibrated) {
      armed = true;
      throttle = 0;
      targetRoll = 0;
      targetPitch = 0;
      resetPID();
      Serial.println("[CMD] ARMED");
    } else {
      Serial.println("[CMD] Cannot arm - not calibrated!");
    }
  }
  // DISARM
  else if (cmd == "DISARM") {
    armed = false;
    stopMotors();
    Serial.println("[CMD] DISARMED");
  }
  // CALIBRATE
  else if (cmd == "CAL") {
    if (!armed) {
      calibrateIMU();
    }
  }
  // CONTROL: CTRL,throttle,roll,pitch
  else if (cmd.startsWith("CTRL,")) {
    if (armed) {
      float t, r, p;
      if (sscanf(buffer, "CTRL,%f,%f,%f", &t, &r, &p) == 3) {
        throttle = constrain(t, 0.0f, 1.0f);
        targetRoll = constrain(r, -45.0f, 45.0f);
        targetPitch = constrain(p, -45.0f, 45.0f);
      }
    }
  }
  // PID: PID,type,P,I,D
  else if (cmd.startsWith("PID,")) {
    char type[3];
    float p, i, d;
    if (sscanf(buffer, "PID,%2s,%f,%f,%f", type, &p, &i, &d) == 4) {
      String t = String(type);
      if (t == "RA") {
        rollAngleP = p; rollAngleI = i; rollAngleD = d;
        Serial.printf("[PID] Roll Angle: %.2f, %.2f, %.3f\n", p, i, d);
      }
      else if (t == "PA") {
        pitchAngleP = p; pitchAngleI = i; pitchAngleD = d;
        Serial.printf("[PID] Pitch Angle: %.2f, %.2f, %.3f\n", p, i, d);
      }
      else if (t == "RR") {
        rollRateP = p; rollRateI = i; rollRateD = d;
        Serial.printf("[PID] Roll Rate: %.2f, %.2f, %.3f\n", p, i, d);
      }
      else if (t == "PR") {
        pitchRateP = p; pitchRateI = i; pitchRateD = d;
        Serial.printf("[PID] Pitch Rate: %.2f, %.2f, %.3f\n", p, i, d);
      }
    }
  }
  // BETA
  else if (cmd.startsWith("BETA,")) {
    float b;
    if (sscanf(buffer, "BETA,%f", &b) == 1) {
      madgwickBeta = constrain(b, 0.01f, 1.0f);
      Serial.printf("[FILTER] Beta = %.3f\n", madgwickBeta);
    }
  }
  // PING
  else if (cmd == "PING") {
    char response[64];
    snprintf(response, sizeof(response), "PONG,%d,%d", armed ? 1 : 0, imuCalibrated ? 1 : 0);
    udp.beginPacket(clientIP, clientPort);
    udp.write((uint8_t*)response, strlen(response));
    udp.endPacket();
  }
  // GETPID
  else if (cmd == "GETPID") {
    char response[256];
    snprintf(response, sizeof(response), 
      "PIDS,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f",
      rollAngleP, rollAngleI, rollAngleD,
      pitchAngleP, pitchAngleI, pitchAngleD,
      rollRateP, rollRateI, rollRateD,
      pitchRateP, pitchRateI, pitchRateD
    );
    udp.beginPacket(clientIP, clientPort);
    udp.write((uint8_t*)response, strlen(response));
    udp.endPacket();
  }
}

void sendTelemetry() {
  if (!clientConnected) return;
  
  char buffer[256];
  snprintf(buffer, sizeof(buffer),
    "TEL,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%d,%d,%d,%d,%d",
    roll, pitch, yaw,
    rollRate, pitchRate, yawRate,
    throttle,
    armed ? 1 : 0,
    motorFL, motorFR, motorRL, motorRR
  );
  
  udp.beginPacket(clientIP, clientPort);
  udp.write((uint8_t*)buffer, strlen(buffer));
  udp.endPacket();
}

// FAILSAFE

void checkFailsafe() {
  if (armed && (millis() - lastCommandTime > 500)) {
    if (!failsafe) {
      failsafe = true;
      Serial.println("[FAILSAFE] Command timeout!");
    }
    armed = false;
    stopMotors();
  } else {
    failsafe = false;
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  
  
  initMPU6050();
  initDShot();
  initWiFi();
  
  stopMotors();
  delay(100);
  
  calibrateIMU();
  
  
  lastLoopTime = micros();
}

void loop() {
  // Fixed loop timing
  uint32_t now = micros();
  if (now - lastLoopTime < LOOP_PERIOD_US) {
    return;
  }
  
  dt = (now - lastLoopTime) / 1000000.0f;
  lastLoopTime = now;
  loopCount++;
  
  // 1. Read IMU (raw MPU data)
  readMPU6050();
  
  // 2. Transform to NED frame (X->front, Y->right, Z->down)
  //    MPU: X->left, Y->back, Z->up
  //    
  //    quand drone est levé:
  //    - MPU sees: accelX=0, accelY=0, accelZ=+1g (Z points up, feels gravity pulling down = +1g)
  //    - NED needs: accelX=0, accelY=0, accelZ=+1g (Z points down = +1g)
  //    
  //    NED_X (front)  = -MPU_Y (front = -back)
  //    NED_Y (right)  = -MPU_X (right = -left)
  //    NED_Z (down)   = +MPU_Z (down sees +g when Z_mpu points up and feels +g)
  float nedAccelX = -accelY;
  float nedAccelY = -accelX;
  float nedAccelZ = accelZ;   // les deux points "with gravity" = positive
  
  float nedGyroX = -gyroY;
  float nedGyroY = -gyroX;
  float nedGyroZ = -gyroZ;
  
  // 3. Update attitude avec NED data
  madgwickUpdate(nedGyroX, nedGyroY, nedGyroZ, nedAccelX, nedAccelY, nedAccelZ, dt);
  quaternionToEuler();
  
  // 4. stocke rates pour PID (en NED frame)
  rollRate = nedGyroX;
  pitchRate = nedGyroY;
  yawRate = nedGyroZ;
  
  // 5. Process commands
  processUDPCommands();
  
  // 6. Failsafe
  checkFailsafe();
  
  // 7. PID & Motor mix
  if (armed && !failsafe) {
    float rollOutput, pitchOutput;
    computeCascadedPID(rollOutput, pitchOutput);
    computeMotorMix(throttle, rollOutput, pitchOutput);
    sendMotorCommands();
  } else {
    stopMotors();
  }
  
  // 8. Telemetry en 50Hz
  if (loopCount % TELEMETRY_DIVIDER == 0) {
    sendTelemetry();
  }
}
