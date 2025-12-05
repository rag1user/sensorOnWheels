#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ESP32Servo.h>

// -------------------- Pin Definitions --------------------

// L298N pins
const int ENA_PIN  = 25; // PWM for left motor
const int IN1_PIN  = 26;
const int IN2_PIN  = 27;

const int ENB_PIN  = 14; // PWM for right motor
const int IN3_PIN  = 32;
const int IN4_PIN  = 33;

// Servo pin
const int SERVO_PIN = 18;

// I2C pins for ESP32 (you can change if needed)
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// -------------------- Motor PWM Channels (ESP32) --------------------
const int MOTOR_A_PWM_CHANNEL = 0;
const int MOTOR_B_PWM_CHANNEL = 1;
const int MOTOR_PWM_FREQ      = 20000; // 20 kHz to keep motors quieter
const int MOTOR_PWM_RES       = 8;     // 0–255

// -------------------- Servo --------------------
Servo steeringServo;
// Center / range angles – tweak for your steering geometry
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 50;
const int SERVO_RIGHT  = 130;

// -------------------- BME680 --------------------
Adafruit_BME680 bme;
const uint8_t BME680_I2C_ADDR = 0x76;

// -------------------- Helper: Motor Control --------------------
// speed: -255..255  (negative = reverse, positive = forward)
void setMotor(int enaPin, int in1Pin, int in2Pin, int pwmChannel, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmChannel, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(pwmChannel, -speed);
  } else {
    // Brake (both LOW or both HIGH, choose your style)
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

// Convenience wrappers
void setDrive(int leftSpeed, int rightSpeed) {
  setMotor(ENA_PIN, IN1_PIN, IN2_PIN, MOTOR_A_PWM_CHANNEL, leftSpeed);
  setMotor(ENB_PIN, IN3_PIN, IN4_PIN, MOTOR_B_PWM_CHANNEL, rightSpeed);
}

void stopDrive() {
  setDrive(0, 0);
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESP32 Sensor-on-Wheels: L298N + BME680 + Servo");

  // I2C setup
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Motor pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // PWM channels for motors
  ledcSetup(MOTOR_A_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(ENA_PIN, MOTOR_A_PWM_CHANNEL);

  ledcSetup(MOTOR_B_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(ENB_PIN, MOTOR_B_PWM_CHANNEL);

  // Initialize motors (stopped)
  stopDrive();

  // Servo setup
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);

  // BME680 setup
  if (!bme.begin(BME680_I2C_ADDR)) {
    Serial.println("Could not find BME680 sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("BME680 found.");

  // Recommended BME680 config (you can tweak later)
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
}

// -------------------- Sensor Read Helper --------------------
void readAndPrintBME680() {
  if (!bme.performReading()) {
    Serial.println("BME680 reading failed");
    return;
  }

  Serial.print("Temp: ");
  Serial.print(bme.temperature);
  Serial.print(" °C,  Humidity: ");
  Serial.print(bme.humidity);
  Serial.print(" %,  Pressure: ");
  Serial.print(bme.pressure / 100.0);
  Serial.print(" hPa,  Gas: ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" kOhms
