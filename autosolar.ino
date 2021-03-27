#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define BIN_1 4   // x-axis motor pins
#define BIN_2 5
#define AIN_1 19  // z-axis motor pins
#define AIN_2 18

#define uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  5        // seconds

/* PWM properties */
const int freq = 5000;
const int motorChannel_1 = 1; // PWM channels
const int motorChannel_2 = 2;
const int motorChannel_3 = 3;
const int motorChannel_4 = 4;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 255;

/* Constants */
const int LDRbuffer = 10;       // adjust based on LDR input fluctuation
const int LDR_max_diff = 100;   // adjust based on LDR input fluctuation
const int int_buffer = 10;      // adjust based on INA input fluctuation

/* Intensity */
float intensity = 0;
float last_intensity = 0;

/* LDR input pins */
int ldrtop = 32;              // top LDR input pin
int ldrbtm = 33;              // bottom LDR input pin
int ldrlt = 34;               // left LDR input pin
int ldrrt = 39;               // right LDR input pin

/* LDR initial values */
int ldr_top = 0;
int ldr_btm = 0;
int ldr_lt = 0;
int ldr_rt = 0;

/* INA219 config */
float shuntvoltage = 0;
float current_mA = 0;

/* Adjustment indicators */
int UDdiff = 0;
int LRdiff = 0;
int UDadj = 0;
int LRadj = 0;

/* Define encoders */
ESP32Encoder encoderx;
ESP32Encoder encoderz;

/* Define INA219 */
Adafruit_INA219 ina219;


void gotosleep() {
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void automaticsolartracker() {      // main function

  // read LDR inputs
  ldr_top = analogRead(ldrtop);
  ldr_btm = analogRead(ldrbtm);
  ldr_lt = analogRead(ldrlt);
  ldr_rt = analogRead(ldrrt);

  // read INA current
  shuntvoltage = ina219.getShuntVoltage_mV();
  current_mA = ina219.getCurrent_mA();

  // calculation
  //intensity = shuntvoltage * current_mA;
  UDdiff = ldr_top - ldr_btm;
  LRdiff = ldr_lt - ldr_rt;

  // Skipping
  if (UDdiff > LDR_max_diff || LRdiff > LDR_max_diff) {     // ESP32 will go to sleep is U/D or L/R LDR input difference too large
    gotosleep();
  }

  // adjustment about x-axis
  if (abs(UDdiff) >= LDRbuffer) {
    if (UDdiff > 0) {                             // if upper LDR sensed more light then rotate panel upward
      ledcWrite(motorChannel_1, LOW);
      ledcWrite(motorChannel_2, MAX_PWM_VOLTAGE);
    }
    else if (UDdiff < 0) {                        // if lower LDR sensed more light then rotate panel downward
      ledcWrite(motorChannel_1, MAX_PWM_VOLTAGE);
      ledcWrite(motorChannel_2, LOW);
    }
  }
  else {
    UDadj = 0;      // if difference within threshold, then no adjustment is needed
  }

  // adjustment about z-axis
  if (abs(LRdiff) >= LDRbuffer) {
    if (LRdiff > 0) {                             // if left LDR sensed more light then rotate panel left
      ledcWrite(motorChannel_3, LOW);
      ledcWrite(motorChannel_4, MAX_PWM_VOLTAGE);
    }
    else if (LRdiff < 0) {                        // if right LDR sensed more light then rotate panel right
      ledcWrite(motorChannel_3, MAX_PWM_VOLTAGE);
      ledcWrite(motorChannel_4, LOW);
    }
  }
  else {
    LRadj = 0;      // if difference within threshold, then no adjustment is needed
  }

  // ESP32 go to sleep if no adjustment is needed
  if (UDadj == 0 && LRadj == 0) {
    gotosleep();
  }
}


void setup() {
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors

  encoderx.attachHalfQuad(12, 27); // Attache pins for use as encoderx pins
  encoderz.attachHalfQuad(26, 25); // Attache pins for use as encoderz pins

  encoderx.setCount(0);  // set starting count value after attaching
  encoderz.setCount(0);  // set starting count value after attaching

  /* configure LED PWM functionalitites */
  ledcSetup(motorChannel_1, freq, resolution);
  ledcSetup(motorChannel_2, freq, resolution);
  ledcSetup(motorChannel_3, freq, resolution);
  ledcSetup(motorChannel_4, freq, resolution);

  /* attach the channel to the GPIO to be controlled */
  ledcAttachPin(BIN_1, motorChannel_1);
  ledcAttachPin(BIN_2, motorChannel_2);
  ledcAttachPin(AIN_1, motorChannel_3);
  ledcAttachPin(AIN_2, motorChannel_4);
}


void loop() {
  automaticsolartracker();
  delay(50); // Wait for 50 millisecond(s)
}
