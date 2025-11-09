#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

/*
  ml_digits.ino
  ----------------
  Simple IMU logger for recording "digit" writing sessions from an
  Adafruit LSM6DSOX sensor. Outputs CSV rows on Serial in the format:
    t_ms,ax,ay,az,gx,gy,gz,state,label

  Usage:
   - Upload to an ESP32 (adjust SDA_PIN/SCL_PIN if needed)
   - Use the accompanying Python `write.py` script to collect labeled
     writing segments via serial and save them under `imu_digits/`.

  Notes:
   - No behavioural changes made; only added header/documentation.
*/

// ===== Config =====
#define SDA_PIN 21
#define SCL_PIN 22
const uint16_t FS_HZ = 200;                    // sample rate
const uint32_t SAMPLE_US = 1000000UL / FS_HZ;
const float THR_AX=1.2f, THR_AY=1.2f, THR_AZ=1.2f;    // accel thresholds (m/s^2)
const float THR_GX=0.35f, THR_GY=0.35f, THR_GZ=0.35f; // gyro thresholds (rad/s)
const float D_ON  = 1.4f;                      // activity to enter WRITING
const float D_OFF = 0.7f;                      // activity to return REST
const uint32_t T_ON_MS  = 80;                  // debounce enter
const uint32_t T_OFF_MS = 250;                 // debounce exit
const uint32_t CAL_MS   = 1500;                // baseline calibration

// ===== Globals =====
Adafruit_LSM6DSOX imu;
uint32_t next_us=0, t0_ms=0;
bool baseline_done=false;
uint32_t cal_end=0; uint32_t ncal=0;
float ax0=0, ay0=0, az0=0, gx0=0, gy0=0, gz0=0;

enum State {REST=0, WRITING=1};
State st=REST;
uint32_t st_since=0;
float Df=0.0f; const float D_BETA=0.2f; // EMA smoothing
char current_label='?';                 // set via Serial by Python

void setup(){
  Serial.begin(115200);
  // Give USB a moment to come up; do NOT wait for Serial monitor
  delay(600);

  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000);
  if(!imu.begin_I2C(0x6A,&Wire) && !imu.begin_I2C(0x6B,&Wire)){
    // Print once; Python ignores lines starting with '#'
    Serial.println("# ERR: LSM6DSOX not found."); while(1){delay(1000); }
  }
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);   // ~200 Hz
  imu.setGyroDataRate(LSM6DS_RATE_208_HZ);

  t0_ms = millis();
  cal_end = t0_ms + CAL_MS;
  next_us = micros();

  Serial.println("# imu_digit_logger_v1");
  Serial.println("# columns: t_ms,ax,ay,az,gx,gy,gz,state,label");
  Serial.println("# send a digit 0-9 over serial to set the label");
}

void handle_serial_label(){
  while(Serial.available()){
    int c = Serial.read();
    if (c >= '0' && c <= '9'){
      current_label = (char)c;
      Serial.print("# label set to "); Serial.println(current_label);
    }
  }
}

void loop(){
  handle_serial_label();

  uint32_t now_us = micros();
  if ((int32_t)(now_us - next_us) < 0) return;
  // do { next_us += SAMPLE_US; } while ((int32_t)(now_us - next_us) >= 0);
  next_us += SAMPLE_US;

  sensors_event_t a,g,t; imu.getEvent(&a,&g,&t);
  uint32_t now_ms = millis();

  // Calibration of baseline means
  if(!baseline_done){
    ax0 = (ax0*ncal + a.acceleration.x)/(ncal+1);
    ay0 = (ay0*ncal + a.acceleration.y)/(ncal+1);
    az0 = (az0*ncal + a.acceleration.z)/(ncal+1);
    gx0 = (gx0*ncal + g.gyro.x)/(ncal+1);
    gy0 = (gy0*ncal + g.gyro.y)/(ncal+1);
    gz0 = (gz0*ncal + g.gyro.z)/(ncal+1);
    ncal++;
    if ((int32_t)(now_ms - cal_end) >= 0) baseline_done=true;
    return;
  }

  // Activity metric (max normalized delta) + EMA smoothing
  float dax=fabsf(a.acceleration.x-ax0)/THR_AX;
  float day=fabsf(a.acceleration.y-ay0)/THR_AY;
  float daz=fabsf(a.acceleration.z-az0)/THR_AZ;
  float dgx=fabsf(g.gyro.x-gx0)/THR_GX;
  float dgy=fabsf(g.gyro.y-gy0)/THR_GY;
  float dgz=fabsf(g.gyro.z-gz0)/THR_GZ;
  float D = max(max(dax,day),daz);
  D = max(D, max(max(dgx,dgy),dgz));
  Df = (1.0f - D_BETA)*Df + D_BETA*D;

  // FSM with hysteresis + debouncing
  if (st==REST){
    if (Df >= D_ON){
      if (st_since==0) st_since = now_ms;
      if (now_ms - st_since >= T_ON_MS){
        st = WRITING;
        st_since = now_ms;
        Serial.printf("# START,%c,%lu\n", current_label, (unsigned long)now_ms);
      }
    } else st_since = 0;
  } else { // WRITING
    if (Df <= D_OFF){
      if (st_since==0) st_since = now_ms;
      if (now_ms - st_since >= T_OFF_MS){
        st = REST;
        st_since = now_ms;
        Serial.printf("# STOP,%c,%lu\n", current_label, (unsigned long)now_ms);
      }
    } else st_since = 0;
  }

  // Stream CSV every sample (both REST and WRITING; we’ll segment in Python)
  Serial.printf("%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%c\n",
    (unsigned long)(now_ms - t0_ms),
    a.acceleration.x, a.acceleration.y, a.acceleration.z,
    g.gyro.x, g.gyro.y, g.gyro.z,
    (int)st, current_label
  );
}
