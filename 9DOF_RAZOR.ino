#include "ITG3200.h"
#include "Adafruit_ADXL345_U.h"
#include "Adafruit_HMC5883_U.h"

// Include Library -> Manage Libraries:
// Uses: https://github.com/adafruit/Adafruit_Sensor
// Uses: https://github.com/adafruit/Adafruit_ADXL345
// Uses: https://github.com/adafruit/Adafruit_HMC5883_Unified
// Uses: https://github.com/Seeed-Studio/Grove_3_Axis_Digital_Gyro

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12340);
ITG3200 gyro;

void displaySensorDetails(const sensor_t &sensor)
{
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value);
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value);
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution);
  Serial.println("------------------------------------");
  Serial.println("");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello World");

  if(!mag.begin()) {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  if(!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  // Configure magnetometer in continuous mode to read data 75hz
  // DO0=0, DO1=1, DO2=1
  mag.write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRA_REG_M, (4+2)<<2);

  gyro.init();

  accel.setRange(ADXL345_RANGE_16_G);
  Serial.print("Accel Data Rate: "); 
  Serial.print(accel.getDataRate());
  Serial.println("");

  Serial.print("Accel Range: "); 
  Serial.print(accel.getRange());
  Serial.println("");

  sensor_t sensor;
  mag.getSensor(&sensor);
  displaySensorDetails(sensor);
  
  accel.getSensor(&sensor);
  displaySensorDetails(sensor);
}

void readMag() {
  sensors_event_t magEvent; 
  mag.getEvent(&magEvent);
  Serial.print("MAG,");
  Serial.print(magEvent.magnetic.x); Serial.print(",");
  Serial.print(magEvent.magnetic.y); Serial.print(",");
  Serial.print(magEvent.magnetic.z); Serial.print(",");
}

void readAccel() {
  sensors_event_t accelEvent; 
  accel.getEvent(&accelEvent);
  Serial.print("ACCEL,");
  Serial.print(accelEvent.acceleration.x); Serial.print(",");
  Serial.print(accelEvent.acceleration.y); Serial.print(",");
  Serial.print(accelEvent.acceleration.z); Serial.print(",");
}

void readGyro() {
  double gyroTemp = gyro.getTemperature();
  Serial.print("TEMP,");
  Serial.print(gyroTemp);
  Serial.print(",");

  int16_t gyroX, gyroY, gyroZ;
  gyro.getXYZ(&gyroX,&gyroY,&gyroZ);

  Serial.print("GYRO,");
  Serial.print(gyroX); Serial.print(",");
  Serial.print(gyroY); Serial.print(",");
  Serial.print(gyroZ); Serial.print(",");
}

// how often to read from in continuous mode?
#define READ_EVERY_MS (1000 / 75) // 75HZ

static int lastMillis = 0;

void loop() {
  int currentMillis = millis();
  if (currentMillis - lastMillis < READ_EVERY_MS) {
    return;
  }
  lastMillis = currentMillis;

  Serial.print("$,");
  Serial.print("TIME,");
  Serial.print(millis());
  Serial.print(",");

  readMag();
  //readAccel();
  //readGyro();

  Serial.println();
}

