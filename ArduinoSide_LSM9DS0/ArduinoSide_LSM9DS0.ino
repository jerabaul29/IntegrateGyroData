#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

/*

   Reading the IMU and GYRO and MAG data from LSM9DS0 and send to computer as a
   struct through USB.

   Connection of LSM9DS0 as I2C.

*/

// Assign a unique base ID for the sensor
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

// event object
sensors_event_t accel, mag, gyro, temp;

// struct for transmitting the data to the computer through USB
struct IMU_data {
  // first acceleration
  float IMU_X;
  float IMU_Y;
  float IMU_Z;

  // then gyroscope
  float GYR_X;
  float GYR_Y;
  float GYR_Z;

  // then magnetometer
  float MAG_X;
  float MAG_Y;
  float MAG_Z;
};

// instanciate one struct
IMU_data IMU_data_holder;

// length of the structure
int len_struct = sizeof(IMU_data_holder);

// send the structure giving the IMU state through serial
void send_IMU_struct() {
  Serial.write('S');
  Serial.write((uint8_t *)&IMU_data_holder, len_struct);
  Serial.write('E');
  return;
}

// function to update the struct with the new data
void set_IMU_data(float IMU_X, float IMU_Y, float IMU_Z, float GYR_X, float GYR_Y, float GYR_Z, float MAG_X, float MAG_Y, float MAG_Z) {
  IMU_data_holder.IMU_X = IMU_X;
  IMU_data_holder.IMU_Y = IMU_Y;
  IMU_data_holder.IMU_Z = IMU_Z;

  IMU_data_holder.GYR_X = GYR_X;
  IMU_data_holder.GYR_Y = GYR_Y;
  IMU_data_holder.GYR_Z = GYR_Z;

  IMU_data_holder.MAG_X = MAG_X;
  IMU_data_holder.MAG_Y = MAG_Y;
  IMU_data_holder.MAG_Z = MAG_Z;
}

// configure the sensor: set range
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

// how often should sample
//unsigned long interval_sampling_micro = 10000L; // 100 HZ
unsigned long interval_sampling_micro = 20000L; // 50 HZ
//unsigned long interval_sampling_micro = 50000L; // 20 HZ
//unsigned long interval_sampling_micro = 100000L; // 10 HZ
//unsigned long interval_sampling_micro = 1000000L; // 1 HZ
unsigned long time_previous;
unsigned long time_current;

void setup() {

  delay(2000);

  Serial.begin(115200);

  Serial.print(F("size of struct: "));
  Serial.print(len_struct);
  Serial.println();

  Serial.println(F("sensor initialisation")); Serial.println("");

  /* Initialise the sensor */
  if (!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }
  Serial.println(F("found sensor"));

  /* Setup the sensor gain and integration time */
  Serial.println(F("configure sensor"));
  configureSensor();

  Serial.println(F("done with setup"));

  time_previous = micros();

}

void loop() {

  // check if time to do new measurement
  time_current = micros();
  if (time_current - time_previous >= interval_sampling_micro) {

    time_previous += interval_sampling_micro;

    // Get a new sensor event
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    // update the struct
    set_IMU_data(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                 gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                 mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    // transmit the struct
    send_IMU_struct();

  }

}
