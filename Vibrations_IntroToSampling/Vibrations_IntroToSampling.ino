#include <i2c_t3.h> // the I2C library that replaces Wire.h for the Teensy 3.2
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <FlexCAN.h>
#include <EEPROM.h>

#define PS0Pin      2
#define resetPin    22
#define IMUIntPin   6
#define STPin       17
#define ledPin      13

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
elapsedMillis ledTimer;
elapsedMillis ledDuration;

boolean ledState = false;

Adafruit_BNO055 bno = Adafruit_BNO055();

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(ledPin, OUTPUT);
 
  pinMode(PS0Pin,OUTPUT);
  digitalWrite(PS0Pin,LOW);
  pinMode(resetPin,OUTPUT);
  digitalWrite(resetPin,LOW);
  pinMode(STPin,OUTPUT);
  digitalWrite(STPin,LOW);

  
  pinMode(IMUIntPin,INPUT);
  
  delay(20);
  digitalWrite(resetPin,HIGH);
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin(bno.OPERATION_MODE_ACCGYRO))  //Accelerometer and Gyro only mode
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  //Increase the limit on the Accelerometer to 16G
  Serial.println(bno.OPERATION_MODE_ACCGYRO);  //print the current operation mode (for debug)
  
  bno.write8(bno.BNO055_PAGE_ID_ADDR, 1); //Change to memory page 1
  Serial.println(bno.read8(bno.BNO055_PAGE_ID_ADDR)); //print the current memory page (for debug)
  
  bno.write8(bno.BNO055_ACCEL_DATA_X_LSB_ADDR, 0x1F); //Change the accel_config register (same as accel_data on page0)
    //to have a 16G limit
  Serial.println(bno.read8(bno.BNO055_ACCEL_DATA_X_LSB_ADDR));  //print the code for the current accel limit (for debug)

  Serial.println(bno.read8(bno.BNO055_PAGE_ID_ADDR)); //print the current memory page (for debug)
  
  bno.write8(bno.BNO055_PAGE_ID_ADDR, 0); //change back to memory page 0
  delay(1000*5);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  if ( ledTimer >= 50){
    ledTimer = 0;
    ledState = !ledState;
    digitalWrite(ledPin,ledState);

    //Write the time to the serial data
    Serial.print(millis()/1000.0);
    Serial.print("\t");

    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
    /* Display the floating linear acceleration data */
    Serial.print(acceleration.x());
    Serial.print("\t");
    Serial.print(acceleration.y());
    Serial.print("\t");
    Serial.print(acceleration.z());
    Serial.print("\t");
    
    imu::Vector<3> angularVelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      /* Display the floating point angular acceleration data */
    Serial.print(angularVelocity.x());
    Serial.print("\t");
    Serial.print(angularVelocity.y());
    Serial.print("\t");
    Serial.print(angularVelocity.z());
    Serial.print("\t");
  
    /*
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
    */
  
    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print(system, DEC);
    Serial.print("\t");
    Serial.print(gyro, DEC);
    Serial.print("\t");
    Serial.print(accel, DEC);
    Serial.print("\t");
    Serial.println(mag, DEC);
    Serial.print("\n");

  }
}
