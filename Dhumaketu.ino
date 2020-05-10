/*
   Description: Model Rocket motor gimbal for long burning motors using 2 servo.
                Uses Arduino PID controler.
               
   Authors: Aishwarya K, Kavya Patil, Prajwal Nayak, Prajwal P
   Date: April 2020
   
   Microcontroller: Teensy 3.2
   IMU: MPU6050
   Servo: 9g Servo
   Barometer: BMP280
   
   Servo Connection    MPU board Connection
   X - 8               SCL - A5 
   Y - 9               SDA - A4                

*/

#include "Arduino.h"
#include <Wire.h>
#include <Servo.h> //servo library
#include <I2Cdev.h>
#include <SPI.h>
#include <SimpleKalmanFilter.h>
#include <SD.h>
#include <MPU6050_6Axis_MotionApps20.h> // Gyroscope and axcelerometer libraries
#include <helper_3dmath.h>
#include <PID_v1.h> // Arduino PID library
#include <Adafruit_BMP280.h>

#define LED_Pin 13
bool blinkState = true;
#define redPin 17
#define greenPin 16
#define bluePin 15
#define Eject_Pin 6
#define SOUND
#define Speaker_Pin 9

#define ON_RAMP 0 // constants for states of state machine
#define IN_FLIGHT 1
#define ABORT_FLIGHT 2
#define RECOVERY 3
int currentState = 0;
// state variables
float h = 0.0, h_max = 0.0, v = 0.0, a = 0.0;

Servo ServoX;   // X axis Servo
Servo ServoY;   // Y axis Servo
int servoAngle = 0;   // servo position in degrees
int startangle = 60;
int endangle = 120;
int centerangleX = 90;
int centerangleY = 90;

// =============================================
// ===              BAROMETER                ===
// =============================================
Adafruit_BMP280 bmp;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
const int sealevel = 1012; //current sea pressure
static float alt;
float est_alt;
float pascal;

// =============================================
// ===              MPU Variables                ===
// =============================================
MPU6050 mpu; // Create  MPU object
SimpleKalmanFilter rollKalmanFilter(1,1,0.01); //reduces noise in gyro readings
SimpleKalmanFilter pitchKalmanFilter(1,1,0.01);
SimpleKalmanFilter yawKalmanFilter(1,1,0.01);
float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;
float rollNew, pitchNew, yawNew; //declare variable that will hold new current gyro values

// MPU control/status vars
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// relative ypr[x] usage based on sensor orientation when mounted, e.g. ypr[PITCH]
#define PITCH   1     // defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  2     // defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted
#define YAW   0     // defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted

// PID stuff
//Define Variables we'll be connecting to
double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;

// These initial tuning parameters need to be tuned
double KpX = 3.55, KiX = 0.005, KdX = 2.05;
double KpY = 3.55, KiY = 0.005, KdY = 2.05;
PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

//calibration stuff
//Change those 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 200;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// =============================================
// ===          MISC Global Vars             ===
// =============================================
//Timers Vars
unsigned long previousMillis = 0;
unsigned long landprev = 0;
unsigned long nextSampleTime; //time when next sample has to be taken
const long SERIAL_REFRESH_TIME = 100;
bool launch = false;
bool pyro = false;
bool landed = false;

//Flight Variables
long launchacc = 2000; //Acceleration threshold to detect launch
long flightStart;
const int pitchabort = 45; //Pitch abort angle
const int timetoapogee = 15000; //Time required to reach apogee to fire Ejection_Pin
float initialAltitude;
float currAlti;
float maxAltitude = 0;
float minAltitude = 15;
int lcc = 0; // land check coutner

// =============================================
// ===              SD CARD                  ===
// =============================================
/*
The circuit:
    SD card attached to SPI bus as follows:
 ** MOSI - pin 11 
 ** MISO - pin 12 
 ** CLK - pin 14 
*/
int CS = 10; //Chipselect on pin 10
char filename[16]; 
File myFile; 
int sd_count = 0;
bool FL = false;
bool fileclosed = false;
/*
   Initial setup
   do the board calibration
   if you use the calibration function do not move the board until the calibration is complete
*/
void setup()
{
  Wire.begin();
  Serial.begin(38400); //baud rate can be changed (38400 is used for Bluetooth)
  while (!Serial);
  Serial.println("Dhumaketu-Mk2 firmware 05-04-2020");
  
  // configure EjectPin, LEDs and Speaker for output
  pinMode(Eject_Pin, OUTPUT);
  pinMode(Eject_Pin, LOW);
  pinMode(LED_Pin, OUTPUT);
  digitalWrite(LED_Pin, HIGH);
  pinMode(redPin, OUTPUT);
  digitalWrite(redPin, HIGH);
  pinMode(greenPin, OUTPUT);
  digitalWrite(greenPin, LOW);
  pinMode(bluePin, OUTPUT);
  digitalWrite(bluePin, LOW);
  pinMode(Speaker_Pin, OUTPUT);
  digitalWrite(Speaker_Pin, LOW);

  startuptone();
  
  ServoX.attach(4);  // attaches the X servo on Digital pin 10
  ServoY.attach(3);  // attaches the Y servo on Digital pin 11
  // set both servo's to 90 degree
  ServoX.write(centerangleX);
  ServoY.write(centerangleY);
  delay(1000);

  //PASS 1: Initialize and Calibrate MPU
  // INPUT CALIBRATED OFFSETS HERE; SPECIFIC FOR EACH UNIT AND EACH MOUNTING CONFIGURATION!!!!
  ax_offset = -1178;
  ay_offset = 809;
  az_offset = 1331;
  gx_offset = 60;
  gy_offset = 0;
  gz_offset = 13;
  calibrate();
  initialize();

  //PASS 2: Initialize Baromter
    initializeBMP();
        
  //PASS 3: Initialize SD Module
    SPI.setSCK(14); //set SCK pin to 14 (13 is default)
    //initializeSD();    

  //PASS 4: Pyro Check
    if (Eject_Pin == HIGH) {
    RED();
    while (1);
    }

  //PASS 5: TVC Check
    servo_sweep(); //check if the tvc can move freely
  //servo_fastsweep();

  Serial.print("MEASURE INITIAL ALTITUDE: ");
  initialAltitude = 0;
  for (int i = 0; i < 10; i++) {
    get_alt();
    initialAltitude += est_alt;
  }
  initialAltitude = initialAltitude / 10;
  Serial.print(initialAltitude);
  Serial.println("m");
  
  digitalWrite(redPin, LOW);
}

void initialize() 
{ 
  Serial.println(F("Initializing MPU 6050 device..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
  //turn the PID on
  //servo's can go from 60 degree's to 120 degree so set the angle correction to + - 30 degrees max
  myPIDX.SetOutputLimits(-30, 30);
  myPIDY.SetOutputLimits(-30, 30);
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
  SetpointX = 0;
  SetpointY = 0;
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed, 2 = DMP configuration updates failed (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed code = "));
    Serial.println(devStatus);
  }
}

// ================================================================
// ===                         BAROMETER                       ===
// ================================================================

void initializeBMP() {

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
    }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println(F("BMP Initialized"));
}

// ================================================================
// ===               SD CARD Begin                       ===
// ================================================================

void initializeSD() {

  Serial.print("Initializing SD card...");
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    Serial.println("Card failed, or not present!");
    RED();
    while (1);
  }
  
  else {
  Serial.println("Card Initialized.");
  }

  //Create a file with new name
  if (!loadSDFile()) {
    Serial.println(F("F-0"));
    while (1);
    RED();
  }
  else {
    Serial.println(F("F-1"));
  }

  Serial.println(filename);

  File myFile = SD.open(filename,FILE_WRITE);
  Serial.println(myFile);
  if (myFile) {
    //Print Header Files  - - alt, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY

    myFile.print("t");
    myFile.print(",");
    myFile.print("alt");
    myFile.print(",");
    myFile.print("M-alt");
    myFile.print(",");
    myFile.print("ax");
    myFile.print(",");
    myFile.print("ay");
    myFile.print(",");
    myFile.print("az");
    myFile.print(",");
    myFile.print("gx");
    myFile.print(",");
    myFile.print("gy");
    myFile.print(",");
    myFile.print("gz");
    myFile.print(",");
    myFile.print("yaw");
    myFile.print(",");
    myFile.print("roll");
    myFile.print(",");
    myFile.print("ServoX");
    myFile.print(",");
    myFile.print("ServoY");
    myFile.print(",");
    myFile.print("Launch");
    myFile.print(",");
    myFile.print("Apogee");
    myFile.print(",");
    myFile.println("Land");

    myFile.close();
    Serial.println(F("Fin-1"));
  }
  else {
    Serial.print(F("Fin-0"));
    RED();
    while (1);
  }
}

/*
   MAIN PROGRAM LOOP
*/
void loop()
{   
  mpu.resetFIFO();    
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ( fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  // check for correct available data length
  if (fifoCount < packetSize)
    return;

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  fifoCount -= packetSize;

  // flush buffer to prevent overflow
  mpu.resetFIFO();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.getAcceleration(&ax,&ay,&az);

  mpuPitch = ypr[PITCH] * 180 / PI;
  mpuRoll = ypr[ROLL] * 180 / PI;
  mpuYaw  = ypr[YAW] * 180 / PI;
  
  rollNew = rollKalmanFilter.updateEstimate(mpuRoll);    //average gyro readings and then pass into kalman filter
  pitchNew = pitchKalmanFilter.updateEstimate(mpuPitch);   
  yawNew = yawKalmanFilter.updateEstimate(mpuYaw);

  /*
    * Detect Launch
    * Enable ABORT - for extreame tilt.
    * Detect Apogee
    * Fire Pyros
    * Detect Land
  */
      switch(currentState) {
      case ON_RAMP: on_ramp(); break;
      case IN_FLIGHT: in_flight(); break;
      case ABORT_FLIGHT: abort_flight(); break;
      case RECOVERY: recovery(); break;
      }   

  //Flight Logs
  if (fileclosed == false) {
    //Serial.println("writing");
    Write();
    sd_count++;

    if (sd_count > 100) {
      myFile.flush();
      sd_count = 0;
      //Serial.println("FLUSH");
    }
  }  
 /* if (launch == true && landed == true && (currentMillis - previousMillis > 2000)) {
    currentMillis = millis();
    previousMillis = currentMillis;
    fileclosed = true;
    myFile.close();
    //Serial.println("FClose");
  }*/
}//voidloop end

void on_ramp() {  
  
    mpu.getAcceleration(&ax,&ay,&az);
    GREEN();
    Serial.println(ay);
    
    //Detect if launch   
    if(ay > 2000) { 
      launch = true;
      flightStart = millis();
      digitalWrite(LED_Pin, LOW);
      Serial.println("Launch detected!");
      digitalWrite(greenPin, LOW);
      tone(Speaker_Pin, 2500, 1000);
      currentState = IN_FLIGHT;
    }
}

void in_flight() {
  
  digitalWrite(bluePin, HIGH);
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_Pin, blinkState);

  // flush buffer to prevent overflow
  mpu.resetFIFO();
  InputX = yawNew;
  myPIDX.Compute();
  InputY = rollNew;
  myPIDY.Compute();
  
  float q1[4];
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  q1[0] = q.w;
  q1[1] = q.x;
  q1[2] = q.y;
  q1[3] = q.z;
  SerialPrintFloatArr(q1, 4);
  
  //if using PID do those
  ServoX.write(-OutputX + 90);
  ServoY.write(OutputY + 90);

  //Serial.println(millis());
  // if you do not want to use the PID
  // ServoX.write(-mpuPitch + 90);
  // ServoY.write(mpuRoll + 90);
  Serial.print(yawNew);
  Serial.print("    ");
  Serial.print(rollNew);
  Serial.print("    ");
  Serial.print(OutputX);
  Serial.print("    ");
  Serial.println(OutputY);

  //Measure Altitude and detect Apogee
  if (millis() > nextSampleTime) {
      get_alt();
      nextSampleTime = millis()+ SERIAL_REFRESH_TIME;
      currAlti = est_alt - initialAltitude;
      
      //detect apogee
      if (currAlti > maxAltitude) {
        maxAltitude = currAlti;           //save highest altitude
        }
  }
  Serial.println(currAlti);
  
  //ABORT when pitched over 45 deg
  if ((mpuRoll > pitchabort || mpuYaw > pitchabort || mpuRoll < -pitchabort || mpuYaw < -pitchabort) && (currAlti < 5)) {
        currentState = ABORT_FLIGHT;
        
  }
  
  //RECOVERY PROGRAM 1-3
  //1
  if((millis() - flightStart) > timetoapogee) {
    currentState = RECOVERY;
    digitalWrite(bluePin, LOW);
    digitalWrite(Eject_Pin, HIGH); //FIRE!!
    pyro = true;
    delay(3000);
    Serial.print("Maximum altitude: ");
    Serial.println(maxAltitude);
   }   
   
  //2 
  /*if (maxAltitude > minAltitude && currAlti < (maxAltitude - 3.0)) { //if height is lower than maximum height by 3m
        currentState = RECOVERY;
        digitalWrite(bluePin, LOW);
        digitalWrite(Eject_Pin, HIGH); //FIRE!!
        pyro = true;
        delay(3000);
          Serial.print("Maximum altitude: ");
  Serial.println(maxAltitude);
        }   
   }*/

  //3
  // if launch is detected and altitude is at least 15 m, 
  // but velocity is negative, assume apogee 
   /* if(currAlti >= minAltitude && v<0.0) {
       digitalWrite(bluePin, LOW);
       digitalWrite(Eject_Pin, HIGH); //FIRE!!
       pyro = true;
       tone(Speaker_Pin, 2500, 200);
       currentState = RECOVERY;
       delay(3000);
         Serial.print("Maximum altitude: ");
  Serial.println(maxAltitude);
         }*/
}

void abort_flight() {
    RED();
}

void recovery() {
  
  // Eject parachute when reaching this state
  Serial.println("Parachute ejected!");
  Serial.print("Current altitude: ");
  Serial.println(currAlti);
  Serial.print("Current vertical velocity: ");
  Serial.println(v);
  
  digitalWrite(Eject_Pin, LOW);
  digitalWrite(LED_Pin, HIGH);
  tone(Speaker_Pin, 2500, 1000);
  
  purpleled();
  // return servos to central position
  ServoX.write(90);
  ServoY.write(90);
  
  //CHECK if it has landed.
  //PASS 1
  if(ax > -600 && ax < 600 && ay > -600 && ay < 600 && millis() - landprev > 1000) {  
    lcc++; // land check coutner
    Serial.println(lcc);
    //PASS 2
    if (ax > -600 && ax < 600 && ay > -600 && ay < 600 && lcc == 5) {
      
      Serial.println("LANDED!");
      landed = true;
      LAND_SIG();
      while(1);
    }
    landprev = millis();
  }  
}

void get_alt() {
  alt = bmp.readAltitude(sealevel);
  pascal = bmp.readPressure();
  est_alt = pressureKalmanFilter.updateEstimate(alt);
}

// ================================================================
// === These 2 functions will format the data                   ===
// ================================================================
void SerialPrintFloatArr(float * arr, int length) {
  for (int i = 0; i < length; i++) {
    SerialFloatPrint(arr[i]);
   // Serial.print(",");
  }
}

void SerialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for (int i = 0; i < 4; i++) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    //Serial.print(c1);
    //Serial.print(c2);
  }
}

/*
   calibration routines those will be executed each time the board is powered up.
   we might want to calibrate it for good on a flat table and save it to the microcontroler eeprom
*/
void calibrate() {
  
  // start message
  Serial.println("\nMPU6050 Calibration Sketch");
  delay(1000);
  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(1000);
  // verify connection
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
  // reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  while (1) {
    if (state == 0) {
      Serial.println("\nReading sensors for first time...");
      meansensors();
      state++;
      delay(100);
    }

    if (state == 1) {
      Serial.println("\nCalculating offsets...");
      calibration();
      state++;
      delay(100);
    }

    if (state == 2) {
      meansensors();
      Serial.print("\nSensor readings with offsets:\t");
      Serial.print(mean_ax);
      Serial.print("\t");
      Serial.print(mean_ay);
      Serial.print("\t");
      Serial.print(mean_az);
      Serial.print("\t");
      Serial.print(mean_gx);
      Serial.print("\t");
      Serial.print(mean_gy);
      Serial.print("\t");
      Serial.println(mean_gz);
      Serial.print("Your offsets:\t");
      Serial.print(ax_offset);
      Serial.print("\t");
      Serial.print(ay_offset);
      Serial.print("\t");
      Serial.print(az_offset);
      Serial.print("\t");
      Serial.print(gx_offset);
      Serial.print("\t");
      Serial.print(gy_offset);
      Serial.print("\t");
      Serial.println(gz_offset);
      Serial.println("\nData is printed as: acelX acelY acelZ gyroX gyroY gyroZ");
      Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
      Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
      Serial.println("Gimbal in use");
      break;
    }
  }
}

void meansensors() {
  
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration() {
  
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");
    blinkState = !blinkState;
    digitalWrite(LED_Pin, blinkState);
    tone(Speaker_Pin, 440, 1000);
    delay(500);
    noTone(Speaker_Pin);
    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}

// ================================================================
// ===                  MISC FUNCTIONS                          ===
// ================================================================

void RED() {

  digitalWrite(greenPin, HIGH);
  digitalWrite(redPin, LOW);
  #ifdef SOUND
  tone(Speaker_Pin, 2500, 100);
  #endif
  delay(200);
  digitalWrite(redPin, HIGH);

  #ifdef SOUND 
  tone(Speaker_Pin, 2500, 100);
  #endif
  delay(200);
  digitalWrite(redPin, LOW);
 
  #ifdef SOUND 
  tone(Speaker_Pin, 2000, 100);
  #endif
  delay(500);
  #ifdef SOUND 
  tone(Speaker_Pin, 2000, 100);
  #endif
}

void GREEN() {
    //Everything is fine.. signal.
  unsigned long interval = 1000;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    digitalWrite(greenPin, LOW);
    #ifdef SOUND
    tone(Speaker_Pin, 2500, 100);
    #endif
  }
  else {
    digitalWrite(greenPin, HIGH);
  }
}

void LAND_SIG() {
  
  //Landed 
  for(int i=0;i<10;i++){
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, HIGH);
  #ifdef SOUND 
  tone(Speaker_Pin, 2500, 300);
  #endif
  delay(100);
  digitalWrite(bluePin, LOW);
  digitalWrite(greenPin, HIGH);
  #ifdef SOUND 
  tone(Speaker_Pin, 2000, 300);
  #endif
  delay(100);

  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, HIGH);
  #ifdef SOUND 
  tone(Speaker_Pin, 2500, 300);
  #endif
  delay(500);
  digitalWrite(bluePin, LOW);
  digitalWrite(greenPin, HIGH);
  #ifdef SOUND 
  tone(Speaker_Pin, 2000, 300);
  #endif
  delay(500); 
  }
}

void servo_fastsweep() {
  
  //X *******************************************
  for (servoAngle = startangle; servoAngle < endangle; servoAngle += 2) //move the micro servo from 0 degrees to 180 degrees
  {
    ServoX.write(servoAngle);
    delay(10);
  }
  delay(100);
  for (servoAngle = endangle; servoAngle > startangle; servoAngle -= 2) //now move back the micro servo from 0 degrees to 180 degrees
  {
    ServoX.write(servoAngle);
    delay(10);
  }
  delay(100);
  ServoX.write(centerangleX);
  delay(200);

  // for Y
  //****************************************
  for (servoAngle = startangle; servoAngle < endangle; servoAngle += 2) //move the micro servo from 0 degrees to 180 degrees
  {
    ServoY.write(servoAngle);
    delay(10);
  }
  delay(100);
  for (servoAngle = endangle; servoAngle > startangle; servoAngle -= 2) //now move back the micro servo from 0 degrees to 180 degrees
  {
    ServoY.write(servoAngle);
    delay(10);
  }
  delay(100);
  
  ServoX.write(centerangleX);
  delay(200);
}

void servo_sweep() {
    
  //X *******************************************   
  for (servoAngle = startangle; servoAngle < endangle; servoAngle++) //move the micro servo from 0 degrees to 180 degrees
  {
    ServoX.write(servoAngle);
    delay(10);
  }
  delay(100);
  for (servoAngle = endangle; servoAngle > startangle; servoAngle--) //now move back the micro servo from 0 degrees to 180 degrees
  {
    ServoX.write(servoAngle);
    delay(10);
   }
  delay(1000);
  ServoX.write(centerangleX);

  // for Y
  //****************************************
  for (servoAngle = startangle; servoAngle < endangle; servoAngle++) //move the micro servo from 0 degrees to 180 degrees
  {
    ServoY.write(servoAngle);
    delay(10);
  }
  delay(100);
  for (servoAngle = endangle; servoAngle > startangle; servoAngle--) //now move back the micro servo from 0 degrees to 180 degrees
  {
    ServoY.write(servoAngle);
    delay(10);
  }
  delay(1000);
   
  ServoX.write(centerangleX);
  ServoY.write(centerangleY);      
}

void startuptone(){  
  #ifdef SOUND
  //Startup Sound
  tone(Speaker_Pin, 2500, 300);
  delay(1000);
  tone(Speaker_Pin, 2000, 300);
  delay(500);
  tone(Speaker_Pin, 3000, 300);
  tone(Speaker_Pin, 3500, 300);
  delay(500);
  tone(Speaker_Pin, 2500, 300);
  delay(1000);
  delay(2000);
  #endif
}

void purpleled() {
 
  unsigned long interval = 3000;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    setColor(80, 0, 80);  // purple
  }  
}

void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

// ==========================================
// ===           SD CARD WRITE            ===
// ==========================================
//Create a new filename everytime.
boolean loadSDFile() {
  
  boolean file = false;
   int n = 0;
  snprintf(filename, sizeof(filename), "data%03d.txt", n); // includes a three-digit sequence number in the file name
  while(SD.exists(filename)) {
    n++;
    snprintf(filename, sizeof(filename), "data%03d.txt", n);
    file = true;
  }
  return file;
}

void Write() {
  myFile = SD.open(filename, FILE_WRITE);

  if (myFile) {

    //Writing in SD Card!
    myFile.print(millis());
    myFile.print(",");
    myFile.print(currAlti);
    myFile.print(",");
    myFile.print(maxAltitude);
    myFile.print(",");
    myFile.print(ax);
    myFile.print(",");
    myFile.print(ay);
    myFile.print(",");
    myFile.print(az);
    myFile.print(",");
    myFile.print(gx);
    myFile.print(",");
    myFile.print(gy);
    myFile.print(",");
    myFile.print(gz);
    myFile.print(",");
    myFile.print(InputX);
    myFile.print(",");
    myFile.print(InputY);
    myFile.print(",");
    myFile.print(-OutputX + 90);
    myFile.print(",");
    myFile.print(OutputY + 90);
    myFile.print(",");
    myFile.print(launch);
    myFile.print(",");
    myFile.print(pyro);
    myFile.print(",");
    myFile.println(landed);
 
    myFile.close();
  } 
}
