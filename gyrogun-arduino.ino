/*
 * Copyright Notice
 * 
 * i2cdevlib MPU6050 examples - MIT Licensed by Jeff Rowberg
 */

#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define ESP_RX 4
#define ESP_TX 3
#define SWITCH 5
#define BUZZ_MOTOR 6
#define GYRO_INT 4

SoftwareSerial esp(ESP_TX, ESP_RX);
MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int switch_state = 0;
int count = 0;
int buzz_counter = 0;

void setup() {// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(GYRO_INT, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(GYRO_INT));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(GYRO_INT), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
  esp.begin(115200);

  pinMode(SWITCH, INPUT_PULLUP);
  pinMode(BUZZ_MOTOR, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
                // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
  }

  float yaw = ypr[0] * 180/M_PI, pitch = ypr[1] *180/M_PI, roll = ypr[2] * 180/M_PI;
  
  bool switch_was_clicked = false;
  
  int switch_val = digitalRead(SWITCH);
  if (switch_val == 0) {
    if(switch_state == 1) {
      switch_was_clicked = true;
      buzz_counter = 5;
    } 
    
    if(buzz_counter > 0) {
      digitalWrite(BUZZ_MOTOR, HIGH);
      buzz_counter--;
    } else {
      digitalWrite(BUZZ_MOTOR, LOW);
    }
  } else {
    digitalWrite(BUZZ_MOTOR, LOW);
  }
  switch_state = switch_val;

  uint8_t packet[16] = {};

  uint32_t message_type = 0;
  if (switch_was_clicked) message_type = 1;

  packet[0] = message_type >> 24 % 256;
  packet[1] = message_type >> 16 % 256;
  packet[2] = message_type >> 8 % 256;
  packet[3] = message_type >> 0 % 256;

  uint32_t yaw_uint = 0;
  memcpy(&yaw_uint, &yaw, sizeof(uint32_t));

  packet[4] = yaw_uint >> 24 % 256;
  packet[5] = yaw_uint >> 16 % 256;
  packet[6] = yaw_uint >> 8 % 256;
  packet[7] = yaw_uint >> 0 % 256;

  uint32_t pitch_uint = 0;
  memcpy(&pitch_uint, &pitch, sizeof(uint32_t));

  packet[8] = pitch_uint >> 24 % 256;
  packet[9] = pitch_uint >> 16 % 256;
  packet[10] = pitch_uint >> 8 % 256;
  packet[11] = pitch_uint >> 0 % 256;

  uint32_t roll_uint = 0;
  memcpy(&roll_uint, &roll, sizeof(uint32_t));

  packet[12] = roll_uint >> 24 % 256;
  packet[13] = roll_uint >> 16 % 256;
  packet[14] = roll_uint >> 8 % 256;
  packet[15] = roll_uint >> 0 % 256;

  bool esp_ok = false;
  while(esp.available() || !esp_ok) {
    if(!esp.available()) continue;
    char val = esp.read();
    if(val == 1) {
      
      Serial.println("ESP OK");
      esp_ok = true;
      digitalWrite(LED_BUILTIN, HIGH);
      break;      
    } else if (val == 0) {
      Serial.println("ESP failure");
      esp.flush();
      esp_ok = false;
      
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.print(val);
    }
  } 

  esp.write(packet, 16);
  delay(7);

  Serial.println(message_type);
}
