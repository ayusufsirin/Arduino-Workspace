#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Servo.h"
#include "PID_v1.h"
#include "sbus.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


Servo servo_r;
Servo servo_p;
Servo servo_y;

#include "Parameters.h"

double Setpoint_r, Input_r, Output_r;
double Setpoint_p, Input_p, Output_p;

PID PID_r(&Input_r, &Output_r, &Setpoint_r, param_pid_roll_kp, param_pid_roll_ki, param_pid_roll_kd, DIRECT);
PID PID_p(&Input_p, &Output_p, &Setpoint_p, param_pid_pitch_kp, param_pid_pitch_ki, param_pid_pitch_kd, DIRECT);


class Plane {
  public:
    int angle_r;
    int angle_p;
    int angle_y;

    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
};

Plane plane;

bfs::SbusRx sbus_rx(&Serial);

enum ModeEnum { Mode_Manual, Mode_FBWA };
ModeEnum mode = Mode_FBWA;


void setup() {
  setupMPU ();

  servo_r.attach(9);
  servo_p.attach(5);
  servo_y.attach(6);

  Setpoint_r = 0;
  Setpoint_p = 0;
  PID_r.SetOutputLimits(-90, 90);
  PID_p.SetOutputLimits(-90, 90);
  PID_r.SetMode(AUTOMATIC);
  PID_p.SetMode(AUTOMATIC);

  sbus_rx.Begin();
}


void loop() {
  if (sbus_rx.Read()) {
   
    Setpoint_r = map(sbus_rx.ch(0), 600, 2200, -90, 90);
    Setpoint_p = map(sbus_rx.ch(1), 600, 2200, -90, 90);
    
    /* Display lost frames and failsafe data */
//    Serial.print(sbus_rx.lost_frame());
//    Serial.print("\t");
//    Serial.println(sbus_rx.failsafe());
  }

  if (mode == Mode_Manual) Manual();
  else if (mode == Mode_FBWA) FBWA();

  servo_r.write(plane.angle_r);
  servo_p.write(plane.angle_p);
  servo_y.write(plane.angle_y);
}

void Manual() {
  plane.angle_r = map(sbus_rx.ch(0), 600, 2200, 30, 150);
  plane.angle_p = map(sbus_rx.ch(1), 600, 2200, 30, 150);
  plane.angle_y = map(sbus_rx.ch(3), 600, 2200, 30, 150);
}

void FBWA() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(plane.ypr, &q, &gravity);
//    Serial.print("ypr\t");
//    Serial.print(ypr[0] * 180 / M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[1] * 180 / M_PI);
//    Serial.print("\t");
//    Serial.println(ypr[2] * 180 / M_PI);

    Input_r = plane.ypr[1] * 180 / M_PI;
    Input_p = plane.ypr[2] * 180 / M_PI;
    PID_r.Compute();
    PID_p.Compute();

    plane.angle_r = map(Output_r, -90, 90, 30, 150);
    plane.angle_p = map(Output_p, -90, 90, 30, 150);
    plane.angle_y = map(Output_r, -90, 90, 30, 150);     

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void setupMPU () {
  // join I2C bus (I2Cdev library doesn't do this automatically)
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
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

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
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}
