/*
     Project Name:      Cervical Trainer
     Author List:       Abhishek Kansara,Bharathi Kumar
     Filename:          MPU6050_Latest_code
     Functions:         GetDMP() , MPUMath(), setup(), loop()
                        ,i2cSetup(),dmpDataReady(),MPU6050Connect()
     Global Variables:  MPUOffsets[],mpuInterrupt, FifoAlive,IsAlive,mpuIntStatus,devStatus,packetSize,fifoCount,
                        fifoBuffer[64],q,aa,aaReal,aaWorld,gravity,euler[],ypr[],StartUP
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


MPU6050 mpu;


#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define  DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif
#define LED_PIN 13

//  XA      YA      ZA      XG      YG      ZG
// MPU Offset Values
int MPUOffsets[6] = { -149, -744, 1052, 98, 183, -47};

// ............................................I2C SETUP Items......................................//

/*
  Function Name:  i2cSetup
  Input:    None
  Output:   It is used to setup I2C protocol
  Logic:    We here are just using the Wire.begin function
            from Wire.h library.Through this function we are
            just invoking the I2C protocol by assiging its
            registers in arduino
*/

void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();

  // 400kHz I2C clock (200kHz if CPU is 8MHz)
  TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

//.....................................INTERRUPT DETECTION ROUTINE......................................//

/*
  Variables
  mpuInterrupt: indicates whether MPU interrupt pin has gone high
*/

/*
  Function Name:  dmpDataReady
  Input:          None
  Output:         when data is coming then the intrrupt will generate
  Logic:          here only variable mpuInterrupt value is changed to true
                  when the interrupt occur
*/

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

//.........................................MPU DMP SETUP..................................................//

// Variables
int FifoAlive = 0; // tests if the interrupt is triggering
int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
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
byte StartUP = 100; // lets get 100 readings from the MPU before we start trusting them (Bot is not trying to balance at this point it is just starting up.)

/*
  Function Name:  MPU6050Connect
  Input:          None
  Output:         The mpu gets initialized and sends offset values
  Logic:          first here mpu gets initialized and checks for the status
                  that the required device has established connection and
                  calibarte the sensor value
*/

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize();
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }

  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, RISING); //pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus();

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();

  delay(1000); // Let it Stabalize

  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt
}

//...........................MPU DMP Get Data............................//

/*
  Function Name:  GetDMP
  Input:          None
  Output:         Get the data in form of data from i2c and processes further
  Logic:          we are getting data from I2C library
                  library acts as data processing from the raw
                  data from MPU and convert it in to euler angle
*/

void GetDMP() {

  // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());

  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) {
    // we have failed Reset and wait till next time!
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) {
      // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // Get the data
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    MPUMath(); //  On success MPUMath()
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}

//........................... MPU Math ...............................//

/*
  Function Name:  MPUMath
  Input:          None
  Output:         We get Yaw,Pitch,Roll in Degrees.
  Logic:          We are using some simple math and map function to calculate
                  this process of coverting it into degrees
*/

float Yaw, Pitch, Roll;
void MPUMath() {

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // conversion from rads to degree
  Yaw = (ypr[0] * 180.0 / M_PI);
  Pitch = (ypr[1] *  180.0 / M_PI);
  Roll = (ypr[2] *  180.0 / M_PI);

  // conversion of data into packets for bluetooth transmission
  int outy = map(Yaw, -180, 180, 0, 360) + 360;
  int outr = map(Pitch, -180, 180, 0, 360) + 360;
  int outp = map(Roll,  180, -180, 0, 360) + 360;

  Serial.print(outy);
  Serial.print("y");
  Serial.print(outp);
  Serial.print("p");
  Serial.print(outr);
  Serial.print("r");
  Serial.println("\t");
}

//.............................................Setup...........................................//


/*
  Function Name:  setup
  Input:          None
  Output:         We are here initializing the I2C protocol,baud rate,Led as an OUTPUT and MPU6050 device
  Logic:          Just Calling the function
*/

void setup() {
  Serial.begin(9600); //115200
  while (!Serial);
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);
}

//.............................................Loop...........................................//

/*
  Function Name:  loop
  Input:          None
  Output:         Whole processing occurs here
  Logic:          Just Call the GetDMP function if interrupt occur
*/

void loop() {
  if (mpuInterrupt ) {
    // wait for MPU interrupt or extra packet(s) available
    GetDMP();
  }
}
