
#include <Servo.h>
#include <SPI.h> //Call SPI library so you can communicate with the nRF24L01+
#include <Wire.h>
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <RF24.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#define NUMPAYLOADPERPACKET 14
#define PACKSIZE 32
#define SENSORNUM 2
#define EXPDEBUG0 true
#define EXPDEBUG1 true
#define EXPDEBUG2 false
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H  0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
  #define Y_FINE_GAIN      0x04
  #define Z_FINE_GAIN      0x05
  #define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
  #define XA_OFFSET_L_TC   0x07
  #define YA_OFFSET_H      0x08
  #define YA_OFFSET_L_TC   0x09
  #define ZA_OFFSET_H      0x0A
  #define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define MPU9250CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif

#define AHRS true         // set to false for basic data read
#define SerialDebug false   // set to true to get Serial output for debugging

#define TIMEGAP 10

//const pin settings
const int pinCE = 4; //This pin is used to set the nRF24 to standby (0) or active mode (1)
const int pinCSN = 5; //This pin is used to tell the nRF24 whether the SPI communication is a command or message to send out
byte gotByte = 0; //used to store user commands from transmit module
byte gotWord = 0; //used to store user commands from transmit module //redundant
volatile int count = 0; //tracks the number of interrupts from IRQ
int pCount = 0; //tracks what last count value was so know when count has been updated
RF24 wirelessSPI(pinCE, pinCSN); // Declare object from nRF24 library (Create your wireless SPI)
const uint64_t wAddress = 0xB00B1E50B1LL;              // Pipe to write or transmit on
const uint64_t rAddress = 0xB00B1E50D2LL;  //pipe to recive data on
word userCommand = 0; //userCommand - need to convert to byte in sensor nodes
// structure to hold experimental sensor data from sensor nodes
typedef struct { //total 32 byte of
  word angularSpeed = 0x0000;  // 2 bytes (speed value : 2byte word 16bit pwm; number sample points wrod 0-63655| byte)
  word msgLength = 0x0000;    // 1-14 data

  word data0 = 0x0000;    // 1 data points in words
  word data1 = 0x0000;    // 1 data points in words
  word data2 = 0x0000;    // 1 data points in words
  word data3 = 0x0000;    // 1 data points in words
  word data4 = 0x0000;    // 1 data points in words
  word data5 = 0x0000;    // 1 data points in words
  word data6 = 0x0000;    // 1 data points in words
  word data7 = 0x0000;    // 1 data points in words
  word data8 = 0x0000;    // 1 data points in words
  word data9 = 0x0000;    // 1 data points in words
  word data10 = 0x0000;   // 1 data points in words
  word data11 = 0x0000;   // 1 data points in words
  word data12 = 0x0000;   // 1 data points in words
  word data13 = 0x0000;   // 1 data points in words
  // total 1 to 13 words of data
} ExperimentPayload;

// the possible states of the state-machine
typedef enum {  NONE, GOT_N, GOT_D, GOT_I, GOT_S, GOT_M, GOT_F, GOT_R, GOT_P, GOT_U } states;
//GOT_D can be a direction indicator
//MOTOR status enums
typedef enum {STOP, RESTART, RUN} motorStates; //how about CL, ACL ?

// structure to hold experimental configuration data from users
typedef struct {
  uint8_t numb_transmit_record = 3;
  unsigned int sensorNumber = SENSORNUM;
  unsigned int rotationSpeedIncrement = 1;   // I length of bootloader hex data (bytes)
  unsigned int measurmentFrequency = 20; //F
  unsigned int requiredSampleNumber = 72; //M
  unsigned int rotationDirection = 1; //1 for clockwise 0 for anticlockwise;
  unsigned int pauseTime = 1; //1 for clockwise 2 for anticlockwise;
  unsigned int restartValue = 1; //1 for clockwise 2 for anticlockwise;
  unsigned int speed_offset = 20;
  unsigned int pwmLimit = 132;
  unsigned int arm = 53;
  float coefficient_yaw = 0.1;
} ExperimentSetting;


typedef struct {
  unsigned int transmitNumber = NUMPAYLOADPERPACKET;
  unsigned int pwmSpeed = 90;   // I length of bootloader hex data (bytes)
  unsigned int sampleCounter = 0; //when came out of the measurement loop then store the counter here and catch up from where left off
  unsigned int waitingDelay = 0; //frequency delay between measuemrents
  bool completeMeasurementLoop = false;
  unsigned int prevRotation = 1;
  float sound_speed = 340.29;
} ExperimentStatus;

typedef struct {
  float xa = 0.0f; //accelerometer
  float ya = 0.0f; //no need of typecast from double 0.0 to float or int 0 to float
  float za = 0.0f; //no extra code by compilers.
  float xg = 0.0f; //gyroscope
  float yg = 0.0f;
  float zg = 0.0f;
  float xm = 0.0f; // magnetometor
  float ym = 0.0f;
  float zm = 0.0f;
  float temp = 0.0f;
} GyroValue;

typedef struct {
  int16_t xa = 0;
  int16_t ya = 0;
  int16_t za = 0;
  int16_t xg = 0;
  int16_t yg = 0;
  int16_t zg = 0;
  int16_t xm = 0;
  int16_t ym = 0;
  int16_t zm = 0;
  int16_t yaw = 0;
  int16_t pitch = 0;
  int16_t roll = 0;
  int16_t temperature = 0;
  float freq = 0.0f;
} GyroOutput;

typedef struct {
  uint8_t sensorID;
  GyroOutput gyroOut;
  uint16_t range = 0;
  bool error = false;
  uint32_t timeStamp = 0;
  int16_t reflected_yaw = 0;
  int16_t reflected_pitch = 0;
  int16_t reflected_roll = 0;
} SensorData;

typedef struct {
  GyroOutput gyroOut;
  bool error = false;
  uint32_t timeStampB = 0;
  uint32_t timeStampE = 0;
} GyroData;

//mpu settings
// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int adoPin = 8;
int myLed = 13;

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t prevMeasure = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;

/*
   time markers
*/
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval
/*
   IMU calculation parameters
*/
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


// current partial number
int currentValue;

ExperimentStatus expLoopStatus;
//declaring the ExperimentPayload
ExperimentPayload expPayload;
//declaring the ExperimentSetting
ExperimentSetting expSetting;
// current state-machine state

//======Sensor and servo init start
Servo myservo; //download the servo library and inspect how it works and use it in C code later
const int esc_sig_pin = 3; // any pwm available pin (3, 6, 9, 11)
const int ultrasonic_sensor_address[SENSORNUM] = {34,24}; //secsor collections};//

//int angular_speed = 0; //initial angular speed
//int limit = 132; // anticlockwise PWM upper speed limit for the ESC
//int arm = 53; //clockwise PWM starting point with highest speed
unsigned int mid_point = (expSetting.pwmLimit - expSetting.arm) / 2 + expSetting.arm; //TODO is it possible to put in the structure?
//int speed_offset = 20; // limit to 5 hz maximum
//int reading_time = 35; // waiting time for a reading in ms( milli seconds) 6.5 per metre
//const int readings = 72; // how many times reading in an angular speed 72 constant for 2 times of statistical basis.
//int waiting_time = 100 / expSetting.measurmentFrequency * expSetting.requiredSampleNumber; //overall time staying in an angular speed
//========Sensor and servo init end


states state = NONE;

motorStates motorStatus = STOP;

void expSettup(const unsigned int value) {
  switch (state) {
    case GOT_I:
      expSetting.rotationSpeedIncrement = value;
      break;
    case GOT_F:
      expSetting.measurmentFrequency = value;
      break;
    case GOT_M:
      expSetting.requiredSampleNumber = value;
      break;
    case GOT_D:
      expSetting.rotationDirection = value;
      break;
    case GOT_P:
      expSetting.pauseTime = value;
      break;
    case GOT_R:
      expSetting.restartValue = value;
      break;
    case GOT_U:
      expSetting.sensorNumber = value;
      break;
    case NONE:
    default:
      break;
  }
  return;
}

void processDisplayCurrentSetting ()
{
  // print all the setting values of this experiment
  Serial.println ("======================================");
  Serial.println ("Current settings are : ");
  Serial.print ("Rotation increment in pwm value = ");
  Serial.println (expSetting.rotationSpeedIncrement);
  Serial.print ("Rotation direction = ");
  Serial.println (expSetting.rotationDirection);
  Serial.print ("The number of required samples in a speed = ");
  Serial.println (expSetting.requiredSampleNumber);
  Serial.print ("Frequency = ");
  Serial.println (expSetting.measurmentFrequency);
  Serial.print ("PauseTime = ");
  Serial.println (expSetting.pauseTime);
  Serial.print ("restart Value = ");
  Serial.println (expSetting.restartValue);
  Serial.println ("======================================");
  return;
} // end of processDisplayCurrentSetting

void processSampleDisp (const unsigned int value) //const int so not changing the value
{
  // should it be displayed or sent to a sensor?
  Serial.print ("Number of samples to display = ");
  Serial.println (value);
}
void setTrasmitNumber() {
  expLoopStatus.transmitNumber = (NUMPAYLOADPERPACKET / (expSetting.sensorNumber * expSetting.numb_transmit_record)) * (expSetting.sensorNumber * expSetting.numb_transmit_record);
}

void setTrasmitNumberUser(const unsigned int value) {
  if (value < NUMPAYLOADPERPACKET) {
    expLoopStatus.transmitNumber = value;
  } else {
    return;
  }
}

void setRotationDirection(const unsigned int value) {
  //expSetting.rotationDirection = value;
  //  if (value == 0) {
  //    expLoopStatus.pwmSpeed = expSetting.arm + expSetting.speed_offset;
  //
  //  } else {
  //    expLoopStatus.pwmSpeed = mid_point;
  //  }
  Serial.print ("Rotation direction value = ");
  Serial.println (value);
}

void changeRotationIncre (const unsigned int value)
{
  expSetting.rotationSpeedIncrement = value;
  //serial line ouput message for debugging
  Serial.print ("Rotation increment in pwm value = ");
  Serial.println (value);
}

void changeSampleM(const unsigned int value)
{
  expSetting.requiredSampleNumber = value;
  //serial line ouput message for debugging
  Serial.print ("changed required Sample Number:");
  Serial.println (expSetting.requiredSampleNumber);
} // end of changeSampleM

void changeFreq(const unsigned int value)
{
  expSetting.measurmentFrequency = value;
  expLoopStatus.waitingDelay = 1000 / expSetting.measurmentFrequency;

  // sending frequency of sensing to a sensor
  Serial.print ("Frequency = ");
  Serial.println (expSetting.measurmentFrequency);
} // end of changeFreq

void restartMotor(const unsigned int value)
{
  expSetting.restartValue = value;
  expLoopStatus.completeMeasurementLoop = true;
  motorStatus = RESTART;

  //restarting the motor with pwm value given
  //serial line ouput message for debugging
  Serial.print ("restarting Motor with pwm value: ");
  Serial.println (expSetting.restartValue);
}
void pauseMotor(const unsigned int value)// stop the motor for the value seconds
{
  expSetting.pauseTime = value;
  motorStatus = STOP;
  //pausing motor for seconds
  Serial.print ("pauseMotor = ");
  Serial.println (expSetting.pauseTime);
}

void handlePreviousState ()
{
  expSettup(currentValue); //setting all the value in the global variable now

  switch (state)
  {
    case GOT_N:
      setTrasmitNumberUser(currentValue);
      break;
    case GOT_D:
      setRotationDirection (currentValue); //no arguments needed
      break;
    case GOT_S:
      processSampleDisp (currentValue);
      break;
    case GOT_M:
      changeSampleM (currentValue);
      break;
    case GOT_F:
      changeFreq (currentValue);
      break;
    case GOT_R:
      restartMotor (currentValue);
      break;
    case GOT_P:
      pauseMotor (currentValue);
      break;
    case GOT_I:
      changeRotationIncre (currentValue);
      break;
    case GOT_U:
      setTrasmitNumber();
      break;
    default:
      //state = NONE;
      break;
  }  // end of switch
  currentValue = 0;
}  // end of handlePreviousState

void processIncomingByte (const byte c)
{
  if (isdigit (c) && c != 0) //need to get multiple bytes of digits
  {
    currentValue *= 10;
    currentValue += c - '0';
  }  // end of digit
  else {    // The end of the number signals a state change

    handlePreviousState (); //exit flag - no more processing inside

    char upperC = toupper (c);
    /*
      if (!upperC){
      Serial.println("Non-alphabet character detected");
      return;
      }
    */
    // set the new state, if we recognize it
    switch (upperC)
    {
      case 'N':
        state = GOT_N;
        break;
      case 'D':
        state = GOT_D;
        break;
      case 'I':
        state = GOT_I;
        break;
      case 'S':
        state = GOT_S;
        break;
      case 'M':
        state = GOT_M;
        break;
      case 'F':
        state = GOT_F;
        break;
      case 'R':
        state = GOT_R;
        break;
      case 'P':
        state = GOT_P;
        break;
      case 'U':
        state = GOT_U;
        break;
      default:
        state = NONE;
        break;
    }  // end of switch on processing incoming bytes
  }// end of non-digit
} // end of processIncomingByte

void fillDummy() { //for debug purpose remove it in the production code
  //dummy payload
  expPayload.angularSpeed = random(35, 155); //simulating the sensor time
  expPayload.msgLength = random(1, NUMPAYLOADPERPACKET); //simulating the sensor time
  expPayload.data0 = random(50, 750); //simulating the sensor time
  expPayload.data1 = random(50, 750); //simulating the sensor time
  expPayload.data2 = random(50, 750); //simulating the sensor time
  expPayload.data3 = random(50, 750); //simulating the sensor time
  expPayload.data4 = random(50, 750); //simulating the sensor time
  expPayload.data5 = random(50, 750); //simulating the sensor time
  expPayload.data6 = random(50, 750); //simulating the sensor time
  expPayload.data7 = random(50, 750); //simulating the sensor time
  expPayload.data8 = random(50, 750); //simulating the sensor time
  expPayload.data9 = random(50, 750); //simulating the sensor time
  expPayload.data10 = random(50, 750); //simulating the sensor time
  expPayload.data11 = random(50, 750); //simulating the sensor time
  expPayload.data12 = random(50, 750); //simulating the sensor time
  expPayload.data13 = random(50, 750); //simulating the sensor time
  return;
}

void sendResultToHost(const void* payload) {
  const char * bytePayload = (const char *)payload;//Test void to byte
  wirelessSPI.stopListening();        //transmitter so stop listening for data

  if (!wirelessSPI.write( bytePayload, 32)) { //if the send fails let the user know over serial monitor
    Serial.println("packet delivery failed");
  }
  wirelessSPI.startListening();
  return;
}

//Servo functions
boolean start_sensor(byte bit8address) {
  boolean errorlevel = 0;
  byte bit7address;
  bit7address = bit8address & B11111110;               //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.

  Wire.beginTransmission((int)bit7address);
  Wire.write(81);           //Read a byte sub command start 81
  errorlevel = Wire.endTransmission() | errorlevel;      //terminate the transmission

  return errorlevel; //1 is good to communicate with sensor
}

uint16_t read_sensor(byte bit8address) {
  boolean errorlevel = 0;
  uint16_t range = 0;
  Wire.beginTransmission((int)bit8address);
  Wire.write(byte(0x01)); // sub command
  errorlevel = Wire.endTransmission();
  int avail = Wire.requestFrom((uint8_t)bit8address, (uint8_t)2);
  if (avail >= 2) {
    range = ( Wire.read() << 8 ) +  Wire.read(); //TODO check if this reports the actual measurement or measuremnt more than 255
  }
  if (errorlevel) {
    return 0;
  }
  else {
    return range;
  }
}

bool measure(const int sensors, SensorData * sensorData) {//sensor array[] and call all and waiting
  //static uint16_t error[SENSORNUM] = {0};  //Create a bit to check for catch errors as needed. array = one by one single = stack all
  //static uint16_t range[SENSORNUM] = {0};  //TODO need to make it as a struct outside and update it.
  //static SensorData sensorData[SENSORNUM]; //TODO make this as a global and refresh
  //initiating all sensors almost simultaneously using interrupts?

  //for (int i = 0; i < SENSORNUM; i++) {
  //uint8_t errorRetry = 5;
  //uint8_t errorCount = 0;
  //while (errorCount < errorRetry) {
  sampleIMUtoSensor(sensorData); //taking timestamp 1 here

  sensorData->error |= !start_sensor(sensors);    //Start the sensor and collect any error codes.
  //if (sensorData[i].error) {
  //errorCount++;
  //} else {
  //sampleIMU();
  //break;
  //}
  //}
  //}
  //if any of the sensor data is available
  //bool errorCombined = false;
  //for (int i = 0; i < SENSORNUM; i++) {
  //errorCombined |= sensorData->error;
  if (sensorData->error) {
    
    //TODO notify error sensors.
    if (SerialDebug) {
      Serial.print("Error Sensor number "); Serial.println(sensorData->sensorID);
    }
    return 0;
  }
//  else {
//    //continue; //next
//  }
  //}
//  if (errorCombined) {
//    //free(sensorData);
//    sampleIMU();
//    return 0; //sending all sensorData with error flag on;
//  }
  //delay(expLoopStatus.waitingDelay); //frequency determiner recommended value > 6.5ms * previous measurement / 100
  uint32_t  measureWaitingNow = 0, measureWaitingPrevMeasure = 0, measureWaitingAccumulated = 0; // used to calculate integration interval
  do {
    if (pCount == count) {
      measureWaitingNow =  millis();
      sampleIMU(); //sample more IMU collecting and linear filter them while waiting. asynchronous sampling
      //shows average angular speed.
      measureWaitingPrevMeasure = millis();
      measureWaitingAccumulated += measureWaitingPrevMeasure - measureWaitingNow;
    } else {
      return 0;
    }
  } while (measureWaitingAccumulated < expLoopStatus.waitingDelay); //no need to be super accurate about this just sample enough to provide
  //time to collect ultrasonic range finder measurements
  //for (int i = 0; i < SENSORNUM; i++) {
//  if (sensorData->error) { //error == 1 then
//    sensorData->range = 0;
//    sampleIMU();
//    return 0;
//    //continue; //next sequence
//  }
  sensorData->range = read_sensor(sensors);   //reading the sensor will return an integer value -- if this value is 0 there was an error
  if(EXPDEBUG1){
    Serial.println(sensorData->range);  
  }
  
  //TODO calculate the time of flight and figure out when was the measurment hit the objects (sensorData->timeStamp - millis())/12.8
  //}
  //TODO calculate the actual yaw from the gyro and time of flight.
  //for (int i = 0; i < SENSORNUM; i++) {
  uint32_t elsp_time = sensorData->timeStamp - (uint16_t)(((float)sensorData->range / 100.0f / expLoopStatus.sound_speed) * 1000); //time1 ms - time2 ms (range / speed of sound in s *1000 s/ms )
  //TODO find out how to modify the yaw angle from the constant gyro value;
  //possibly parting the mahony linear filtering equation or from general q to yaw.
  sensorData->reflected_yaw = sensorData->gyroOut.yaw; // (uint16_t)(sensorData->gyroOut.yaw + (float)(elsp_time * sensorData->gyroOut.zg)/1000000.0f * expSetting.coefficient_yaw);
  //TODO make this in 3D space

  //}
  return 1; //early return for a valid measurement check available.Z
}
uint16_t result[NUMPAYLOADPERPACKET]  = {0}; //initialising the result packet
void measure_cycle() {
  //check pause or restart - timer needed

  //TODO: gyro angular speed gy.

  for (; expLoopStatus.sampleCounter < expSetting.requiredSampleNumber; ) {
    if (pCount == count) {

      bool measurement_check = 0;

      //TODO Make asynchronous triggering
      SensorData measurements[SENSORNUM];
      for (int i = 0; i < SENSORNUM; i++) {
        measurements[i].sensorID = i;
        measurement_check |= measure(ultrasonic_sensor_address[i], &measurements[i]); //reading the sensor and append to the reading

      }
  
//      if (measurement_check) {
//        //prepare the results to send through the radio
//
//        //value check range 0 or 675 than redo the measurement Do it again?
//        for (size_t i = 0; i < SENSORNUM; i++) {
//          if (measurements[i].range == 0 || measurements[i].range == 765) {
//            measurement_check = measure(ultrasonic_sensor_address[i], &measurements[i]); //only one of them update?
//          }
//        }
//      } else {
//        if (SerialDebug) {
//          Serial.println("something wrong with all sensors");
//        }
//        //TODO send warning signs through nrf24
//        return;
//      }
      for (size_t i = 0; i < SENSORNUM; i++) {
        //uint16_t measurement = measure(ultrasonic_sensor_address[i]); //reading the sensor and append to the reading
        //uint16_t: 2 bytes -> [0-65535] or [0x0000-0xFFFF]
        //TODO change it to fill up the payload buffer
        if (measurements[i].range != 0 && measurements[i].range != 765) {
          result[expPayload.msgLength++] = (uint16_t)measurements[i].sensorID;
          result[expPayload.msgLength++] = (uint16_t)measurements[i].reflected_yaw;
          result[expPayload.msgLength++] = (uint16_t)measurements[i].range; //one variable case change array case
          if(EXPDEBUG1){
          Serial.print(expPayload.angularSpeed); Serial.print(" ");
          //Serial.print(expPayload.msgLength - 3); Serial.print(" ");
          Serial.print(result[expPayload.msgLength - 3]); Serial.print(" ");
          //Serial.print(expPayload.msgLength - 2); Serial.print(" ");
          Serial.print(result[expPayload.msgLength - 2]); Serial.print(" ");
          //Serial.print(expPayload.msgLength - 1); Serial.print(" ");
          Serial.println(result[expPayload.msgLength - 1]);
          }
        }
      }
      //error check? if any of them are 001 style and log it and more than tollerence.
      bool errorCombined = 0;
      uint16_t SumAngSpeed = 0;
      uint16_t countAngSpeed = 0;
      for (size_t i = 0; i < SENSORNUM; i++) {
        errorCombined &= (uint16_t)measurements[i].error; //if any sensor initiated reports 0; then 1
        if (!errorCombined) {
          SumAngSpeed += (uint16_t)measurements[i].gyroOut.zg; //in the prototype experiment only z axis of gyro
          countAngSpeed++;
        }
      }
      if (countAngSpeed) {
        expPayload.angularSpeed = (uint16_t)(SumAngSpeed / countAngSpeed);
      }
      //decide to send by sample count (expLoopStatus.sampleCounter==expLoopStatus.transmitNumber/(expSetting.sensorNumber* expSetting.numb_transmit_record)
      expLoopStatus.sampleCounter++;
      //expPayload.msgLength= expPayload.msgLength + expSetting.sensorNumber;
      //Serial.print(expPayload.msgLength); Serial.print(" ");Serial.print(expLoopStatus.sampleCounter); Serial.println(" ");
      
      if (expPayload.msgLength == expLoopStatus.transmitNumber || expLoopStatus.sampleCounter > expSetting.requiredSampleNumber - 1) { //full buffer then send it and prepare it
        //prepare the data, by mix and match method, into a payload structure and send it
        const uint16_t payload[] = {expPayload.angularSpeed, expPayload.msgLength, result[0], result[1],
                                    result[2], result[3], result[4], result[5], result[6], result[7],
                                    result[8], result[9], result[10], result[11], result[12], result[13]
                                   }; //initialising and terminating in this scope only
//        if(EXPDEBUG1){
//        for (int i = 0; i < (int)expPayload.msgLength; ++i ) {
//          Serial.print(payload[2 + i]);
//          Serial.print("  ");
//          }
//        Serial.println();
//        }

        sendResultToHost(payload); //sending only we need?
        expPayload.msgLength = 0;
        memset(result, 0, NUMPAYLOADPERPACKET * sizeof & result[0]); //set all field of payload array to 0 //probably unneccesary
      }

    } else {
      return;// early return due to the unprocessed packet from radio
    }
  }//iteration loop
  //initialising it after looping
  expLoopStatus.completeMeasurementLoop = true; //increment pwm speed outter loop
  expLoopStatus.sampleCounter = 0; //initialise the counter to zero to get new loop later
  return; //successful return after all the the
}

void slowDown(unsigned int required_speed, unsigned int interval) {
  myservo.write(required_speed - (required_speed - expLoopStatus.pwmSpeed) / 3); //expSetting.rotationDirection ==1
  delay(interval);//TODO find alternative code not to use the inter timer0 disable timer 0 later
  myservo.write(required_speed - 2 * (required_speed - expLoopStatus.pwmSpeed) / 3); //expSetting.rotationDirection ==1
  delay(interval); //these stops everything exclude isr
  myservo.write(required_speed); //stopping status
  expLoopStatus.pwmSpeed = required_speed;
  return;
}

int motorStatusFunc() {
  switch (motorStatus) {
    case STOP:
      if (expLoopStatus.pwmSpeed != mid_point) {
        slowDown(mid_point, 1000);
      }
      return 1; //getting out of this function
    case RESTART:
      if (expLoopStatus.pwmSpeed != expSetting.restartValue) {
        slowDown(expSetting.restartValue, 500);
        expLoopStatus.pwmSpeed = expSetting.restartValue;
      }
      motorStatus = RUN;
      break;
    case RUN: // all run?
    default:
      //do nothing just go out of the switch and continue the next script
      //continue; //to the end of the current loop
      break;
  }//how much clocks to do these?
  return 0;
}

void ucxl_measurements() {
  if (pCount < count) {
    return;
  }
  expPayload.angularSpeed = expLoopStatus.pwmSpeed;//preparing the first field of the payload to host
  if (motorStatusFunc()) {
    return;
  }
  if (SerialDebug) {
    Serial.println("pwm set up");
  }
  expLoopStatus.prevRotation = expSetting.rotationDirection;
  if (expSetting.rotationDirection == 1) { //clockwise direction
    for (; expLoopStatus.pwmSpeed > expSetting.arm + expSetting.speed_offset; )
    {
      if (expLoopStatus.prevRotation != expSetting.rotationDirection) {
        break;
      }
      if (SerialDebug) {
        Serial.println("rotation direction 0");
      }
      if (expLoopStatus.completeMeasurementLoop == true) {
        expLoopStatus.pwmSpeed -= expSetting.rotationDirection * expSetting.rotationSpeedIncrement;
        expLoopStatus.completeMeasurementLoop = false;
      }
      if (SerialDebug) {
        Serial.println(expLoopStatus.pwmSpeed);
      }

      myservo.write(expLoopStatus.pwmSpeed); //constant speed during the iteration of measurements
      if (SerialDebug) {
        Serial.println("servo written");
      }
      measure_cycle();
      if (SerialDebug) {
        Serial.println("measured cycle");
      }
      if (pCount != count) {
        return; //early return to process return or break?
      }
      //if measure_cycle loop finsihed all the sampling request then move to next one
      //check the expSetting.requiredSampleNumber == count then increment <- inside
    }
    if (expLoopStatus.pwmSpeed < expSetting.arm + expSetting.speed_offset + 1) {
      slowDown(mid_point, 1000);
    }

  } else if (expSetting.rotationDirection == 0) { //anticlockwise direction
    for (; expLoopStatus.pwmSpeed < expSetting.pwmLimit; )
    {
      if (expLoopStatus.prevRotation != expSetting.rotationDirection) {
        break;
      }
      if (expLoopStatus.completeMeasurementLoop == true) {
        expLoopStatus.pwmSpeed += expSetting.rotationSpeedIncrement;
        expLoopStatus.completeMeasurementLoop = false;
      }

      myservo.write(expLoopStatus.pwmSpeed);
      measure_cycle();
      if (pCount != count) {
        return;
      }
    }
    if (expLoopStatus.pwmSpeed > expSetting.pwmLimit - 1) {
      slowDown(mid_point, 1000);
    }
  } else {
    //sending error message to the host
    if (SerialDebug) {
      char msg[] = "invalid direction param -1, 1";
      sendResultToHost(&msg); //need & to convert it to void
    }
  }
  return;
}

void setup ()
{
  //Servo and I2C set up
  myservo.attach(esc_sig_pin); //attaching the selected pwm pin
  Wire.begin(); //Strarting the I2C communication
  //nrf24 modem set up
  wirelessSPI.begin();  //Start the nRF24 module
  wirelessSPI.setAutoAck(1);                    // Ensure autoACK is enabled so rec sends ack packet to let you know it got the transmit packet payload
  wirelessSPI.enableAckPayload();         //allows you to include payload on ack packet
  wirelessSPI.maskIRQ(1, 1, 0);             //mask all IRQ triggers except for receive (1 is mask, 0 is no mask)
  wirelessSPI.setPALevel(RF24_PA_LOW); //Set power level to low, won't work well at higher levels (interfer with receiver)
  wirelessSPI.openWritingPipe(wAddress);        //open writing or transmit pipe
  wirelessSPI.openReadingPipe(1, rAddress); //open reading or recieve pipe
  wirelessSPI.startListening();                 // Start listening for messages

  Serial.begin (115200); //optional in a release

  //pin initialising
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(adoPin, OUTPUT);
  digitalWrite(adoPin, HIGH);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  if (SerialDebug) {
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  }
  delay(200);
  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    if (SerialDebug) {
      Serial.println("MPU9250 is online, now self-testing");
    }

    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values

    if (SerialDebug) {
      Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
      Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
      Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
      Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
      Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
      Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
    }
    delay(500);

    getAres();
    getGres();
    getMres();

    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    if (SerialDebug) {
      Serial.println(" MPU9250 calibrated and its bias");
      Serial.println(" x   y   z  ");
      Serial.print(" ");
      Serial.print((int)(1000 * accelBias[0])); Serial.print(" ");
      Serial.print((int)(1000 * accelBias[1])); Serial.print(" ");
      Serial.print((int)(1000 * accelBias[2]));
      Serial.println("mg");
      Serial.print(gyroBias[0]); Serial.print(" ");
      Serial.print(gyroBias[1]); Serial.print(" ");
      Serial.print(gyroBias[2]);
      Serial.println("o/s");
    }
    delay(500);

    initMPU9250();

    if (SerialDebug) {
      Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    }
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
    if (SerialDebug) {
      Serial.println("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    }
    delay(100);

    // Get magnetometer calibration from AK8963 ROM
    initAK8963(magCalibration);  if (SerialDebug) {
      Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
    }
    {
      //magcalMPU9250(magBias, magScale);
      float magbias[3] = {0, 0, 0};
      magbias[0] = 54.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
      magbias[1] = 280.;  // User environmental x-axis correction in milliGauss
      magbias[2] = -448.;  // User environmental x-axis correction in milliGauss

      magBias[0] = (float) magbias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
      magBias[1] = (float) magbias[1] * mRes * magCalibration[1];
      magBias[2] = (float) magbias[2] * mRes * magCalibration[2];

      // Get soft iron correction estimate hardcoded now but it can be monitored and corrected when new soft iron is introduced.
      magScale[0] = 0.92;
      magScale[1] = 1.03;
      magScale[2] = 1.05;
    }

    if (SerialDebug) {
      Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
    }

    delay(500);
  }
  else
  {
    if (SerialDebug) {
      Serial.print("Could not connect to MPU9250: 0x");
      Serial.println(c, HEX);
    }
    while (1) ; // Loop forever if communication doesn't happen
  }

  expLoopStatus.transmitNumber = (NUMPAYLOADPERPACKET / (expSetting.sensorNumber * expSetting.numb_transmit_record)) * (expSetting.sensorNumber * expSetting.numb_transmit_record); //limit number per packet
   if (EXPDEBUG2) {
      Serial.print(expLoopStatus.transmitNumber); Serial.print("  ");
   }
  randomSeed(analogRead(0));    //use random ADC value to seed random number algorithm
  Serial.println(F("sensor calibration compeleted"));
  char msg[] = "sensor calibration compeleted";
  sendResultToHost(&msg); //need & to convert it to void
}  // end of setup
void loop ()
{
  if (pCount < count) { //read the command after interrupt came from the modem
    //possibly detache the interupt here for a while
    byte inByte[PACKSIZE] {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                          };//initialising and terminating in this scope only

    while (wirelessSPI.available()) { //get data sent from transmit
      wirelessSPI.read( &inByte, sizeof inByte);
      //Serial.println(gotByte);
      unsigned int inByteLength = sizeof inByte;
      for (unsigned int i = 0; i < inByteLength; ++i) {
        processIncomingByte(inByte[i]); //read one byte of data and store it in gotByte variable
      }
    }
    count = pCount; //resetting interrupt flags
  }
  else { //measuring or sensing if not getting any incomming message
    /*
      { //sensor reading function
      fillDummy();
      }
      const word payload[] = {expPayload.angularSpeed, expPayload.msgLength, expPayload.data0, expPayload.data1,
                            expPayload.data2, expPayload.data3, expPayload.data4, expPayload.data5,
                            expPayload.data6, expPayload.data7, expPayload.data8, expPayload.data9,
                            expPayload.data10, expPayload.data11, expPayload.data12, expPayload.data13
                           }; //initialising and terminating in this scope only
      //TODO just get the structure sent
      {
      delay(expLoopStatus.waitingDelay);
      //measuing sensors instead of delay

      sendResultToHost(payload); //TODO check if actually sending it well from
      }
    */
    ucxl_measurements();
    //attach the interrupt back to recv the incomming messages
    attachInterrupt(0, interruptFunction, FALLING);  //Create interrupt: 0 for pin 2 or 1 for pin 3, the name of the interrupt function or ISR, and condition to trigger interrupt
  }
}  // end of loop

void interruptFunction() {
  detachInterrupt(0); //interrupt on pin D2
  count++; //up the receive counter
}

//===================================================================================================================
//====== Set of helper function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================
void sampleIMU() {
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt from mpu9250, check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes;  //- accelBias[0]/2;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes;  //- accelBias[1]/2;
    az = (float)accelCount[2] * aRes;  //- accelBias[2]/2;

    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes;  // - gyroBias[0]/2; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;  // - gyroBias[1]/2;
    gz = (float)gyroCount[2] * gRes;  // - gyroBias[2]/2;

    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
    mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];

    mx *= magScale[0];
    my *= magScale[1];
    mz *= magScale[2];
  }

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  //MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, my, mx, mz);

  return;
}

void sampleIMUtoSensor(SensorData * sensorData) {
  sampleIMU();

  if (!AHRS) { //if AHRS false
    delt_t = millis() - prevMeasure;
    if (delt_t > TIMEGAP) {

      if (SerialDebug) {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000 * ax); Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000 * ay); Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000 * az); Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(gx); Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(gy); Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(gz); Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG");

        tempCount = readTempData();  // Read the adc values
        temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade TODO need to change it to the
        sensorData->gyroOut.temperature = (uint16_t)(temperature);
        // Print temperature in degrees Centigrade
        //Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
      }
      prevMeasure = millis();
    }
  }
  else { //if AHRS true
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - prevMeasure;
    if (delt_t > TIMEGAP) { //independent report to serial
      if (SerialDebug) {
        Serial.print("ax = "); Serial.print((int)1000 * ax);
        Serial.print(" ay = "); Serial.print((int)1000 * ay);
        Serial.print(" az = "); Serial.print((int)1000 * az); Serial.println(" mg");
        Serial.print("gx = "); Serial.print( gx);
        Serial.print(" gy = "); Serial.print( gy);
        Serial.print(" gz = "); Serial.print( gz); Serial.println(" deg/s");
        Serial.print("mx = "); Serial.print( (int)mx );
        Serial.print(" my = "); Serial.print( (int)my );
        Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");

        Serial.print("q0 = "); Serial.print(q[0]);
        Serial.print(" qx = "); Serial.print(q[1]);
        Serial.print(" qy = "); Serial.print(q[2]);
        Serial.print(" qz = "); Serial.println(q[3]);
      }

      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth.
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI;
      yaw   -= 10.9; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      roll  *= 180.0f / PI;

      sensorData->gyroOut.yaw = (uint16_t)(yaw);
      sensorData->gyroOut.pitch = (uint16_t)(pitch);
      sensorData->gyroOut.roll = (uint16_t)(roll);
      sensorData->gyroOut.freq = (float)(sumCount / sum);

      //printing all results
      if (SerialDebug) {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(yaw);
        Serial.print(", ");
        Serial.print(pitch);
        Serial.print(", ");
        Serial.println(roll);
        Serial.print("rate = "); Serial.print((float)sumCount / sum); Serial.println(" Hz");
      }
      // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
      // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
      // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
      // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
      // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
      // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
      // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
      // This filter update rate should be fast enough to maintain accurate platform orientation for
      // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
      // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
      // The 3.3 V 8 MHz Pro Mini is doing pretty well!
      digitalWrite(myLed, !digitalRead(myLed));
      prevMeasure = millis();
      sumCount = 0;
      sum = 0;
    }
  }

  sensorData->gyroOut.xa = (uint16_t)(1000 * ax);
  sensorData->gyroOut.ya = (uint16_t)(1000 * ay);
  sensorData->gyroOut.za = (uint16_t)(1000 * az);

  sensorData->gyroOut.xg = (uint16_t)(gx);
  sensorData->gyroOut.yg = (uint16_t)(gy);
  sensorData->gyroOut.zg = (uint16_t)(gz);

  sensorData->gyroOut.xm = (uint16_t)(mx);
  sensorData->gyroOut.ym = (uint16_t)(my);
  sensorData->gyroOut.zm = (uint16_t)(mz);

  sensorData->timeStamp = millis();
}

void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250()
{
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, MPU9250CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in MPU9250CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, MPU9250CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x10);  // Set gyro full-scale to 1000 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases to push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t  aAvg[3] = {0}, aSTAvg[3] = {0}, gAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, MPU9250CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.; // Report percent differences
    destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
  }
}

void magcalMPU9250(float * dest1, float * dest2)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(1000);

  // shoot for ~fifteen seconds of mag data
  if (Mmode == 0x02) sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
  if (Mmode == 0x06) sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
  for (ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if (Mmode == 0x02) delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
    if (Mmode == 0x06) delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
  }
  Serial.println("avg mag x   y   z:");
  Serial.print(" "); Serial.print((mag_max[0] + mag_min[0] ) / 2); Serial.print(" "); Serial.print((mag_max[1] + mag_min[1] ) / 2);; Serial.print(" "); Serial.println((mag_max[2] + mag_min[2] ) / 2);

  //    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  //    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  //    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes * magCalibration[1];
  dest1[2] = (float) mag_bias[2] * mRes * magCalibration[2];
  Serial.println("avg magBias x   y   z:");
  Serial.print(" "); Serial.print(dest1[0]); Serial.print(" "); Serial.print(dest1[1]);; Serial.print(" "); Serial.println(dest1[2]);

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts


  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = avg_rad / ((float)mag_scale[0]);
  dest2[1] = avg_rad / ((float)mag_scale[1]);
  dest2[2] = avg_rad / ((float)mag_scale[2]);
  Serial.println("avg magScale x   y   z:");
  Serial.print(" "); Serial.print(dest2[0]); Serial.print(" "); Serial.print(dest2[1]);; Serial.print(" "); Serial.println(dest2[2]);

  Serial.println("Mag Calibration done!");
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  //Wire.endTransmission();        // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  //  Wire.requestFrom(address, 1);  // Read one byte from slave register address
  Wire.requestFrom(address, (size_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  //Wire.endTransmission();  // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  //        Wire.requestFrom(address, count);  // Read bytes from slave register address
  Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}



// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }
  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}
