
#include <Servo.h>
#include <SPI.h> //Call SPI library so you can communicate with the nRF24L01+
#include <Wire.h>
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <RF24.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#define NUMPAYLOADPERPACKET 14
#define PACKSIZE 32
#define SENSORNUM 4
#define GYRO_ADDRESS 0x69 //gyro address
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
  unsigned int sensorNumber = 1;
  unsigned int rotationSpeedIncrement = 1;   // I length of bootloader hex data (bytes)
  unsigned int measurmentFrequency = 20; //F
  unsigned int requiredSampleNumber = 72; //M
  unsigned int rotationDirection = 1; //1 for clockwise 0 for anticlockwise;
  unsigned int pauseTime = 1; //1 for clockwise 2 for anticlockwise;
  unsigned int restartValue = 1; //1 for clockwise 2 for anticlockwise;
  unsigned int speed_offset = 20;
  unsigned int pwmLimit = 132;
  unsigned int arm = 53;
} ExperimentSetting;


typedef struct {
  unsigned int transmitNumber = NUMPAYLOADPERPACKET;
  unsigned int pwmSpeed = 90;   // I length of bootloader hex data (bytes)
  unsigned int sampleCounter = 0; //when came out of the measurement loop then store the counter here and catch up from where left off
  unsigned int waitingDelay = 0; //frequency delay between measuemrents
  bool completeMeasurementLoop = false;
  unsigned int prevRotation = 1;
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
  int16_t temp = 0;

} GyroOutput;

typedef struct {
  float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
  float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
  float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
  float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
  float pitch, yaw, roll;
  float deltat = 0.0f;                              // integration interval for both filter schemes
  uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
  uint32_t Now = 0;                                 // used to calculate integration interval
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
} GyroParameter;

typedef struct{
  GyroOutput gyroOut;
  uint16_t range =0;
} SensorData;

//SensorData sensorData;
GyroParameter gyroPara;
GyroValue gyroValue;
GyroOutput gyroOut;

ExperimentStatus expLoopStatus;
//declaring the ExperimentPayload
ExperimentPayload expPayload;
//declaring the ExperimentSetting
ExperimentSetting expSetting;
// current state-machine state
states state = NONE;

motorStates motorStatus = STOP;
// current partial number
int currentValue;
//======Sensor and servo init start
Servo myservo; //download the servo library and inspect how it works and use it in C code later
const int esc_sig_pin = 3; // any pwm available pin (3, 6, 9, 11)
const int ultrasonic_sensor_address[SENSORNUM] = {112, 110, 114, 116}; //ultrasonic address 224` to 110.

//int angular_speed = 0; //initial angular speed
//int limit = 132; // anticlockwise PWM upper speed limit for the ESC
//int arm = 53; //clockwise PWM starting point with highest speed
unsigned int mid_point = (expSetting.pwmLimit - expSetting.arm) / 2 + expSetting.arm; //TODO is it possible to put in the structure?
//int speed_offset = 20; // limit to 5 hz maximum
//int reading_time = 35; // waiting time for a reading in ms( milli seconds) 6.5 per metre
//const int readings = 72; // how many times reading in an angular speed 72 constant for 2 times of statistical basis.
//int waiting_time = 100 / expSetting.measurmentFrequency * expSetting.requiredSampleNumber; //overall time staying in an angular speed
//========Sensor and servo init end

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
  expLoopStatus.transmitNumber = NUMPAYLOADPERPACKET / expSetting.sensorNumber;
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

uint16_t* measure(const int *sensors) {//sensor array[] and call all and waiting
  boolean errorCombined = 0;
  static uint16_t error[SENSORNUM] = {0};  //Create a bit to check for catch errors as needed. array = one by one single = stack all
  static uint16_t range[SENSORNUM] = {0};  //TODO need to make it as a struct outside and update it.
  static SensorData sensorData[SENSORNUM];
  //initiating all sensors almost simultaneously using interrupts?

  for (int i = 0; i < SENSORNUM; i++) {
    uint8_t errorRetry = 5;
    uint8_t errorCount = 0;

    while (errorCount < errorRetry) {
      error[i] |= start_sensor(sensors[i]);    //Start the sensor and collect any error codes.
      //TODO Gyro reading in to a structure; str[sensor#, gyro str[]]
      errorCount++;
    }
    //if still fails;
    errorCombined |= error[i]; //if any sensor initinated not initiated 0 any initiated 1 reinitiate it if not ready? here?
  }

  if (errorCombined) {                 //If you had an error starting the sensor there is little point in reading it as you will get old data.
    delay(expLoopStatus.waitingDelay); //frequency determiner recommended value > 6.5ms * previous measurement / 100
    //TODO improvement timer instead of delay;
    for (int i = 0; i < SENSORNUM; i++) {
      if (!error[i]) { //error == 0 then
        range[i] = 0;
        continue; //next sequence
      }
      range[i] = read_sensor(sensors[i]);   //reading the sensor will return an integer value -- if this value is 0 there was an error
      //TODO triple sensor : insert to a struct or array of int or double - global variable
    }
    return range; //early return for a valid measurement check available.

  } else {
    return error; //returning status array //in the case of all initialisation has failed 000
  }
}

void measure_cycle() {
  //check pause or restart - timer needed
  //if not pause then keep checking

  //int count = 0; //TODO to global structure member
  uint16_t result[NUMPAYLOADPERPACKET]  = {0}; //initialising the result packet
  expPayload.angularSpeed = expLoopStatus.pwmSpeed;

  for (; expLoopStatus.sampleCounter < expSetting.requiredSampleNumber; ) {
    if (pCount == count) {
      //Serial.print("expLoopStatus.sampleCounter ");
      //Serial.println(expLoopStatus.sampleCounter);
      uint8_t totalSensorTransmit = expSetting.sensorNumber * expLoopStatus.transmitNumber;
      expPayload.msgLength =  expLoopStatus.sampleCounter % totalSensorTransmit; //preparing the second field of the payload to host //TODO to the settings structure
      uint16_t *measurements;
      measurements = measure(ultrasonic_sensor_address); //reading the sensor and append to the reading

      //error check? if any of them are 001 style and log it and more than tollerence.
      boolean errorCombined = 0;
      for (int i = 0; i < SENSORNUM; i++) {
        errorCombined &= measurements[i]; //if any sensor initiated reports 0; then 1
      }

      if (errorCombined) { //if any error exists continue
        free(measurements); //contains 010 eror codes
        continue; //not incrementing as the first tab of the for loop not initialised
      } else {
        //prepare the results to send through the radio
        for (int i = 0; i < SENSORNUM; i++) {
          //uint16_t measurement = measure(ultrasonic_sensor_address[i]); //reading the sensor and append to the reading
          //uint16_t: 2 bytes -> [0-65535] or [0x0000-0xFFFF]

          //TODO change it to fill up the payload buffer
          result[expPayload.msgLength] = measurements[i]; //one variable case change array case
          expPayload.msgLength++; //= expPayload.msgLength + expSetting.sensorNumber;
        }
        expLoopStatus.sampleCounter++;
        //expPayload.msgLength= expPayload.msgLength + expSetting.sensorNumber;
        free(measurements);

        if (expPayload.msgLength == totalSensorTransmit || expLoopStatus.sampleCounter == expSetting.requiredSampleNumber || expLoopStatus.sampleCounter > expSetting.requiredSampleNumber ) { //full buffer then send it and prepare it
          //prepare the data, by mix and match method, into a payload structure and send it
          const uint16_t payload[] = {expPayload.angularSpeed, expPayload.msgLength, result[0], result[1],
                                      result[2], result[3], result[4], result[5], result[6], result[7],
                                      result[8], result[9], result[10], result[11], result[12], result[13]
                                     }; //initialising and terminating in this scope only

          sendResultToHost(payload); //sending only we need?

          memset(result, 0, NUMPAYLOADPERPACKET * sizeof & result[0]); //set all field of payload array to 0
        }
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
  Serial.println("pwm set up");
  expLoopStatus.prevRotation = expSetting.rotationDirection;
  if (expSetting.rotationDirection == 1) { //clockwise direction
    for (; expLoopStatus.pwmSpeed > expSetting.arm + expSetting.speed_offset; )
    {
      if (expLoopStatus.prevRotation != expSetting.rotationDirection) {
        break;
      }
      Serial.println("rotation direction 0");
      if (expLoopStatus.completeMeasurementLoop == true) {
        expLoopStatus.pwmSpeed -= expSetting.rotationDirection * expSetting.rotationSpeedIncrement;
        expLoopStatus.completeMeasurementLoop = false;
      }

      Serial.println(expLoopStatus.pwmSpeed);

      myservo.write(expLoopStatus.pwmSpeed); //constant speed during the iteration of measurements
      Serial.println("servo written");
      measure_cycle();
      Serial.println("measured cycle");
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
    char msg[] = "invalid direction param -1, 1";
    sendResultToHost(&msg); //need & to convert it to void
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

  randomSeed(analogRead(0));    //use random ADC value to seed random number algorithm
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

    ucxl_measurements();
    //attach the interrupt back to recv the incomming messages
    attachInterrupt(0, interruptFunction, FALLING);  //Create interrupt: 0 for pin 2 or 1 for pin 3, the name of the interrupt function or ISR, and condition to trigger interrupt
  }
}  // end of loop

void interruptFunction() {
  detachInterrupt(0); //interrupt on pin D2
  count++; //up the receive counter
}
