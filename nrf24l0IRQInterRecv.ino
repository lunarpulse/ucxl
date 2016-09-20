
#include <SPI.h> //Call SPI library so you can communicate with the nRF24L01+
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <RF24.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
const int pinCE = 4; //This pin is used to set the nRF24 to standby (0) or active mode (1)
const int pinCSN = 5; //This pin is used to tell the nRF24 whether the SPI communication is a command or message to send out
word gotWord = 0; //used to store payload from transmit module
volatile int count = 0; //tracks the number of interrupts from IRQ
int pCount = 0; //tracks what last count value was so know when count has been updated
RF24 wirelessSPI(pinCE, pinCSN); // Declare object from nRF24 library (Create your wireless SPI)
const uint64_t wAddress = 0xB00B1E50D2LL;              // Pipe to write or transmit on
const uint64_t rAddress = 0xB00B1E50B1LL;  //pipe to recive data on
word userCommand = 0; //userCommand
const unsigned int MAX_INPUT = 32;
// structure to hold experimental sensor data from sensor nodes
typedef struct { //total 32 byte of
  word startWord = 0x0000;  // 2 bytes (speed value : 2byte word 16bit pwm; number sample points wrod 0-255| byte)
  //word samepleNWord = 0x0000;    // 1 data points in word( 8byte *2) - to unsigned int or float(32 byte - 4 words) later in receiver
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
typedef enum {  NONE, GOT_D, GOT_I, GOT_S, GOT_M, GOT_F, GOT_R, GOT_P } states;

states state = NONE;
// current partial number
unsigned int currentValue;

word buildCommand() {
  //build command 2 bytes word format depends on state
  //'C'(1)| number(2) (0- 255)
  word command = 0;
  switch (state)
  {
    case GOT_I:
      command = (byte)'I';
      break;
    case GOT_M:
      command = (byte)'M';
      break;
    case GOT_F:
      command = (byte)'F';
      break;
    case GOT_R:
      command = (byte)'R';
      break;
    case GOT_P:
      command = (byte)'P';
      break;
  }  // end of switch
  command <<= 8;
  command += currentValue;
  /*
     test - works previos byte to word word to byte works
    byte lb = command & 0x00FF;
    byte hb = (command  >> 8) & 0x00FF;

    Serial.print(hb);
    Serial.print(" ,");
    Serial.println(lb);
  */
  //state = NONE;
  return command;
}
int sendToSensor(const char* outByte) {
  //userCommand = buildCommand();
  wirelessSPI.stopListening();        // stop listening while sending commands

  if (!wirelessSPI.write( outByte, 32 )) { //if the send fails let the user know over serial monitor
    Serial.println("packet delivery failed");
    return 0;
  }
  wirelessSPI.startListening();                 // Back to listening mode
  return 1;
}

void processIncomingByte (const byte inByte)
{
  static char outByte[MAX_INPUT];//msg char array for 32 byte payload to transmit commands
  static unsigned short outByteSequence = 0;

  switch (inByte)
  {

    case '\n':   // new line 13
      for (int i = outByteSequence; i<MAX_INPUT; ++i ){
        outByte [i]=0;
      }
      //outByte [outByteSequence] = 0;  // terminating with null
      // terminator reached! sending the byte to sensors, multiple sensor recognise itself by serial number in the packet
      //Serial.println("-------------------------------------------------");
      //Serial.println(outByte);
      //Serial.println("-------------------------------------------------");

      /*
        {
        unsigned int outByteSize = sizeof outByte;
        for (int i = 0; i < outByteSize; ++i) {
          Serial.print(i);
          Serial.print(" : ");
          Serial.print(outByte[i]); //read one byte of data and store it in gotByte variable
        }
        Serial.println();
        }
      */
      sendToSensor (outByte);
      // reset buffer for next time
      outByteSequence = 0;
      break;

    case '\r':   // discard carriage return
      break;

    default:
      // keep adding until full ... allow for terminating null byte
      if (outByteSequence < (MAX_INPUT - 1))
        outByte [outByteSequence++] = inByte;
      break;

  }  // end of switch
} // end of processIncomingByte

void setup ()
{
  wirelessSPI.begin();  //Start the nRF24 module
  wirelessSPI.setAutoAck(1);                // Ensure autoACK is enabled so rec sends ack packet to let you know it got the transmit packet payload
  wirelessSPI.enableAckPayload();           //allows you to include payload on ack packet
  wirelessSPI.maskIRQ(1, 1, 0);             //mask all IRQ triggers except for receive (1 is mask, 0 is no mask)
  wirelessSPI.setPALevel(RF24_PA_LOW);      //Set power level to low, won't work well at higher levels (interfer with receiver)
  wirelessSPI.openWritingPipe(wAddress);    //open writing(transmit) pipe this will become receive pipe from sensor nodes
  wirelessSPI.openReadingPipe(1, rAddress); // open reading(recieve) pipe - > transmit pipe for sensor nodes, up to 6 pipes to listen from this node
  wirelessSPI.startListening();             // Start listening for messages

  Serial.begin (115200);
}  // end of setup
void loop ()
{

  while (Serial.available ())//only parsing when it is available, if not just go back to the parsing area.
    processIncomingByte(Serial.read ()); //receiving user commands from the user and process it to transmit or display messages

  if (pCount < count) { //If this is true it means count was interated and another interrupt occurred

    word payload[16] = {0x0000, 0x0000, 0x0000, 0x0000,
                        0x0000, 0x0000, 0x0000, 0x0000,
                        0x0000, 0x0000, 0x0000, 0x0000,
                        0x0000, 0x0000, 0x0000, 0x0000
                       }; //or 0x0000

    while (wirelessSPI.available()) { //get data sent from transmit
      //need to reverse the words to an array and display it accordingly
      wirelessSPI.read( &payload, 32 ); //read 32 byte of data and store it in payload[16] - 32 byte
    }
    ExperimentPayload expPayload;
//declaring the ExperimentSetting
    //TODO function sorting the message
    {
      expPayload.startWord = payload[0];
      //expPayload.samepleNWord = payload[1];
      expPayload.msgLength = payload[1];
      expPayload.data0 = payload[2];
      expPayload.data1 = payload[3];
      expPayload.data2 = payload[4];
      expPayload.data3 = payload[5];
      expPayload.data4 = payload[6];
      expPayload.data5 = payload[7];
      expPayload.data6 = payload[8];
      expPayload.data7 = payload[9];
      expPayload.data8 = payload[10];
      expPayload.data9 = payload[11];
      expPayload.data10 = payload[12];
      expPayload.data11 = payload[13];
      expPayload.data12 = payload[14];
      expPayload.data13 = payload[15];
    }

    //print all the variables
    //Serial.print(F("speed: "));
    String msgDisplay = "";
    msgDisplay += payload[0];
    msgDisplay += "  ";
    for (int i = 0; i < expPayload.msgLength; ++i ) {
      msgDisplay += payload[2 + i];
      msgDisplay += "  ";
    }
    Serial.println(msgDisplay);

    //    Serial.print(payload[0]);
    //    Serial.print(F("  "));
    //
    //    for(int i = 0; i<expPayload.msgLength; ++i ){
    //      Serial.print(payload[2+i]);
    //      Serial.print("  ");
    //    }
    //    Serial.println(); // displaying the message
    //TODO gotWord to receivedData and multiple display to the length of the message by words.
    count = pCount; // synchronising the count
  }
  else
  { //listen to the incomming again
    attachInterrupt(1, interruptFunction, FALLING);  //Create interrupt: 0 for pin 2 or 1 for pin 3, the name of the interrupt function or ISR, and condition to trigger interrupt
  }
}  // end of loop

void interruptFunction() {
  detachInterrupt(1); //interrupt on pin D3
  count++; //up the receive counter
}
