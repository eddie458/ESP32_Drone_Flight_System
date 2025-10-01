#include <SPI.h>
//#include "printf.h"
#include "RF24.h"
//#include <ezButton.h>

#define CE_PIN 4
#define CSN_PIN 5

#define VRX_PIN_1 36
#define VRY_PIN_1 39

#define VRX_PIN_2 34
#define VRY_PIN_2 35

#define SW_PIN_1   16
#define SW_PIN_2   17

RF24 radio(CE_PIN, CSN_PIN);

float SendValue[] = {0, 0, 0, 0, 0, 0, 0, 0};

struct PayloadStruct 
{
  //char message[15];  // only using 6 characters for TX & RX payloads
  int Throttle;
  int Roll;
  int Pitch;
  int Yaw;
  //uint8_t
};

PayloadStruct payload;

void setup() 
{
  // Serial.begin(115200);

  // while (!Serial) 
  // {
  //   // some boards need to wait to ensure access to serial over USB
  // }
  
  //Serial.println("Setup");
  if (!radio.begin()) 
  {
    //Serial.println(F("Radio hardware is not responding!!"));
    while (1) 
    {
      // hold in infinite loop
    }  
  }
  
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(0x1234567890LL);

  //memcpy(payload.message, "Hello ", 6);
  //payload.counter = 1;
  pinMode(2, OUTPUT);      //Turn on the internal LED to show system on
  digitalWrite(2, HIGH); 
}

void loop() 
{

  // payload.Roll = map(analogRead(VRY_PIN_2), 0, 4095, 2000, 1000);
  // payload.Pitch = map(analogRead(VRX_PIN_2), 0, 4095, 2000, 1000);
  // payload.Throttle = map(analogRead(VRX_PIN_1), 0, 4095, 2000, 1000);
  // payload.Yaw = map(analogRead(VRY_PIN_1), 0, 4095, 2000, 1000);

  SendValue[0] = map(analogRead(VRY_PIN_2), 0, 4095, 2000, 1000);
  SendValue[1] = map(analogRead(VRX_PIN_2), 0, 4095, 2000, 1000);
  SendValue[2] = map(analogRead(VRX_PIN_1), 0, 4095, 2000, 1000);
  SendValue[3] = map(analogRead(VRY_PIN_1), 0, 4095, 2000, 1000);

  if(SendValue[0] >= 1400 && SendValue[0] <= 1600)
  {
    SendValue[0] = 1500;
  }
  if(SendValue[0] < 1400)
  {
    SendValue[0] = map(SendValue[0],1000,1400,1000,1500);
  }
  if(SendValue[0] > 1600)
  {
    SendValue[0] = map(SendValue[0],1600,2000,1500,2000);
  }

  if(SendValue[1] >= 1400 && SendValue[1] <= 1600)
  {
    SendValue[1] = 1500;
  }
  if(SendValue[1] < 1400)
  {
    SendValue[1] = map(SendValue[1],1000,1400,1000,1500);
  }
  if(SendValue[1] > 1600)
  {
    SendValue[1] = map(SendValue[1],1600,2000,1500,2000);
  }

  if(SendValue[3] >= 1400 && SendValue[3] <= 1600)
  {
    SendValue[3] = 1500;
  }
  if(SendValue[3] < 1400)
  {
    SendValue[3] = map(SendValue[3],1000,1400,1000,1500);
  }
  if(SendValue[3] > 1600)
  {
    SendValue[3] = map(SendValue[3],1600,2000,1500,2000);
  }

  // Serial.print("Throttle = ");
  // Serial.print(payload.Throttle);
  // Serial.print("\t\t Roll = ");
  // Serial.print(payload.Roll);
  // Serial.print("\t Pitch = ");
  // Serial.print(payload.Pitch);
  // Serial.print("\t Yaw = ");
  // Serial.println(payload.Yaw);

  payload.Roll = SendValue[0];
  payload.Pitch = SendValue[1];
  payload.Throttle = SendValue[2];
  payload.Yaw = SendValue[3];
  
  //Serial.println("Loop");
  // put your main code here, to run repeatedly:
  radio.write(&payload, sizeof(payload));
  //memcpy(payload.message, "Hello ", 6);  // set the outgoing message
  //radio.stopListening();                 // put radio in TX mode
  //payload.counter = payload.counter + 1;
  //delay(1000);
}
