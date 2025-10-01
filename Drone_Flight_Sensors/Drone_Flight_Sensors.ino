#include <SPI.h>      //Used for SPI communication
//#include "printf.h"
#include "RF24.h"     //Used for NF24 RF communication
#include <Wire.h>     //Used for I2C Communication.
#include <TinyGPSPlus.h>    //Used for GPS module. 

#define CE_PIN 4
#define CSN_PIN 5

#define ADC 36

//Gyroscope variables
float RatePitch, RateRoll, RateYaw;   //Write output of the sensor to this variables.
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;      //Calibration values to substract error. 
int RateCalibrationNumber;    //Variable to keep track of values already recorded.

float VoltageConvert;
float ADCValue;

RF24 radio(CE_PIN, CSN_PIN);

struct PayloadStruct 
{
  //char message[15];  // only using 6 characters for TX & RX payloads
  float Roll;
  float Pitch;
  float Yaw;
  float Latitude; 
  float Longitude;
  float Voltage;
  //int Throttle;
  //int Roll;
  //int Pitch;
  //int Yaw;
  //uint8_t
};

PayloadStruct payload;

TinyGPSPlus gps;    // The TinyGPSPlus object

void gyro_signals(void)       //Function used to get data from the gyroscope.
{
  Wire.beginTransmission(0x68);     //Start I2C communication with the gyro using default address of 0x68. 
  Wire.write(0x1A);                 //Access configuration register 26 with address 1A.
  Wire.write(0x05);                 //Select low pass filter of 10 Hz. 
  Wire.endTransmission();           //Terminate communication with gyroscope.  
  Wire.beginTransmission(0x68);     //Start I2C communication with the gyro using default address of 0x68. 
  Wire.write(0x1B);                 //Access gyro configuration register 27 with address 1B.
  Wire.write(0x08);                 //Set the sensitivity scale factor of 65.5 LSB/(degrees/second). [0 0 0 0 1 0 0 0  = 0x08].
  Wire.endTransmission();           //Terminate communication with gyroscope.     
  Wire.beginTransmission(0x68);     //Start I2C communication with the gyro using default address of 0x68. 
  Wire.write(0x43);                 //Access gyro measurement registers storing gyro measurements [registers 43, 44, 45, 46, 47 and 48].     
  Wire.endTransmission();           //Terminate communication with gyroscope.  

  Wire.requestFrom(0x68,6);         //Request 6 bytes to read gyro measurement registers information from 43:48. 

  int16_t GyroX = Wire.read() << 8 | Wire.read();     //Read the gyro measurements around the X axis and combine both values read from registers 43 and 44.
  int16_t GyroY = Wire.read() << 8 | Wire.read();     //Read the gyro measurements around the Y axis and combine both values read from registers 45 and 46.
  int16_t GyroZ = Wire.read() << 8 | Wire.read();     //Read the gyro measurements around the Z axis and combine both values read from registers 47 and 48.

  RateRoll = (float)GyroX / 65.5;       //Convert the measurements from LSB/(degrees/second) to degrees/second.
  RatePitch = (float)GyroY / 65.5;      //Since LSB Scale factor is 65.5 LSB/(degrees/second), just divide by this value to get degrees/second.
  RateYaw = (float)GyroZ / 65.5;        //Do this for X, Y and Z axis. 
}

void setup() 
{
  // put your setup code here, to run once:

  Serial.begin(115200);
  while (!Serial) 
  {
    // some boards need to wait to ensure access to serial over USB
  }

  Serial.println("Setup");  

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
  radio.openWritingPipe(0x0987654321LL);

  Serial2.begin(9600);

  //Start communication with gyroscope. 
  Wire.setClock(400000);          //Set the I2C clock to 400kHz. 
  Wire.begin();                   //Start I2C communication. 
  delay(250);                     //Give the MPU6050 time to start. 
  Wire.beginTransmission(0x68);   //Start I2C communication with the gyro using default address of 0x68. 
  Wire.write(0x6B);               //Access power management register 0x6B.
  Wire.write(0x00);               //Set all bits of register to 0 to make MPU6050 start and stay on. 
  Wire.endTransmission();         //Terminate communication with gyroscope.  

  //For loop that performs the calibration measurements. Don't move quadcopter when calibration is taking place. 
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) 
  {   
    //Read 2000 measurements of gyro. 
    gyro_signals();   //Read value of gyroscope. 
    RateCalibrationRoll += RateRoll;    //Add all measured values to calibration variable. 
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);   //Wait 1 millisecond between reads. 
  }

  RateCalibrationRoll /= 2000;    //Get the medium of all values by dividing values by 2000. 
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

}

void loop() 
{
  Serial.println("Loop");  
  // put your main code here, to run repeatedly:
  
  while (Serial2.available() > 0)
  {
    if (gps.encode(Serial2.read()))
    {
    if (gps.location.isValid())
    {
      payload.Latitude = gps.location.lat(), 6;
      
      payload.Longitude = gps.location.lng(), 6;
      
      //Serial.print(gps.location.lat(), 6);
      //Serial.print(gps.location.lng(), 6);
    }
      
    }
    
  }

  Serial.println(payload.Latitude); 
  Serial.println(payload.Longitude); 

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    //Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  
  gyro_signals();     //Call the predefined function to read the gyro measurements. 
  
  RateRoll -= RateCalibrationRoll;    //Substract calibration error from measurements. 
  RatePitch -= RateCalibrationPitch;  //Correct the measurements. 
  RateYaw -= RateCalibrationYaw;

  payload.Roll = RateRoll;
  payload.Pitch = RatePitch;
  payload.Yaw = RateYaw;

  ADCValue = analogRead(ADC);
  //VoltageConvert = ((ADCValue * 3.3)/(4095));   // Since 3.3V = 2^10 - 1 = 1023 and 1/5 voltage divider gives 1023 / (3.3 * 5) = 62.
  VoltageConvert = (18*((ADCValue * 3.3)/(4095)+0.109)) / 3.178;    //Real Voltage = (Max Voltage Physical * Read Voltage ADC) / Max Voltage ESP32 using voltage divider with R1 of 2164 ohms and R2 of 466 ohms.
  

  payload.Voltage = VoltageConvert;

  Serial.println(payload.Roll); 
  Serial.println(payload.Pitch); 
  Serial.println(payload.Yaw); 

  Serial.println(payload.Voltage); 
  Serial.println(((ADCValue * 3.3)/(4095))); 

  radio.write(&payload, sizeof(payload));

  delay(250);

}
