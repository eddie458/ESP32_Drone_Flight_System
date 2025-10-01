#include <Wire.h>
//#include <PulsePosition.h>
#include <BasicLinearAlgebra.h>
#include <SPI.h>      //Used for SPI communication
#include "printf.h"   //Used for serial print format
#include "RF24.h"     //Used for NF24 RF communication

using namespace BLA;
//PulsePositionInput ReceiverInput(RISING);

//SPI GPIO
#define CE_PIN 4
#define CSN_PIN 5

#define ADC 36

//Read PPM Variables
byte interruptPin = 13;
unsigned long previousMicros = 0;
unsigned long microsAtLastPulse = 0;
unsigned long time_read = 0;
unsigned long previousTime_read = 0;
int minValue = 1000;
int maxValue = 2000;
int defaultValue = 1000;
int blankTime = 2100;
int pulseCounter = 0;
int channelAmount = 4;
unsigned long rawValues[] = {0, 0, 0, 0, 0, 0, 0, 0};

//PWM GPIO
const int ledPin1 = 26; //ESC 1 - Top Right
const int ledPin2 = 25; //ESC 2 - Botton Right
const int ledPin3 = 33; //ESC 3 - Botton Left
const int ledPin4 = 32; //ESC 4 - Top Left

//PWM variables
const int freq = 250;
const int resolution = 12;

const int ledChannel1 = 0;
const int ledChannel2 = 1;
const int ledChannel3 = 2;
const int ledChannel4 = 3;

//Gyroscope variables
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

//Receiver variables
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0; 

//Powerswitch variables
float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed=0;
float BatteryDefault=1300;

//Define the parameter containing the length of each control loop.
uint32_t LoopTimer;

//All variables necessary for the PID control loop are declared in this part, including the values for the P, I and D parameters.
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

//PID control loop output. 
float PIDReturn[]={0, 0, 0};

//Each PID control loop contains 3 variables and 3 constants. 
float PRateRoll=0.6; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03; float DRatePitch=DRateRoll; float DRateYaw=0;

//Declare the input variables to the motors.
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t  dig_P6, dig_P7, dig_P8, dig_P9; 
float AltitudeBarometer, AltitudeBarometerStartUp;
float AccZInertial;
float AltitudeKalman, VelocityVerticalKalman;
BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;
float DesiredVelocityVertical, ErrorVelocityVertical;
float PVelocityVertical=3.5; float IVelocityVertical=0.0015; float DVelocityVertical=0.01; 
float PrevErrorVelocityVertical, PrevItermVelocityVertical;

//RF24 radio(CE_PIN, CSN_PIN);

// struct PayloadStruct 
// {
//   //char message[15];  // only using 6 characters for TX & RX payloads
//   int Throttle;
//   int Roll;
//   int Pitch;
//   int Yaw;
//   //uint8_t
// };

//PayloadStruct received;

void IRAM_ATTR isr() 
{
  previousTime_read = time_read;
  previousMicros = microsAtLastPulse;
  microsAtLastPulse = micros();
  time_read = microsAtLastPulse - previousMicros;
  
  // if (time_read > maxValue || time_read < minValue)
  // {
  //   time_read = previousTime_read;
  // }
  // else time_read = time_read;
  if (time_read > blankTime) 
  {
    // Blank detected: restart from channel 1 
    pulseCounter = 0;
  }
  else 
  {
    // Store times between pulses as channel values
    if (pulseCounter < channelAmount) 
    {
      rawValues[pulseCounter] = time_read;
      ++pulseCounter;
    }
  }
}

void kalman_2d(void)
{
  Acc = {AccZInertial};
  S=F*S+G*Acc;
  P=F*P*~F+Q;
  L=H*P*~H+R;
  K=P*~H*Invert(L); 
  M = {AltitudeBarometer};
  S=S+K*(M-H*S);
  AltitudeKalman=S(0,0); 
  VelocityVerticalKalman=S(1,0); 
  P=(I-K*H)*P;
}

void barometer_signals(void)
{
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76,6);
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int )dig_T1 <<1)))* ((signed long int )dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T>>4) - ((signed long int )dig_T1)))>> 12) * ((signed long int )dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )dig_P6);
  var2 = var2 + ((var1*((signed long int )dig_P5)) <<1);
  var2 = (var2>>2)+(((signed long int )dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13 ))>>3)+((((signed long int )dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int )dig_P1)) >>15);
  if (var1 == 0) { p=0;}    
  p = (((unsigned long int )(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
  else { p = (p / (unsigned long int )var1) * 2;  }
  var1 = (((signed long int )dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((signed long int )(p>>2)) * ((signed long int )dig_P8))>>13;
  p = (unsigned long int)((signed long int )p + ((var1 + var2+ dig_P7) >> 4));
  double pressure=(double)p/100;
  AltitudeBarometer=44330*(1-pow(pressure
     /1013.25, 1/5.255))*100;
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) 
{
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (
  KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void battery_voltage(void) 
{
  Voltage = (float)analogRead(ADC) / 62;   //Read ADC input from PIN 15 from Teensy (Change to ADC pin of ESP32). Since 3.3V = 2^10 - 1 = 1023 and 1/5 voltage divider gives 1023 / (3.3 * 5) = 62.
  //Since 3.3V = 2^12 - 1 = 4095 and 1/5 voltage divider gives 4095 / (3.3 * 5) = 62.
}

void read_receiver(void)
{
  // ChannelNumber = ReceiverInput.available();  
  // if (ChannelNumber > 0) {
  //        for (int i=1; i<=ChannelNumber;i++){
  //           ReceiverValue[i-1]=ReceiverInput.read(i);
  //        }
  // }

  // if(radio.available())
  // {
  //   PayloadStruct received;
    
  //   radio.read(&received, sizeof(received));
  //   //Serial.println(received.message + received.counter);
  //   //Serial.println(received.message);  // print incoming message
  //   //Serial.println(received.counter);  // print incoming counter

  //   // Serial.print("Throttle 1 = ");
  //   // Serial.print(received.Throttle);
  //   // Serial.print("\t\t Roll 1 = ");
  //   // Serial.print(received.Roll);
  //   // Serial.print("\t Pitch 1 = ");
  //   // Serial.print(received.Pitch);
  //   // Serial.print("\t Yaw 1 = ");
  //   // Serial.println(received.Yaw);

  //   ReceiverValue[0] = received.Roll;
  //   ReceiverValue[1] = received.Pitch;
  //   ReceiverValue[2] = received.Throttle;
  //   ReceiverValue[3] = received.Yaw;

  //   //ledcWrite(ledChannel, 1.024 * received.Throttle);

  // }
  
  for (byte channel = 1; channel <= channelAmount; ++channel) 
  {
    //Serial.print("Ch");
    //Serial.print(channel);
    //Serial.print(": ");
    //Serial.print(rawValues[channel - 1]);
    //Serial.print(value);

    ReceiverValue[0] = rawValues[0];
    ReceiverValue[1] = rawValues[1];
    ReceiverValue[2] = rawValues[2];
    ReceiverValue[3] = rawValues[3];
    
    //if(channel < channelAmount)
    //{
    //  //Serial.print('\t');
    //} 

  }  
  //Serial.println();

}

void gyro_signals(void) 
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096 - 0.03;          //Calibration 
  AccY=(float)AccYLSB/4096 + 0.03;          //Calibration
  AccZ=(float)AccZLSB/4096 - 0.09;          //Calibration
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) 
{
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void) 
{
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
  PrevErrorVelocityVertical=0; 
  PrevItermVelocityVertical=0;
}

void setup() 
{

  // Serial.begin(115200);

  pinMode(interruptPin, INPUT);
	attachInterrupt(interruptPin, isr, RISING);

  //Serial.println("Setup");

  ledcSetup(ledChannel1, freq, resolution);
  ledcAttachPin(ledPin1, ledChannel1);

  ledcSetup(ledChannel2, freq, resolution);
  ledcAttachPin(ledPin2, ledChannel2);

  ledcSetup(ledChannel3, freq, resolution);
  ledcAttachPin(ledPin3, ledChannel3);

  ledcSetup(ledChannel4, freq, resolution);
  ledcAttachPin(ledPin4, ledChannel4);

  //Make motors stop beeping and disrupting calibration. 
  ledcWrite(ledChannel1, 1000);
  ledcWrite(ledChannel2, 1000);
  ledcWrite(ledChannel3, 1000);
  ledcWrite(ledChannel4, 1000);

  delay(4000);   //Wait 4 seconds for ESCs and motors to stop vibrating. 

  // pinMode(5, OUTPUT);
  // digitalWrite(5, HIGH);
  // pinMode(13, OUTPUT);
  // digitalWrite(13, HIGH);

  //Start communication with gyroscope. 
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);   
  Wire.endTransmission();
  Wire.beginTransmission(0x76); 
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();   
  Wire.beginTransmission(0x76);
  Wire.write(0xF5); 
  Wire.write(0x14);
  Wire.endTransmission();   
  uint8_t data[24], i=0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76,24);  

  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  } 

  dig_T1 = (data[1] << 8) | data[0]; 
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6]; 
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11]<< 8) | data[10];
  dig_P4 = (data[13]<< 8) | data[12];
  dig_P5 = (data[15]<< 8) | data[14];
  dig_P6 = (data[17]<< 8) | data[16];
  dig_P7 = (data[19]<< 8) | data[18];
  dig_P8 = (data[21]<< 8) | data[20];
  dig_P9 = (data[23]<< 8) | data[22]; delay(250);
  
  for (RateCalibrationNumber=0; 
        RateCalibrationNumber<2000;
        RateCalibrationNumber ++) {
          gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    barometer_signals();
    AltitudeBarometerStartUp+=
        AltitudeBarometer; delay(1);
  }

  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  AltitudeBarometerStartUp/=2000;

  // if (!radio.begin()) 
  // {
  //   //Serial.println(F("Radio hardware is not responding!!"));
  //   while (1) 
  //   {
  //     // hold in infinite loop
  //   }  
  // }
  
  // radio.setChannel(5);
  // radio.setDataRate(RF24_1MBPS);
  // radio.setPALevel(RF24_PA_LOW);
  // radio.openReadingPipe(1, 0x1234567890LL);
  // radio.startListening();
  
  F = {1, 0.004,
            0, 1};  
  G = {0.5*0.004*0.004,
            0.004};
  H = {1, 0};
  I = {1, 0,
           0, 1};
  Q = G * ~G*10*10;
  R = {30*30};
  P = {0, 0,
           0, 0};
  S = {0,
           0};

  // analogWriteFrequency(1, 250);
  // analogWriteFrequency(2, 250);
  // analogWriteFrequency(3, 250);
  // analogWriteFrequency(4, 250);
  // analogWriteResolution(12);

  // pinMode(6, OUTPUT);
  // digitalWrite(6, HIGH);

  battery_voltage();

  if (Voltage > 7.5) 
  { 
    digitalWrite(16, LOW);   //The red LED at pin 12 is turned off. 
  }

  // if (Voltage > 8.3) { digitalWrite(5, LOW); BatteryAtStart=BatteryDefault; }
  // else if (Voltage < 7.5) {BatteryAtStart=30/100*BatteryDefault ;}
  // else { digitalWrite(5, LOW);BatteryAtStart=(82*Voltage-580)/100*BatteryDefault; }
  // ReceiverInput.begin(14);
  // while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050) {
  //   read_receiver();
  //   delay(4);
  // }

  pinMode(2, OUTPUT);      //Turn on the internal LED to show system on
  digitalWrite(2, HIGH); 

  read_receiver();     //Start reading PPM stream from PIN 14. 

  //Serial.println("Move Controller");

  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050)  //If throttle is not in lowest position. 
  {
    read_receiver();

    // Serial.print(ReceiverValue[0]);
    // Serial.print('\t');
    // Serial.print(ReceiverValue[1]);
    // Serial.print('\t');
    // Serial.print(ReceiverValue[2]);
    // Serial.print('\t');
    // Serial.print(ReceiverValue[3]);
    // Serial.println();

    // Serial.print("Throttle 2 = ");
    // Serial.print(ReceiverValue[2]);
    // Serial.print("\t\t Roll 2 = ");
    // Serial.print(ReceiverValue[0]);
    // Serial.print("\t Pitch 2 = ");
    // Serial.print(ReceiverValue[1]);
    // Serial.print("\t Yaw 2 = ");
    // Serial.println(ReceiverValue[3]);

    delay(4);
  }

  //Illuminate green led to show off that setup process is finished. 
  pinMode(17, OUTPUT);     //Turn off the red LED and turn on the green LED
  digitalWrite(17, HIGH);

  LoopTimer=micros();
}

void loop() 
{
  //Serial.println("Loop");
  gyro_signals();

  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  AccZInertial=-sin(AnglePitch*(3.142/180))*AccX+cos(AnglePitch*(3.142/180))*sin(AngleRoll*(3.142/180))* AccY+cos(AnglePitch*(3.142/180))*cos(AngleRoll*(3.142/180))*AccZ;   
  AccZInertial=(AccZInertial-1)*9.81*100;
  barometer_signals();
  AltitudeBarometer-=AltitudeBarometerStartUp;
  kalman_2d();

  read_receiver();

  // Serial.print(ReceiverValue[0]);
  // Serial.print('\t');
  // Serial.print(ReceiverValue[1]);
  // Serial.print('\t');
  // Serial.print(ReceiverValue[2]);
  // Serial.print('\t');
  // Serial.print(ReceiverValue[3]);
  // Serial.println();

  DesiredAngleRoll=0.10*(ReceiverValue[0]-1500);
  DesiredAnglePitch=0.10*(ReceiverValue[1]-1500);
  DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
  DesiredVelocityVertical=0.3*(ReceiverValue[2]-1500);

  ErrorVelocityVertical=DesiredVelocityVertical-VelocityVerticalKalman;

  pid_equation(ErrorVelocityVertical, PVelocityVertical, IVelocityVertical, DVelocityVertical, PrevErrorVelocityVertical, PrevItermVelocityVertical);
  InputThrottle=1500+PIDReturn[0]; 
  PrevErrorVelocityVertical=PIDReturn[1]; 
  PrevItermVelocityVertical=PIDReturn[2];
  ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
  ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;

  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll,PrevItermAngleRoll);     
  DesiredRateRoll=PIDReturn[0]; 
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];

  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch=PIDReturn[0]; 
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];
  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;
  
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll=PIDReturn[0];
  PrevErrorRateRoll=PIDReturn[1]; 
  PrevItermRateRoll=PIDReturn[2];
  
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch=PIDReturn[0]; 
  PrevErrorRatePitch=PIDReturn[1]; 
  PrevItermRatePitch=PIDReturn[2];
  
  pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw,PrevItermRateYaw);
  InputYaw=PIDReturn[0]; 
  PrevErrorRateYaw=PIDReturn[1]; 
  PrevItermRateYaw=PIDReturn[2];

  if (InputThrottle > 1800) InputThrottle = 1800;
  MotorInput1= 1.024*(InputThrottle-InputPitch-InputRoll-InputYaw);
  MotorInput2= 1.024*(InputThrottle+InputPitch-InputRoll+InputYaw);
  MotorInput3= 1.024*(InputThrottle+InputPitch+InputRoll-InputYaw);
  MotorInput4= 1.024*(InputThrottle-InputPitch+InputRoll+InputYaw);

  if (MotorInput1 > 2000)MotorInput1 = 1999;
  if (MotorInput2 > 2000)MotorInput2 = 1999; 
  if (MotorInput3 > 2000)MotorInput3 = 1999; 
  if (MotorInput4 > 2000)MotorInput4 = 1999;

  int ThrottleIdle=1180;

  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  int ThrottleCutOff=1000;

  if (ReceiverValue[2]<1050) 
  {
    MotorInput1=ThrottleCutOff; 
    MotorInput2=ThrottleCutOff;
    MotorInput3=ThrottleCutOff; 
    MotorInput4=ThrottleCutOff;
    reset_pid();
  }

  ledcWrite(ledChannel1, MotorInput1);
  ledcWrite(ledChannel2, MotorInput2);
  ledcWrite(ledChannel3, MotorInput3);
  ledcWrite(ledChannel4, MotorInput4);

  battery_voltage();
  if (Voltage < 7.5) 
  {
    digitalWrite(16, HIGH);
  }
  else 
  {
    digitalWrite(16, LOW);
  }

  // CurrentConsumed=Current*1000*0.004/3600+CurrentConsumed;
  // BatteryRemaining=(BatteryAtStart-CurrentConsumed)/BatteryDefault*100;
  // if (BatteryRemaining<=30) digitalWrite(5, HIGH);
  // else digitalWrite(5, LOW);

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}
