#include <SPI.h>      //Used for SPI communication
#include "printf.h"   //Used for serial print format
#include "RF24.h"     //Used for NF24 RF communication
#include <Wire.h>     //Used for I2C Communication.
#include <Adafruit_GFX.h>       //Used for OLED display.
#include <Adafruit_SSD1306.h>   //Used for OLED display.
#include <LiquidCrystal_I2C.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <Arduino.h>
#include <AsyncTCP.h>
#include "SPIFFS.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// // Replace with your network credentials
// const char* ssid = "Turbo_Casita";
// const char* password = "stmEKq&jcBA@";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ip";
const char* PARAM_INPUT_4 = "gateway";

//Variables to save values from HTML form
String ssid;
String pass;
String ip;
String gateway;

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ipPath = "/ip.txt";
const char* gatewayPath = "/gateway.txt";

IPAddress localIP;
//IPAddress localIP(192, 168, 1, 200); // hardcoded

// Set your Gateway IP address
IPAddress localGateway;
//IPAddress localGateway(192, 168, 1, 1); //hardcoded
IPAddress subnet(255, 255, 0, 0);

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

//SPI GPIO
#define CE_PIN 4
#define CSN_PIN 5

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

PayloadStruct received;

float RollRead = 0;
float PitchRead = 0;
float YawRead = 0;
float LatitudeRead = 0;
float LongitudeRead = 0;
float VoltageRead = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">

<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  
  <style>
    html 
    {
      font-family: Century Gothic; 
      display:inline-block; 
      margin: 0px auto; 
      text-align: center; 
      background-color: OldLace;
    }
    h1
    {
      color: #0F3376; 
      padding: 2vh;
    }
    p
    {
      font-size: 1.5rem;
    }
  </style>

</head>

<body>
  <h1>ESP32 Drone Server</h1>
  <h2>Roll: </h2>
  <p>
    <span id="Roll">%Roll%</span>
  </p>
  <h2>Pitch: </h2>
  <p>
    <span id="Pitch">%Pitch%</span>
  </p>
  <h2>Yaw: </h2>
  <p>
    <span id="Yaw">%Yaw%</span>
  </p>
  <h2>Latitude: </h2>
  <p>
    <span id="Latitude">%Latitude%</span>
  </p>
  <h2>Longitude: </h2>
  <p>
    <span id="Longitude">%Longitude%</span>
  </p>
  <h2>Voltage: </h2>
  <p>
    <span id="Voltage">%Voltage%</span>
  </p>

</body>

<script>
setInterval(function ( ) 
{
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() 
  {
    if (this.readyState == 4 && this.status == 200) 
    {
      document.getElementById("Roll").innerHTML = this.responseText;
    }
  };

  xhttp.open("GET", "/Roll", true);
  xhttp.send();
}, 3000 ) ;

setInterval(function ( ) 
{
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() 
  {
    if (this.readyState == 4 && this.status == 200) 
    {
      document.getElementById("Pitch").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/Pitch", true);
  xhttp.send();
}, 3000 ) ;

setInterval(function ( ) 
{
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() 
  {
    if (this.readyState == 4 && this.status == 200) 
    {
      document.getElementById("Yaw").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/Yaw", true);
  xhttp.send();
}, 3000 ) ;

setInterval(function ( ) 
{
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() 
  {
    if (this.readyState == 4 && this.status == 200) 
    {
      document.getElementById("Latitude").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/Latitude", true);
  xhttp.send();
}, 3000 ) ;

setInterval(function ( ) 
{
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() 
  {
    if (this.readyState == 4 && this.status == 200) 
    {
      document.getElementById("Longitude").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/Longitude", true);
  xhttp.send();
}, 3000 ) ;

setInterval(function ( ) 
{
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() 
  {
    if (this.readyState == 4 && this.status == 200) 
    {
      document.getElementById("Voltage").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/Voltage", true);
  xhttp.send();
}, 3000 ) ;

</script>
</html>
)rawliteral";

String readRoll() {
  float a = RollRead;
  //Serial.println(h);
  return String(a);
}

String readPitch() {
  float b = PitchRead;
  //Serial.println(h);
  return String(b);
}

String readYaw() {
  float c = YawRead;
  //Serial.println(h);
  return String(c);
}

String readLatitude() {
  float d = LatitudeRead;
  //Serial.println(h);
  return String(d);
}

String readLongitude() {
  float e = LongitudeRead;
  //Serial.println(h);
  return String(e);
}

String readVoltage() {
  float f = VoltageRead;
  //Serial.println(h);
  return String(f);
}

// Replaces placeholder with values
String processor(const String& var){
  //Serial.println(var);
  if(var == "Roll"){
    return readRoll();
  }
  else if(var == "Pitch"){
    return readPitch();
  }
  else if(var == "Yaw"){
    return readYaw();
  }
  else if(var == "Latitude"){
    return readLatitude();
  }
  else if(var == "Longitude"){
    return readLongitude();
  }
  else if(var == "Voltage"){
    return readVoltage();
  }
  return String();
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
}

// Initialize WiFi
bool initWiFi() {
  if(ssid=="" || ip==""){
    Serial.println("Undefined SSID or IP address.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  localIP.fromString(ip.c_str());
  localGateway.fromString(gateway.c_str());


  if (!WiFi.config(localIP, localGateway, subnet)){
    Serial.println("STA Failed to configure");
    return false;
  }
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.println("Connecting to WiFi...");

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while(WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("Failed to connect.");
      return false;
    }
  }

  Serial.println(WiFi.localIP());
  return true;
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

  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();

  //LCD Message
  lcd.clear(); 
  lcd.setCursor(0, 0);
  lcd.print("Program Starting");
  delay(5000);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
  Serial.println(F("SSD1306 allocation failed"));
  for(;;);
  }

  initSPIFFS();

  // Load values saved in SPIFFS
  ssid = readFile(SPIFFS, ssidPath);
  pass = readFile(SPIFFS, passPath);
  ip = readFile(SPIFFS, ipPath);
  gateway = readFile (SPIFFS, gatewayPath);
  Serial.println(ssid);
  Serial.println(pass);
  Serial.println(ip);
  Serial.println(gateway);

  // // Connect to Wi-Fi
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi..");
  // }

  // // Print ESP32 Local IP Address
  // Serial.println(WiFi.localIP());

  if (!radio.begin()) 
  {
    //Serial.println(F("Radio hardware is not responding!!"));
    while (1) 
    {
      // hold in infinite loop
    }  
  }

  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, 0x0987654321LL);
  radio.startListening();

  //LCD Message
  lcd.clear(); 
  lcd.setCursor(0, 0);
  lcd.print("Setup Finished");
  delay(5000);

  //LCD Message
  lcd.clear(); 
  lcd.setCursor(0, 0);
  lcd.print("Starting WIFI");
  delay(5000);

  if(initWiFi()) 
  {
    //LCD Message
    lcd.clear(); 
    lcd.setCursor(0, 0);
    lcd.print("WIFI Connected");
    lcd.setCursor(0, 1);
    lcd.print("Starting Server on:");
    lcd.setCursor(0, 2);
    lcd.print(ip);
    delay(10000);

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
    });
    server.on("/Roll", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readRoll().c_str());
    });
    server.on("/Pitch", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readPitch().c_str());
    });
    server.on("/Yaw", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readYaw().c_str());
    });
    server.on("/Latitude", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readLatitude().c_str());
    });
    server.on("/Longitude", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readLongitude().c_str());
    });
    server.on("/Voltage", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readVoltage().c_str());
    });

    // Start server
    server.begin();
  }
  else 
  {
    // Connect to Wi-Fi network with SSID and password
    Serial.println("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER", NULL);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP); 

    //LCD Message
    lcd.clear(); 
    lcd.setCursor(0, 0);
    lcd.print("Can't connect WIFI");
    lcd.setCursor(0, 1);
    lcd.print("Go to WIFI Manager:");
    lcd.setCursor(0, 2);
    lcd.print(IP);
    delay(10000);

    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });
    
    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid);
            // Write file to save value
            writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Write file to save value
            writeFile(SPIFFS, passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip);
            // Write file to save value
            writeFile(SPIFFS, ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway);
            // Write file to save value
            writeFile(SPIFFS, gatewayPath, gateway.c_str());
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + ip);
      delay(3000);
      ESP.restart();
    });
    server.begin();
  }
  
  //LCD Message
    lcd.clear(); 
    lcd.setCursor(0, 0);
    lcd.print("Setup Completed");
    delay(5000);

  lcd.clear(); 
}

void loop() 
{
  Serial.println("Loop");  
  // put your main code here, to run repeatedly:
  if(radio.available())
  {
    PayloadStruct received;
    radio.read(&received, sizeof(received));
    //Serial.println(received.message + received.counter);
    RollRead = received.Roll;
    PitchRead = received.Pitch;
    YawRead = received.Yaw;
    LatitudeRead = received.Latitude;
    LongitudeRead = received.Longitude;
    VoltageRead = received.Voltage;
    
  }

  Serial.println(RollRead);  // print incoming message
  Serial.println(PitchRead);  // print incoming counter
  Serial.println(YawRead);  // print incoming message
  Serial.println(LatitudeRead);  // print incoming counter
  Serial.println(LongitudeRead);  // print incoming message
  Serial.println(VoltageRead); 

  display.clearDisplay();
  // Display static text
  display.setTextSize(1.5);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.print("Roll: ");
  display.println(RollRead);

  display.setTextSize(1.5);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.print("Pitch: ");
  display.println(PitchRead);

  display.setTextSize(1.5);
  display.setTextColor(WHITE);
  display.setCursor(0, 30);
  display.print("Yaw: ");
  display.println(YawRead);
  display.display(); 

  //lcd.setCursor(14,0);
  //lcd.print("   ");
  lcd.setCursor(0, 0);
  lcd.print("Latitude: ");
  lcd.print(LatitudeRead);

  //lcd.setCursor(15,1);
  //lcd.print("   ");
  lcd.setCursor(0,1);
  lcd.print("Longitude: ");
  lcd.print(LongitudeRead);

  //lcd.setCursor(13,2);
  //lcd.print("   ");
  lcd.setCursor(0,2);
  lcd.print("Voltage: ");
  lcd.print(VoltageRead);

  delay(250);
}
