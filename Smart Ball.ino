

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
SoftwareSerial sim800l(10, 11);
#include <SoftwareSerial.h>
#include <TinyGPS.h>
SoftwareSerial mySerial(7, 8);
TinyGPS gps;



// Create instances of the BMP180 and MPU-6050 sensors
Adafruit_BMP085 bmp;
MPU6050 mpu;


const char *APN = "www";
const char *APN_USERNAME = "";
const char *APN_PASSWORD = "";


const char *URL = "http://rocky-retreat-74526.herokuapp.com/sensor";







int buttonData = 0;

int button = 3;


volatile int flow_frequency;
unsigned int l_hour;
unsigned char flowsensor = 2; // Change this to 9
unsigned long currentTime2;
unsigned long cloopTime2;

void sendATCommand(String command, bool printResponse = true) {

  sim800l.println(command);
  delay(500);

  String response = "";
  while (sim800l.available()) {
    char c = sim800l.read();
    response += c;
  }

  if (response.length() > 0 && printResponse) {
    Serial.println(response);
  }
}

void setupGPRS() {
  sendATCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  sendATCommand("AT+SAPBR=3,1,\"APN\",\"" + String(APN) + "\"");
  if (strlen(APN_USERNAME) > 0) {
    sendATCommand("AT+SAPBR=3,1,\"USER\",\"" + String(APN_USERNAME) + "\"");
  }
  if (strlen(APN_PASSWORD) > 0) {
    sendATCommand("AT+SAPBR=3,1,\"PWD\",\"" + String(APN_PASSWORD) + "\"");
  }

  sendATCommand("AT+SAPBR=1,1");
  delay(5000);
}

void sendHTTPPOSTRequest(int id, float temperature, float pressure, float flow, float acoustic, float ax, float ay, float az, float gx, float gy, float gz, float lat, float lon) {
   String postData = "{\"t\":"+String(temperature)+",\"p\":"+String(pressure)+",\"f\":"+String(flow)+",\"a\":"+String(acoustic)+",\"ax\":"+String(ax)+",\"ay\":"+String(ay)+",\"az\":"+String(az)+",\"gx\":"+String(gz)+",\"gy\":"+String(gy)+",\"gz\":"+String(gz)+",\"lat\":"+String(lat)+",\"lon\":"+String(lon)+"}";

    Serial.print("Sending POST data: ");
    Serial.println(postData);




  sendATCommand("AT+HTTPINIT");
  sendATCommand("AT+HTTPPARA=\"CID\",1");
  sendATCommand("AT+HTTPPARA=\"URL\",\"" + String(URL) + "\"");
  sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  sendATCommand("AT+HTTPDATA=" + String(postData.length()) + ",10000");
  
  delay(1000); // Wait for the ">" prompt

  sendATCommand(postData, false);
  delay(5000); // Wait for the data to be sent

  sendATCommand("AT+HTTPACTION=1");
  delay(5000); // Wait for the response

  

  sendATCommand("AT+HTTPREAD");
  String httpResponse = "";
while (sim800l.available()) {
  char c = sim800l.read();
  httpResponse += c;
}
Serial.print("HTTP response: ");
Serial.println(httpResponse);
  sendATCommand("AT+HTTPTERM");
}

void flow() {
  flow_frequency++;
}

void setup() {


    // Initialize I2C communication
  Wire.begin();
Serial.begin(9600);

  sim800l.begin(9600);
  delay(1000);
  setupGPRS();
  mySerial.begin(9600);
  delay(1000);







  pinMode(A3, INPUT);
  pinMode(A6, INPUT);

    pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);
  Serial.begin(9600);
  attachInterrupt(0, flow, RISING); // Change this to interrupt number 1
  sei();
  currentTime2 = millis();
  cloopTime2 = currentTime2;

  // Initialize serial communication
  



  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {}
  }

  // Initialize MPU-6050 sensor
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU-6050 not connected properly, check wiring!");
    while (1) {}
  }



  


  
}

void loop() {
     long lat, lon;
  float flat, flon;
  if(buttonData == 0){
    
  Serial.println("Press Button to start reading");


  }



  while(buttonData == 0 && analogRead(button) < 1020){}
  buttonData = 1;
  

  float sound = analogRead(6);
  Serial.print("SOUND: ");
  Serial.println(sound);
  currentTime2 = millis();
  if (currentTime2 >= (cloopTime2 + 1000)) {
    cloopTime2 = currentTime2;

    l_hour = (flow_frequency * 60 / 7.5);
    flow_frequency = 0;
    Serial.print(l_hour, DEC);
    Serial.println(" L/hour");
  }


  // Read temperature and pressure from BMP180
  float temperature = bmp.readTemperature();
  long pressure = bmp.readPressure();

  // Read accelerometer and gyroscope data from MPU-6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print data from both sensors
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print("Accel: ");
  Serial.print(ax);
  Serial.print(" ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.println(az);

  Serial.print("Gyro: ");
  Serial.print(gx);
  Serial.print(" ");
  Serial.print(gy);
  Serial.print(" ");
  Serial.println(gz);

  bool newdata = false;
  unsigned long startGPS = millis();
 while (millis() - startGPS < 5000) 
  {
    if (mySerial.available()) 
    
    {
      char c = mySerial.read();
      //Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c)) 
      {
        newdata = true;
        break;  // uncomment to print new data immediately!
      }
    }
  }

  if (newdata) 
  {

  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
 // Serial.print(flat);


  }


 sendHTTPPOSTRequest(123, temperature, pressure, l_hour, sound, ax, ay, az, gx, gy, gz, 0,0);


}


