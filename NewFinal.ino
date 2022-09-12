#include <ArduinoHttpClient.h>
//#include <WiFi101.h>
#include <ArduinoJson.h>
#include <TouchScreen.h>

#include <MotorDriver.h>
#include <SPI.h>
#include <WiFiNINA.h>

//#include "secrets.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros


///////please enter your sensitive data in the Secret tab/arduino_secrets.h
/////// WiFi Settings ///////
char ssid[] = "Hotspot";
char pass[] = "esw_lab10";

int keyIndex = 0;            // your network key Index number (needed only for WEP)
//WiFiClient  client;

unsigned long myChannelNumber = 1848245;
const char * myWriteAPIKey = "WGU883V6XLSQR7ME";
String cse_ip = "192.168.43.247"; // YOUR IP from ipconfig/ifconfig
String cse_port = "8080";
String serverName = "http://esw-onem2m.iiit.ac.in:443/~/in-cse/in-name/";
String ae = "ESW-ultrasonic-sensor";
String cnt = "node1";
char serverAddress[] = "192.168.43.247";  // server address
int port = 443;

WiFiServer server(80); 
WiFiClient wifi = server.available();

int status = WL_IDLE_STATUS;

float distanceCm;
int trigPin = 4;
int echoPin = 12;
float duration;
int i = 0;

#define SOUND_SPEED 0.034


HttpClient client = HttpClient(wifi, "esw-onem2m.iiit.ac.in", port);
MotorDriver m;

long duration1;                                                           //duration of ultrasonic pulse
long duration2;
long duration3;
int distanceCm1;                                                          //distance in cm
int distanceCm2;
int distanceCm3;
String distance1, distance2, distance3, dir;

void turnRight() {
  m.motor(1, FORWARD, 0);
  m.motor(2, FORWARD, 0); 
  m.motor(3, FORWARD, 0);
  m.motor(4, FORWARD, 0);  
  m.motor(1, BACKWARD, 200);
  m.motor(2, FORWARD, 200); 
  m.motor(3, FORWARD, 200);
  m.motor(4, BACKWARD, 200);
}

void turnLeft() {
  m.motor(1, FORWARD, 0);
  m.motor(2, FORWARD, 0); 
  m.motor(3, FORWARD, 0);
  m.motor(4, FORWARD, 0);  

  m.motor(1, FORWARD, 200);
  m.motor(2, BACKWARD, 200); 
  m.motor(3, BACKWARD, 200);
  m.motor(4, FORWARD, 200);
}

void goForward() {
  m.motor(1, FORWARD, 200);
  m.motor(2, FORWARD, 200);
  m.motor(3, FORWARD, 200);
  m.motor(4, FORWARD, 200);  
}

void goBackward() {
  m.motor(1, BACKWARD, 200);
  m.motor(2, BACKWARD, 200);
  m.motor(3, BACKWARD, 200);
  m.motor(4, BACKWARD, 200);  
}






void createCI(String& val){
// add the lines in step 3-6 inside this function


  String postData = "{\"m2m:cin\": {\"lbl\": [ \"Team-16\" ],\"con\": \"" + String(val)+ "\"}}";

  Serial.println(postData);
  client.beginRequest();
  client.post("/~/in-cse/in-name/Team-16/Node-1/Data");
  client.sendHeader("Content-Type", "application/json;ty=4");
  client.sendHeader("X-M2M-Origin", "w961iw:JEhiH5");
//  client.sendHeader("Content-Length", String(postData.length()));
  client.sendHeader("Connection", "keep-alive");
  client.beginBody();
  client.print(postData);
  client.endRequest();

//  // read the status code and body of the response
  int statusCode = client.responseStatusCode();
  String response = client.responseBody();

  Serial.print("Status code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void enable_WiFi() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }
}

void connect_WiFi() {
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(500);
  }
}

// time code
long previousMillis = 0;
long interval = 5000;

void setup() {
  pinMode(A1, OUTPUT);                                                    //Analog pin A1 connected to TRIG
  pinMode(A0, INPUT);                                                     //Analog pin A0 connected to ECHO
  pinMode(A3, OUTPUT);
  pinMode(A2, INPUT);
  pinMode(A5, OUTPUT);
  pinMode(A4, INPUT);

  Serial.begin(9600);
  enable_WiFi();
  connect_WiFi();

  server.begin();
  printWifiStatus();
  
  
  ThingSpeak.begin(wifi);

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  
}

void loop()
{
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);                                                 //give a pulse of 10us on TRIG
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  duration1 = pulseIn(A0, HIGH);                                          //check time elasped in receiving back the pulse on ECHO
  delay(100);
  digitalWrite(A3, LOW);
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);                                                 //give a pulse of 10us on TRIG
  delayMicroseconds(10);
  digitalWrite(A3, LOW);
  duration2 = pulseIn(A2, HIGH);
  delay(100);
  digitalWrite(A5, LOW);
  delayMicroseconds(2);
  digitalWrite(A5, HIGH);
  delayMicroseconds(10);
  digitalWrite(A5, LOW);
  duration3 = pulseIn(A4, HIGH);
  delay(100);
  m.motor(1, FORWARD, 200);
  m.motor(2, FORWARD, 200); 
  m.motor(3, FORWARD, 200);
  m.motor(4, FORWARD, 200);
  distanceCm1 = (duration1 * 0.034 )/ 2;                                    //convert to distance in cm
  distanceCm2 = (duration2 * 0.034 )/ 2;
  distanceCm3 = (duration3 * 0.034 )/ 2;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval){
    Serial.println("30s over");
    previousMillis = currentMillis;
    String val = String(distanceCm1);
    createCI(val);
  }
  
  if(distanceCm1 <= 10)                                                   //if distance less than 20cm display on monitor
  {
      if(distanceCm2 <= 10 && distanceCm3 <= 10) {
          goBackward();
          distance1 = String(distanceCm1);
          distance2 = String(distanceCm2);
          distance3 = String(distanceCm3);
          dir = String(4);
          ThingSpeak.setField(1, distance1);
          ThingSpeak.setField(2, distance2);
          ThingSpeak.setField(3, distance3);
          ThingSpeak.setField(4, "4");
      }
      else if(distanceCm3 <= 10) {
          turnLeft();
          distance1 = String(distanceCm1);
          distance2 = String(distanceCm2);
          distance3 = String(distanceCm3);
          dir = String(3);
          ThingSpeak.setField(1, distance1);
          ThingSpeak.setField(2, distance2);
          ThingSpeak.setField(3, distance3);
          ThingSpeak.setField(4, "3");
      }
      else if(distanceCm2 <= 10) {
        turnRight();
        distance1 = String(distanceCm1);
        distance2 = String(distanceCm2);
        distance3 = String(distanceCm3);
        dir = String(2);
        ThingSpeak.setField(1, distance1);
        ThingSpeak.setField(2, distance2);
        ThingSpeak.setField(3, distance3);
        ThingSpeak.setField(4, "2");
      }
  }
  else {
      goForward();
      distance1 = String(distanceCm1);
      distance2 = String(distanceCm2);
      distance3 = String(distanceCm3);
      dir = String(1);
      ThingSpeak.setField(1, distance1);
      ThingSpeak.setField(2, distance2);
      ThingSpeak.setField(3, distance3);
      ThingSpeak.setField(4, "1");
  }
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
}
