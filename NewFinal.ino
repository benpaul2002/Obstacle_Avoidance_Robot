#include <ArduinoHttpClient.h>
#include <ArduinoMqttClient.h>
#include <AESLib.h>
#define TS_ENABLE_SSL // For HTTPS SSL connection

//#include <WiFi101.h>
#include <ArduinoJson.h>
#include <TouchScreen.h>

#include <MotorDriver.h>
#include <SPI.h>
#include <WiFiNINA.h>


//#include "secrets.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros

#define SECRET_SHA1_FINGERPRINT "e9fbd108ae081d28ca3eba673207fa99ec196aad"
const char * fingerprint = SECRET_SHA1_FINGERPRINT;
 

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
/////// WiFi Settings ///////
//char ssid[] = "otspott";
//char pass[] = "esw_team10";
char ssid[] = "weeeee";
char pass[] = "KiaraCoke";
int keyIndex = 0;            // your network key Index number (needed only for WEP)
//WiFiClient  client;

unsigned long myChannelNumber = 1848245;
const char * myWriteAPIKey = "WGU883V6XLSQR7ME";
String cse_ip = "10.42.0.1"; // YOUR IP from ipconfig/ifconfig
String cse_port = "8080";
String serverName = "http://esw-onem2m.iiit.ac.in:443";
String ae = "ESW-ultrasonic-sensor";
String cnt = "node1";
char serverAddress[] = "10.42.0.1";  // server address
int port = 443;

WiFiServer server(80); 
WiFiClient wifi = server.available();
MqttClient mqttclient(wifi);

int status = WL_IDLE_STATUS;

float distanceCm;
int trigPin = 4;
int echoPin = 12;
float duration;
int i = 0;
long onFlag = 0;

#define SOUND_SPEED 0.034


HttpClient client1 = HttpClient(wifi, "esw-onem2m.iiit.ac.in", port);
MotorDriver m;

long duration1;                                                           //duration of ultrasonic pulse
long duration2;
long duration3;
int distanceCm1;                                                          //distance in cm
int distanceCm2;
int distanceCm3;
String distance1, distance2, distance3, dir;

void stop_car(){
  m.motor(1, BACKWARD, 0);
  m.motor(2, FORWARD, 0); 
  m.motor(3, FORWARD, 0);
  m.motor(4, BACKWARD, 0); 

}

void turnRight() {
  m.motor(1, BACKWARD, 0);
  m.motor(2, FORWARD, 0); 
  m.motor(3, FORWARD, 0);
  m.motor(4, BACKWARD, 0); 
  
  m.motor(1, BACKWARD, 200);
  m.motor(2, FORWARD, 200); 
  m.motor(3, FORWARD, 200);
  m.motor(4, BACKWARD, 200);
}

void turnLeft() {
  m.motor(1, FORWARD, 0);
  m.motor(2, BACKWARD, 0); 
  m.motor(3, BACKWARD, 0);
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

void thingspeak_write(String dir){
  ThingSpeak.setField(1, distance1);
  ThingSpeak.setField(2, distance2);
  ThingSpeak.setField(3, distance3);
  ThingSpeak.setField(4, dir);
}

void ping_sensor(){
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);                                                 //give a pulse of 10us on TRIG
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  duration1 = pulseIn(A0, HIGH);                                          //check time elasped in receiving back the pulse on ECHO
  delay(50);
  digitalWrite(A3, LOW);
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);                                                 //give a pulse of 10us on TRIG
  delayMicroseconds(10);
  digitalWrite(A3, LOW);
  duration2 = pulseIn(A2, HIGH);
  delay(50);
  digitalWrite(A5, LOW);
  delayMicroseconds(2);
  digitalWrite(A5, HIGH);
  delayMicroseconds(10);
  digitalWrite(A5, LOW);
  duration3 = pulseIn(A4, HIGH);
  delay(50);

  distanceCm1 = (duration1 * 0.034 )/ 2;                                    //convert to distance in cm
  distanceCm2 = (duration2 * 0.034 )/ 2;
  distanceCm3 = (duration3 * 0.034 )/ 2;

  Serial.println(distanceCm1);
  Serial.println(distanceCm2);
  Serial.println(distanceCm3);
}
void createCI(String& val){
// add the lines in step 3-6 inside this function
  uint8_t key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};
  int i = 0;
  int cond = 0;
  
  char data1[] = "0123456789012345";
  char data2[] = "0123456789012345";
  if (val.length() < 16 )
  {
   
    for(i=0;i<16;i++){
      data1[i]=val[i];
    }
    
    aes256_enc_single(key, data1); # 16 bit block of encrypted data
    
    
  }
  
  else
  { 
    # WILL HAVE TO USE 2 BLOCKS AS DATA LENGTH > 16 BYTES
    cond = 1
    for(i=0;i<val.length();i++){
      data1[i]=val[i];
    }
    while(i<16){
      data1[i]="A";
      i++;
    }
    int j = 0;
    while(i<val.length()){
      data2[j]=val[i];
      j++;
      i++;
    }
    while(j<16){
      data2[j]="A";
      j++;
    }
    aes256_enc_single(key, data1); # both 16 bit blocks of encrypted data
    aes256_enc_single(key, data2); # both 16 bit blocks of encrypted data
  }

  String postData2 = "{\"m2m:cin\": {\"lbl\": [ \"Team-16\" ],\"con\": \"45\"}}";
  Serial.println(postData2);
  String postData = "{\"m2m:cin\": {\"lbl\": [ \"Team-16\" ],\"con\": ";
  postData += "\"";
  postData += String(val);
  postData += "\"";
  postData += "}}";
  Serial.println(postData);
  
  
//  auto var = std::format("{\"m2m:cin\": {\"lbl\": [ \"Team-16\" ],\"con\": \"{}\"}}",val);
//  Serial.println(var);

  
  client1.beginRequest();
  client1.post("/~/in-cse/in-name/Team-16/Node-1/Data");
  client1.sendHeader("Content-Type", "application/json;ty=4");
  client1.sendHeader("X-M2M-Origin", "w961iw:JEhiH5");
  client1.sendHeader("Content-Length", String(postData.length()));
//  client.sendHeader("Connection", "keep-alive");
  client1.beginBody();
  client1.print(postData);
  client1.endRequest();

//  // read the status code and body of the response
  int statusCode = client1.responseStatusCode();
  String response = client1.responseBody();

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
long interval = 30000;

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

void navigate(){
  if(distanceCm2 > distanceCm3 && distanceCm2 <= 1000) {
      turnLeft();
      distance1 = String(distanceCm1);
      distance2 = String(distanceCm2);
      distance3 = String(distanceCm3);
      dir = String(3);
      thingspeak_write(dir);
      delay(1500);
      
    }
    if(distanceCm3 > distanceCm2 && distanceCm3 <= 1000) {
      turnRight();
      distance1 = String(distanceCm1);
      distance2 = String(distanceCm2);
      distance3 = String(distanceCm3);
      dir = String(2);
      thingspeak_write(dir);
      delay(1500);
    }
    ping_sensor();
    if(distanceCm3 >= 1000 ||distanceCm2 >= 1000  )                                                   //if distance less than 25cm display on monitor
    {
  
        stop_car();
        delay(1000);
        goBackward();
        delay(2500);
        stop_car();
        delay(1000);
        navigate();
    }
}

void loop()
{
  Serial.println(onFlag +"running");onFlag = ThingSpeak.readLongField(1856320, 1, "NUAMJ7MY4VAX7V8U");
  while(onFlag == 0) {
    onFlag = ThingSpeak.readLongField(1856320, 1, "NUAMJ7MY4VAX7V8U");
    Serial.println("0 Waiting");
    m.motor(1, FORWARD, 0);
    m.motor(2, FORWARD, 0); 
    m.motor(3, FORWARD, 0);
    m.motor(4, FORWARD, 0);

  }
  ping_sensor();
  m.motor(1, FORWARD, 200);
  m.motor(2, FORWARD, 200); 
  m.motor(3, FORWARD, 200);
  m.motor(4, FORWARD, 200);


  

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval){
    Serial.println("30s over");
    previousMillis = currentMillis;
    String val = "[";
    val += String(distanceCm1);
    val += ",";
    val += String(distanceCm2);
    val += ",";
    val += String(distanceCm3);
    val += "]";
    val += ",";
    val += String(dir);

    Serial.println(val);
    
    createCI(val);
  }
  
  if(distanceCm2 <= 75 || distanceCm3 <= 75 )                                                   //if distance less than 25cm display on monitor
  {

      stop_car();
      delay(1000);
      goBackward();
      delay(2500);
      stop_car();
      delay(1000);
      
      navigate();

      goForward();
      distance1 = String(distanceCm1);
      distance2 = String(distanceCm2);
      distance3 = String(distanceCm3);
      dir = String(1);
      thingspeak_write(dir);

      
  }
//      
  else {
      goForward();
      distance1 = String(distanceCm1);
      distance2 = String(distanceCm2);
      distance3 = String(distanceCm3);
      dir = String(1);
      thingspeak_write(dir);

  }
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
}
