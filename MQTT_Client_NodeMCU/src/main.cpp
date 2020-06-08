#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <SoftwareSerial.h>
// #define ssid "Tran Tuan Khai"
// #define password "k1i9e9t90913948951"
SoftwareSerial s(D5,D6);

#define mqtt_server "192.168.1.11" 
#define topic_humi "channel2/humi"
#define topic_temp "channel2/temp"   
#define mqtt_topic_sub "channel1/device1"

unsigned long t=0;
float Temp, Humi;
uint8_t data[5];
char msg_Temp[50];
char msg_Humi[50];
char topic_sub[50];

const uint16_t mqtt_port = 1883; 
const byte ledPin = D0;

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
int value = 0;
// void setup_wifi() {
//   delay(10);
//   Serial.println();
//   Serial.print("Connecting to ");
//   Serial.println(ssid);
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("");
//   Serial.println("WiFi connected");
//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP());
// }

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String tp(topic);
  for (int i = 0; i < length; i++) {
    char receivedChar = (char)payload[i];
    Serial.print(receivedChar);
    // Kiểm tra nếu tin nhận được là 1 thì bật LED và ngược lại
    if (receivedChar == '1')
      { sprintf(topic_sub, "channel1/device1");
      String(topic);
        
       if (tp == "location1/channel1/device1") {
          s.print('A');
          s.print('1');
          s.print('0');
        }

        if (tp == "location1/channel1/device2") {
          s.print('A');
          s.print('2');
          s.print('0');
        }
        
        if (tp == "location2/channel1/device1") {
          s.print('B');
          s.print('1');
          s.print('0');
        }

        if (tp == "location2/channel1/device2") {
          s.print('B');
          s.print('2');
          s.print('0');
        }


      }
      
    if (receivedChar == '0')
    { 
      
      if (tp == "location1/channel1/device1") {
        s.print('A');
        s.print('1');
        s.print('1');
      }
      if (tp == "location1/channel1/device2") {
        s.print('A');
        s.print('2');
        s.print('1');
      }

      if (tp == "location2/channel1/device1") {
        s.print('B');
        s.print('1');
        s.print('1');
      }
      if (tp == "location2/channel1/device2") {
        s.print('B');
        s.print('2');
        s.print('1');
      }

    }
      
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  s.begin(9600);
  //WiFiManager
  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.autoConnect("Kiet dep trai");
  Serial1.println("Connected to the wifi....");

  client.setServer(mqtt_server, mqtt_port); 
  client.setCallback(callback);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
}
// Hàm kết nối wifi

// Hàm call back để nhận dữ liệu

// Hàm reconnect thực hiện kết nối lại khi mất kết nối với MQTT Broker
void reconnect() {
  // Chờ tới khi kết nối
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Thực hiện kết nối với mqtt user và pass
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");

      client.subscribe("location1/channel1/device1");
      client.subscribe("location1/channel1/device2");
      client.subscribe("location2/channel1/device1");
      client.subscribe("location2/channel1/device2");


    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Đợi 5s
      delay(5000);
    }
  }
}
void loop() {
  // Kiểm tra kết nối
  if (!client.connected()) {
    reconnect();
  }
 
 //Get temp & Humi data from Serial port
if (s.available()>=5){
    s.readBytes(data,5);
    if(data[0] == 0x23){
      Temp = (uint16_t)(data[1]<<8) | (data[2] & 0xFC);
      Temp *=175.72f;
      Temp /=65536.0f;
      Temp -=46.85f;
      Humi = (uint16_t)(data[3]<<8) | (data[4] & 0xFC);
      Humi *=125.0f;
      Humi /=65536.0f;
      Humi -=6.0f;
    }
  }

  
//Publish temp & humi every 2 secs to the broker
if(millis()-t>1000){
  sprintf(msg_Temp, "%2.2f", Temp);
  sprintf(msg_Humi, "%2.2f", Humi);
  client.publish(topic_temp, msg_Temp );
  client.publish(topic_humi, msg_Humi );
  t=millis();
  }
  client.loop();
}