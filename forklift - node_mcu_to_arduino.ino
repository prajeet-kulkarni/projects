#include <ESP8266WiFi.h>
#include<math.h>
#include<SoftwareSerial.h>

SoftwareSerial send1(13,15);


// WiFi credentials
 char* ssid = "Realme";                    //Enter your wifi hotspot ssid
 char* password ="narzo123";               //Enter your wifi hotspot password
const uint16_t port = 8002;
char* host = "192.168.238.238";                   //Enter the ip address of your laptop after connecting it to wifi hotspot


char incomingPacket[80];
WiFiClient client;

String msg = "0";
String imsg = "0";



void setup()
{
  send1.begin(9600);
  //Serial.begin(9600);
  Serial.begin(115200);                          //Serial to print data on Serial Monitor
  //S.begin(9600);        //Serial to transfer data between ESP and AVR. The Serial connection is inbuilt.
  //S.write(con);
  
  //Connecting to wifi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.println("...");
  }
 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());

  
  
}


void loop() 
{
  if(!client.connect(host, port))
  {
    Serial.println("Connection to host failed");
    delay(200);
    return;
   }
  
  while(1)
  { 
     if(client.available()>0)
     {
                msg = client.readStringUntil('*');
                Serial.println(msg); 
                send1.println(msg);
                
                Serial.print("Sending message to Arduino");
                         
         
     }
       
  }    
}
