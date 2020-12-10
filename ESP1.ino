#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
SoftwareSerial s(D6, D5);

WiFiClient client;
WiFiServer server(80);
const String tenwifi = "CCC";
const String mkwifi = "98765431";
String data="";
int dulieu;

void setup() {
  Serial.begin(115200);
  s.begin(9600);
  pinMode(16,OUTPUT);
  pinMode(2,OUTPUT); 
  digitalWrite(16,LOW);
  digitalWrite(2,LOW);
  Serial.print("Ket noi den wifi ");
  Serial.println(tenwifi);
  WiFi.begin(tenwifi,mkwifi);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi da duoc ket noi");
  Serial.println("Dia chi IP la: ");
  Serial.println(WiFi.localIP());
  server.begin();    
}

void loop() {
  client = server.available();
  if(!client){
    return;
  }
  data = checkClient();
  Serial.println(data);
  if(data == "left"){
    digitalWrite(2,LOW);
    dulieu = 1;
  }
  if(data == "right"){
    digitalWrite(2,HIGH);
    dulieu = 2;
  }
  if(data == "up"){
    digitalWrite(16,LOW);
    dulieu = 3;
  }
  if(data == "down"){
    digitalWrite(16,HIGH);
    dulieu = 4;
  }
  if(data == "stop"){
    digitalWrite(16,HIGH);
    dulieu = 5;
  }
  if(data == "BAN1"){
    dulieu = 6;
  }
  if(data == "BAN2"){
    dulieu = 7;
  }
    if(data == "BAN3"){
    dulieu = 8;
  }
    if(data == "BAN4"){
    dulieu = 9;
  }
  Serial.println(dulieu);
  if (s.available() > 0)
  {
    s.write(dulieu);
    delay(1000);
  }
  
  
}
String checkClient(){
  while(!client.available())
  {
    delay(1);
  }
  String request = client.readStringUntil('\r');
  request.remove(0,5);
  request.remove(request.length()-9,9);
  return request;
}
