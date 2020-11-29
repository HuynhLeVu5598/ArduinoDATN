#include <ESP8266WiFi.h>
WiFiClient client;
WiFiServer server(80);
const String tenwifi = "TUYET";
const String mkwifi = "123456789";
String data="";

void setup() {
  Serial.begin(115200);
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
  if(data == "B1"){
    digitalWrite(2,LOW);
  }
  if(data == "T1"){
    digitalWrite(2,HIGH);
  }
  if(data == "B2"){
    digitalWrite(16,LOW);
  }
  if(data == "T2"){
    digitalWrite(16,HIGH);
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
