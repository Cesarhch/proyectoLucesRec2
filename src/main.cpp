#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

//funcion enlazada a codigo "c" compilada con compilador "c++"
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();


uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const int rele = 14;
const int pinPulsador = 35;
int i = 1;
int pulsador =0;
int analogPin = 33; // KY-028 detector de temperatura
int valAnalogTemp;
int confirmacion;
int grados;

void funcionPulsador(){
  if(digitalRead(pinPulsador)==HIGH){
    digitalWrite(rele, HIGH);
    pulsador=1;
  }else if(digitalRead(pinPulsador)==LOW){
    pulsador=0;
  }
}

typedef struct struct_message {
  int a;
} struct_message;

// Se crean dos variables con tipo estructura myData
struct_message myDataRec;
struct_message myDataSen;

esp_now_peer_info_t peerInfo;

// Llamada para el envio del mensaje
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS){
   confirmacion=1;
  }else{
   confirmacion=2;
  }
}
 
// Funcion que recibe el mensaje
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myDataRec, incomingData, sizeof(myDataRec));
  if(myDataRec.a==2){confirmacion=1;}
  if (myDataRec.a==2 || digitalRead(pinPulsador)==HIGH){
    digitalWrite(rele,HIGH);
  } else {
    digitalWrite(rele,LOW);
    Serial.println("estoy ondatarecv segundo rele low");
  }
  
}

void setup() {
  // Iniciar Serial Monitor
  Serial.begin(9600);
  pinMode(rele, OUTPUT);
  pinMode(pinPulsador, INPUT_PULLDOWN);
  pinMode(analogPin,INPUT_PULLDOWN);
  // Configuracion del modo Wi-Fi 
  WiFi.mode(WIFI_STA);

  // Iniciar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Funcion para enviar el mensaje
  esp_now_register_send_cb(OnDataSent);
  
  //Este apartado del peer es util para enviar el paquete
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer
          
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // Funcion para recibir el mensaje
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  funcionPulsador();
  valAnalogTemp=analogRead(analogPin);  
  grados=((temprature_sens_read() - 32) / 1.8);
  if(grados>=65 || valAnalogTemp<=900 || valAnalogTemp>=1300){
    Serial.flush();
    esp_sleep_enable_timer_wakeup(5e6);
    esp_deep_sleep_start();
    Serial.println("he entrado en deep sleep");
  }
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myDataSen, sizeof(myDataSen));
  while(confirmacion==2 && pulsador==0){
    digitalWrite(rele, LOW);
    Serial.println("estoy en while rele low");
    delay(2000);
  }
delay(2000);
}