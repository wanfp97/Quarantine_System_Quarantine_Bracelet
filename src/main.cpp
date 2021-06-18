#define DEBUG

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "RF24.h"
#include "LowPower.h"

void qb_transmit();
void qb_opened();
void qb_emergency();

enum QUARANTINE_BAND_STATUS {
  NOT_INIT  = 0,
  PRESENT   = 1,
  OPENED    = 2,
  MISSING   = 3,
  EMERGENCY = 4
};

RF24 nrf24(8, 9); // using pin 8 for the CE pin, and pin 9 for the CSN pin

SoftwareSerial IG(6, 7);     //pin 6 Rx, pin 7 Tx for connecting to IG

bool qb_initialized = 0;

bool opened = 0;

struct settings{
  uint16_t  channel;
  uint32_t ig_addr;
  uint32_t qb_addr;
}qb_settings;

struct qb_message
{
  uint8_t qb_num;
  uint8_t status;
}qb_msg;


void setup() {
  IG.begin(9600);
  while(!IG);   //wait until uart port ready

  pinMode(2, INPUT_PULLUP);     //use pin2 for lock
  pinMode(3, INPUT_PULLUP);     //use pin3 for emergency button
  pinMode(10, OUTPUT);    // used for buzzer
  digitalWrite(10, LOW);

  #ifdef DEBUG
    Serial.begin(9600);   //uart for debugging
  #endif

  //set all unused pin to output low to save power
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  for(int i = 14; i<=17; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  attachInterrupt(2, qb_opened, RISING);
  attachInterrupt(3, qb_emergency, FALLING);

  ADCSRA = ADCSRA & B01111111;      //disable ADC, set ADEN to 0
  ACSR = B10000000;       //disable analog comparator, set ACD to 1
  DIDR0 = DIDR0 | B00111111;      //disable digital input buffer on analog pins
  PRR = PRR | B00000001;      //write 1 to PRADC to disable the clock to ADC

  #ifdef DEBUG
   Serial.println("starting nrf24...");
  #endif

  if (!nrf24.begin()) {
    #ifdef DEBUG
      Serial.println(F("NRF24 ERROR!!"));
    #endif
    while (1) {} // hold in infinite loop
  }

  #ifdef DEBUG
   Serial.println("start sync");
  #endif

  char* incoming_str = (char*)malloc(sizeof("connect"));
  char expecting[] = "connect";

  Retry_sync:
  while(IG.available()<=0);
  IG.readBytes(incoming_str, sizeof(expecting));

  #ifdef DEBUG
    delay(100);
    Serial.print(F("Received: "));
    Serial.println(incoming_str);
  #endif

  if(strncmp(incoming_str, expecting, sizeof(expecting))==0) {
    char ack[] = "connected";
    IG.write(ack, sizeof(ack));

    #ifdef DEBUG
      Serial.println("Sent: connected");
    #endif

    struct qb_sync_message_incoming {
      uint8_t ig_channel;
      uint32_t ig_addr;
      uint32_t qb_addr;
      uint8_t qb_num;
    }qb_sync_info;

    while(IG.available()<=0);   //wait for serial input
    IG.readBytes(reinterpret_cast<char*>(&qb_sync_info), sizeof(qb_sync_info));

    qb_settings.channel = qb_sync_info.ig_channel;
    #ifdef DEBUG
      Serial.print(F("qb_settings.channel = "));
      Serial.println(qb_settings.channel);
    #endif

    qb_settings.ig_addr =  qb_sync_info.ig_addr;   
    #ifdef DEBUG
      Serial.print(F("qb_settings.ig_addr = "));
      Serial.println(qb_settings.ig_addr, 16);
    #endif

    qb_settings.qb_addr =  qb_sync_info.qb_addr;
    #ifdef DEBUG
      Serial.print(F("qb_settings.qb_addr = "));
      Serial.println(qb_settings.qb_addr, 16);
    #endif

    qb_msg.qb_num =  qb_sync_info.qb_num;
    #ifdef DEBUG
      Serial.print(F("qb_msg.qb_num = "));
      Serial.println(qb_msg.qb_num);
    #endif

    nrf24.setPALevel(RF24_PA_MAX);      //set max signal amplification
    nrf24.setCRCLength(RF24_CRC_8);     //set 8 bit crc
    nrf24.setPayloadSize(2);      //1byte user number + 1 byte status byte
    nrf24.setAutoAck(true);       //enable auto acknoledgement
    nrf24.setChannel(qb_settings.channel);      //set channel
    nrf24.setAddressWidth(4);               //set addr width
    nrf24.openReadingPipe(0, qb_settings.qb_addr);     //set receiving addr
    nrf24.openWritingPipe(qb_settings.ig_addr);         //set to write to ig
    nrf24.stopListening();     //start receiving

    IG.write(reinterpret_cast<char*>(&qb_sync_info), sizeof(qb_sync_info));

    #ifdef DEBUG
      Serial.println(F("nrf24 parameter setting done"));
    #endif

    if(digitalRead(2)){
      qb_msg.status = PRESENT;
    }
    else {
      qb_msg.status = OPENED;
    }
    
    delay(1000);   //delay for IG to be ready

    #ifdef DEBUG
      Serial.println(F("transmitting to IG"));
    #endif

    bool report = nrf24.write(&qb_msg, sizeof(qb_message));      // transmit & save the report

    if(!report) {
      #ifdef DEBUG
        delay(100);
        Serial.println(F("transmission failed"));
        Serial.println(F("goto Retry sync"));
      #endif
      goto Retry_sync;
    }
    else {
      #ifdef DEBUG
        delay(100);
        Serial.println(F("sync successful"));
      #endif  
    }
  }
  else {
    goto Retry_sync;
  }
  
  

  //sync successful

  Retry_test_update:
  while(IG.available()<=0);
  IG.readBytes(incoming_str, sizeof("update")-1);
  if(strncmp(incoming_str,"update", sizeof("update")-1)==0) {
    if(digitalRead(2)){
      qb_msg.status = PRESENT;
    }
    else {
      qb_msg.status = OPENED;
    }
    IG.print(F("start"));
    #ifdef DEBUG
      Serial.println(F("Sending update message"));
    #endif
    if(!nrf24.write(&qb_msg, sizeof(qb_message))) {            // transmit & save the report
      #ifdef DEBUG
        Serial.println(F("Sending message failed"));
      #endif
      goto Retry_test_update;
    }
  }
  else {
    goto Retry_test_update;
  }
  #ifdef DEBUG
    Serial.println(F("Update message sent"));
  #endif
}

void loop() {
  qb_transmit();
  nrf24.powerDown();    
  for(int i = 0; i<37; i++) {     //loop to sleep for 5mins
    LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_OFF);        //sleep for 8s and leave ADC as default (disabled previously)
  }
}

void qb_opened() {
  qb_msg.status = OPENED;
  digitalWrite(10, HIGH);
  delay(2000);
  digitalWrite(10, LOW);
  qb_transmit();
}

void qb_emergency() {
  qb_msg.status = EMERGENCY;
  digitalWrite(10, HIGH);
  delay(5000);
  digitalWrite(10, LOW);
  qb_transmit();
}

void qb_transmit() {
  Retransmit:
  uint8_t retransmit_count = 0;
  nrf24.powerUp();
  if(!nrf24.write(&qb_msg, sizeof(qb_message))) {
    nrf24.powerDown();
    digitalWrite(10, HIGH);
    retransmit_count ++;
    if(retransmit_count>10) {
      qb_msg.status = MISSING;
      #ifdef DEBUG
        Serial.println(F("Update info failed"));
      #endif
    }
    delay(100);
    digitalWrite(10, LOW);
    for(int j = 0; j<7; j++) {     //loop to sleep for 1min
      LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_OFF);        //sleep for 8s and leave ADC as default (disabled previously)
    }
    goto Retransmit;
  }
  #ifdef DEBUG
    Serial.println(F("Update info success"));
  #endif
  nrf24.powerDown();
}

