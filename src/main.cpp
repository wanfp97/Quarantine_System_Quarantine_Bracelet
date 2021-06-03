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
  MISSING   = 3,      //never used by qb (for ig only)
  EMERGENCY = 4
};

RF24 nrf24(8, 9); // using pin 7 for the CE pin, and pin 8 for the CSN pin

SoftwareSerial IG(7, 6);     //pin 5 Rx, pin4 Tx fot connecting to IG

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
  uint8_t status = 0;
}qb_message;


void setup() {
  IG.begin(9600);
  while(!IG);   //wait until uart port ready

  pinMode(2, INPUT);     //use pin2 for lock, externally connect to pull up resistor
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

  Retry_sync:
  while(IG.available()<=0);
  String incoming_str = IG.readString();

  #ifdef DEBUG
    delay(100);
    Serial.print(F("Received: "));
    Serial.println(incoming_str);
  #endif

  if(incoming_str == "connect") {
    IG.print(F("connected"));

    #ifdef DEBUG
      Serial.println("connected sent");
    #endif

    while(IG.available()<=0);   //wait for serial input
    incoming_str = IG.readString();

    #ifdef DEBUG
      delay(100);
      Serial.print(F("Received: "));
      Serial.println(incoming_str);
    #endif

    void* incoming_ptr = incoming_str.begin();
    qb_settings.channel = *(uint16_t*) incoming_ptr;
    qb_settings.ig_addr = *(unsigned int*) incoming_ptr + 2;    //offset to ig_addr (refer to message sent by ig)
    qb_settings.qb_addr = *(unsigned int*) incoming_ptr + 6;    //offset to qb_addr
    qb_message.qb_num = *(uint8_t*) incoming_ptr + 10;          //offset to qb_num
    IG.print(incoming_str);

    nrf24.setPALevel(RF24_PA_MAX);      //set max signal amplification
    nrf24.setCRCLength(RF24_CRC_8);     //set 8 bit crc
    nrf24.setPayloadSize(2);      //1byte user number + 1 byte status byte
    nrf24.setAutoAck(true);       //enable auto acknoledgement
    nrf24.setChannel(qb_settings.channel);      //set channel
    nrf24.setAddressWidth(4);               //set addr width
    nrf24.openReadingPipe(0, qb_settings.qb_addr);     //set receiving addr
    nrf24.openWritingPipe(qb_settings.ig_addr);         //set to write to ig
    nrf24.stopListening();     //start receiving

    delay(100);

    #ifdef DEBUG
      delay(100);
      Serial.print(F("nrf24 parameter setting done"));
    #endif

    qb_message.status = PRESENT;

    bool report = nrf24.write(&qb_message, sizeof(qb_message));      // transmit & save the report

    if(!report) {
      goto Retry_sync;
      #ifdef DEBUG
        delay(100);
        Serial.print(F("transmission failed"));
      #endif
    }
  }
  else {
    goto Retry_sync;
  }
  

  //sync successful

  Retry_test_update:
  while(IG.available()<=0);
  incoming_str = IG.readString();
  if(incoming_str == "update") {
    IG.print(F("start"));
    if(!nrf24.write(&qb_message, sizeof(qb_message))) {            // transmit & save the report
      goto Retry_test_update;
    }
  }
  else {
    goto Retry_test_update;
  }
}

void loop() {
  qb_transmit();
  nrf24.powerDown();    
  delay(50);        //delay for the power down nrf24 to complete
  for(int i = 0; i<37; i++) {     //loop to sleep for 5mins
    LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_OFF);        //sleep for 8s and leave ADC as default (disabled previously)
  }
}

void qb_opened() {
  qb_message.status = OPENED;
  digitalWrite(10, HIGH);
  delay(2000);
  digitalWrite(10, LOW);
  qb_transmit();
}

void qb_emergency() {
  qb_message.status = EMERGENCY;
  digitalWrite(10, HIGH);
  delay(5000);
  digitalWrite(10, LOW);
  qb_transmit();
}

void qb_transmit() {
  Retransmit:
  nrf24.powerUp();
  if(!nrf24.write(&qb_message, sizeof(qb_message))) {
    nrf24.powerDown();
    digitalWrite(10, HIGH);
    delay(100);
    digitalWrite(10, LOW);
    for(int j = 0; j<7; j++) {     //loop to sleep for 1min
      LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_OFF);        //sleep for 8s and leave ADC as default (disabled previously)
    }
    goto Retransmit;
  }
  nrf24.powerDown();
}

