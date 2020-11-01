/*

HobbyIoT NET End device THR Type 1 sensor node

A simple MQTT-SN over 802.15.4 Client implementation based on MRF24J40 and Arduino
Visit project website at https://sites.google.com/view/hobbyiot/projects/mqtt-sn-802-15-4
Twitter: @sivanovbg

This is the HobbyIoT NET End device End device THR Type 1 sensor node.
It implements temperature and humidity sensor alongside with magnetic (reed) sensor.
It implements ATMega328P Arduino compatible MCU together with
802.15.4 2.4 GHz physical radio interface based on Microchip's MRF24J40MA transceiver
Current implementation (V2) incorporates also the DHT11 temperature and humidity sensor
 
Work is ongoing optimizing the powerconsumption close to the sleep current of the chips

The implementation is based ot MQTT-SN Specification Version 1.2.
If Arduino Pro Mini @ 3.3 V is used, the 3.3V regulator has to be removed or disconnected.
Topics should be predefined on the client and gateway sides
and are fixed to 2 positions alpha-numeric string.

Messages supported:

CONNECT
CONNACK
PINGREQ
PINGRESP
PUBLISH

*/

//#define DEBUG_MODE

#include "HIoT-THR_Cfg-homes.h"
#include <SPI.h>
#include <mrf24j.h>   // *** Please use the modified library found within the same repo on GitHub ***
#include "DHT.h"
#include "LowPower.h"

const int pin_reset = 6;
const int pin_cs = 10;
const int pin_interrupt = 2;

#define DHTPWR 8
#define LED 7

#define DHTPIN 4
#define DHTTYPE DHT11

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

boolean message_received = false;
boolean node_connected = false;
boolean connection_timeout = true;
boolean node_subscribed = false;
char rx_buffer[127];
char tx_buffer[127];
uint8_t rx_len;

float temperature;
float humidity;
long batt_value;

uint8_t temperature_string[10] = "24.00";
uint8_t humidity_string[10];
uint8_t batt_value_string[10];

uint8_t timeout_timer = 3;
uint8_t connect_timer = 4;

// Change 0x44 and 0x96 with the 802.15.4 short address bytes of your Client (this node)

uint8_t CONNECT_MSG[] = { 0x0A, 0x04, 0x00, 0x01, 0x03, 0x84, 0xFF, 0xFF, OWN_A_MSB, 0x96 }; // 0x012C = 5 min; 0x0258 = 10 min; 0x0384 = 15 min; 0x0834 = 35 min duration

uint8_t PINGREQ_MSG[] = { 0x0A, 0x16, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, OWN_A_MSB, 0x96 };

// Several sample messages used within the current implementation set up

uint8_t PUBLISH_MSGON[] = { 0x0B, 0x0C, 0x00, 'o', msg_n, OWN_A_MSB, 0x96, ' ', 'R', 'E', 'D' };
uint8_t PUBLISH_MSGOFF[] = { 0x0B, 0x0C, 0x00, 'o', msg_n, OWN_A_MSB, 0x96, 'D', 'a', 'r', 'k' };
uint8_t PUBLISH_MSGTMP[] = { 0x0C, 0x0C, 0x00, 't', msg_n, OWN_A_MSB, 0x96, '-', '-', '-', '-', '-' };
uint8_t PUBLISH_MSGHUM[] = { 0x0C, 0x0C, 0x00, 'h', msg_n, OWN_A_MSB, 0x96, '-', '-', '-', '-', '-' };

uint8_t PUBLISH_MSGALR[] = { 0x0C, 0x0C, 0x00, 'r', msg_n, OWN_A_MSB, 0x96, 'A', 'L', 'E', 'R', 'T' };
uint8_t PUBLISH_MSGOPN[] = { 0x0C, 0x0C, 0x00, 'r', msg_n, OWN_A_MSB, 0x96, 'O', 'P', 'E', 'N', ' ' };
uint8_t PUBLISH_MSGCLS[] = { 0x0C, 0x0C, 0x00, 'r', msg_n, OWN_A_MSB, 0x96, 'C', 'L', 'O', 'S', 'E' };

uint8_t PUBLISH_MSGBAT[] = { 0x0C, 0x0C, 0x00, 'b', msg_n, OWN_A_MSB, 0x96, 'b', 'a', 't', '0', '1' };

uint8_t SUBSCRIBE_MSG[] = { 0x07, 0x12, 0x00, OWN_A_MSB, 0x96, 's', msg_n };

typedef struct
{
  uint8_t Length;
  uint8_t MsgType;
  uint8_t Var[127];
} Message;

Message Msg;

#define CONNACK   0x05
#define PINGRESP  0x17
#define PUBLISH   0x0C
#define PUBACK    0x0D
#define SUBACK    0x13

DHT dht(DHTPIN, DHTTYPE);

void setup() { 

  #ifdef DEBUG_MODE
  Serial.begin(19200);

  Serial.println();
  Serial.println("|HobbyIoT Sensor node Type 1 THR|");
  Serial.println("|-------------------------------|");
  Serial.print("|Own address: 0x"); Serial.println(OWN_ADDRESS,HEX);
  Serial.println("|          DEBUG_MODE           |");
  Serial.println("|-------------------------------|");
  Serial.println();
  #endif

//  #ifndef DEBUG_MODE
////  pinMode(67, INPUT_PULLUP); // 67 should be PD0 pin /UART TXI
////  pinMode(66, INPUT_PULLUP); // 67 should be PD0 pin /UART TXI
//  digitalWrite(66, HIGH); //    66 should be PD1 pin / UART TXO
//                          //    according pins_arduino.h
//  #endif

  attachInterrupt(0, interrupt_routine, FALLING);

  attachInterrupt(1, reed_routine, CHANGE);

  pinMode(2,INPUT_PULLUP); // define input for MRF24J40 interrupt, PU is internal
//  pinMode(2,INPUT); // define input for MRF24J40 interrupt, PU is external
  pinMode(3,INPUT); // define input for reed sensor, PU is external

  pinMode(DHTPWR, OUTPUT); // power for DHT11 OK

  digitalWrite(DHTPWR, HIGH);  // DHT11 Power on
  
//  pinMode(9, OUTPUT); // power for MRF24J40MA Not used

//  pinMode(8, OUTPUT); // Test LED
//  digitalWrite(9, HIGH); // Test LED ON
  
  interrupts();

  dht.begin();

  pinMode(DHTPIN, INPUT); // remove DHTPIN internal pull up to save power

//  dht_off(); // switch off DHT11

  mrf_init();

}

void interrupt_routine() {

    delay(200);
    mrf.interrupt_handler(); // mrf24 object interrupt routine

//    pinMode(pin_led,OUTPUT);
}

void reed_routine() {

  noInterrupts();
  delay(100);
  #ifdef DEBUG_MODE
  Serial.println("----------- REED Routine -----------");
  #endif
  mrf_wake();
  delay(100);
  if(digitalRead(3) == 1)
  {
    #ifdef DEBUG_MODE
    Serial.println("Door/window just OPENED!");
    #endif
    mrf.send16(GW_ADDRESS,PUBLISH_MSGOPN,sizeof(PUBLISH_MSGOPN));
  }
  else
  {
    #ifdef DEBUG_MODE
    Serial.println("Door/window just CLOSED!");
    #endif
    mrf.send16(GW_ADDRESS,PUBLISH_MSGCLS,sizeof(PUBLISH_MSGCLS));
  }

//  Serial.println("INT1 Change, .............sending alert!");
  
//  mrf.send16(GW_ADDRESS,PUBLISH_MSGALR,sizeof(PUBLISH_MSGALR));

  delay(200);
  interrupts();
  timed_sleep(SHORT_CONNECT_INTERVAL);
}
// *************** MAIN LOOP STARTS HERE ***************
void loop() {
  
  if(node_connected == true) {
    
    connect_timer = 4;
    exchange_keepalive();
    exchange_data();
    timed_sleep(SLEEP_INTERVAL);
  }
  else {
    get_connected();
    if(node_connected == false) {
      #ifdef DEBUG_MODE
      Serial.println("Not connected");
      #endif
      if(connect_timer > 0) {
        connect_timer --;
        #ifdef DEBUG_MODE
        Serial.println("Waiting for SHORT connect interval");
        #endif
        timed_sleep(SHORT_CONNECT_INTERVAL);        
      }

      else {
        #ifdef DEBUG_MODE
        Serial.println("Waiting for LONG connect interval");
        #endif
        timed_sleep(LONG_CONNECT_INTERVAL);
      }
    }
  }

  if(timeout_timer == 0) {
    #ifdef DEBUG_MODE
    Serial.println("Connection timeout ...");
    #endif
    node_connected = false;
  }
}

// *************** MAIN LOOP ENDS HERE ***************

void get_connected() {

  int i;
        
  #ifdef DEBUG_MODE
  Serial.print("Connecting ... ");
  #endif

  mrf.send16(GW_ADDRESS,CONNECT_MSG,sizeof(CONNECT_MSG));

  delay(100);
  
  mrf.check_flags(&handle_rx, &handle_tx);
  
  if(message_received) {
    
    Msg.Length = rx_buffer[0];  // Fill in the message fields
    Msg.MsgType = rx_buffer[1];
    for(i=2;i<Msg.Length;i++) {
        Msg.Var[i-2] = rx_buffer[i];
    }

    if(Msg.MsgType == CONNACK) {
     #ifdef DEBUG_MODE
      Serial.println("CONNACK received. Connected");
      #endif
      message_received = false;
      node_connected = true;
      connection_timeout = false;
      timeout_timer = 3;
   }
  }
}

void exchange_keepalive() {

  int i;
        
  #ifdef DEBUG_MODE
  Serial.println("Sending PINGREQ");
  #endif
  mrf.send16(GW_ADDRESS,PINGREQ_MSG,sizeof(PINGREQ_MSG));
  
  delay(100);

   mrf.check_flags(&handle_rx, &handle_tx);

   if(message_received) {
     Msg.Length = rx_buffer[0];  // Fill in the message fields
     Msg.MsgType = rx_buffer[1];
     for(i=2;i<Msg.Length;i++) {
      Msg.Var[i-2] = rx_buffer[i];
     }

     if(Msg.MsgType == PINGRESP) {
        #ifdef DEBUG_MODE
        Serial.println("PINGRESP received");
        #endif
        message_received = false;
        connection_timeout = false;
        timeout_timer = 3;
     }
 }
 else {
      timeout_timer--;   
 }
}

void timed_sleep(char sleep_count) {

  #ifdef DEBUG_MODE
  Serial.println("Wireless module TIMED SLEEP");
  delay(10);
  #endif

    mrf.write_short(MRF_SLPACK,0x48); // bits 0..6 of WAKECNT
    mrf.write_short(MRF_RFCTL,mrf.read_short(MRF_RFCTL)|0x08); // bit 7 of WAKECNT = 1
    mrf.write_short(MRF_RFCTL,mrf.read_short(MRF_RFCTL)&0xEF); // bit 8 of WAKECNT = 0

  // 6. Set MAINCNT value and put MRF into sleep mode
  // Will sleep for 60 s: MAINCNT = 0x005B8D80
    mrf.write_long(MRF_MAINCNT0,0x80);
    mrf.write_long(MRF_MAINCNT1,0x8D);
    mrf.write_long(MRF_MAINCNT2,sleep_count); // 0x05 = 15 s; 0x17 = 60 s; 0x64 = 5 min; 0xA5 = 7 min; 0xC8 = 10 min?
    mrf.write_long(MRF_MAINCNT3,0x00);

    mrf.write_long(MRF_MAINCNT3,mrf.read_long(MRF_MAINCNT3)|0x80); // Put MRF into timed sleep!

    delay(500); // Before spi_off

    spi_off();

    #ifdef DEBUG_MODE
    Serial.println("Arduino is going to SLEEP");
    delay(5);
    #endif

//    digitalWrite(9, LOW); // Test LED OFF

    delay(500); // SPI OFF to Arduino OFF

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Sistem will sleep until MRF counts down the desired sleep period
    // and wakes the Arduino MCU module after that

    delay(100); // To wake up slowly :)
    
    mrf.read_short(MRF_INTSTAT); // Read INTSTAT register to clear the interrupt

    #ifdef DEBUG_MODE
    Serial.println("System wakes up");
    #endif

//    digitalWrite(9, HIGH); // Test LED ON
    
    spi_on();
    
}

void dht_off() {
      digitalWrite(DHTPWR, LOW);  // DHT11 Power off
//      pinMode(DHTPIN, INPUT); // set to only Imput to remove the internal pull up
      #ifdef DEBUG_MODE
      Serial.println("DHT11 Power OFF");
      #endif
}

void dht_on() {
      digitalWrite(DHTPWR, HIGH);  // DHT11 Power on
      #ifdef DEBUG_MODE
      Serial.println("DHT11 Power ON");
      #endif
}

void spi_off() {

//  pinMode(12, INPUT); // MISO OK
//  pinMode(SS, INPUT_PULLUP);   // CS
//  delay(100);
//  pinMode(MOSI, INPUT_PULLUP); // Due leaves HIGH (MOSI)
//  pinMode(SCK, INPUT_PULLUP);  // Due leaves LOW

  digitalWrite(SS, HIGH);
  delay(100);
  digitalWrite(MOSI, HIGH);
  digitalWrite(SCK, HIGH);
  
  #ifdef DEBUG_MODE
  Serial.println("SPI interface OFF");
  delay(5);
  #endif
  
}

void spi_on() {

//  pinMode(MISO, INPUT_PULLUP); // MISO OK
  pinMode(SS, OUTPUT);   // CS
  pinMode(MOSI, OUTPUT); // Due leaves HIGH (MOSI)
  pinMode(SCK, OUTPUT);  // Due leaves LOW
  delay(100);
  mrf_init();
  #ifdef DEBUG_MODE
  Serial.println("SPI interface ON (w/mrf_init)");
  #endif
  
}

long readVcc() {
  long result; // Read 1.1V reference against AVcc
  
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void mrf_init() {
  
  mrf.reset();
  mrf.init();

  mrf.set_pan(0xcfce); // pan ID = 0xABCD // Enter your 802.15.4 PAN ID here
  mrf.address16_write(OWN_ADDRESS);

  // MRF24J40 Inits for timed sleep mode ***************************************************

  mrf.write_short(MRF_INTCON,mrf.read_short(MRF_INTCON)&0xBF); // Enable wake interrupt (bit 6)

  // Must be initial settings...
  
  // 1. Select 100 KHz internal oscillator (default setting) by clearing SLPCLKEN bit
  //  mrf.write_long(MRF_SLPCON0,mrf.read_long(MRF_SLPCON0)&0xFE);
  //  delay(10);
  
  // 2. Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator)
  //    mrf.write_long(MRF_RFCON7,0x80);
  
  // 3. Set CLKOUTEN = 1 and SLPCLKDIV = 0x01 done during MRF init phase

  // 3.5 Calibrate
  //    mrf.write_long(MRF_SLPCAL2,mrf.read_long(MRF_SLPCAL2)&0x10);
  //    delay(1000);

  // 4. Set the WAKETIME to 0x0D2 -> 2.1 ms
    mrf.write_long(MRF_WAKETIMEL,0xD2);
    mrf.write_long(MRF_WAKETIMEH,0x00);    
}

void handle_rx() {
    uint8_t i;
    
    delay(100);
    rx_len = mrf.rx_datalength();
    for (i = 0; i < rx_len; i++)
    {
          rx_buffer[i] = mrf.get_rxinfo()->rx_data[i];
    }
//    client_address = rx_buffer[7];
    message_received = true;
}

void handle_tx() {
//    if (mrf.get_txinfo()->tx_ok) {
//        Serial.println("802.15.4 TX went ok, got ack");
//    } else {
//        Serial.print("802.15.4 TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
//    }
}

void exchange_data() {
  
  int i;

  mrf_sleep();
  dht_on();
  
  #ifdef DEBUG_MODE
  Serial.println("Reading temperature...");
  #endif
  temperature = dht.readTemperature(); // get the current temperature
                                           // and prepare the MQTT-SN message
  dtostrf(temperature,0,0,temperature_string);
  uint8_t tmp_len = strlen(temperature_string);

  for(i=0;i<tmp_len;i++) {
    PUBLISH_MSGTMP[i+7] = temperature_string[i];
  }
  PUBLISH_MSGTMP[i+7] = 0x00;
  PUBLISH_MSGTMP[0] = 7+i; // +2 deleted, MQTT-SN message prepared

  #ifdef DEBUG_MODE
  Serial.println("Reading humidity...");
  #endif
  humidity = dht.readHumidity(); // get the current humidity

  dht_off();
                                           // and prepare the MQTT-SN message
  dtostrf(humidity,0,0,humidity_string);
  uint8_t hum_len = strlen(humidity_string);

  for(i=0;i<tmp_len;i++) {
    PUBLISH_MSGHUM[i+7] = humidity_string[i];
  }
  PUBLISH_MSGHUM[i+7] = 0x00;
  PUBLISH_MSGHUM[0] = 7+i; // +2 deleted, MQTT-SN message prepared      

  #ifdef DEBUG_MODE
  Serial.println("Reading battery voltage...");
  #endif
  batt_value = readVcc();

  dtostrf(batt_value,0,0,batt_value_string);

  uint8_t batt_len = strlen(batt_value_string);

  for(i=0;i<batt_len;i++) {
    PUBLISH_MSGBAT[i+7] = batt_value_string[i];
  }
  PUBLISH_MSGBAT[i+7] = 0x00;
  PUBLISH_MSGBAT[0] = 7+i; // +2 deleted, MQTT-SN message prepared
      
  mrf_wake();               // Ready for sending, powering the MRF on.

  #ifdef DEBUG_MODE
  Serial.println("Sending temperature...");
  #endif
  mrf.send16(GW_ADDRESS,PUBLISH_MSGTMP,sizeof(PUBLISH_MSGTMP)); // send the temperature value
  delay(100);

  #ifdef DEBUG_MODE
  Serial.println("Sending humidity...");
  #endif
  mrf.send16(GW_ADDRESS,PUBLISH_MSGHUM,sizeof(PUBLISH_MSGHUM)); // send the humidity value
  delay(100);
     
  #ifdef DEBUG_MODE
  Serial.println("Sending battery voltage...");
  #endif
  mrf.send16(GW_ADDRESS,PUBLISH_MSGBAT,sizeof(PUBLISH_MSGBAT)); // send the battery value
  delay(100);
}

void mrf_wake() {
  // Perform register wake-up
  mrf.write_short(MRF_WAKECON,mrf.read_short(MRF_WAKECON)|0x40); // Wake up sequence start
  delay(10);
  mrf.write_short(MRF_WAKECON,mrf.read_short(MRF_WAKECON)&0xBF); // Wake up sequence end

  delay(10);
  // RF State mashine reset
  mrf.write_short(MRF_RFCTL,mrf.read_short(MRF_RFCTL)|0x04); // RF State mashine reset
  delay(10);
  mrf.write_short(MRF_RFCTL,mrf.read_short(MRF_RFCTL)&0xFB); // RF State mashine release

  delay(100); // 20 ms shoud be enough...

  #ifdef DEBUG_MODE
  Serial.println("Wireless module is ON");
  #endif

}

void mrf_sleep()
{

  #ifdef DEBUG_MODE
  Serial.println("Wireless module IMMEDIATE SLEEP");
  #endif

//  delay(50);
  
  mrf.write_short(MRF_SOFTRST,mrf.read_short(MRF_SOFTRST)|0x04); // LP - Power management reset
//  delay(10);
//  mrf.write_short(MRF_SLPACK,0xC8); // LP - Go to Sleep together with the required WAKECNT bits.
  mrf.write_short(MRF_SLPACK,mrf.read_short(MRF_SLPACK)|0x80); // LP - Go to Sleep

//  delay(50);

}
