/////////////////////////////////////////////////////////////////////////
//////////////////////   RF TO SERIAL TRANSDUCER ////////////////////////
/////////////////////////////////////////////////////////////////////////
//
// Receives manchester-encoded  RF data packets and sends them through a
// Software Serial link. Intended to interface arduino systems via cheap 
// 433 MHz transmitters with MCUs without an available an implementation 
// of an RF library, using a minimum of additional hardware.
//
// The data packets received and sent are  sensed_buf_t  structs defined 
// in sensed.h. sensed is a class I use for sensor data but this program
// can be adapted to any other type of message.
//
// Manchester encoding library by mchr3k
//    https://github.com/mchr3k/
//    https://github.com/mchr3k/arduino-libs-manchester
//    https://mchr3k.github.io/arduino-libs-manchester/
//
// Serial comms are implemented by the minimalistic send-only library 
// by Nick Gammon 
//    https://github.com/nickgammon
//    https://github.com/nickgammon/SendOnlySoftwareSerial
//
// Robustness of serial comms is enhanced by the SerialTransfer packet-
// oriented library SerialTransfer by PowerBroker2
//    https://github.com/PowerBroker2
//    https://github.com/PowerBroker2/SerialTransfer
//
// To make this library fit into an AtTiny85 with 512 bytes of free RAM
// I've made serveral changes to this library, begining with prefixing
// source filenames with 'mod_'
//
// * Changed the CRC library and implemented a lame XOR checksum that 
//   does not need a large CRC table. See mod_SerialTransfer library. 
//   The receiving side must use the same library to avoid CRC errorss.
//
// * The send and receive code segments of the library are conditionally
//   compiled based on whether the symbols COMPILE_SND and COMPILE_RCV 
//   are defined or not. REMEMBER TO ADD OR REMOVE THE DEFINITIONS AS 
//   APPROPIATE. For this library to fit compfortably into an AtTiny85
//   only the SND psart shoud be included.
// 
// When Using with AtTiny with ISP (e. g. Arduino as ISP)
//     REMEMBER TO SET CLOCK SPEED TO 8 Mz AND RUN BOOTLOADER
// With DigisPark DigiStump: Testetd with with AtTiny AVR core 8 MHz no USB option. 
//     Programmer:Micronucleus
//
// WARNING: Digisspark attiny core uses TIMER1 for timekeeping. TIMER1 is
//          used by the Manchester RF library, so when initializing it 
//          all time-related functions cease to work. Options are 1) use
//          another core (TIMER0 is used by other cores for timekeeping)
//          2) modify the Manchester Library to use TIMER0, or 3) avoid
//          using millis(), micros(), delay() and the like. Tis last
//          option is used here since, fortunately, the SendOnlySeraial
//          library uses calibrated loops for timing purposes.
//
// I've also made this work in an Arduino Nano using Andreas Rohner 
// RFTransmitter/RFReceiver libraries which make use of PC interrups
// through Andreas's own PinChangeInterruptHandler class. I've been
// unable to make this work with a Digistump board, but kept the code
// as an option (see RF433_BROADCAST symbol). In this case the TF input
// pin must be assigned to a PCI capacble pin.
//
//
/////////////////////////////////////////////////////////////////////////
//////////////////////   CONFIGURATION   ////////////////////////////////
/////////////////////////////////////////////////////////////////////////
//
// For Digispark Digistump AtTiny85 // Board / IC Pin / Usage
const int TX_PIN        =    0;     //  P0   / Pin 5  / Tx Data output 
const int LED_PIN       =    1;     //  P1   / Pin 6  / On-board LED (Model A) (P0 in model B)
const int RF_INPUT_PIN  =    4;     //  P4   / Pin 3  / RF Receiver input 

 
/////////////////////////////////////////////////////////////////////////
/////////////////////////// RF BROADCAST DRIVER /////////////////////////
/////////////////////////////////////////////////////////////////////////
// 
// Andreas Rohner library doesn't seem to work with AtTiny85 (at least
// not without some rewriting.
//
// I've used Manchester Encoding library with Arduino Nano and AtTiny85
// (Digispark Digistump AVR @ 16.5 MHz)
//  
// Sources: 
//   mchr3k (Manchester encoding)
//      https://github.com/mchr3k/arduino-libs-manchester
//   Andreas Rohner:
//      https://github.com/zeitgeist87/RFTransmitter (usend in transmitter)
//      https://github.com/zeitgeist87/RFReceiver  (used here)

#define RF433_ZEITGEIST    1    // Andreas Rohner (zeitgeist87) library
#define RF433_MANCHESTER   2    // mchr3k Library 

//----------------------- SELECTT RF PROTOCOL & DRIVER 

#define RF433_BROADCAST RF433_MANCHESTER

//-------------------------------------------------------- zeitgeist87's
#if RF433_BROADCAST == RF433_ZEITGEIST
#include <PinChangeInterruptHandler.h>
#include <RFReceiver.h>
unsigned int PULSE_LENGTH = 333;  // Pulse length in microseconds must agree with transmitter
RFReceiver receiver(RF_INPUT_PIN, PULSE_LENGTH);  // Packet receiver
byte senderId = 0;                                // Sender id
byte packageId = 0;                               // packet id

#elif RF433_BROADCAST == RF433_MANCHESTER
//--------------------------------------------------------- mchr3k's 
#include <Manchester.h>

// 1200 had a much worse error ratio

#define MANCHESTER_SPEED MAN_600  
#else
#error "*** RADIO DRIVER NOT CONFIGURED ***"
#endif

/////////////////////////////////////////////////////////////////////////
//////////////////////// SERIAL COMMS LIBRARY ///////////////////////////
/////////////////////////////////////////////////////////////////////////

/* 
 * Librería de Software Serial sólo envío. No utiliza interrupciones,
 * así que no colisiona con PinChangeInterruptHandler como sí le
 * pcurre a SoftwareSerial
 */

#include <SendOnlySoftwareSerial.h>
SendOnlySoftwareSerial SSerial(TX_PIN);

/* SerialTransfer implementa un protocolo para intercambio de paquetes por 
 * puerto serie (Stream). Esta versión está modificada para caber en un 
 * AtTiny85
 */
 
#include "mod_SerialTransfer.h"  // Version modified to fit, with fake CRC
SerialTransfer myTransfer;

/////////////////////////////////////////////////////////////////////////
/////////////////////// SENSED SENSOR PACKETS LIB ///////////////////////
/////////////////////////////////////////////////////////////////////////

// Packet lib
// Configure sensed frame size according to RFReceiver limits
#include <sensed.h>
sensed sensor_data(SS_NOT_SET);

/////////////////////////////////////////////////////////////////////////
///////////////////////////  other globals  /////////////////////////////
/////////////////////////////////////////////////////////////////////////

bool received = false;    // A packet has been received
byte len;                 // Length of received data

/////////////////////////////////////////////////////////////////////////
//////////////////////   LED SIGNALING   ////////////////////////////////
/////////////////////////////////////////////////////////////////////////
unsigned long LED_blink_count = 0;       // 0..LED_BLINK_LIMIT
unsigned long LED_wink_count = 0;        // LED_WINK_LIMIT..0 when winking
unsigned long LED_BLINK_LIMIT = 8000;    // approx 2.5 blinks/s on attiny85 @ 8Mhz
unsigned long LED_WINK_LIMIT = 100000UL; // approx 1s wink in attiny @ 8 Mhz 
unsigned int led_status = HIGH;

/////////////////////////////////////////////////////////////////////////
////////////////////////    OTROS PROTOTIPOS   //////////////////////////
/////////////////////////////////////////////////////////////////////////

void blink_led();
void wink_led();
void set_led(byte val);

#define TUNECOUNT 519UL // Timing tuning for my_delay()

void my_delay(unsigned long msaprox);

/////////////////////////////////////////////////////////////////////////
///////////////////////////  *** SETUP ***  /////////////////////////////
/////////////////////////////////////////////////////////////////////////
void setup() {

  pinMode(LED_PIN, OUTPUT); //
  pinMode(TX_PIN, OUTPUT);
  set_led(LOW);

    // --- Initialize RF
  /**/ 
  pinMode(RF_INPUT_PIN, INPUT);
  #if RF433_BROADCAST == RF433_MANCHESTER  
    man.setupReceive(RF_INPUT_PIN, MANCHESTER_SPEED);
    man.beginReceiveArray(sizeof(sensed_buf_t)+1, (uint8_t *)(sensor_data.x_bufptr()));
  #elif RF433_BROADCAST == RF433_ZEITGEIST
    receiver.begin();
  #else
    #error "INVALID RF433 OPTION: "   RF433_BROADCAST
  #endif 

   // --- Initialize Serial
  
  SSerial.begin(9600);
  myTransfer.begin(SSerial);

   // --- Signal end of setup
  
  wink_led();

}
/////////////////////////////////////////////////////////////////////////
/////////////////////  *** M A I N   L O O P  ***  //////////////////////
/////////////////////////////////////////////////////////////////////////
void loop() {

  // --- Receive and check packet

  #if RF433_BROADCAST == RF433_MANCHESTER  
    while (!man.receiveComplete()) blink_led(); 
    man.stopReceive();
    len = sensor_data.buflen();
  #else
    senderId = 0;
    packageId = 0;
    while (!receiver.ready()) blink_led();  
    len = receiver.recvPackage((byte *)(sensor_data.bufptr()), &senderId, &packageId);
  #endif
  set_led (LOW);

  if (len<=0) {
    sensor_data.set_name("!ER");
    sensor_data.set((float)0.0);
  } else {
    int chk = sensor_data.check_checksum();
    if (chk != 0) {
      sensor_data.set_name("!CK");
      sensor_data.set((float)chk);
    }
  }
  received = true;
  
  // --- Send through serial (note the use of msglen(). 
  //     Manchester received len includes buffer lenght

  myTransfer.txArray(sensor_data.bufptr(), sensor_data.msglen());
  myTransfer.sendData(len);

 // --- Prepare for another packet 

  #if RF433_BROADCAST == RF433_MANCHESTER  
  my_delay(10);  // Hope it helps
  man.beginReceiveArray(sizeof(sensed_buf_t)+1, (uint8_t *)(sensor_data.x_bufptr()));
  #endif

  // --- Signal packet sent


  wink_led();

}


//////////////////////////////////////////////////////////////////////
// Switch on or off the led
//////////////////////////////////////////////////////////////////////
void set_led(byte s) {
 digitalWrite(LED_PIN, s);
 led_status = s;
}

//////////////////////////////////////////////////////////////////////
// Alternately turn on and off led after LED_BLINK_LIMIT iterations 
//////////////////////////////////////////////////////////////////////
void blink_led() {
  if (LED_wink_count > 0) {
      LED_wink_count--;
      LED_blink_count++;
  } else {
    LED_blink_count++;
    if (LED_blink_count >= LED_BLINK_LIMIT) {
          led_status = led_status==HIGH? LOW: HIGH;
          digitalWrite(LED_PIN, led_status);
          LED_blink_count= 0;    
    }
  }
}

//////////////////////////////////////////////////////////////////////
// Turn on led for a certain amount of (approximate) miliseconds
//////////////////////////////////////////////////////////////////////
void wink_led() {
  if (LED_wink_count < 10) {
    LED_wink_count = LED_WINK_LIMIT;
    set_led(HIGH);
  }
}

//////////////////////////////////////////////////////////////////////
// Don't ask me why delay has been implemented in these two functions
//////////////////////////////////////////////////////////////////////
void boring_count() {
  byte leds = led_status;
  for (unsigned long xx = 0; xx<TUNECOUNT; xx++)
    led_status = !led_status;
  led_status = leds;
 
}
void my_delay(unsigned long msaprox) {
  for (unsigned long k=0; k<msaprox; k++) 
    boring_count();
}
