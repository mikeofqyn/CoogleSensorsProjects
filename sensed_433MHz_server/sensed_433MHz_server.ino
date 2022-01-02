/* 
 * Log sensed packets received via 433 radio to an OLED display and/or
 * serial monitor.
 * 
 */

/////////////////////////////////////////////////////////////////////////
//////////////////////   CONFIGURATION   ////////////////////////////////
/////////////////////////////////////////////////////////////////////////

const int OLED_RESET     =  4;  // Adafruit SSD1306 OLED display reset pin
const int LED_PIN        = 13;  // Led to flash with every packet received
const int RF_INPUT_PIN  =   4;  // Input from RF receiver, MUST be PCINT interrupt-enabled

// LED Control
unsigned long lastMillisLED = millis();    
unsigned const int MaxMillisLED = 1000;

// Verbose debug: UNcomment

#define VERBOSE_DEBUG
 
/////////////////////////////////////////////////////////////////////////
/////////////////////////////// DISPLAY /////////////////////////////////
/////////////////////////////////////////////////////////////////////////

#ifdef OLED_DISPLAY

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#endif

/////////////////////////////////////////////////////////////////////////
////////////////////////////// RF BROADCAST /////////////////////////////
/////////////////////////////////////////////////////////////////////////
// Sources: 
//   mchr3k (Manchester encoding)
//      https://github.com/mchr3k/arduino-libs-manchester
//   Andreas Rohner:
//      https://github.com/zeitgeist87/RFTransmitter (usend in transmitter)
//      https://github.com/zeitgeist87/RFReceiver  (used here)

#define RF433_ZEITGEIST    1    // Andreas Rohner (zeitgeist87) library
#define RF433_MANCHESTER   2    // mchr3k Library 
#define RF433_BROADCAST RF433_MANCHESTER

//-------------------------------------------------------- zeitgeist87's
#if RF433_BROADCAST == RF433_ZEITGEIST
#include <PinChangeInterruptHandler.h>
#include <RFReceiver.h>
unsigned int PULSE_LENGTH = 333;  // Pulse length in microseconds must agree with transmitter
RFReceiver receiver(RF_INPUT_PIN, PULSE_LENGTH);

#elif RF433_BROADCAST == RF433_MANCHESTER
//--------------------------------------------------------- mchr3k's 
#include <Manchester.h>
#define MANCHESTER_SPEED MAN_1200
#else
#error "*** RADIO BROADCAST NOT CONFIGURED ***"
#endif

/////////////////////////////////////////////////////////////////////////
////////////////////////  SENSOR PACKET LIBRARY /////////////////////////
/////////////////////////////////////////////////////////////////////////

// Configure sensed frame size according to RFReceiver limits

#include <sensed.h>
sensed sensor_data(SS_NOT_SET);

/////////////////////////////////////////////////////////////////////////
///////////////////////////  other globals  /////////////////////////////
/////////////////////////////////////////////////////////////////////////

bool received = false;
byte senderId = 0;
byte packageId = 0;
byte len;

/////////////////////////////////////////////////////////////////////////
///////////////////////////  *** SETUP ***  /////////////////////////////
/////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(1);

#ifdef OLED_DISPLAY
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.display();
  display.clearDisplay();
  display.display();
#endif  

#if RF433_BROADCAST == RF433_MANCHESTER  
  man.setupReceive(RF_INPUT_PIN, MANCHESTER_SPEED);
  man.beginReceiveArray(sizeof(sensed_buf_t), (uint8_t *)(sensor_data.x_bufptr()));
#elif RF433_BROADCAST == RF433_ZEITGEIST
  receiver.begin();
#else
#error "INVALID RF433 OPTION: "   RF433_BROADCAST
#endif

  pinMode(LED_PIN, OUTPUT); //
  Serial.print("Mode ");Serial.print(RF433_BROADCAST);
  Serial.println(" initialized. Output: NODE# PKG# NAME VALUE"); 
}

/////////////////////////////////////////////////////////////////////////
////////////////////  *** GEIGER COUNTER LOOP ***  //////////////////////
/////////////////////////////////////////////////////////////////////////
void loop() {

  #ifdef VERBOSE_DEBUG
  Serial.print("Waiting...");
  #endif
  // --- Receive and check packet
  senderId = 0;
  packageId = 0;

  #if RF433_BROADCAST == RF433_MANCHESTER  
  while (!man.receiveComplete()) { delay(1); };
  len = sensor_data.buflen();
  Serial.println(len); /////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<
  #else
  len = receiver.recvPackage((byte *)(sensor_data.bufptr()), &senderId, &packageId);
  #endif

  #ifdef VERBOSE_DEBUG
  Serial.print(" received ");Serial.println(len);
  #endif

  if (len<1) {
    senderId = 0;
    packageId = 0;
    sensor_data.set_name("!ER");
    sensor_data.set((float)0.0);
  } else {
    int chk = sensor_data.check_checksum();
    if (chk != 0) {
      sensor_data.set_name("!CK");
      sensor_data.set((float)chk);
      Serial.print(">");Serial.println((unsigned long)(sensor_data.bufptr()));
    }
  }
  received = true;

  // --- Update LED
  if (received)   {
      digitalWrite(LED_PIN, HIGH);
      if ((millis() - lastMillisLED) >= MaxMillisLED)  {
          received = false;
          digitalWrite(LED_PIN, LOW);
      }
  } else {
    lastMillisLED = millis();    
  }  

  // --- Refresh display
  update_display();

  // --- Ask for another packet 

  #if RF433_BROADCAST == RF433_MANCHESTER  
  man.beginReceiveArray(sizeof(sensed_buf_t), (uint8_t *)(sensor_data.x_bufptr()));
  #endif

}


/////////////////////////////////////////////////////////////////////////
////////////////////////  *** UPDATE DISPLAY *** ////////////////////////
/////////////////////////////////////////////////////////////////////////

void update_display() {

#ifdef OLED_DISPLAY
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(80,9);
  display.println(sensor_data.name());
  }
  // TEXT ON DISPLAY
  float f;
  f = sensor_data.get_float();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(4,0);
  if (f <= 9.99) display.println(f,  1); else display.println((int)f);
  display.setCursor(4,18);
  display.setTextSize(1);
  display.print("Node: "); display.println(senderId);
  display.setCursor(54,0);
  display.println(packageId);
#endif

#define SP(t) Serial.print(t);Serial.print("\t")
    SP(senderId);SP(packageId);SP(sensor_data.name());SP(sensor_data.get_float());
    Serial.println();
#ifdef VERBOSE_DEBUG
    sensor_data.dump(&Serial);
    Serial.println();
#endif  
}
