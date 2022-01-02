 
///////////////////////////////////////////////////////////////////////////////////
// COOGLEIOT FRAMEWORK AND PUBSUBCLIENT MQTT CLIENT FOR SENSED LIBRARY PACKETS
//
// Listen on (software) serial line and RF packets for data packets (sensed buffers
// defined in sensed.h). Format them as jSON strings and handle to a MQTT broker.
//
//
// Versión 4.0 - 01 jan 2022 - Adapted to use the CoogleSensors Library.
// This library uses John Coggeshall's CoogleIOT Framework for IOT systems using WiFi
//    http://www.thissmarthouse.net/
//    https://www.arduinolibraries.info/libraries/coogle-iot
//    https://github.com/ThisSmartHouse/CoogleIOT
//
// Nick O'Leary's PubSubClient MQTT Client (used by CoogleIOT)
//    https://github.com/knolleary
//    https://github.com/knolleary/pubsubclient
//    https://pubsubclient.knolleary.net/api.html    << READ THIS
//
//                **** W A R N I N G ****
//
// Some configuration has to be made on the PubSubClient library which cannot be
// made here, since #defines in the main .ino files aren't seen by the compiler 
// while compiling the libraries' .cpp files. 
//
// The default packet size of 128 won't work with CoogleIOT as explained in the 
// documentation (MQTT Client Notes in the GitHub project page).
//
// CHANGES MADE TO PubSubCient.h
 /*  
 **   
 **  // MQTT_MAX_PACKET_SIZE : Maximum packet size  
 *>  // --  APR 15 2020 CHANGED FROM 128 TO 1024 FOR COMPATIBILITY WITH COOGLEIOT
 **  #ifndef MQTT_MAX_PACKET_SIZE
 *>  #define MQTT_MAX_PACKET_SIZE 1024  // WAS 128
 **  #endif
 **  
 */
#include <time.h>

//--------------------------------------------------------------------------------
// TelnetSerial allows a Serial-like interface through Telenet connection
//

#include <TelnetSerial.h>

TelnetSerial Tty(115200);

#define TELNET_CHECK_INTERVAL 200 // Check fo incoming connections each 200 ms

unsigned long last_telnet_check = 0;

//--------------------------------------------------------------------------------
// CoogleSensor provides a configuration web portal and MQTT integration using the
// CoogleIOT library by John Coggeshall
// (https://github.com/ThisSmartHouse/CoogleIOT
//                                                                          

#include <CoogleSensors.h>

CoogleSensors CS(Tty);

#include "CoogleIOT_hub.h"


///////////////////////////////////////////////////////////////////////////////////
// NRF24Lite LIBRARY FOR NRF24L01 RADIO MODULES - Library and globals
//
//
#include <SPI.h>
#include <NRFLite.h>

NRFLite  radioNRF;

//
//  SPI pins on nodeMCU are: CSN-D8, MOSI-D7, MISO-D6, SCK-D5
//  These are passed in NRF24Lite initializarion
//
#define CE_PIN   D3  // GPIO0  Can be any digital pin
#define CS_PIN   D8  // GPIO15 Fixed. NodeMCU SPI pins: CSN-D8, MOSI-D7, MISO-D6, SCK-D5
//

const static uint8_t   RADIO_ID = 0;  // This station ID

///////////////////////////////////////////////////////////////////////////////////
// SERIALTRANSFER LIBRARY AND SOFTWARE SERIAL
//
// Robustness of serial comms is enhanced by the SerialTransfer packet-oriented 
// library SerialTransfer by PowerBroker2:
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
//   COMPILE_RCV MUST BE DEFINED FOR THIS PROGRAM
//
// The AtTiny side uses the SendOnlySoftwareSerial library by NickGammon. Here
// we use standard SoftwareSerial (in the hope that they interoperate).
//

// Arduino NodeMCU core uses GPIO pin numbers.
#define SERIAL_RX_PIN 4   // GPIO 04 Board pin D2
#define SERIAL_TX_PIN 5   // GPIO 05 Board pin D1

#define SOFTWARE_UART_SPEED 9600 // Must match that of the transmitter

#include <SoftwareSerial.h>

SoftwareSerial sw_UART (SERIAL_RX_PIN,SERIAL_TX_PIN);      

#include "mod_SerialTransfer.h"  // Version modified to fit, with fake CRC

SerialTransfer myTransfer;

///////////////////////////////////////////////////////////////////////////////////
// SENSED  PACKETS LIB 
//
// Be careful to check sensed max data frame size against other libraries limits
//
#include <sensed.h>
#include <sensed_extensions.h>

sensed serial_data_1(SS_NOT_SET);
sensed serial_data_2(SS_NOT_SET);
sensed radio_data_1(SS_NOT_SET);
sensed radio_data_2(SS_NOT_SET);

// Packets may be resent by the sender to ensure delivery. In this case two 
// consecutive packets will be identical, includen microsec timestamp

sensed *serial_input_cur = &serial_data_1;
sensed *serial_input_prv = &serial_data_2;
sensed *radio_input_cur = &radio_data_1;
sensed *radio_input_prv = &radio_data_2;
sensed *data_tmp = NULL;

#define swap_bufs(cur, prv) {data_tmp = cur; cur = prv; prv = data_tmp; };

#define IOT_HUB_MAX_FLOATS_PER_MESSGE  10


///////////////////////////////////////////////////////////////////////////////////
// Stats
//

// Packets
unsigned long serial_rcvd = 0;  // Received packets
unsigned long serial_dupl = 0;  // Duplicate packets
unsigned long serial_errs = 0;  // SerialTransfer errors
unsigned long radio_rcvd = 0;   // NRF24L01 Received packets
unsigned long radio_dupl = 0;   // NRF24L01 Duplicate packets
unsigned long radio_errs = 0;   // NRF24L01 errors
unsigned long chksum_errs = 0;  // checksum errors detected by sensed library
unsigned long sensed_sent = 0;  // sensed packets handled to mqtt
unsigned long json_buf_ovf = 0; // Number of json buffer overflows (should be 0)
unsigned long json_msg_ovf = 0; // Number of json message overflows (should be 0)
unsigned long serialize_errors = 0;  // Errors serializing to buffer (shold never happen)
unsigned long publish_errors = 0;    // Unpublished messages due to errore
unsigned long invalid_messages = 0;  // Number of invalid SENSED messages

///////////////////////////////////////////////////////////////////////////////////
// Other parameters and globals
//

unsigned long last_status_millis = 0;


///////////////////////////////////////////////////////////////////////////////////
// ArduinoJson
//
// TO DO: ArduinoJson.h is included in CoogleIOT directory, must decide whether to
// use this an move it to the libraries directory or delete it and install it anew 
// from the libraries manager and then check for compatibility issues. (try a diff?)
//
// Apart from the excellent book, much help was provided by the online assistant:
//
// https://arduinojson.org/v5/assistant/
//
          
#include <ArduinoJson.h>

//
// Sizes estimated with ample margin. See README_json.txt
//
#define IOT_HUB_JSON_BUFFER_SIZE        800   // Memory used by ArduinoHson library
#define IOT_HUB_MAX_JSON_MESSAGE_LEN    640   // Ouput message max size

///////////////////////////////////////////////////////////////////////////////////
// Prototypes 
// Not needed in Arduino, but old programmers cherish healthy habits
//

static const char notavailable[] = "N/A";
const char* na(const char *data) {return data? data: notavailable; }

//
// Publishing 
//
int iot_hub_publish_stats();
int iot_hub_publish_sensed (sensed *data, const char *from = NULL);


///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// UTILITY ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void process_packet(sensed **cur, sensed **prv, unsigned long &duplic, const char *from)
{
  sensed *pkt_cur = *cur;
  sensed *pkt_prv = *prv;
 
  //
  // Check checksum
  //
  int chk = pkt_cur->check_checksum();
  if (chk != 0) {  // Sensed packet corrupted?
     chksum_errs++;  // Checksum errors
     CS.publish_error(IOT_HUB_ERROR_CHECKSUM, "sensed", from);
     return;
  } 

  //
  // Check for duplicates
  //
  if (memcmp(pkt_cur->bufptr(), pkt_prv->bufptr(), pkt_cur->msglen()) == 0) {
    duplic++;  // Duplicate packets
    // CS.logPrintf(DEBUG, "Duplcate packet: Type %d name %s", (int)pkt_cur->type(), pkt_prv->name());
    return;
  }

  //
  // Publish data
  //
  iot_hub_publish_sensed (pkt_cur, from);

  //
  // Swap current<->previous for duplicates checking
  //
  swap_bufs(*cur, *prv);  // Swap current <--> previous   <== IMPORTANT !!
}

///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// SETUP / ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void setup()
{
  //
  // CoogleSensors library
  //

    CS.begin();

    last_telnet_check = millis();

  
  //
  // SerialTransfer
  //
   sw_UART.begin(SOFTWARE_UART_SPEED);
   myTransfer.begin(sw_UART);
   CS.logPrintf(INFO, "Serial input started on GPIO pin %d at %d bps", SERIAL_RX_PIN, SOFTWARE_UART_SPEED);

 //
 // NRF24L01 radio
 //
  CS.info("Initializing NRF24 transceiver");
  if (!radioNRF.init(RADIO_ID, CE_PIN, CS_PIN))
  {
    CS.error("IOT Hub: couldn't initialize transceiver");
    CS.flashStatus(200, 50); // flash 50 times, 200ms on 200ms on
  }

  //
  // Finished 
  //
  CS.info("Hub initialized");

}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// MAIN LOOP ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void loop()
{
  ////////////////////////////////////////////////
  // Important, do CoogleIOT housekeeping stuff //
  ////////////////////////////////////////////////
  CS.loop();   // IMPORTANT


  //
  // Check for incoming telnet connections
  //
  if ( (millis()-last_telnet_check) > TELNET_CHECK_INTERVAL) {
    Tty.check_conn();
    last_telnet_check = millis();
  }

  //
  // Input from radio?
  //
    if (radioNRF.hasData())
    {
      radio_rcvd++;
      memset (radio_input_cur->bufptr(), 0, radio_input_cur->bufsize());
      // No errors, either good data or no data at all
      radioNRF.readData(radio_input_cur->bufptr()); // Warning buffer must be 32 bytes long at least
      // DEBUG
      radio_input_cur->dump(&Serial);
      process_packet (&radio_input_cur, &radio_input_prv, radio_dupl, "radio");
    }

  //
  // Input from serial?
  //
  uint8_t len = myTransfer.available();
  if(len)
  {
    serial_rcvd++;  // Received packets
    myTransfer.rxArray((uint8_t *)(serial_input_cur->bufptr()), len);
    // DEBUG: NOTIFY RCV
    // CS.logPrintf(DEBUG, "Reveived serial packet: %d", (int)len);  
    // DUMP BUFFER
    // for (int k2=0; k2<len; k2++) {Serial.print(*(serial_input_cur->bufptr()+k2));Serial.print(" "); } Serial.println();
    // DUMP OBJECT
    // serial_input_cur->dump(&Serial);
    if(myTransfer.status < 0) {   // Serial transfer error?
        const char *err = IOT_HUB_ERROR_UNKNOWN_SERIAL;
        switch (myTransfer.status)
        {
            case -1:  err = IOT_HUB_ERROR_CRC;             break;
            case -2:  err = IOT_HUB_ERROR_PAYLOAD;         break;
            case -3:  err = IOT_HUB_ERROR_STOP_BYTE;       break;
        }
        CS.publish_error(err, "SerialTransfer");
        serial_errs++;
    } 
    else
    {
        process_packet (&serial_input_cur, &serial_input_prv, serial_dupl, "serial");
    }
  }
  
  // LOOP AGAIN
}



///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// FUNCTIONS FOR PUBLISHING ////////////////////////
///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
// Publish SENSED  data packet
//
int iot_hub_publish_sensed (sensed *data, const char *from) {
  
  int len = 0;  
  StaticJsonBuffer<IOT_HUB_JSON_BUFFER_SIZE> jb;
  JsonObject& obj = jb.createObject();
  //
  // Add headers
  //
  CS.JSON_header(obj);
  
  //
  // Build message
  //
  uint8_t msgtype = data->type();
  int msgclass = sensor_class(msgtype);
  int datalen  = data->len();

  /*DEBUG CS.logPrintf(INFO, "Type: 0x%02X  class:%d  len:%d ", data->type(), msgclass, datalen); */

  int nelements = datalen / sizeof(float);
  int typeindex = sensed_typeindex(msgtype);

  char typestr[8];
  sprintf(typestr, "0x%02X", msgtype);
  
  obj["msg_version"] = data->version();
  obj["name"] = na(data->name());
  obj["id"]   = data->id();
  obj["class"] = msgclass;
  obj["type"] = typestr;
  obj["type_name"] = sensed_typename(typeindex);
  obj["elapsed_usec"] = data->elapsed();
  obj["data_lenght"] = datalen;
  obj["thru"] = from;

  obj["n_data"] = nelements;
  
  const char *vname;
  float varray[SENSED_MAX_FLOATS_PER_MESSGE];
  int expected;

  switch (msgclass)
  {
    case 0:  // No data with optional string message
    case 1:  // 1 String message
      vname = sensed_valuename(typeindex, 1);
      if (datalen > 0) obj[vname] = data->get_str();
      break;
    case 2:  // 1 float vaule
    case 3:  // array of n float values 
      if ( (datalen % sizeof(float)) != 0) {
        obj["ERROR"] = "Inconsistent data array length for message class";
        break;
      }
      expected = sensed_nvalues(typeindex);
      if ( nelements != expected ) {
        obj["ERROR"] = "invalid number of data elements";
        obj["EXPECTED_ELEMENTS"] = expected;
        invalid_messages++;
        break;
      }
      data->move_bytes(varray, datalen);
      for (int i=0; i<nelements; i++) {
        if (sensed_vtype(typeindex) != sensed_flt) {
          obj["ERROR"] = "invalid type for data element";
          obj["EMENT_NUMBER"] = i;
          break;
        }
        vname = sensed_valuename(typeindex, i);
        obj[vname] = varray[i];
      }
      break;
    default:  // Other classes 
        obj["ERROR"] = "Sensor class not supported";
  }
  //
  // Compose topic
  //
  char topic [sizeof(IOT_HUB_SENSOR_TOPIC_PREFIX)+SENSED_SENSOR_NAME_LENGTH + 1] ="";  // SENSED_SENSOR_NAME_LENGTH in sensed.h
  strncpy(topic, IOT_HUB_SENSOR_TOPIC_PREFIX, sizeof(IOT_HUB_SENSOR_TOPIC_PREFIX));
  strcat(topic, na(data->name())); // sensed::name() ensures name is null-terminated
  
  //
  // Send
  // 
  int iret = CS.publish_JSON(topic, obj, true);
  if (iret >= 0)
  {
    sensed_sent++;
  }
  return iret;
}
