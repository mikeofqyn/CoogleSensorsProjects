#ifndef COGGLEIOT_HUB_H
#define COGGLEIOT_HUB_H

//
// Configuration
//

#define IOT_HUB_VERSION  "2.0"  // Jan. 2022


//
// Topics to publish to
//
const char IOT_HUB_SENSOR_TOPIC_PREFIX[]  =  "sensed/hub/";  // prefix for sensed library packets

//
// Status srtings and error codes
//
const char IOT_HUB_ERROR_CHECKSUM[]       =  "BAD_CHECKSUM";
const char IOT_HUB_ERROR_CRC[]            =  "CRC_ERROR";
const char IOT_HUB_ERROR_PAYLOAD[]        =  "PAYLOAD_ERROR";
const char IOT_HUB_ERROR_STOP_BYTE[]      =  "STOP_BYTE_ERROR";
const char IOT_HUB_ERROR_UNKNOWN_SERIAL[] =  "UNKNOWN_SERIAL_ERROR";

#include <Arduino.h>

#endif
