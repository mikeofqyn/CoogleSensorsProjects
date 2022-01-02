///////////////////////////////////////////////////////////////////////////////////
// Send WiFi signal strength data to an MQTT broker using CoogleSensor lib.
// See CoogleSensor's README.md for documentation
//
// See https://eyesaas.com/wi-fi-signal-strength/
// 
///////////////////////////////////////////////////////////////////////////////////


//
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

#include <CoogleSensors.h>

#define MEASUREMENT_NAME  "WiFi"  // Match SENSOR_TYPE (can't stringify)

                             
unsigned long MEASUREMENT_INTERVAL = 10000;  // 10s 

unsigned long last_measurement = millis();

//-----------------------------------------------------------------------------------
// Labels and read values 

char* labels[] = { "dBm" };
float values[] = {  0. };

const int num_values = sizeof(values)/sizeof(float); // 1, hopefullly

//
// SETUP
///////////////////////////////////////////////////////////////////////////////////

void setup()
{
    // CoogleSensors library

    CS.begin();

	  last_telnet_check = millis();

}

//
// MAIN LOOP 
///////////////////////////////////////////////////////////////////////////////////


void loop()
{
    ///
    // Important, do CoogleIOT housekeeping stuff 
    //////////////////////////////////////////////
    CS.loop();   

  	//
  	// Check for incoming telnet connections
  	//
  	if ( (millis()-last_telnet_check) > TELNET_CHECK_INTERVAL) {
  		Tty.check_conn();
  		last_telnet_check = millis();
  	}
	
    //
    // Check if measurement interval has elapsed
    //
    if ((millis() - last_measurement) < MEASUREMENT_INTERVAL) {
        return;
    }
    last_measurement = millis();
  
    //
    // Read data
    //
    long rssi = WiFi.RSSI();

    //
    // Publish
    //
    if (CS.is_online()) {
      values[0] = (float) rssi;
      CS.publish_measurement(MEASUREMENT_NAME, num_values, labels, values);
    } 

    //
    // Echo to console
    //
    Tty.print("MQTT: "); Tty.print(CS.is_online()? "ONLINE" : "OFFLINE");
    Tty.print(" \t "); Tty.print(MEASUREMENT_NAME); 
    Tty.print(" \tRSSI (dBm): ");
    Tty.print(values[0]); 
    Tty.print(" \t Heap: (bytes)");
    Tty.print(ESP.getFreeHeap());
    Tty.println();
  
}
