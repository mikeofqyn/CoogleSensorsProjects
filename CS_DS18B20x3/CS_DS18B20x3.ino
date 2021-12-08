  ///////////////////////////////////////////////////////////////////////////////////
// Send temperature data to an MQTT broker using CoogleSensor lib.
// Temperature is read from multiple DS18B20 sensors
// See CoogleSensor's README.md for documentation
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

//----------------------------------------------------------------------------------
// DS18B20 Temperature sensors library 
// Requires OneWire library
// All data pins are conected in parallel to digital pin 2 and collectivelly pulled
// up with a 4.7K resistor
//

#include <OneWire.h>
#include <DallasTemperature.h>

#define MEASUREMENT_NAME  "DS18B20" 

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);  

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

unsigned long DS_MEASUREMENT_INTERVAL = 20000;  // 20s 

unsigned long last_measurement = millis();

//-----------------------------------------------------------------------------------
// Labels and read values 

char* labels[] = { "temp_1", "temp_2", "temp_3", "temp_4", "temp_5", "temp_6" };
float values[] = { 0., 0., 0., 0., 0., 0. };

const int maxdevices = sizeof(values) / sizeof(float);

int deviceCount = 0;



//
// SETUP
///////////////////////////////////////////////////////////////////////////////////

void setup()
{
    // CoogleSensors library
 
    CS.begin();

	  last_telnet_check = millis();

    // DS18B20

    sensors.begin();  // Start up the library
    deviceCount = sensors.getDeviceCount();

    CS.logPrintf(INFO, "Found %d DS18B20 sensors", deviceCount);
    if ( (deviceCount<1) || (deviceCount > maxdevices) ) {
      CS.logPrintf(ERROR, "Invalid sensor count. Min 1 max %d", maxdevices);
      CS.error("Please change connections");
    }
    while ( (deviceCount<1) || (deviceCount > maxdevices) ) {
      delay(1000);
    } 
    CS.logPrintf(INFO, "Found %d DS18B20 sensors", deviceCount);
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
    if ((millis() - last_measurement) < DS_MEASUREMENT_INTERVAL) {
        return;
    }
    last_measurement = millis();
  
    //
    // Read from sensors
    //
    sensors.requestTemperatures(); 
    
    // Read and display temperature from each sensor
    Tty.print("Temperature: ");
    for (int i = 0;  i < deviceCount;  i++)
    {
      values[i]= sensors.getTempCByIndex(i);
      Tty.print(i+1);
      Tty.print(": ");
      Tty.print(values[i]);
      Tty.print("C  |  ");
    }    
    Tty.print(" |  Free heap: ");
    Tty.print(ESP.getFreeHeap());
    Tty.println(" bytes");

    //
    // Publish
    //
    if (CS.is_online()) { 
      CS.publish_measurement(MEASUREMENT_NAME, deviceCount, labels, values);
    } 
}
