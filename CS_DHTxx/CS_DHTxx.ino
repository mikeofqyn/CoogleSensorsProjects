///////////////////////////////////////////////////////////////////////////////////
// Send temperature and humidity data to an MQTT broker using CoogleSensor lib.
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
// DHT Humidity / Temperature sensors library 
// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <DHT.h>

#define SENSOR_TYPE DHT22    // DHT11 or DHT22
                             // FOR DHT22 A 19K PULL-UP BETWEEN DATA PIN AND VCC IS NEEDED
#define DHT_SENSOR_PIN 2     // DHT SENSOR DATA PIN IN GPIO2
                             
#define MEASUREMENT_NAME  "DHT22" 
                          
DHT dht(DHT_SENSOR_PIN, SENSOR_TYPE) ;


unsigned long DHT_MEASUREMENT_INTERVAL = 20000;  // 20s 

unsigned long last_measurement = millis();

//-----------------------------------------------------------------------------------
// Labels and read values 

char* labels[] = { "temperature", "humidity" };
float values[] = { 0., 0. };

//
// SETUP
///////////////////////////////////////////////////////////////////////////////////

void setup()
{
    // CoogleSensors library

    CS.begin();

	  last_telnet_check = millis();

    // DHT

    dht.begin();

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
    if ((millis() - last_measurement) < DHT_MEASUREMENT_INTERVAL) {
        return;
    }
    last_measurement = millis();
  
    //
    // Read from sensor 
    //
    dht.read();
    delay(300);
    float temperature = dht.readTemperature();
    delay(300);
    float humidity = dht.readHumidity();

    //
    // Publish
    //
    if (CS.is_online()) {
      values[0] = temperature;
      values[1] = humidity;
      CS.publish_measurement(MEASUREMENT_NAME, 2, labels, values);
    } 

    //
    // Echo to console
    //
    Tty.print("MQTT: "); Tty.print(CS.is_online()? "ONLINE" : "OFFLINE");
    Tty.print(" \t "); Tty.print(MEASUREMENT_NAME); 
    Tty.print(" \tHumidity (%): ");
    Tty.print(humidity, 1);
    Tty.print(" %\tTemp: (C)");
    Tty.print(temperature, 1);
    Tty.print(" \t Heap: (bytes)");
    Tty.print(ESP.getFreeHeap());
    Tty.println();
  
}
