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


//----------------------------------------------------------------------------------
// DHT Humidity / Temperature sensors library 
// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
//
// dec 19 2021 added monitoring of wifi strenght


#define MONITOR_WIFI  // comment to disable wifi strenght monitoring


#include <DHT.h>

#define MEASUREMENT_NAME  "DHT11"  // Must match SENSOR_TYPE with quotes (can't stringify)
#define SENSOR_TYPE DHT11          // DHT11 or DHT22
                                   // FOR DHT22 A 10K PULL-UP BETWEEN DATA PIN AND VCC IS NEEDED


                             
#define DHT_SENSOR_PIN 2     // DHT SENSOR DATA PIN IN GPIO2
                             
                          
DHT dht(DHT_SENSOR_PIN, SENSOR_TYPE) ;


unsigned long DHT_MEASUREMENT_INTERVAL = 20000;  // 20s 

unsigned long last_measurement = millis();


//-----------------------------------------------------------------------------------
// Labels and read values 

char* labels[] = { "temperature", "humidity", "dBm" };
float values[] = { 0., 0., 0. };

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

#ifdef MONITOR_WIFI
    values[2] = (float) WiFi.RSSI();;
    int nvalues = 3;
#else    
    int nvalues = 2;
#endif

    //
    // Publish
    //
    if (CS.is_online()) {
      values[0] = temperature;
      values[1] = humidity;
      CS.publish_measurement(MEASUREMENT_NAME, nvalues, labels, values);
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
    Tty.print(" \t Heap: (bytes) ");
    Tty.print(ESP.getFreeHeap());
#ifdef MONITOR_WIFI
    Tty.print(" \tRSSI (dBm): ");
    Tty.print(values[2]); 
#endif
    Tty.println();
  
}
