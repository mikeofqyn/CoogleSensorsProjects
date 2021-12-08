#include <ESP8266WiFi.h>
#include <CS_static_IPs.h>  // Parte de librería CoogleSensors

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n\n\n\n\n\n\n");
  Serial.print("MCU inicializado - ");
  Serial.println(WiFi.macAddress());
  Serial.println("\n\n\n");
  pinMode(LED_BUILTIN, OUTPUT);

  csStaticConfigEntry* scfg = cs_GetStaticConfig(WiFi.macAddress().c_str());

#define printlin(lbl,add) Serial.print(lbl); Serial.println(scfg->add.toString());

  if (scfg) {
    Serial.println("**  STATIC NETWORK CONFIGURATION HARDWIRED FOR THIS MAC ADDRESS **\n");
    Serial.print  ("    MAC: "); Serial.println(scfg->mac_addr);
        printlin("       IP: ",address);
        printlin("  gateway: ",gateway);
        printlin("     mask: ",subnet);
        printlin("     dns1: ",dns1);
        printlin("     dns2: ",dns2);
  } else {
    Serial.println("-- NO STATIC NETWORK CONFIGURATION HARDWIRED IN CODE --");
  }
  Serial.println();
  
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);   
  // Turn the LED on (Note that LOW is the voltage level
  // but actually the LED is on; this is because
  // it is active low on the ESP-01)
  delay(400);                       // Wait .4 seconds
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  delay(800);                      //  Wait .8 seconds (to demonstrate the active low LED)

}
