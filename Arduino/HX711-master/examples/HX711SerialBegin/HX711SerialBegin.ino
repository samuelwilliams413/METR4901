#include "HX711.h"

HX711 scale;

void setup() {
  Serial.begin(38400);
  Serial.println("HX711 Demo");

  Serial.println("Initializing the scale");
  // parameter "gain" is ommited; the default value 128 is used by the library
  // HX711.DOUT	- pin #A1
  // HX711.PD_SCK	- pin #A0
  scale.begin(A1, A0);

}

void loop() {
  Serial.println(scale.read());

  scale.power_down();			        // put the ADC in sleep mode
  delay(50);
  scale.power_up();
}

