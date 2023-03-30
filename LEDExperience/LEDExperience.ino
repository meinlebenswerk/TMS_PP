#include "RGBConverterLib.h"
#include "Adafruit_LC709203F.h"


#define PIN_R 14 
#define PIN_G 15 
#define PIN_B 16 

// Battery monitoring
Adafruit_LC709203F lc;
void initialize_battery_monitoring() {
  lc.begin();
  Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);
  lc.setThermistorB(3950);
  Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());
  lc.setPackSize(LC709203F_APA_1000MAH);
  lc.setAlarmVoltage(3.8);
}

// enum DeviceState {
//   IDLE,
//   LOCKING,
//   UNLOCKING,
//   INVALID_CARD
// };

// Global?


// unsigned long last_status_light_update = 0;
// unsigned long last_status_light_battery_update = 0;
// void update_status_light(DeviceState status) {
//   switch(status) {
//     case IDLE:
//       Serial.print("IDLE");
//       return;
//   }
// }

// void print_battery_state() {
//   // We can query this every few (~3s)
//   Serial.print("Batt_Voltage:");
//   Serial.print(lc.cellVoltage(), 3);
//   Serial.print("\t");
//   Serial.print("Batt_Percent:");
//   Serial.print(lc.cellPercent(), 1);
//   Serial.print("\t");
//   Serial.print("Batt_Temp:");
//   Serial.println(lc.getCellTemperature(), 1);
// }

void setup() {


  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);

  initialize_battery_monitoring();
}

void write_color(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(PIN_R, 255 - r);
  analogWrite(PIN_G, 255 - g);
  analogWrite(PIN_B, 255 - b);
}

void loop() {
  uint8_t r, g, b;

  // Get the battery state
  Serial.print("Batt_Percent:");
  Serial.println(lc.cellPercent(), 1);

  // 0 to 120
  // RGBColor color = ColorConverter.HSItoRGB(lc.cellPercent() * 120.0 , 100, 100);
  RGBConverter::HslToRgb(lc.cellPercent() * 120.0, saturation, lighting, red, green, blue);
  write_color(color.red, color.green, color.blue);

  // Delay
  delay(3000);
}
