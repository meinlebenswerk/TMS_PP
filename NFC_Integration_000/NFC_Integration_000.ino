#include <SPI.h>
#include <MFRC522.h>
#include <Preferences.h>
#include <ESP32Servo.h>
// #include "Adafruit_LC709203F.h"


// Defines

#define VERSION "0.0.1"
#define UUID_BUFFER_CAPACITY 32

#define RST_PIN           13
#define SS_PIN            12
#define IRQ_PIN           11
// #define SERVO_PIN         18
#define SERVO_PIN         17


#define PIN_R 14 
#define PIN_G 15 
#define PIN_B 16 


// Lock Servo Limits
#define SERVO_REST 90
// #define SERVO_OPEN 90 - 25
// #define SERVO_CLOSED 90 + 40
#define SERVO_OPEN 90 + 15
#define SERVO_CLOSED 90 - 15



// Global variables
MFRC522 mfrc522(SS_PIN, RST_PIN);
// Adafruit_LC709203F lc;

char uuid_buffer[UUID_BUFFER_CAPACITY];
size_t uuid_buffer_size = 0;
Preferences preferences;

Servo servo;

bool locked = false;  // For now in RAM, later in NVStorage
char bound_uuid[UUID_BUFFER_CAPACITY];



// NFC Init
void initialize_nfc() {
  Serial.println("Initializing NFC");
  SPI.begin();
  mfrc522.PCD_Init();
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
  delay(4);

  Serial.println(F("MFRC522 Digital self test"));
  Serial.println(F("*****************************"));
  mfrc522.PCD_DumpVersionToSerial();  // Show version of PCD - MFRC522 Card Reader
  Serial.println(F("-----------------------------"));
  Serial.println(F("Only known versions supported"));
  Serial.println(F("-----------------------------"));
  Serial.println(F("Performing test..."));
  bool result = mfrc522.PCD_PerformSelfTest(); // perform the test
  Serial.println(F("-----------------------------"));
  Serial.print(F("Result: "));
  if (result)
    Serial.println(F("OK"));
  else
    Serial.println(F("DEFECT or UNKNOWN"));
  Serial.println();

  // Zero out the uuid buffer
  memset(uuid_buffer, 0, sizeof(uuid_buffer));
}

// Lock and NVStorage init
void initialize_lock() {
  Serial.println("Initializing Lock...");

  // Init and begin preferences
  preferences.begin("peak-protector", false);

  // locked = preferences.getBool("locked", false);
  locked = false;
  
  // If we're locked, load the UUID from storage
  if(locked) {
    size_t bound_uuid_size = preferences.getInt("bound_uuid_size");
  } else {
    memset(bound_uuid, 0, sizeof(bound_uuid));
  }

  // Initialize the Servo
  servo.attach(SERVO_PIN);
  servo.write(SERVO_REST);

  // Update state
  // servo.write(locked? SERVO_CLOSED : SERVO_OPEN);
}

// Battery monitoring
// void initialize_battery_monitoring() {
//   lc.begin();
//   Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);
//   lc.setThermistorB(3950);
//   Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());
//   lc.setPackSize(LC709203F_APA_1000MAH);
//   lc.setAlarmVoltage(3.8);
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

void initialize_leds() {
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  write_led_color(0, 0, 0);
}

void write_led_color(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(PIN_R, 255 - r);
  analogWrite(PIN_G, 255 - g);
  analogWrite(PIN_B, 255 - b);
} 


void read_in_uuid(char* buffer, size_t* size) {
  buffer[0] = mfrc522.uid.sak;
  memcpy(buffer + 1, mfrc522.uid.uidByte, mfrc522.uid.size);
  *size = mfrc522.uid.size + 1;
}

void setup() {
	Serial.begin(115200);
  // while(!Serial){};
  Serial.println(F("PeakGuard v" VERSION " starting..."));
  
  initialize_lock();
  initialize_nfc();
  initialize_leds();
  // initialize_battery_monitoring();
}

void loop() {
  // Do nothing when there's no new card
	if (!mfrc522.PICC_IsNewCardPresent()) {
    // Serial.println("No new card...");
		return;
	}

	// Select one of the cards
  // TODO! figure out what this means
	if (!mfrc522.PICC_ReadCardSerial()) {
    // Serial.println("Cannot read card serial");
		return;
	}

  // Get the uid of the card
  read_in_uuid(uuid_buffer, &uuid_buffer_size);

  if(locked) {
    // Check if the uuids are equal
    if(memcmp(bound_uuid, uuid_buffer, uuid_buffer_size) == 0) {
      // Yes, unlock
      Serial.println("Unlocked!");

      // Unlock
      locked = false;
      memset(bound_uuid, 0, sizeof(bound_uuid));

      // Update lock state
      servo.write(locked? SERVO_CLOSED : SERVO_OPEN);

      // Update LEDs
      write_led_color(0, 255, 16);
      delay(500);
      write_led_color(0, 0, 0);
    } else {
      // No - warning!
      Serial.println("Go away!");

      // Flash LEDs red
      for(int i=0; i<2; i++) {
        write_led_color(255, 0, 0);
        delay(200);
        write_led_color(0, 0, 0);
        delay(200);
      }
      // And turn off
      write_led_color(0, 0, 0);
    }
  } else {
    // Bind to card:
    Serial.println("Bound to new card!");

    // Lock
    locked = true;
    memcpy(bound_uuid, uuid_buffer, uuid_buffer_size);

    // Update lock state
    servo.write(locked? SERVO_CLOSED : SERVO_OPEN);

    // Update LEDs
    write_led_color(0, 64, 255);
    delay(500);
    write_led_color(0, 0, 0);
  }

  // Do this to stop the loop?
  mfrc522.PICC_HaltA();
}
