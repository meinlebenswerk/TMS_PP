#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>


// Defines

#define VERSION "0.0.1"
#define UUID_BUFFER_CAPACITY 32

#define RST_PIN         13
#define SS_PIN          12
#define IRQ_PIN         11




// Global variables
MFRC522 mfrc522(SS_PIN, RST_PIN);
Adafruit_NeoPixel strip(23, 10, NEO_GRB + NEO_KHZ800);

char uuid_buffer[UUID_BUFFER_CAPACITY];
size_t uuid_buffer_size = 0;
Preferences preferences;

bool locked = false;  // For now in RAM, later in NVStorage
char bound_uuid[UUID_BUFFER_CAPACITY];



// NFC Init
void initialize_nfc() {
  Serial.println("Initializing NFC");
  SPI.begin();
  mfrc522.PCD_Init();
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
  delay(4);

  // Zero out the uuid buffer
  memset(uuid_buffer, 0, sizeof(uuid_buffer));
}

// Lock and NVStorage init
void initialize_lock() {
  Serial.println("Initializing Lock...");

  // Init and begin preferences
  preferences.begin("peak-protector", false);



  locked = false;
  memset(bound_uuid, 0, sizeof(bound_uuid));
}

void set_strip_color(uint32_t color) {
  for(int i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void initialize_leds() {
  strip.begin();
  strip.show();
  strip.setBrightness(255);
}


void read_in_uuid(char* buffer, size_t* size) {
  buffer[0] = mfrc522.uid.sak;
  memcpy(buffer + 1, mfrc522.uid.uidByte, mfrc522.uid.size);
  *size = mfrc522.uid.size + 1;
}

void setup() {
	Serial.begin(115200);

  Serial.println(F("PeakGuard v" VERSION " starting..."));
  
  initialize_lock();
  initialize_nfc();
  initialize_leds();

  Serial.println(sizeof(uuid_buffer));
}

void loop() {
  // Do nothing when there's no new card
	if (!mfrc522.PICC_IsNewCardPresent()) {
		return;
	}

	// Select one of the cards
  // TODO! figure out what this means
	if (!mfrc522.PICC_ReadCardSerial()) {
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

      // Update LEDs
      colorWipe(strip.Color(0, 255, 16), 33);
      // And turn off
      set_strip_color(strip.Color(0, 0, 0));
      
    } else {
      // No - warning!
      Serial.println("Go away!");

      // Flash LEDs red
      for(int i=0; i<2; i++) {
        set_strip_color(strip.Color(255, 0, 0));
        delay(200);
        set_strip_color(strip.Color(0, 0, 0));
        delay(200);
      }
      // And turn off
      set_strip_color(strip.Color(0, 0, 0));
    }
  } else {
    // Bind to card:
    Serial.println("Bound to new card!");

    // Lock
    locked = true;
    memcpy(bound_uuid, uuid_buffer, uuid_buffer_size);

    // Update LEDs
    colorWipe(strip.Color(0, 64, 255), 33);
    // And turn off
    set_strip_color(strip.Color(0, 0, 0));
  }

  // Do this to stop the loop?
  mfrc522.PICC_HaltA();
}
