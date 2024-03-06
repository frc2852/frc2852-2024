#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN     6  // The pin your LED strip is connected to
#define NUM_LEDS    50 // Number of LEDs in your strip

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

enum LEDMode {
  ON,
  OFF
};

LEDMode currentMode = OFF;

void setup() {
  Wire.begin(0x01); 
  Wire.onReceive(receiveEvent); // Register event for receiving I2C data
  strip.begin();
  updateLEDs(currentMode);
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  updateLEDs(currentMode);
  // You can add any continuous animations or checks here
}

void receiveEvent(int howMany) {
  while (Wire.available()) {
    int mode = Wire.read(); // Receive byte as an integer
    currentMode = (LEDMode)mode; // Cast integer to LEDMode enum
    updateLEDs(currentMode); // Update LEDs based on the mode
  }
}

void updateLEDs(LEDMode mode) {
  switch (mode) {
    case ON:
      setAllLEDs(strip.Color(0, 0, 255)); // Green
      break;
    case OFF:
      setAllLEDs(strip.Color(0, 0, 0));
      break;
  }
  strip.show();
}


void setAllLEDs(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
}
