#include <Adafruit_NeoPixel.h>

#define LED_PIN 6   // The pin your LED strip is connected to
#define NUM_LEDS 50 // Number of LEDs in your strip

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  Serial.begin(9600);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop()
{

  if (Serial.available() > 0)
  {
    String rgbString = Serial.readStringUntil('\n'); // Read the incoming RGB string
    int r, g, b;
    parseRGBString(rgbString, r, g, b);
    setAllLEDs(strip.Color(255, 0, 0)); // Set the color of the LEDs
    strip.show();
  }
}

void parseRGBString(String rgbString, int &r, int &g, int &b)
{
  int firstCommaIndex = rgbString.indexOf(',');
  int secondCommaIndex = rgbString.indexOf(',', firstCommaIndex + 1);

  r = rgbString.substring(0, firstCommaIndex).toInt();
  g = rgbString.substring(firstCommaIndex + 1, secondCommaIndex).toInt();
  b = rgbString.substring(secondCommaIndex + 1).toInt();
}

void setAllLEDs(uint32_t color)
{
  for (int i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, color);
  }
}
