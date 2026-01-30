
void updateAllLed(uint32_t color)
{
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = color; // CRGB::Blue;
    }
}

void performRainbow()
{
    static uint8_t hueOffset = 0;

    // Verteilung: 256 / 25 LEDs = ~10.24 Farbwerte pro LED
    for (int i = 0; i < NUM_LEDS; i++)
    {
        uint8_t hue = hueOffset + (i * 256 / NUM_LEDS);
        leds[i] = CHSV(hue, 255, 255);
    }

    hueOffset++; // Rotiert den gesamten Ring
}