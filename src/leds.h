
static uint8_t program = 2;
static uint8_t programCount = 6;
long long lastTimeJump = 0;
PositionSwitch lastRightSwitchPosition = Middle;

void setup_leds()
{
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
}

void checkRightSwitchState()
{
    if (lastRightSwitchPosition != input->switchRightOutside)
    {
        lastRightSwitchPosition = input->switchRightOutside;

        Serial.println("switch value changed");

        if (lastRightSwitchPosition == Top)
        {
            if (program < programCount - 1)
            {
                program++;
            }
        }
        else if (lastRightSwitchPosition == Bottom)
        {
            if (program > 0)
            {
                program--;
            }
        }
    }
}

void updateAllLed(uint32_t color)
{
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = color; // CRGB::Blue;
    }
}

void lightUpLed(int led, int radius, CRGB color)
{
    leds[led] = color;

    int pos = led;
    for (int i = 1; i <= radius; i++)
    {
        pos++;

        if (pos >= NUM_LEDS)
        {
            pos = 0;
        }
        leds[pos] = color;
    }

    pos = led;

    for (int i = 1; i <= radius; i++)
    {
        pos--;

        if (pos < 0)
        {
            pos = NUM_LEDS - 1;
        }
        leds[pos] = color;
    }
}

bool performStatus(CRGB color)
{
    if (lastTimeJump + 100 < millis())
    {
        updateAllLed(CRGB::Black);

        if (robot->currentMovingLeg == 0)
        {
            lightUpLed(12, 1, color);
        }
        else if (robot->currentMovingLeg == 1)
        {
            lightUpLed(16, 1, color);
        }
        else if (robot->currentMovingLeg == 2)
        {
            lightUpLed(20, 1, color);
        }
        else if (robot->currentMovingLeg == 3)
        {
            lightUpLed(3, 1, color);
        }
        else if (robot->currentMovingLeg == 4)
        {
            lightUpLed(7, 1, color);
        }

        return true;
    }
    return false;
}

bool performRainbow()
{
    if (lastTimeJump + 5 < millis())
    {
        static uint8_t hueOffset = 0;

        // Verteilung: 256 / 25 LEDs = ~10.24 Farbwerte pro LED
        for (int i = 0; i < NUM_LEDS; i++)
        {
            uint8_t hue = hueOffset + (i * 256 / NUM_LEDS);
            leds[i] = CHSV(hue, 255, 255);
        }

        hueOffset++; // Rotiert den gesamten Ring

        return true;
    }
    return false;
}

bool performRotatingColor(CRGB color)
{
    if (lastTimeJump + 25 < millis())
    {
        static uint8_t offset = 0;

        updateAllLed(CRGB::Black);

        lightUpLed(offset, 2, color);

        offset++;

        if (offset >= NUM_LEDS)
        {
            offset = 0;
        }

        return true;
    }
    return false;
}

void loop_leds()
{
    checkRightSwitchState();

    switch (program)
    {
    case 0:
        if (performStatus(CRGB::Red))
        {
            lastTimeJump = millis();
        }
        break;
    case 1:
        if (performRainbow())
        {
            lastTimeJump = millis();
        }
        break;
    case 2:
        if (performRotatingColor(CRGB::Red))
        {
            lastTimeJump = millis();
        }
        break;
    case 3:
        if (performRotatingColor(CRGB::Blue))
        {
            lastTimeJump = millis();
        }
        break;
    case 4:
        if (performRotatingColor(CRGB::Green))
        {
            lastTimeJump = millis();
        }
        break;
    case 5:
        updateAllLed(CRGB::Black);
        // nothing, turn all off
        break;
    }
}