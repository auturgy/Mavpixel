/*
 * Source below found here: http://www.kasperkamperman.com/blog/arduino/arduino-programming-hsb-to-rgb/
 */

//Color conversion and pixel functions

void hsvToRgb24(hsvColor_t* c, CRGB* r)
{
    uint16_t val = c->v;
    uint16_t sat = 255 - c->s;
    uint32_t base;
    uint16_t hue = c->h;

    if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
        r->r = val;
        r->g = val;
        r->b = val;
    } else {

        base = ((255 - sat) * val) >> 8;

        switch (hue / 60) {
            case 0:
            r->r = val;
            r->g = (((val - base) * hue) / 60) + base;
            r->b = base;
            break;
            case 1:
            r->r = (((val - base) * (60 - (hue % 60))) / 60) + base;
            r->g = val;
            r->b = base;
            break;

            case 2:
            r->r = base;
            r->g = val;
            r->b = (((val - base) * (hue % 60)) / 60) + base;
            break;

            case 3:
            r->r = base;
            r->g = (((val - base) * (60 - (hue % 60))) / 60) + base;
            r->b = val;
            break;

            case 4:
            r->r = (((val - base) * (hue % 60)) / 60) + base;
            r->g = base;
            r->b = val;
            break;

            case 5:
            r->r = val;
            r->g = base;
            r->b = (((val - base) * (60 - (hue % 60))) / 60) + base;
            break;

        }
    }
}

#ifdef LED_STRIP

void ledSetup() {
  FastLED.addLeds<NEOPIXEL, NEO_PIN1>(ledrgb, 8);
  FastLED.addLeds<NEOPIXEL, NEO_PIN2>(&ledrgb[8], 8);
  FastLED.addLeds<NEOPIXEL, NEO_PIN3>(&ledrgb[16], 8);
  FastLED.addLeds<NEOPIXEL, NEO_PIN4>(&ledrgb[32], 8);
}

void scaleLedValue(uint16_t index, const uint8_t scalePercent)
{
    ledhsv[index].v = ((uint16_t)ledhsv[index].v * scalePercent / 100);
}
 
void setLedHsv(uint16_t index, const hsvColor_t *color)
{
    ledhsv[index].h = color->h;
    ledhsv[index].s = color->s;
    ledhsv[index].v = color->v;
}

void getLedHsv(uint16_t index, hsvColor_t *color)
{
    color->h = ledhsv[index].h;
    color->s = ledhsv[index].s;
    color->v = ledhsv[index].v;
}

void setBrightness(uint8_t v){
  FastLED.setBrightness(v);
}

void show() {
  for (int i = 0; i < MAX_LED_STRIP_LENGTH; i++) 
    hsvToRgb24(&ledhsv[i], &ledrgb[i]);
  FastLED.show();
}

#endif
