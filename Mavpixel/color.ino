/*
 * Source below found here: http://www.kasperkamperman.com/blog/arduino/arduino-programming-hsb-to-rgb/
 */

//Color conversion and pixel functions

void rgbToHsv24(const CRGB* r, hsvColor_t* h)
{
    uint8_t min, max, delta;
    int16_t hue;

    if(r->r < r->g) min = r->r; else min = r->g;
    if(r->b < min) min = r->b;
    if(r->r > r->g) max = r->r; else max = r->g;
    if(r->b > max) max = r->b;
    h->v = max;                // v, 0..255
 
    delta = max - min;                      // 0..255, < v
    if( max != 0 ) h->s = (int)(delta)*255 / max;        // s, 0..255
    else {// r = g = b = 0        // s = 0, v is undefined
      h->s = 0;
      h->h = 0;
      return;
    }
    if( r->r == max ) hue = (r->g - r->b) * 60 / delta;        // between yellow & magenta
    else if( r->g == max ) hue = 120 + (r->b - r->r) * 60 / delta;    // between cyan & yellow
    else hue = 240 + (r->r - r->g) * 60 / delta;    // between magenta & cyan
 
    if( hue < 0 ) hue += 360;
    h->h = hue;
}

void hsvToRgb24(const hsvColor_t* c, CRGB* r)
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
  hsvColor_t h;
  rgbToHsv24(&ledrgb[index], &h);
  h.v = ((uint16_t)ledhsv[index].v * scalePercent / 100);
  hsvToRgb24(&h, &ledrgb[index]);
}
 
void setLedHsv(uint16_t index, const hsvColor_t *color)
{
  hsvToRgb24(color, &ledrgb[index]);  
  //rgbColor24bpp_t* r = hsvToRgb24(color);
    //leds[index].setRGB(r->rgb.r, r->rgb.g, r->rgb.b);
//    ledhsv[index].h = color->h;
//    ledhsv[index].s = color->s;
//    ledhsv[index].v = color->v;
}

void getLedHsv(uint16_t index, hsvColor_t *color)
{
    //uint32_t c;
    rgbToHsv24(&ledrgb[index], color);
//    color->h = ledhsv[index].h;
//    color->s = ledhsv[index].s;
//    color->v = ledhsv[index].v;
}

void setBrightness(uint8_t v){
  FastLED.setBrightness(v);
}

void show() {
  FastLED.show();
}

#endif
