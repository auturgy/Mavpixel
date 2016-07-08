/*
 * Mavpixel Mavlink Neopixel bridge
 * (c) 2016 Nick Metcalfe
 * This file is derived from Cleanflight.
 *
 * Mavpixel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Please read licensing, redistribution, modifying, authors and 
 * version numbering from main sketch file. This file contains only
 * a minimal header.
 *
 * Mavpixel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Mavpixel.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Source below found here: http://www.kasperkamperman.com/blog/arduino/arduino-programming-hsb-to-rgb/
 */

void rgbToHsv24(uint8_t r, uint8_t g, uint8_t b, hsvColor_t* h)
{
    uint8_t min, max, delta;
    int16_t hue;

    if(r < g) min = r; else min = g;
    if(b < min) min = b;
    if(r > g) max = r; else max = g;
    if(b > max) max = b;
    h->v = max;                // v, 0..255
 
    delta = max - min;                      // 0..255, < v
    if( max != 0 ) h->s = (int)(delta)*255 / max;        // s, 0..255
    else {// r = g = b = 0        // s = 0, v is undefined
      h->s = 0;
      h->h = 0;
      return;
    }
    if( r == max ) hue = (g - b) * 60 / delta;        // between yellow & magenta
    else if( g == max ) hue = 120 + (b - r) * 60 / delta;    // between cyan & yellow
    else hue = 240 + (r - g) * 60 / delta;    // between magenta & cyan
 
    if( hue < 0 ) hue += 360;
    h->h = hue;
}

void hsvToRgb24(const hsvColor_t* c, rgbColor24bpp_t* r)
{
    uint16_t val = c->v;
    uint16_t sat = 255 - c->s;
    uint32_t base;
    uint16_t hue = c->h;

    if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
        r->rgb.r = val;
        r->rgb.g = val;
        r->rgb.b = val;
    } else {

        base = ((255 - sat) * val) >> 8;

        switch (hue / 60) {
            case 0:
            r->rgb.r = val;
            r->rgb.g = (((val - base) * hue) / 60) + base;
            r->rgb.b = base;
            break;
            case 1:
            r->rgb.r = (((val - base) * (60 - (hue % 60))) / 60) + base;
            r->rgb.g = val;
            r->rgb.b = base;
            break;

            case 2:
            r->rgb.r = base;
            r->rgb.g = val;
            r->rgb.b = (((val - base) * (hue % 60)) / 60) + base;
            break;

            case 3:
            r->rgb.r = base;
            r->rgb.g = (((val - base) * (60 - (hue % 60))) / 60) + base;
            r->rgb.b = val;
            break;

            case 4:
            r->rgb.r = (((val - base) * (hue % 60)) / 60) + base;
            r->rgb.g = base;
            r->rgb.b = val;
            break;

            case 5:
            r->rgb.r = val;
            r->rgb.g = base;
            r->rgb.b = (((val - base) * (60 - (hue % 60))) / 60) + base;
            break;

        }
    }
}

#ifdef LED_STRIP

//LEDS have been split into separate strips due to timing issues with the serial receive interrupt.
// See: https://github.com/FastLED/FastLED/wiki/Interrupt-problems

void ledSetup() {
  strip[0] = new Adafruit_NeoPixel(8, NEO_PIN1, NEO_FLAGS);
  strip[0]->begin();
  strip[1] = new Adafruit_NeoPixel(8, NEO_PIN2, NEO_FLAGS);
  strip[1]->begin();
  strip[2] = new Adafruit_NeoPixel(8, NEO_PIN3, NEO_FLAGS);
  strip[2]->begin();
  strip[3] = new Adafruit_NeoPixel(8, NEO_PIN4, NEO_FLAGS);
  strip[3]->begin();
}

void scaleLedValue(uint16_t index, const uint8_t scalePercent)
{
  int s = index / 8;
  int l = index % 8;
  uint32_t c = strip[s]->getPixelColor(l);
  strip[s]->setPixelColor(l, (uint16_t)((c >> 16) & 0xff) * scalePercent / 100, 
                         (uint16_t)((c >> 8) & 0xff) * scalePercent / 100,
                         (uint16_t)(c & 0xff) * scalePercent / 100); 
}
 
void setLedHsv(uint16_t index, const hsvColor_t *color)
{
    rgbColor24bpp_t rgb;
    int s = index / 8;
    int l = index % 8;
    hsvToRgb24(color, &rgb);
    strip[s]->setPixelColor(l, rgb.rgb.r, rgb.rgb.g, rgb.rgb.b);
}

void getLedHsv(uint16_t index, hsvColor_t *color)
{
    int s = index / 8;
    int l = index % 8;
    uint32_t c = strip[s]->getPixelColor(l);
    rgbToHsv24((uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c, color); 
}

void setBrightness(uint8_t v){
  strip[0]->setBrightness(v);
  strip[1]->setBrightness(v);
  strip[2]->setBrightness(v);
  strip[3]->setBrightness(v);
  writeEEPROM(STRIP_BRIGHT, v);
}

void show() {
  flush();
  strip[0]->show();
  flush();
  strip[1]->show();
  flush();
  strip[2]->show();
  flush();
  strip[3]->show();
}

#endif

