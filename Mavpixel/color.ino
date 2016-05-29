/*
 * Source below found here: http://www.kasperkamperman.com/blog/arduino/arduino-programming-hsb-to-rgb/
 */

//Color conversion and pixel functions

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
rgbColor24bpp_t* hsvToRgb24(const hsvColor_t* c)
{
    static rgbColor24bpp_t r;

    uint16_t val = c->v;
    uint16_t sat = 255 - c->s;
    uint32_t base;
    uint16_t hue = c->h;

    if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
        r.rgb.r = val;
        r.rgb.g = val;
        r.rgb.b = val;
    } else {

        base = ((255 - sat) * val) >> 8;

        switch (hue / 60) {
            case 0:
            r.rgb.r = val;
            r.rgb.g = (((val - base) * hue) / 60) + base;
            r.rgb.b = base;
            break;
            case 1:
            r.rgb.r = (((val - base) * (60 - (hue % 60))) / 60) + base;
            r.rgb.g = val;
            r.rgb.b = base;
            break;

            case 2:
            r.rgb.r = base;
            r.rgb.g = val;
            r.rgb.b = (((val - base) * (hue % 60)) / 60) + base;
            break;

            case 3:
            r.rgb.r = base;
            r.rgb.g = (((val - base) * (60 - (hue % 60))) / 60) + base;
            r.rgb.b = val;
            break;

            case 4:
            r.rgb.r = (((val - base) * (hue % 60)) / 60) + base;
            r.rgb.g = base;
            r.rgb.b = val;
            break;

            case 5:
            r.rgb.r = val;
            r.rgb.g = base;
            r.rgb.b = (((val - base) * (60 - (hue % 60))) / 60) + base;
            break;

        }
    }
    return &r;
}

#ifdef LED_STRIP
void setLedHsv(uint16_t index, const hsvColor_t *color)
{
    rgbColor24bpp_t* r = hsvToRgb24(color);
    strip.setPixelColor(index, r->rgb.r, r->rgb.g, r->rgb.b);
}

void getLedHsv(uint16_t index, hsvColor_t *color)
{
    uint32_t c = strip.getPixelColor(index);
    rgbToHsv24((uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c, color);    
}
#endif
