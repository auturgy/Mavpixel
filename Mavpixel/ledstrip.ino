/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#include <platform.h>

//#include <build_config.h>

#ifdef LED_STRIP

//#include "color.h"
//#include "maths.h"
//#include <common/typeconversion.h>
//#include <common/printf.h>
//#include <common/axis.h>
//#include <common/utils.h>

//#include "config/parameter_group.h"
//#include "config/parameter_group_ids.h"

//#include "drivers/light_ws2811strip.h"
//#include "drivers/system.h"
//#include "drivers/serial.h"

//#include "flight/pid.h"
//#include "flight/failsafe.h"

//#include "io/rc_controls.h"

//#include "sensors/battery.h"

//#include "io/ledstrip.h"

//#include "config/runtime_config.h"
//#include "config/config.h"
//#include "config/feature.h"

//PG_REGISTER_ARR_WITH_RESET_FN(ledConfig_t, MAX_LED_STRIP_LENGTH, ledConfigs, PG_LED_STRIP_CONFIG, 0);
//PG_REGISTER_ARR_WITH_RESET_FN(hsvColor_t, CONFIGURABLE_COLOR_COUNT, colors, PG_COLOR_CONFIG, 0);


static bool ledStripInitialised = false;
static bool ledStripEnabled = true;

static void ledStripDisable(void);

//#define USE_LED_ANIMATION
//#define USE_LED_RING_DEFAULT_CONFIG

// timers
#ifdef USE_LED_ANIMATION
static uint32_t nextAnimationUpdateAt = 0;
#endif

static uint32_t nextIndicatorFlashAt = 0;
static uint32_t nextWarningFlashAt = 0;
static uint32_t nextRotationUpdateAt = 0;

#define LED_STRIP_20HZ ((1000 * 1000) / 20)
#define LED_STRIP_10HZ ((1000 * 1000) / 10)
#define LED_STRIP_5HZ ((1000 * 1000) / 5)

//#if MAX_LED_STRIP_LENGTH > WS2811_LED_STRIP_LENGTH
//#error "Led strip length must match driver"
//#endif

//                          H    S    V
#define LED_BLACK        {  0,   0,   0}
#define LED_WHITE        {  0, 255, 255}
#define LED_RED          {  0,   0, 255}
#define LED_ORANGE       { 30,   0, 255}
#define LED_YELLOW       { 60,   0, 255}
#define LED_LIME_GREEN   { 90,   0, 255}
#define LED_GREEN        {120,   0, 255}
#define LED_MINT_GREEN   {150,   0, 255}
#define LED_CYAN         {180,   0, 255}
#define LED_LIGHT_BLUE   {210,   0, 255}
#define LED_BLUE         {240,   0, 255}
#define LED_DARK_VIOLET  {270,   0, 255}
#define LED_MAGENTA      {300,   0, 255}
#define LED_DEEP_PINK    {330,   0, 255}

hsvColor_t hsv_black       = LED_BLACK;
hsvColor_t hsv_white       = LED_WHITE;
hsvColor_t hsv_red         = LED_RED;
hsvColor_t hsv_orange      = LED_ORANGE;
hsvColor_t hsv_yellow      = LED_YELLOW;
hsvColor_t hsv_limeGreen   = LED_LIME_GREEN;
hsvColor_t hsv_green       = LED_GREEN;
hsvColor_t hsv_mintGreen   = LED_MINT_GREEN;
hsvColor_t hsv_cyan        = LED_CYAN;
hsvColor_t hsv_lightBlue   = LED_LIGHT_BLUE;
hsvColor_t hsv_blue        = LED_BLUE;
hsvColor_t hsv_darkViolet  = LED_DARK_VIOLET;
hsvColor_t hsv_magenta     = LED_MAGENTA;
hsvColor_t hsv_deepPink    = LED_DEEP_PINK;

#define LED_DIRECTION_COUNT 6

//const hsvColor_t * const defaultColors[] = {
hsvColor_t* colors[] = {
        &hsv_black,
        &hsv_white,
        &hsv_red,
        &hsv_orange,
        &hsv_yellow,
        &hsv_limeGreen,
        &hsv_green,
        &hsv_mintGreen,
        &hsv_cyan,
        &hsv_lightBlue,
        &hsv_blue,
        &hsv_darkViolet,
        &hsv_magenta,
        &hsv_deepPink,
        &hsv_black,
        &hsv_black
};

typedef enum {
    COLOR_BLACK = 0,
    COLOR_WHITE,
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_LIME_GREEN,
    COLOR_GREEN,
    COLOR_MINT_GREEN,
    COLOR_CYAN,
    COLOR_LIGHT_BLUE,
    COLOR_BLUE,
    COLOR_DARK_VIOLET,
    COLOR_MAGENTA,
    COLOR_DEEP_PINK,
} colorIds;

typedef enum {
    DIRECTION_NORTH = 0,
    DIRECTION_EAST,
    DIRECTION_SOUTH,
    DIRECTION_WEST,
    DIRECTION_UP,
    DIRECTION_DOWN
} directionId_e;


// Note, the color index used for the mode colors below refer to the default colors.
// if the colors are reconfigured the index is still valid but the displayed color might
// be different.
// See colors[] and defaultColors[] and applyDefaultColors[]

//Single array for efficiency now there are many more modes
modeColorIndexes_t modeColors[] = {
//Stabilize  
  {COLOR_GREEN, COLOR_DARK_VIOLET, COLOR_GREEN, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Acro
  {COLOR_LIME_GREEN, COLOR_DARK_VIOLET, COLOR_GREEN, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Althold
  {COLOR_BLUE, COLOR_DARK_VIOLET, COLOR_GREEN, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Auto
  {COLOR_CYAN, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Guided
  {COLOR_MINT_GREEN, COLOR_DARK_VIOLET, COLOR_ORANGE, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Loiter
  {COLOR_LIGHT_BLUE, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//RTL
  {COLOR_ORANGE, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Circle
  {COLOR_DEEP_PINK, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Land
  {COLOR_MAGENTA, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Drift
  {COLOR_DARK_VIOLET, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Sport
  {COLOR_YELLOW, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Flip
  {COLOR_RED, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Autotune
  {COLOR_WHITE, COLOR_ORANGE, COLOR_RED, COLOR_ORANGE, COLOR_BLUE, COLOR_ORANGE},
//Poshold
  {COLOR_BLUE, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Brake
  {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_BLUE, COLOR_ORANGE},
//Throw
  {COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_BLUE, COLOR_ORANGE},
//Manual
  {COLOR_YELLOW, COLOR_DARK_VIOLET, COLOR_GREEN, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//FBWA
  {COLOR_BLUE, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//FBWB
  {COLOR_MAGENTA, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Training
  {COLOR_RED, COLOR_DARK_VIOLET, COLOR_ORANGE, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
//Cruise
  {COLOR_DARK_VIOLET, COLOR_DARK_VIOLET, COLOR_RED, COLOR_DEEP_PINK, COLOR_BLUE, COLOR_ORANGE},
};  

uint8_t ledGridWidth;
uint8_t ledGridHeight;
uint8_t ledCount;
uint8_t ledsInRingCount;

#ifdef USE_LED_RING_DEFAULT_CONFIG
const ledConfig_t defaultLedStripConfig[] = {
    { CALCULATE_LED_XY( 2,  2), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 2,  1), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 2,  0), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 1,  0), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 0,  0), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 0,  1), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 0,  2), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 1,  2), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 1,  1), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 1,  1), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 1,  1), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 1,  1), 3, LED_FUNCTION_THRUST_RING},
};
#else
//const ledConfig_t defaultLedStripConfig[] = {
ledConfig_t ledConfigs[MAX_LED_STRIP_LENGTH] = {
    { CALCULATE_LED_XY(15, 15), 0, LED_DIRECTION_SOUTH | LED_DIRECTION_EAST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
    { CALCULATE_LED_XY(15,  8), 0, LED_DIRECTION_EAST | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY(15,  7), 0, LED_DIRECTION_EAST | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY(15,  0), 0, LED_DIRECTION_NORTH | LED_DIRECTION_EAST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
    { CALCULATE_LED_XY( 8,  0), 0, LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE },
    { CALCULATE_LED_XY( 7,  0), 0, LED_DIRECTION_NORTH | LED_FUNCTION_FLIGHT_MODE },
    { CALCULATE_LED_XY( 0,  0), 0, LED_DIRECTION_NORTH | LED_DIRECTION_WEST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
    { CALCULATE_LED_XY( 0,  7), 0, LED_DIRECTION_WEST | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY( 0,  8), 0, LED_DIRECTION_WEST | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY( 0, 15), 0, LED_DIRECTION_SOUTH | LED_DIRECTION_WEST | LED_FUNCTION_INDICATOR | LED_FUNCTION_ARM_STATE },
    { CALCULATE_LED_XY( 7, 15), 0, LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY( 8, 15), 0, LED_DIRECTION_SOUTH | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY( 7,  7), 0, LED_DIRECTION_UP | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY( 8,  7), 0, LED_DIRECTION_UP | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY( 7,  8), 0, LED_DIRECTION_DOWN | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY( 8,  8), 0, LED_DIRECTION_DOWN | LED_FUNCTION_FLIGHT_MODE | LED_FUNCTION_WARNING },
    { CALCULATE_LED_XY( 8,  9), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 9, 10), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY(10, 11), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY(10, 12), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 9, 13), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 8, 14), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 7, 14), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 6, 13), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 5, 12), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 5, 11), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 6, 10), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 7,  9), 3, LED_FUNCTION_THRUST_RING},
    { CALCULATE_LED_XY( 0,  0), 0, 0},
    { CALCULATE_LED_XY( 0,  0), 0, 0},
    { CALCULATE_LED_XY( 0,  0), 0, 0},
    { CALCULATE_LED_XY( 0,  0), 0, 0}
};
#endif


//void pgResetFn_ledConfigs(ledConfig_t *instance)
//{
//    memcpy(instance, &defaultLedStripConfig, sizeof(defaultLedStripConfig));
//}

//void pgResetFn_colors(hsvColor_t *instance)
//{
//    for (uint8_t colorIndex = 0; colorIndex < ARRAYLEN(defaultColors); colorIndex++) {
//        *instance++ = *defaultColors[colorIndex];
//    }
//}

/*
 * 6 coords @nn,nn
 * 4 direction @##
 * 6 modes @####
 * = 16 bytes per led
 * 16 * 32 leds = 512 bytes storage needed worst case.
 * = not efficient to store led configs as strings in flash.
 * = becomes a problem to send all the data via cli due to serial/cli buffers
 */

typedef enum {
    X_COORDINATE,
    Y_COORDINATE,
    DIRECTIONS,
    FUNCTIONS,
    RING_COLORS
} parseState_e;

#define PARSE_STATE_COUNT 5

static const char chunkSeparators[PARSE_STATE_COUNT] = {',', ':', ':',':', '\0' };

static const char directionCodes[] = { 'N', 'E', 'S', 'W', 'U', 'D' };
#define DIRECTION_COUNT 6//(sizeof(directionCodes) / sizeof(directionCodes[0]))
static const uint16_t directionMappings[DIRECTION_COUNT] = {
    LED_DIRECTION_NORTH,
    LED_DIRECTION_EAST,
    LED_DIRECTION_SOUTH,
    LED_DIRECTION_WEST,
    LED_DIRECTION_UP,
    LED_DIRECTION_DOWN
};

static const char functionCodes[] = { 'I', 'W', 'F', 'A', 'T', 'R', 'C' };
#define FUNCTION_COUNT 7//(sizeof(functionCodes) / sizeof(functionCodes[0]))
static const uint16_t functionMappings[FUNCTION_COUNT] = {
    LED_FUNCTION_INDICATOR,
    LED_FUNCTION_WARNING,
    LED_FUNCTION_FLIGHT_MODE,
    LED_FUNCTION_ARM_STATE,
    LED_FUNCTION_THROTTLE,
	LED_FUNCTION_THRUST_RING,
	LED_FUNCTION_COLOR
};

// grid offsets
uint8_t highestYValueForNorth;
uint8_t lowestYValueForSouth;
uint8_t highestXValueForWest;
uint8_t lowestXValueForEast;

void determineLedStripDimensions(void)
{
    ledGridWidth = 0;
    ledGridHeight = 0;

    uint8_t ledIndex;
    const ledConfig_t *ledConfig;

    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {
        ledConfig = &ledConfigs[ledIndex];

        if (GET_LED_X(ledConfig) >= ledGridWidth) {
            ledGridWidth = GET_LED_X(ledConfig) + 1;
        }
        if (GET_LED_Y(ledConfig) >= ledGridHeight) {
            ledGridHeight = GET_LED_Y(ledConfig) + 1;
        }
    }
}

void determineOrientationLimits(void)
{
    bool isOddHeight = (ledGridHeight & 1);
    bool isOddWidth = (ledGridWidth & 1);
    uint8_t heightModifier = isOddHeight ? 1 : 0;
    uint8_t widthModifier = isOddWidth ? 1 : 0;

    highestYValueForNorth = (ledGridHeight / 2) - 1;
    lowestYValueForSouth = (ledGridHeight / 2) + heightModifier;
    highestXValueForWest = (ledGridWidth / 2) - 1;
    lowestXValueForEast = (ledGridWidth / 2) + widthModifier;
}

void updateLedCount(void)
{
    const ledConfig_t *ledConfig;
    uint8_t ledIndex;
    ledCount = 0;
    ledsInRingCount = 0;

    for (ledIndex = 0; ledIndex < MAX_LED_STRIP_LENGTH; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        if (ledConfig->flags == 0 && ledConfig->xy == 0) {
            break;
        }

        ledCount++;

        if ((ledConfig->flags & LED_FUNCTION_THRUST_RING)) {
            ledsInRingCount++;
        }
    }
}

void reevalulateLedConfig(void)
{
    updateLedCount();
    determineLedStripDimensions();
    determineOrientationLimits();
}



#define CHUNK_BUFFER_SIZE 10

#define NEXT_PARSE_STATE(parseState) ((parseState + 1) % PARSE_STATE_COUNT)


bool parseLedStripConfig(uint8_t ledIndex, const char *config)
{
    char chunk[CHUNK_BUFFER_SIZE];
    uint8_t chunkIndex;
    uint8_t val;

    uint8_t parseState = X_COORDINATE;
    bool ok = true;

    if (ledIndex >= MAX_LED_STRIP_LENGTH) {
        return !ok;
    }

    ledConfig_t *ledConfig = &ledConfigs[ledIndex];
    memset(ledConfig, 0, sizeof(ledConfig_t));

    while (ok) {

        char chunkSeparator = chunkSeparators[parseState];

        memset(&chunk, 0, sizeof(chunk));
        chunkIndex = 0;

        while (*config && chunkIndex < CHUNK_BUFFER_SIZE && *config != chunkSeparator) {
            chunk[chunkIndex++] = *config++;
        }

        if (*config++ != chunkSeparator) {
            ok = false;
            break;
        }

        switch((parseState_e)parseState) {
            case X_COORDINATE:
                val = atoi(chunk);
                ledConfig->xy |= CALCULATE_LED_X(val);
                break;
            case Y_COORDINATE:
                val = atoi(chunk);
                ledConfig->xy |= CALCULATE_LED_Y(val);
                break;
            case DIRECTIONS:
                for (chunkIndex = 0; chunk[chunkIndex] && chunkIndex < CHUNK_BUFFER_SIZE; chunkIndex++) {
                    for (uint8_t mappingIndex = 0; mappingIndex < DIRECTION_COUNT; mappingIndex++) {
                        if (directionCodes[mappingIndex] == chunk[chunkIndex]) {
                            ledConfig->flags |= directionMappings[mappingIndex];
                            break;
                        }
                    }
                }
                break;
            case FUNCTIONS:
                for (chunkIndex = 0; chunk[chunkIndex] && chunkIndex < CHUNK_BUFFER_SIZE; chunkIndex++) {
                    for (uint8_t mappingIndex = 0; mappingIndex < FUNCTION_COUNT; mappingIndex++) {
                        if (functionCodes[mappingIndex] == chunk[chunkIndex]) {
                            ledConfig->flags |= functionMappings[mappingIndex];
                            break;
                        }
                    }
                }
                break;
            case RING_COLORS:
                if (atoi(chunk) < CONFIGURABLE_COLOR_COUNT) {
                    ledConfig->color = atoi(chunk);
                } else {
                    ledConfig->color = 0;
                }
                break;
            default :
                break;
        }

        parseState++;
        if (parseState >= PARSE_STATE_COUNT) {
            break;
        }
    }

    if (!ok) {
        memset(ledConfig, 0, sizeof(ledConfig_t));
    }

    reevalulateLedConfig();

    return ok;
}

void printLedConfig(uint8_t ledIndex)
{
    uint8_t index;
    uint8_t mappingIndex;

    ledConfig_t *ledConfig = &ledConfigs[ledIndex];

    Serial.print(GET_LED_X(ledConfig));
    Serial.print(",");
    Serial.print(GET_LED_Y(ledConfig));    
    Serial.print(":");
    
    for (mappingIndex = 0, index = 0; mappingIndex < DIRECTION_COUNT; mappingIndex++) {
        if (ledConfig->flags & directionMappings[mappingIndex]) {
            Serial.print(directionCodes[mappingIndex]);
        }
    }
    Serial.print(":");

    for (mappingIndex = 0, index = 0; mappingIndex < FUNCTION_COUNT; mappingIndex++) {
        if (ledConfig->flags & functionMappings[mappingIndex]) {
            Serial.print(functionCodes[mappingIndex]);
        }
    }
    Serial.print(":");
    Serial.print(ledConfig->color);
}


void applyDirectionalModeColor(const uint8_t ledIndex, const ledConfig_t *ledConfig, const modeColorIndexes_t *modeColors)
{
    // apply up/down colors regardless of quadrant.
    if ((ledConfig->flags & LED_DIRECTION_UP)) {
        setLedHsv(ledIndex, colors[modeColors->up]);
    }

    if ((ledConfig->flags & LED_DIRECTION_DOWN)) {
        setLedHsv(ledIndex, colors[modeColors->down]);
    }

    // override with n/e/s/w colors to each n/s e/w half - bail at first match.
    if ((ledConfig->flags & LED_DIRECTION_WEST) && GET_LED_X(ledConfig) <= highestXValueForWest) {
        setLedHsv(ledIndex, colors[modeColors->west]);
    }

    if ((ledConfig->flags & LED_DIRECTION_EAST) && GET_LED_X(ledConfig) >= lowestXValueForEast) {
        setLedHsv(ledIndex, colors[modeColors->east]);
    }

    if ((ledConfig->flags & LED_DIRECTION_NORTH) && GET_LED_Y(ledConfig) <= highestYValueForNorth) {
        setLedHsv(ledIndex, colors[modeColors->north]);
    }

    if ((ledConfig->flags & LED_DIRECTION_SOUTH) && GET_LED_Y(ledConfig) >= lowestYValueForSouth) {
        setLedHsv(ledIndex, colors[modeColors->south]);
    }

}

void applyQuadrantColor(const uint8_t ledIndex, const ledConfig_t *ledConfig, const quadrant_e quadrant, const hsvColor_t *color)
{
    switch (quadrant) {
        case QUADRANT_NORTH_EAST:
            if (GET_LED_Y(ledConfig) <= highestYValueForNorth && GET_LED_X(ledConfig) >= lowestXValueForEast) {
                setLedHsv(ledIndex, color);
            }
            return;

        case QUADRANT_SOUTH_EAST:
            if (GET_LED_Y(ledConfig) >= lowestYValueForSouth && GET_LED_X(ledConfig) >= lowestXValueForEast) {
                setLedHsv(ledIndex, color);
            }
            return;

        case QUADRANT_SOUTH_WEST:
            if (GET_LED_Y(ledConfig) >= lowestYValueForSouth && GET_LED_X(ledConfig) <= highestXValueForWest) {
                setLedHsv(ledIndex, color);
            }
            return;

        case QUADRANT_NORTH_WEST:
            if (GET_LED_Y(ledConfig) <= highestYValueForNorth && GET_LED_X(ledConfig) <= highestXValueForWest) {
                setLedHsv(ledIndex, color);
            }
            return;
    }
}

void applyLedModeLayer(void)
{
    const ledConfig_t *ledConfig;

    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        if (!(ledConfig->flags & LED_FUNCTION_THRUST_RING)) {
            if (ledConfig->flags & LED_FUNCTION_COLOR) {
                setLedHsv(ledIndex, colors[ledConfig->color]);
            } else {
                setLedHsv(ledIndex, &hsv_black);
            }
        }

        if (!(ledConfig->flags & LED_FUNCTION_FLIGHT_MODE)) {
            if (ledConfig->flags & LED_FUNCTION_ARM_STATE) {
                if (isArmed) {
                    setLedHsv(ledIndex, &hsv_green);
                } else {
                    setLedHsv(ledIndex, &hsv_blue);
                }
            }
            continue;
        }

        applyDirectionalModeColor(ledIndex, ledConfig, &modeColors[flMode]);

/*        if (1){//FLIGHT_MODE(HEADFREE_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &headfreeModeColors);
#ifdef MAG
        } else if (0){//FLIGHT_MODE(MAG_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &magModeColors);
#endif
#ifdef BARO
        } else if (0){//FLIGHT_MODE(BARO_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &baroModeColors);
#endif
        } else if (0){//FLIGHT_MODE(HORIZON_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &horizonModeColors);
        } else if (0){//FLIGHT_MODE(ANGLE_MODE)) {
            applyDirectionalModeColor(ledIndex, ledConfig, &angleModeColors);
        }
*/    }
}


typedef enum {
    WARNING_FLAG_NONE = 0,
    WARNING_FLAG_LOW_BATTERY = (1 << 0),
    WARNING_FLAG_FAILSAFE = (1 << 1),
    WARNING_FLAG_ARMING_DISABLED = (1 << 2)
} warningFlags_e;

static uint8_t warningFlags = WARNING_FLAG_NONE;

void applyLedWarningLayer(uint8_t updateNow)
{
    const ledConfig_t *ledConfig;
    uint8_t ledIndex;
    static uint8_t warningFlashCounter = 0;

    if (updateNow && warningFlashCounter == 0) {
        warningFlags = WARNING_FLAG_NONE;
        if (cellVoltage > 1 && ((cellVoltage < lowBattVolt) || (iob_battery_remaining_A < lowBattPct))) {
            warningFlags |= WARNING_FLAG_LOW_BATTERY;
        }
        /*if (feature(FEATURE_FAILSAFE) && failsafeIsActive()) {
            warningFlags |= WARNING_FLAG_FAILSAFE;
        }
        if (!ARMING_FLAG(ARMED) && !ARMING_FLAG(OK_TO_ARM)) {
            warningFlags |= WARNING_FLAG_ARMING_DISABLED;
        }*/
    }

    if (warningFlags || warningFlashCounter > 0) {
        const hsvColor_t *warningColor = &hsv_black;

        if ((warningFlashCounter & 1) == 0) {
            if (warningFlashCounter < 4 && (warningFlags & WARNING_FLAG_ARMING_DISABLED)) {
                warningColor = &hsv_green;
            }
            if (warningFlashCounter >= 4 && warningFlashCounter < 12 && (warningFlags & WARNING_FLAG_LOW_BATTERY)) {
                warningColor = &hsv_red;
            }
            if (warningFlashCounter >= 12 && warningFlashCounter < 16 && (warningFlags & WARNING_FLAG_FAILSAFE)) {
                warningColor = &hsv_yellow;
            }
        }  else {
            if (warningFlashCounter >= 12 && warningFlashCounter < 16 && (warningFlags & WARNING_FLAG_FAILSAFE)) {
                warningColor = &hsv_blue;
            }
        }

        for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {

            ledConfig = &ledConfigs[ledIndex];

            if (!(ledConfig->flags & LED_FUNCTION_WARNING)) {
                continue;
            }
            setLedHsv(ledIndex, warningColor);
        }
    }

    if (updateNow && (warningFlags || warningFlashCounter)) {
        warningFlashCounter++;
        if (warningFlashCounter == 20) {
            warningFlashCounter = 0;
        }
    }
}

#define INDICATOR_DEADBAND 25

void applyLedIndicatorLayer(uint8_t indicatorFlashState)
{
    const ledConfig_t *ledConfig;
    static const hsvColor_t *flashColor;

    //if (!rxIsReceivingSignal()) {
    //    return;
    //}

    if (indicatorFlashState == 0) {
        flashColor = &hsv_orange;
    } else {
        flashColor = &hsv_black;
    }


    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        if (!(ledConfig->flags & LED_FUNCTION_INDICATOR)) {
            continue;
        }

        if (iob_chan1 > 1500 + INDICATOR_DEADBAND) {
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_NORTH_EAST, flashColor);
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_SOUTH_EAST, flashColor);
        }

        if (iob_chan1 < 1500 - INDICATOR_DEADBAND) {
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_NORTH_WEST, flashColor);
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_SOUTH_WEST, flashColor);
        }

        if (iob_chan2 > 1500 + INDICATOR_DEADBAND) {
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_NORTH_EAST, flashColor);
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_NORTH_WEST, flashColor);
        }

        if (iob_chan2 < 1500 - INDICATOR_DEADBAND) {
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_SOUTH_EAST, flashColor);
            applyQuadrantColor(ledIndex, ledConfig, QUADRANT_SOUTH_WEST, flashColor);
        }
    }
}


void applyLedThrottleLayer()
{
    const ledConfig_t *ledConfig;
    hsvColor_t color;

    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {
        ledConfig = &ledConfigs[ledIndex];
        if (!(ledConfig->flags & LED_FUNCTION_THROTTLE)) {
            continue;
        }

        getLedHsv(ledIndex, &color);

        int scaled = (iob_throttle * 1.2f) - 60;//scaleRange(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX, -60, +60);
        scaled += HSV_HUE_MAX;
        color.h = (color.h + scaled) % HSV_HUE_MAX;
        setLedHsv(ledIndex, &color);
    }
}

#define ROTATION_SEQUENCE_LED_COUNT 6 // 2 on, 4 off
#define ROTATION_SEQUENCE_LED_WIDTH 2

#define RING_PATTERN_NOT_CALCULATED 255

void applyLedThrustRingLayer(void)
{
    const ledConfig_t *ledConfig;
    hsvColor_t ringColor;
    uint8_t ledIndex;

    // initialised to special value instead of using more memory for a flag.
    static uint8_t rotationSeqLedCount = RING_PATTERN_NOT_CALCULATED;
    static uint8_t rotationPhase = ROTATION_SEQUENCE_LED_COUNT;
    bool nextLedOn = false;

    uint8_t ledRingIndex = 0;
    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        if ((ledConfig->flags & LED_FUNCTION_THRUST_RING) == 0) {
            continue;
        }

        bool applyColor = false;
        if (isArmed) {
            if ((ledRingIndex + rotationPhase) % rotationSeqLedCount < ROTATION_SEQUENCE_LED_WIDTH) {
                applyColor = true;
            }
        } else {
            if (nextLedOn == false) {
                applyColor = true;
            }
            nextLedOn = !nextLedOn;
        }

        if (applyColor) {
            ringColor = *colors[ledConfig->color];
        } else {
            ringColor = hsv_black;
        }

        setLedHsv(ledIndex, &ringColor);

        ledRingIndex++;
    }

    uint8_t ledRingLedCount = ledRingIndex;
    if (rotationSeqLedCount == RING_PATTERN_NOT_CALCULATED) {
        // update ring pattern according to total number of ring leds found

        rotationSeqLedCount = ledRingLedCount;

        // try to split in segments/rings of exactly ROTATION_SEQUENCE_LED_COUNT leds
        if ((ledRingLedCount % ROTATION_SEQUENCE_LED_COUNT) == 0) {
            rotationSeqLedCount = ROTATION_SEQUENCE_LED_COUNT;
        } else {
            // else split up in equal segments/rings of at most ROTATION_SEQUENCE_LED_COUNT leds
            while ((rotationSeqLedCount > ROTATION_SEQUENCE_LED_COUNT) && ((rotationSeqLedCount % 2) == 0)) {
                rotationSeqLedCount >>= 1;
            }
        }

        // trigger start over
        rotationPhase = 1;
    }

    rotationPhase--;
    if (rotationPhase == 0) {
        rotationPhase = rotationSeqLedCount;
    }
}

#ifdef USE_LED_ANIMATION
static uint8_t previousRow;
static uint8_t currentRow;
static uint8_t nextRow;

void updateLedAnimationState(void)
{
    static uint8_t frameCounter = 0;

    uint8_t animationFrames = ledGridHeight;

    previousRow = (frameCounter + animationFrames - 1) % animationFrames;
    currentRow = frameCounter;
    nextRow = (frameCounter + 1) % animationFrames;

    frameCounter = (frameCounter + 1) % animationFrames;
}

int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax) {
    long int a = ((long int) destMax - (long int) destMin) * ((long int) x - (long int) srcMin);
    long int b = (long int) srcMax - (long int) srcMin;
    return ((a / b) - (destMax - destMin)) + destMax;
}

static void applyLedAnimationLayer(void)
{
    const ledConfig_t *ledConfig;

    if (isArmed) {
        return;
    }

    uint8_t ledIndex;
    for (ledIndex = 0; ledIndex < ledCount; ledIndex++) {

        ledConfig = &ledConfigs[ledIndex];

        if (GET_LED_Y(ledConfig) == previousRow) {
            setLedHsv(ledIndex, &hsv_white);
            scaleLedValue(ledIndex, 50);

        } else if (GET_LED_Y(ledConfig) == currentRow) {
            setLedHsv(ledIndex, &hsv_white);
        } else if (GET_LED_Y(ledConfig) == nextRow) {
            scaleLedValue(ledIndex, 50);
        }
    }
}
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))

void updateLedStrip(void)
{

	if (!(ledStripInitialised)) {// && isWS2811LedStripReady())) {
        return;
    }

    if (false) {//IS_RC_MODE_ACTIVE(BOXLEDLOW)) {
        if (ledStripEnabled) {
            ledStripDisable();
            ledStripEnabled = false;
        }
    } else {
        ledStripEnabled = true;
    }
    
    if (!ledStripEnabled){
        return;
    }
    

    uint32_t now = micros();

    bool indicatorFlashNow = (int32_t)(now - nextIndicatorFlashAt) >= 0L;
    bool warningFlashNow = (int32_t)(now - nextWarningFlashAt) >= 0L;
    bool rotationUpdateNow = (int32_t)(now - nextRotationUpdateAt) >= 0L;
#ifdef USE_LED_ANIMATION
    bool animationUpdateNow = (int32_t)(now - nextAnimationUpdateAt) >= 0L;
#endif
    if (!(
            indicatorFlashNow ||
            rotationUpdateNow ||
            warningFlashNow
#ifdef USE_LED_ANIMATION
            || animationUpdateNow
#endif
    )) {
        return;
    }

    static uint8_t indicatorFlashState = 0;

    // LAYER 1
    applyLedModeLayer();
    applyLedThrottleLayer();

    // LAYER 2

    if (warningFlashNow) {
        nextWarningFlashAt = now + LED_STRIP_10HZ;
    }
    applyLedWarningLayer(warningFlashNow);

    // LAYER 3

    if (indicatorFlashNow) {

        uint8_t rollScale = ABS(iob_chan1 - 1500) / 50;
        uint8_t pitchScale = ABS(iob_chan2 - 1500) / 50;
        uint8_t scale = MAX(rollScale, pitchScale);
        nextIndicatorFlashAt = now + (LED_STRIP_5HZ / MAX(1, scale));

        if (indicatorFlashState == 0) {
            indicatorFlashState = 1;
        } else {
            indicatorFlashState = 0;
        }
    }

    applyLedIndicatorLayer(indicatorFlashState);

#ifdef USE_LED_ANIMATION
    if (animationUpdateNow) {
        nextAnimationUpdateAt = now + LED_STRIP_20HZ;
        updateLedAnimationState();
    }
    applyLedAnimationLayer();
#endif

    if (rotationUpdateNow) {

        applyLedThrustRingLayer();

        uint8_t animationSpeedScale = 1;

        if (isArmed) {
            animationSpeedScale = iob_throttle / 10; //scaleRange(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX, 1, 10);
        }

        nextRotationUpdateAt = now + LED_STRIP_5HZ/animationSpeedScale;
    }
    show();
}


bool parseColor(uint8_t index, const char *colorConfig)
{
    const char *remainingCharacters = colorConfig;

    hsvColor_t *color = colors[index];

    bool ok = true;

    uint8_t componentIndex;
    for (componentIndex = 0; ok && componentIndex < HSV_COLOR_COMPONENT_COUNT; componentIndex++) {
        uint16_t val = atoi(remainingCharacters);
        switch (componentIndex) {
            case HSV_HUE:
                if (val > HSV_HUE_MAX) {
                    ok = false;
                    continue;

                }
                colors[index]->h = val;
                break;
            case HSV_SATURATION:
                if (val > HSV_SATURATION_MAX) {
                    ok = false;
                    continue;
                }
                colors[index]->s = (uint8_t)val;
                break;
            case HSV_VALUE:
                if (val > HSV_VALUE_MAX) {
                    ok = false;
                    continue;
                }
                colors[index]->v = (uint8_t)val;
                break;
        }
        remainingCharacters = strstr(remainingCharacters, ",");
        if (remainingCharacters) {
            remainingCharacters++;
        } else {
            if (componentIndex < 2) {
                ok = false;
            }
        }
    }

    if (!ok) {
        memset(color, 0, sizeof(hsvColor_t));
    }

    return ok;
}

void printColorConfig(uint8_t index)
{
    hsvColor_t *color = colors[index];
    Serial.print(colors[index]->h);
    Serial.print(",");
    Serial.print(colors[index]->s);
    Serial.print(",");
    Serial.print(colors[index]->v);
}

bool parseMode(const char *modeConfig)
{
    const char *remainingCharacters = modeConfig;
    uint16_t mode, color;
    uint8_t componentIndex;
    for (componentIndex = 0; componentIndex < MCI_COMPONENT_COUNT; componentIndex++) {
        uint16_t val = atoi(remainingCharacters);
        switch (componentIndex) {
            case MCI_MODE:
                if (val > MAX_MODES) return false;
                mode = val;
                break;
            case MCI_COLOR:
                if (val >= CONFIGURABLE_COLOR_COUNT) return false;
                color = val;
                break;
            case MCI_INDEX:
                if (val > MODE_COLOR_INDEX_MAX) return false;
                break;
        }
        remainingCharacters = strstr(remainingCharacters, ",");
        if (remainingCharacters) remainingCharacters++;
        else if (componentIndex != 2) return false;
        else modeColors[mode][val] = color;
    }
    return true;
}

void printModeConfig(uint8_t mode)
{
    Serial.print(modeColors[mode][0]);
    for (int i = 1; i <= MODE_COLOR_INDEX_MAX; i++) {
      Serial.print(",");
      Serial.print(modeColors[mode][i]);
    }
}

void ledStripInit(void)
{
    ledStripInitialised = false;
    //Led driver init
    ledSetup();
}

void ledStripEnable(void)
{
    reevalulateLedConfig();
    ledStripInitialised = true;
}

static void ledStripDisable(void)
{
//	setStripColor(&hsv_black);
    
//	ws2811UpdateStrip();
}
#endif
