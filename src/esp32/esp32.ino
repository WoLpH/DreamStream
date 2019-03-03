/********************************************************************************
* Copyright (c) 2018 by Hugo Peters. http://hugo.fyi/                           *
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
*********************************************************************************/

///////////////////////////////////////////////////////////////////////////////
/// SETUP
///////////////////////////////////////////////////////////////////////////////

// WiFi setup
#define WIFI_SSID                   "..."
#define WIFI_PASS                   "..."

// FastLED
#define USING_FASTLED

#ifdef USING_FASTLED
#define FASTLED_LED_TYPE            WS2812B
#define FASTLED_LED_COLOR_ORDER     RGB
#define FASTLED_PIN_ID              0
#define FASTLED_NUM_LEDS            100
#define FASTLED_SMOOTHING           200
#endif // USING_FASTLED

#define ENABLE_GRADIENT

#if (defined(FASTLED_SMOOTHING) && FASTLED_SMOOTHING > 0)
#define FASTLED_ENABLE_SMOOTHING
#endif


///////////////////////////////////////////////////////////////////////////////
/// STANDARD DEFINES
///////////////////////////////////////////////////////////////////////////////

// DreamScreen
#define DS_PORT                     8888
#define DS_MAGIC                    0xFC

// Packets
#define PKT_BUFFER_SIZE             512
#define PKT_RESPONSE                (1UL << 6)
#define PKT_BROADCAST               (1UL << 5)
#define PKT_RESPONSE_REQUEST        (1UL << 4)
#define PKT_WRITE                   (1UL << 1)
#define PKT_WRITE_CONSTANT          (1UL << 0)
#define PKT_IS(x, y)                (((x) & y) != 0)

// Stuff
#define FORCEINLINE                 __attribute__((always_inline))

#ifdef USING_FASTLED
// Required for ESP32, otherwises crashes on FastLED.show();
#define FASTLED_ALLOW_INTERRUPTS    0
#endif// USING_FASTLED

// update interval in milliseconds
#define UPDATE_INTERVAL             1

///////////////////////////////////////////////////////////////////////////////
/// INCLUDES
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <Arduino.h>
#include <Preferences.h>
#include "esp_system.h"

// https://stackoverflow.com/questions/41093090/esp8266-error-macro-min-passed-3-arguments-but-takes-just-2
#undef min
#undef max

// Module
#include "StaticString.h"
#include "PacketHelpers.h"
#include "../../../../../.platformio/packages/toolchain-xtensa32/xtensa-esp32-elf/include/c++/5.2.0/cstdint"

// Wifi
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#ifdef USING_FASTLED
#include <FastLED.h>
#endif // USING_FASTLED
// #include "BackgroundTask.h"

///////////////////////////////////////////////////////////////////////////////
/// OBJECTS
///////////////////////////////////////////////////////////////////////////////

WiFiUDP g_udp;
Preferences preferences;
TaskHandle_t TaskA;

void update(void * parameter);

uint8_t g_read_buffer[PKT_BUFFER_SIZE];
uint8_t g_write_buffer[PKT_BUFFER_SIZE];
bool g_show_leds = false;
bool updating = false;
hw_timer_t *timer = NULL;

#ifdef USING_FASTLED

CRGB g_fastled_leds[FASTLED_NUM_LEDS];

#ifdef FASTLED_ENABLE_SMOOTHING
CRGB g_fastled_targetleds[FASTLED_NUM_LEDS];
CHSV sectors[12];
#else
CRGB sectors[12];
#endif // FASTLED_ENABLE_SMOOTHING

CRGB* fastledGetLeds() {
  #ifdef FASTLED_ENABLE_SMOOTHING
  return g_fastled_targetleds;
  #else
  return g_fastled_leds;
  #endif // FASTLED_ENABLE_SMOOTHING
}

#endif // USING_FASTLED

// Code taken from DreamScreen V2 WiFi UDP Protocol Rev 4
// http://dreamscreen.boards.net/thread/293/dreamscreen-wifi-udp-protocol
const uint8_t uartComm_crc8_table[] PROGMEM =
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23,
    0x2A, 0x2D, 0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41,
    0x54, 0x53, 0x5A, 0x5D, 0xE0, 0xE7, 0xEE, 0xE9,	0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF,
    0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD, 0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC,
    0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE,
    0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A, 0x27, 0x20,
    0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74,
    0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8,
    0xAD, 0xAA, 0xA3, 0xA4, 0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6,
    0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 0x19, 0x1E, 0x17, 0x10, 0x05, 0x02,
    0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34, 0x4E, 0x49, 0x40, 0x47,
    0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39,
    0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D,
    0x84, 0x83, 0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF,
    0xFA, 0xFD, 0xF4, 0xF3
};

///////////////////////////////////////////////////////////////////////////////
/// ENUMS
///////////////////////////////////////////////////////////////////////////////

namespace DEVICE_TYPE
{
    enum
    {
        DreamScreenHD       = 0x1,
        DreamScreen4K       = 0x2,
        SideKick            = 0x3
    };
}

namespace DEVICE_MODE
{
    enum
    {
        Sleep               = 0x0,
        Video               = 0x1,
        Music               = 0x2,
        Ambient             = 0x3
    };
}

namespace AMBIENT_MODE
{
    enum
    {
        RGB                 = 0x0,
        Scene               = 0x1
    };
}

namespace AMBIENT_SCENE
{
    enum
    {
        RandomColor         = 0x0,
        Fireside            = 0x1,
        Twinkle             = 0x2,
        Ocean               = 0x3,
        Rainbow             = 0x4,
        July4th             = 0x5,
        Holiday             = 0x6,
        Pop                 = 0x7,
        EnchantedForest     = 0x8
    };
}

///////////////////////////////////////////////////////////////////////////////
/// COMMANDS
///////////////////////////////////////////////////////////////////////////////

union CMD
{
    uint8_t regions[2];
    uint16_t code;
};

namespace COMMANDS
{
    enum Type
    {
        #define DECLARE_COMMAND(enumType, ...) enumType,
        #include "CommandsList.h"
        #undef DECLARE_COMMAND
        NUM
    };

    static const CMD Descriptors[NUM] PROGMEM
    {
        #define DECLARE_COMMAND(enumType, cmdUpper, cmdLower) {cmdUpper, cmdLower},
        #include "CommandsList.h"
        #undef DECLARE_COMMAND
    };
}

namespace COMMANDS_IDS
{
    enum Type
    {
        #define DECLARE_COMMAND(enumType, cmdUpper, cmdLower) enumType = ((cmdUpper << 8) | cmdLower),
        #include "CommandsList.h"
        #undef DECLARE_COMMAND
    };
}

///////////////////////////////////////////////////////////////////////////////
/// STATE
///////////////////////////////////////////////////////////////////////////////

struct DeviceState
{
    uint8_t m_device_type           = DEVICE_TYPE::SideKick;
    uint8_t m_brightness            = 255;
    uint8_t m_ambient_mode_type     = AMBIENT_MODE::RGB;
    uint8_t m_ambient_show_type     = AMBIENT_SCENE::RandomColor;
    uint8_t m_fade_rate             = 4;
    uint8_t m_group_number          = 1;
    uint8_t m_mode                  = DEVICE_MODE::Ambient;
    uint8_t m_esp_firmware[2]       = { 0, 0 };
    uint8_t m_ambient_color[3]      = { 0xFF, 0, 0 };
    uint8_t m_saturation[3]         = { 0xFF, 0xFF, 0xFF };
    uint8_t m_sector_assignment[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    StaticString<16> m_name         = "DreamStream";
    StaticString<16> m_group_name   = "unassigned";
} g_state;

///////////////////////////////////////////////////////////////////////////////
/// STRUCTS
///////////////////////////////////////////////////////////////////////////////

struct PacketHeader
{
    uint8_t m_magic;
    uint8_t m_length;
    uint8_t m_groupaddr;
    uint8_t m_flags;
    uint8_t m_cmd_upper;
    uint8_t m_cmd_lower;
};

///////////////////////////////////////////////////////////////////////////////
/// FORWARD DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

inline uint8_t* payloadWriteBuffer() FORCEINLINE;
inline void handleIncomingPacket(uint8_t flag, uint8_t cmdUpper, uint8_t cmdLower, const uint8_t* pktPayload, int32_t pktPayloadLen) FORCEINLINE;
inline void handleRequestCurrentState(const IPAddress& ip) FORCEINLINE;
inline void handleWriteAmbientColor() FORCEINLINE;
inline void handleWriteBrightness() FORCEINLINE;
inline void handleWriteSectorData(const PacketSectorData* data) FORCEINLINE;
inline void handleWriteMode() FORCEINLINE;
inline void sendPacket(const IPAddress& ip, uint8_t flag, COMMANDS::Type type, int32_t pktPayloadLen) FORCEINLINE;
inline uint8_t calculateCrc8(const uint8_t* data, int32_t dataSize) FORCEINLINE;

void Task1(void * parameter);
void update(void * parameter);
///////////////////////////////////////////////////////////////////////////////
/// MAIN
///////////////////////////////////////////////////////////////////////////////

void setName(const uint8_t *pld);

void setGroupName(const uint8_t *pld);

void setMode(const uint8_t *pld);

void Task1(void * parameter){
  for(;;){
    update();
    delayMicroseconds(100);
  }
}

void setup()
{
   xTaskCreatePinnedToCore(
     Task1,                  /* pvTaskCode */
     "Workload1",            /* pcName */
     1000,                   /* usStackDepth */
     NULL,                   /* pvParameters */
     1,                      /* uxPriority */
     &TaskA,                 /* pxCreatedTask */
     0);                     /* xCoreID */

    preferences.begin("dreamscreen", false);
    g_state.m_device_type = preferences.getUChar("m_device_type", DEVICE_TYPE::SideKick);
    g_state.m_brightness             = preferences.getUChar("m_brightness", 32);
    g_state.m_ambient_mode_type      = preferences.getUChar("m_ambient_mode_type", AMBIENT_MODE::RGB);
    g_state.m_ambient_show_type      = preferences.getUChar("m_ambient_show_type", AMBIENT_SCENE::RandomColor);
    g_state.m_fade_rate              = preferences.getUChar("m_fade_rate", 4);
    g_state.m_group_number           = preferences.getUChar("m_group_number", 1);
    g_state.m_mode                   = preferences.getUChar("m_mode", DEVICE_MODE::Video);
    preferences.getBytes("m_esp_firmware", g_state.m_esp_firmware, 2);
    preferences.getBytes("m_ambient_color", g_state.m_ambient_color, 3);
    preferences.getBytes("m_saturation", g_state.m_saturation, 3);
    preferences.getBytes("m_sector_assignment", g_state.m_sector_assignment, 15);
//    g_state.m_name = preferences.getString("m_name", "DreamStream");
//    g_state.m_group_name = preferences.getString("m_group_name", "unassigned");

    memset(g_read_buffer, 0, sizeof(g_read_buffer));
    memset(g_write_buffer, 0, sizeof(g_write_buffer));

    Serial.begin(115200);

    /*
     * Connect to Wifi
     */
    {
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);

        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
        }

        MDNS.begin(HOSTNAME);
    }

    Serial.println("Connected to wifi");

    ArduinoOTA.setHostname(HOSTNAME);

    ArduinoOTA
      .onStart([]() {
        updating = true;
        g_udp.stop();
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        updating = false;
        Serial.println("\nEnd");
        ESP.restart();
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        updating = false;
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        ESP.restart();
      });

    ArduinoOTA.begin();

    /*
     * Setup UDP
     */
    {
        g_udp.begin(DS_PORT);
    }

    /*
     * Setup FastLED
     */
    #ifdef USING_FASTLED
    FastLED.addLeds<FASTLED_LED_TYPE, FASTLED_PIN_ID, FASTLED_LED_COLOR_ORDER>(g_fastled_leds, FASTLED_NUM_LEDS).setCorrection(TypicalLEDStrip);
    #if FASTLED_NUM_LEDS > 50
    FastLED.setDither(0);
    #endif
    #endif // USING_FASTLED

    // timer = timerBegin(0, 80, true);
    // timerAttachInterrupt(timer, &periodicUpdate, true);
    // timerAlarmWrite(timer, UPDATE_INTERVAL * 1000, false);
    // timerAlarmEnable(timer);

    Serial.println("Entering main loop");
}

void calculateColor(){
  if (g_state.m_mode == DEVICE_MODE::Video){
      // WIP

//        // horizontal
//        fillPaletteFromSectorData(sectorPalette_TR_TL, data->m_sectors, sectorIndices_TR_TL, 50);
//        //fillPaletteFromSectorData(sectorPalette_BL_BR, data->m_sectors, sectorIndices_BL_BR, 5);
//
//        // vertical
//        fillPaletteFromSectorData(sectorPalette_BR_TR, data->m_sectors, sectorIndices_BR_TR, 25);
//        fillPaletteFromSectorData(sectorPalette_TL_BL, data->m_sectors, sectorIndices_TL_BL, 25);

       // CRGBPalette16 finalPalette;
      // finalPalette.loadDynamicGradientPalette(sectorPalette_BL_BR);

      /* Sectors
         6  5  4  3   2
         7            1
         8  9  10 12  0
      */

      CRGB* leds = fastledGetLeds();

      #ifdef ENABLE_GRADIENT

      fill_gradient(
          leds,
          25,
          sectors[7],
          sectors[8],
          sectors[9]);

      fill_gradient(
          leds + 25,
          25,
          sectors[7],
          sectors[6],
          sectors[5]);

      fill_gradient(
          leds + 50,
          25,
          sectors[5],
          sectors[4],
          sectors[3]);

      fill_gradient(
          leds + 75,
          25,
          sectors[3],
          sectors[2],
          sectors[1]);

      #else

      int index = 0;
      fill_solid(leds + index, 8, sectors[8]);
      index += 8;
      fill_solid(leds + index, 9, sectors[7]);
      index += 9;
      fill_solid(leds + index, 18, sectors[6]);
      index += 18;

      fill_solid(leds + index, 10, sectors[5]);
      index += 10;
      fill_solid(leds + index, 10, sectors[4]);
      index += 10;
      fill_solid(leds + index, 10, sectors[3]);
      index += 10;
      fill_solid(leds + index, 18, sectors[2]);
      index += 18;

      fill_solid(leds + index, 9, sectors[1]);
      index += 9;
      fill_solid(leds + index, 8, sectors[0]);

      #endif

//        for (int i = 0; i < 12; ++i)
//        {
//            fastledGetLeds()[i] = CRGB(data->m_sectors[i].rgb[0], data->m_sectors[i].rgb[1], data->m_sectors[i].rgb[2]);
//        }
  }
}

void update(){
      #ifdef USING_FASTLED
      #ifdef FASTLED_ENABLE_SMOOTHING

      EVERY_N_MILLISECONDS(1000/120)
      {
          for (int i = 0; i < FASTLED_NUM_LEDS; ++i)
          {
              g_fastled_leds[i] = blend(g_fastled_leds[i], g_fastled_targetleds[i], FASTLED_SMOOTHING);
          }
      }
      #endif // FASTLED_ENABLE_SMOOTHING

      FastLED.show();
      #endif // USING_FASTLED
      ArduinoOTA.handle();
}

void loop() {
    /*
     * Main Loop
     */
    int                 sktAvailable = 0;
    IPAddress           pktSender;
    int                 pktLen = 0;
    const PacketHeader* pktHdr = (const PacketHeader*)(g_read_buffer);
    const uint8_t*      pktPayload = (const uint8_t*)(g_read_buffer + sizeof(PacketHeader));
    int32_t             pktPayloadLen = 0;

    sktAvailable = g_udp.parsePacket();
    if (sktAvailable > 0)
    {
        pktSender = g_udp.remoteIP();
        pktLen = g_udp.read(g_read_buffer, sizeof(g_read_buffer));

        if (pktLen > sizeof(PacketHeader))
        {
            if (pktHdr->m_magic == DS_MAGIC)
            {
                // good enough, maybe check CRC if we really want to?
                pktPayloadLen = pktHdr->m_length - 5;
                handleIncomingPacket(pktSender, pktHdr->m_flags, pktHdr->m_cmd_upper, pktHdr->m_cmd_lower, pktPayload, pktPayloadLen);
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

uint8_t* payloadWriteBuffer() { return g_write_buffer + sizeof(PacketHeader); }

void handleIncomingPacket(const IPAddress& ip, uint8_t flag, uint8_t cmdUpper, uint8_t cmdLower, const uint8_t* pld, int32_t pldLen)
{
    uint16_t cmd = ((cmdUpper << 8) | cmdLower);

    switch (cmd)
    {
        case COMMANDS_IDS::ResetESP: break;
        case COMMANDS_IDS::Name:
            setName(pld);
            break;
        case COMMANDS_IDS::GroupName:
            setGroupName(pld);
            break;
        case COMMANDS_IDS::GroupNumber:
            setMode(pld);
            break;
        case COMMANDS_IDS::CurrentState:                handleRequestCurrentState(ip); break;
        case COMMANDS_IDS::Ping:                        sendPacket(ip, PKT_RESPONSE, COMMANDS::Ping, 0); break;
        case COMMANDS_IDS::SubscribeToSectorData:       payloadWriteBuffer()[0] = 1; sendPacket(ip, PKT_RESPONSE_REQUEST, COMMANDS::SubscribeToSectorData, 1); break;
        case COMMANDS_IDS::ReadBootloaderMode: break;

        case COMMANDS_IDS::ReadPICVersionNumber: break;
        case COMMANDS_IDS::ReadDiagnostics: break;

        case COMMANDS_IDS::Mode:                        g_state.m_mode = pld[0]; handleWriteMode(); break;
        case COMMANDS_IDS::Brightness:                  g_state.m_brightness = pld[0]; handleWriteBrightness(); break;
        case COMMANDS_IDS::Zones: break;
        case COMMANDS_IDS::ZonesBrightness: break;
        case COMMANDS_IDS::AmbientColor:                memcpy(g_state.m_ambient_color, pld, sizeof(g_state.m_ambient_color)); handleWriteAmbientColor(); break;
        case COMMANDS_IDS::Saturation:                  memcpy(g_state.m_saturation, pld, sizeof(g_state.m_saturation)); break;
        case COMMANDS_IDS::AmbientModeType:             g_state.m_ambient_mode_type = pld[0]; break;
        case COMMANDS_IDS::MusicModeType: break;
        case COMMANDS_IDS::MusicModeColors: break;
        case COMMANDS_IDS::MusicModeWeights: break;
        case COMMANDS_IDS::MinimumLuminosity: break;
        case COMMANDS_IDS::AmbientScene: break;
        case COMMANDS_IDS::IndicatorLightAutoOff: break;
        case COMMANDS_IDS::USBPowerEnable: break;
        case COMMANDS_IDS::SectorData:                  handleWriteSectorData((const PacketSectorData*)pld); break;
        case COMMANDS_IDS::SectorAssignment: break;
        case COMMANDS_IDS::SectorBroadcastControl: break;
        case COMMANDS_IDS::SectorBroadcastTiming: break;
        case COMMANDS_IDS::HDMIInput: break;
        case COMMANDS_IDS::MusicModeSource: break;
        case COMMANDS_IDS::HDMIInputName1: break;
        case COMMANDS_IDS::HDMIInputName2: break;
        case COMMANDS_IDS::HDMIInputName3: break;
        case COMMANDS_IDS::CECPassthroughEnable: break;
        case COMMANDS_IDS::CECSwitchingEnable: break;
        case COMMANDS_IDS::HPDEnable: break;
        case COMMANDS_IDS::VideoFrameDelay: break;
        case COMMANDS_IDS::LetterboxingEnable: break;
        case COMMANDS_IDS::HDMIActiveChannels: break;
        case COMMANDS_IDS::ColorBoost: break;
        case COMMANDS_IDS::CECPowerEnable: break;
        case COMMANDS_IDS::PillarboxingEnable: break;
        case COMMANDS_IDS::SKUSetup: break;
        case COMMANDS_IDS::FlexSetup: break;
        case COMMANDS_IDS::HDRToneRemapping: break;

        case COMMANDS_IDS::BootloaderFlags: break;
        case COMMANDS_IDS::ResetPIC: break;
        case COMMANDS_IDS::ESPConnectedToWiFi: break;
        case COMMANDS_IDS::OtherConnectedToWiFi: break;
    }
}

void setMode(const uint8_t *pld) {
    g_state.m_mode = pld[0];
    preferences.putUChar("m_mode", g_state.m_mode);
}

void setGroupName(const uint8_t *pld) {
    g_state.m_group_name = (const char*)pld;
    preferences.putString("m_group_name", g_state.m_group_name);
}

void setName(const uint8_t *pld) {
    g_state.m_name = (const char*)pld;
    preferences.putString("m_name", g_state.m_name);
}

void handleRequestCurrentState(const IPAddress& ip)
{
    uint8_t* buff = payloadWriteBuffer();

    memcpy(&buff[0], g_state.m_name.Get(), 16);
    memcpy(&buff[16], g_state.m_group_name, 16);
    buff[32] = g_state.m_group_number;
    buff[33] = g_state.m_mode;
    buff[34] = g_state.m_brightness;
    memcpy(&buff[35], g_state.m_ambient_color, 3);
    memcpy(&buff[38], g_state.m_saturation, 3);
    buff[41] = g_state.m_fade_rate;
    memcpy(&buff[42], g_state.m_sector_assignment, 15);
    memcpy(&buff[57], g_state.m_esp_firmware, 2);
    buff[59] = g_state.m_ambient_mode_type;
    buff[60] = g_state.m_ambient_show_type;
    buff[61] = g_state.m_device_type;

    sendPacket(ip, PKT_RESPONSE, COMMANDS::CurrentState, 62);
}

void handleWriteAmbientColor()
{
    #ifdef USING_FASTLED

    for (int i = 0; i < FASTLED_NUM_LEDS; ++i)
        fastledGetLeds()[i] = CRGB(g_state.m_ambient_color[0], g_state.m_ambient_color[1], g_state.m_ambient_color[2]);

    preferences.putBytes("m_ambient_color", g_state.m_ambient_color, 3);
    g_show_leds = true;

    #endif // USING_FASTLED
}

void handleWriteBrightness()
{
    #ifdef USING_FASTLED

    preferences.putUChar("m_brightness", g_state.m_brightness);
    FastLED.setBrightness(map(g_state.m_brightness, 0, 100, 0, 255));

    #endif // USING_FASTLED
}

//TDynamicRGBGradientPalette_byte sectorPalette_BR_TR[25 * 3];
//TDynamicRGBGradientPalette_byte sectorPalette_TR_TL[50 * 5];
//TDynamicRGBGradientPalette_byte sectorPalette_TL_BL[25 * 3];
//TDynamicRGBGradientPalette_byte sectorPalette_BL_BR[0 * 5];
//
//const uint8_t sectorIndices_BR_TR[] PROGMEM = { 0, 1, 2 };
//const uint8_t sectorIndices_TR_TL[] PROGMEM = { 2, 3, 4, 5, 6 };
//const uint8_t sectorIndices_TL_BL[] PROGMEM = { 6, 7, 8 };
//const uint8_t sectorIndices_BL_BR[] PROGMEM = { 8, 9, 10, 11, 0 };
//
//void fillPaletteFromSectorData(TDynamicRGBGradientPalette_byte* target, const SectorRGB* data, const uint8_t* indices, uint8_t numSteps)
//{
//    uint8_t indexDelta = 255 / (numSteps - 1);
//    int counter = 0;
//
//    for (uint8_t i = 0; i < numSteps; ++i)
//    {
//        target[4 * i] = (i == (numSteps - 1)) ? 255 : indexDelta * i;
//
//        for (uint8_t y = 0; y < 3; ++y)
//            target[4 * i + 1 + y] = data[indices[i]].rgb[y];
//    }
//}

void handleWriteSectorData(const PacketSectorData* data)
{

  // static int update_counter = 0;
  // static float last_update = 0;
  // static float slowest = 0;
  // update_counter++;
  //
  // float seconds = millis() / 1000.0;
  // slowest = max(slowest, seconds - last_update);
  // last_update = seconds;
  //
  // if(update_counter % 10 == 0){
  //   Serial.printf("updating colors: %d :: %.3f :: %.3f\n", update_counter, slowest, last_update);
  //   slowest = 0;
  // }

    #ifdef USING_FASTLED
    #ifdef ENABLE_GRADIENT

    for(int i=0; i<12; i++){
        sectors[i] = rgb2hsv_approximate(CRGB(
            data->m_sectors[i].rgb[0],
            data->m_sectors[i].rgb[1],
            data->m_sectors[i].rgb[2]));
    };

    #else

    for(int i=0; i<12; i++){
        sectors[i] = CRGB(
            data->m_sectors[i].rgb[0],
            data->m_sectors[i].rgb[1],
            data->m_sectors[i].rgb[2]);
    };


    #endif

    calculateColor();
    #endif // USING_FASTLED
}

void handleWriteMode()
{
    #ifdef USING_FASTLED

    switch (g_state.m_mode)
    {
        case DEVICE_MODE::Ambient:
        {
            for (int i = 0; i < FASTLED_NUM_LEDS; ++i)
                fastledGetLeds()[i] = CRGB(g_state.m_ambient_color[0], g_state.m_ambient_color[1], g_state.m_ambient_color[2]);
        }; break;

        case DEVICE_MODE::Video:
        {
            memset(fastledGetLeds(), 0, sizeof(g_fastled_leds));
            FastLED.clear();
        };

        case DEVICE_MODE::Sleep:
        {
            memset(fastledGetLeds(), 0, sizeof(g_fastled_leds));
            FastLED.clear();
        };
    }

    #endif // USING_FASTLED
}

void sendPacket(const IPAddress& ip, uint8_t flag, COMMANDS::Type type, int32_t pktPayloadLen)
{
    const uint8_t pktLength = uint8_t(sizeof(PacketHeader) + pktPayloadLen + 1);
    const uint8_t pktLengthMinOne = pktLength - 1;
    const uint8_t pktLengthMinTwo = pktLength - 2;

    CMD cmd = COMMANDS::Descriptors[type];

    g_udp.beginPacket(ip, DS_PORT);
    g_write_buffer[0] = DS_MAGIC;
    g_write_buffer[1] = pktLengthMinTwo;
    g_write_buffer[2] = g_state.m_group_number;
    g_write_buffer[3] = flag;
    g_write_buffer[4] = cmd.regions[0];
    g_write_buffer[5] = cmd.regions[1];
    g_write_buffer[pktLengthMinOne] = calculateCrc8(g_write_buffer, pktLengthMinOne);
    g_udp.write(g_write_buffer, pktLength);
    g_udp.endPacket();
}

uint8_t calculateCrc8(const uint8_t* data, int32_t dataSize)
{
    uint8_t cntr = 0x00;
    uint8_t crc = 0x00;

    while (cntr < dataSize)
    {
        crc = uartComm_crc8_table[uint8_t(crc ^ data[cntr]) & 0xFF];
        ++cntr;
    }

    return crc;
}
