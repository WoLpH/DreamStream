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

#include "DeviceEmulator.h"
#include "Commands.h"
#include "Packet.h"
#include "DeviceManager.h"
#include "DeviceType.h"
#include "PacketHelpers.h"
#include "Enums.h"
#include <time.h>

#define FASTLED_ALLOW_INTERRUPTS 0

#ifdef USING_FASTLED
#include "FastLED.h"

#define NUM_LEDS 1
#define LED_TYPE WS2812B
#define DATA_PIN 5
#define COLOR_ORDER GRB

CRGB leds[5];
#endif

DeviceSideKickEmu::DeviceSideKickEmu(DeviceManager* manager)
    : DeviceSideKick(manager, "127.0.0.1")
    , m_has_active_sector_subscription(false)
{
    SetEmulated(true);

    m_group_number = 0x1;

    m_name = "DreamStream-mac";

    #ifdef USING_FASTLED
    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(100);
    #endif
}

DeviceSideKickEmu::~DeviceSideKickEmu()
{

}

void DeviceSideKickEmu::Update()
{
    if (m_has_active_sector_subscription)
    {
        if (m_sector_subscription_timeout.Elapsed() > SECTOR_SUBSCRIPTION_TIMEOUT_MS)
        {
            // timeout!
            m_has_active_sector_subscription = false;

        #ifndef ENABLE_CLIENT_DEVICES
            SetHostAddress(nullptr);
        #endif

            LOG_INFO("Sector subscription timed out!");
        }
    }

//    printf("Setting color: #%02x%02x%02x\n", m_ambient_color[0], m_ambient_color[1], m_ambient_color[2]);
    if (m_mode == DeviceMode::AMBIENT)
    {
        #ifdef USING_FASTLED
        leds[0] = CRGB(m_ambient_color[0], m_ambient_color[1], m_ambient_color[2]);
        #endif
    }

    #ifdef USING_FASTLED
    FastLED.show();
    #endif
}

void DeviceSideKickEmu::HandleChangeBrightness()
{
    #ifdef USING_FASTLED
    FastLED.setBrightness(m_brightness);
    #endif
}

void DeviceSideKickEmu::HandlePacket(HandleState& hs, const UDPMessageInfo& msg, const PacketInfo& pkt)
{
    Super::HandlePacket(hs, msg, pkt);

    if (PKT_IS(PKT_RESPONSE | PKT_BROADCAST, pkt.m_flag))
    {
        switch (pkt.m_type)
        {
            case Commands::SectorData: HandlePacket_Response_SectorData(msg, pkt); hs.SetHandled(); break;
            default: hs.SetUnhandled(); break;
        }
    }

    if (PKT_IS(PKT_RESPONSE_REQUEST, pkt.m_flag))
    {
        switch (pkt.m_type)
        {
            case Commands::SubscribeToSectorData: HandlePacket_Read_SubscribeToSectorData(msg, pkt); hs.SetHandled(); break;
            case Commands::CurrentState: HandlePacket_Read_CurrentState(msg, pkt); hs.SetHandled(); break;
            case Commands::Ping: SendPacket(msg.m_ipv4addr, Commands::Ping, PKT_RESPONSE); hs.SetHandled(); break;
            default: hs.SetUnhandled(); break;
        }
    }

    if (PKT_IS(PKT_WRITE_CONSTANT, pkt.m_flag))
    {
        switch (pkt.m_type)
        {
            case Commands::AmbientColor: memcpy(m_ambient_color, pkt.m_payload, 3);  hs.SetHandled(); break;
            case Commands::Brightness: PacketUtils::TrySetFromPayload(m_brightness, pkt); HandleChangeBrightness(); hs.SetHandled(); break;
            case Commands::Mode: PacketUtils::TrySetFromPayload(m_mode, pkt);  hs.SetHandled(); break;
            case Commands::GroupNumber: PacketUtils::TrySetFromPayload(m_group_number, pkt);  hs.SetHandled(); break;
            case Commands::Name: PacketUtils::TrySetFromPayload(m_name, pkt);  hs.SetHandled(); break;
            case Commands::GroupName: PacketUtils::TrySetFromPayload(m_name, pkt);  hs.SetHandled(); break;
            default: hs.SetUnhandled(); break;
        }
    }
}

void DeviceSideKickEmu::HandlePacket_Read_CurrentState(const UDPMessageInfo& msg, const PacketInfo& pkt)
{
    uint8_t payload[DS_STATE_PAYLOAD_SIZE];
    memset(payload, 0, sizeof(payload));
    int32_t realPayloadSize = SetStateToPayload(payload);
    SendPacket(msg.m_ipv4addr, Commands::CurrentState, PKT_RESPONSE, payload, realPayloadSize);
}

void DeviceSideKickEmu::HandlePacket_Read_SubscribeToSectorData(const UDPMessageInfo& msg, const PacketInfo& pkt)
{
    if (!m_has_active_sector_subscription)
    {
    #ifndef ENABLE_CLIENT_DEVICES
        // We are not bound to any device at this point, so let's bind to this one.
        SetHostAddress(msg.m_ipv4addr);
    #endif

        m_has_active_sector_subscription = true;
        LOG_INFO("Received SubscribeToSectorData broadcast, requesting subscription to sector data...");
    }

    // send back the acknowledgement
    uint8_t flag = 1;
    SendCommandCustom(Commands::SubscribeToSectorData, PKT_RESPONSE_REQUEST, &flag, 1);
    m_sector_subscription_timeout.Reset();
}

double millis(){
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);

    return (t.tv_sec * 1000) + (t.tv_nsec / 1.0e6);
}

void DeviceSideKickEmu::HandlePacket_Response_SectorData(const UDPMessageInfo& msg, const PacketInfo& pkt)
{
    if (PKT_IS(PKT_RESPONSE, pkt.m_hdr->m_flags))
    {
        if (const PacketSectorData* data = PacketUtils::GetPayloadAs<PacketSectorData>(pkt))
        {
            #ifdef ENABLE_LOGGING
            static double boot = millis();
            static double start = millis();
            static double end = millis();
            start = end;
            end = millis();
            double duration = end - start;
            // if duration is below 50ms, green, else red
            int color = duration < 100 ? 32 : 31;

            printf("\033[%dmcolors: %6.0f %4.0f\033[0m ", color, end - boot, duration);

            for(int j=0; j<12; j++){
                printf("\033[48;2;%d;%d;%dm#%02x%02x%02x\033[0m ",
                       data->m_sector_1[(3*j) + 0],
                       data->m_sector_1[(3*j) + 1],
                       data->m_sector_1[(3*j) + 2],
                       data->m_sector_1[(3*j) + 0],
                       data->m_sector_1[(3*j) + 1],
                       data->m_sector_1[(3*j) + 2]);
            }
            printf("\n");
            fflush(stdout);
            #endif

//                printf("Setting color: #%02x%02x%02x\n", m_ambient_color[0], m_ambient_color[1], m_ambient_color[2]);
            #ifdef USING_FASTLED
            leds[0] = CRGB(data->m_sector_1[0], data->m_sector_1[1], data->m_sector_1[2]);
            #endif
        }
    }
}
