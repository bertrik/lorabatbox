#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include "arduino_lmic_hal_boards.h"

#include <SPI.h>
#include <SSD1306.h>

#include "soc/efuse_reg.h"
#include "soc/rtc.h"

#include "HardwareSerial.h"
#include "EEPROM.h"

// This EUI must be in BIG-ENDIAN format, or "msb".
// For TTN issued EUIs the first bytes should be 0x70, 0xB3, 0xD5.
static const u1_t APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xEF, 0x86 };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the console can be copied as-is.
static const u1_t APPKEY[] = {
    0x27, 0x70, 0x0B, 0xF7, 0xB7, 0xD8, 0x84, 0x5E, 0x74, 0x16, 0x3F, 0x1E, 0xAE, 0x8C, 0x75, 0x83
};

#define OLED_I2C_ADDR 0x3C

#define PIN_OLED_RESET  16
#define PIN_OLED_SDA    4
#define PIN_OLED_SCL    15
#define PIN_BUTTON      0
#define PIN_VEXT        21

#define OTAA_MAGIC      "MAGIC"

typedef struct {
    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    u1_t dn2Dr;
    u1_t rx1DrOffset;
    u1_t rxDelay;
    u4_t channelFreq[MAX_CHANNELS];
    u2_t channelDrMap[MAX_CHANNELS];
    u4_t channelDlFreq[MAX_CHANNELS];
    u2_t channelMap;

    char magic[8];
} otaa_data_t;

typedef struct {
    bool update;
    // 1st line: LoRa address
    char loraDevEui[32];
    // 2nd line: LoRa status
    char loraStatus[32];
    // 3rd line: PM10
    String dust1;
    // 4th line: PM2.5
    String dust2;
} screen_t;

// Pin mapping
const lmic_pinmap lmic_pins = *Arduino_LMIC::GetPinmap_ThisBoard();

// stored in "little endian" format
static uint8_t deveui[8];
static SSD1306 display(OLED_I2C_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
static screen_t screen;

static uint32_t time_offset = 0;
static uint32_t time_seq = 0;

static void inittime(void)
{
    rtc_clk_32k_bootstrap(1024);
    rtc_clk_32k_enable(true);
    rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);
}

static void adjtime(uint32_t t)
{
    time_offset += t;
}

static uint32_t gettime(void)
{
    uint64_t rtc = rtc_time_get() / 32768;
    return time_offset + rtc;
}

// This should also be in little endian format, see above.
void os_getDevEui(u1_t * buf)
{
    for (int i = 0; i < 8; i++) {
        buf[i] = deveui[7 - i];
    }
}

void os_getArtEui(u1_t * buf)
{
    for (int i = 0; i < 8; i++) {
        buf[i] = APPEUI[7 - i];
    }
}

void os_getDevKey(u1_t * buf)
{
    memcpy(buf, APPKEY, 16);
}

static void setLoraStatus(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(screen.loraStatus, sizeof(screen.loraStatus), fmt, args);
    va_end(args);

    screen.update = true;
}

const char *event_names[] = { LMIC_EVENT_NAME_TABLE__INIT };

static void otaa_save(void)
{
    otaa_data_t otaa_data;

    LMIC_getSessionKeys(&otaa_data.netid, &otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
    otaa_data.dn2Dr = LMIC.dn2Dr;
    otaa_data.rx1DrOffset = LMIC.rx1DrOffset;
    otaa_data.rxDelay = LMIC.rxDelay;

    memcpy(otaa_data.channelFreq, LMIC.channelFreq, sizeof(otaa_data.channelFreq));
    memcpy(otaa_data.channelDrMap, LMIC.channelDrMap, sizeof(otaa_data.channelDrMap));
    memcpy(otaa_data.channelDlFreq, LMIC.channelDlFreq, sizeof(otaa_data.channelDlFreq));
    otaa_data.channelMap = LMIC.channelMap;

    strcpy(otaa_data.magic, OTAA_MAGIC);
    EEPROM.put(0, otaa_data);
    EEPROM.commit();
}

static bool otaa_restore(void)
{
    otaa_data_t otaa_data;

    EEPROM.get(0, otaa_data);
    if (strcmp(otaa_data.magic, OTAA_MAGIC) != 0) {
        return false;
    }
    LMIC_setSession(otaa_data.netid, otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
    LMIC.dn2Dr = otaa_data.dn2Dr;
    LMIC.rx1DrOffset = otaa_data.rx1DrOffset;
    LMIC.rxDelay = otaa_data.rxDelay;

    memcpy(LMIC.channelFreq, otaa_data.channelFreq, sizeof(LMIC.channelFreq));
    memcpy(LMIC.channelDrMap, otaa_data.channelDrMap, sizeof(LMIC.channelDrMap));
    memcpy(LMIC.channelDlFreq, otaa_data.channelDlFreq, sizeof(LMIC.channelDlFreq));
    LMIC.channelMap = otaa_data.channelMap;

    return true;
}

static void onEventCallback(void *user, ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    Serial.println(event_names[ev]);

    switch (ev) {
    case EV_JOINING:
        setLoraStatus("OTAA JOIN...");
        break;
    case EV_JOINED:
        otaa_save();
        setLoraStatus("JOIN OK!");
        break;
    case EV_JOIN_FAILED:
        setLoraStatus("JOIN failed!");
        break;
    case EV_REJOIN_FAILED:
        setLoraStatus("REJOIN failed!");
        break;
    case EV_TXCOMPLETE:
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println("Received ack");
        if (LMIC.dataLen) {
            Serial.print("Received ");
            Serial.print(LMIC.dataLen);
            Serial.println(" bytes of payload");
        }
        setLoraStatus("%08X-%d", LMIC.devaddr, LMIC.seqnoUp);
        break;
    case EV_TXSTART:
        setLoraStatus("Transmit SF%d", getSf(LMIC.rps) + 6);
        break;
    case EV_RXSTART:
        setLoraStatus("Receive SF%d", getSf(LMIC.rps) + 6);
        break;
    case EV_JOIN_TXCOMPLETE:
        setLoraStatus("JOIN sent");
        break;
    default:
        Serial.print("Unknown event: ");
        Serial.println((unsigned) ev);
        break;
    }
}

static void send_message(void)
{
    uint8_t buf[20];
    int idx = 0;

    if ((LMIC.opmode & (OP_TXDATA | OP_TXRXPEND)) == 0) {
        time_t t = time(NULL);
        buf[idx++] = (t >> 24) & 0xFF;
        buf[idx++] = (t >> 16) & 0xFF;
        buf[idx++] = (t >> 8) & 0xFF;
        buf[idx++] = (t >> 0) & 0xFF;

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, buf, idx, 0);
    }
}

static void screen_update(void)
{
    if (screen.update) {
        display.clear();

        // 1st line
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, screen.loraDevEui);

        // 2nd line
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 12, screen.loraStatus);

        // 3rd
        display.drawString(0, 30, screen.dust1);

        // 4th line
        display.drawString(0, 46, screen.dust2);

        display.display();
        screen.update = false;
    }
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Starting...");

    // VEXT config: 0 = enable Vext
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, 0);

    // LED config
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 0);

    // button config
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    // init the OLED
    pinMode(PIN_OLED_RESET, OUTPUT);
    digitalWrite(PIN_OLED_RESET, LOW);
    delay(50);
    digitalWrite(PIN_OLED_RESET, HIGH);

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    // setup of unique ids
    uint64_t chipid = ESP.getEfuseMac();
    deveui[0] = (chipid >> 56) & 0xFF;
    deveui[1] = (chipid >> 48) & 0xFF;
    deveui[2] = (chipid >> 40) & 0xFF;
    deveui[3] = (chipid >> 32) & 0xFF;
    deveui[4] = (chipid >> 24) & 0xFF;
    deveui[5] = (chipid >> 16) & 0xFF;
    deveui[6] = (chipid >> 8) & 0xFF;
    deveui[7] = (chipid >> 0) & 0xFF;
    snprintf(screen.loraDevEui, sizeof(screen.loraDevEui),
             "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", deveui[0], deveui[1], deveui[2], deveui[3],
             deveui[4], deveui[5], deveui[6], deveui[7]);

    // RTC
    inittime();

    // LMIC init
    os_init();
    LMIC_reset();
    LMIC_registerEventCb(onEventCallback, NULL);

    EEPROM.begin(512);
    if (otaa_restore()) {
        setLoraStatus("Resume OTAA");
    } else {
        LMIC_startJoining();
    }
}

void loop(void)
{
    static unsigned long button_ts = 0;

    // check for long button press to restart OTAA
    unsigned long ms = millis();
    if (digitalRead(PIN_BUTTON) == 0) {
        if ((ms - button_ts) > 2000) {
            LMIC_reset();
            LMIC_startJoining();
            button_ts = ms;
        }
    } else {
        button_ts = ms;
    }

    // update screen
    screen_update();

    // run LoRa process
    os_runloop_once();
}

