/*******************************************************************************
 * The Things Network - Sensor Data Example
 *
 * Example of sending a valid LoRaWAN packet with DHT22 temperature and
 * humidity data to The Things Networ using a Feather M0 LoRa.
 *
 * Learn Guide: https://learn.adafruit.com/the-things-network-for-feather
 *
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 * Copyright (c) 2018 Brent Rubell, Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *******************************************************************************/
#include <arduino_lmic.h>
#include <hal/hal.h>
#include <RTCZero.h> // Arduino Zero Real Time Clock Library
#include <SPI.h>
#include <CayenneLPP.h>
#include <DHT.h> // include the DHT22 Sensor Library

#define serial Serial1

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 120; // 5 mins

// DHT digital pin and sensor type
#define DHTPIN 10
#define DHTTYPE DHT22

// LMIC Device State
#define STATE_IDLE 0
#define STATE_STANDBY_READY 1
#define STATE_DO_JOB 2
static uint8_t state = STATE_IDLE;
// //
// // For normal use, we require that you edit the sketch to replace FILLMEIN
// // with values assigned by the TTN console. However, for regression tests,
// // we want to be able to compile these scripts. The regression tests define
// // COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// // working but innocuous value.
// //
// #ifdef COMPILE_REGRESSION_TEST
// #define FILLMEIN 0
// #else
// #warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
// #define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
// #endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00,0x00, 0x00,0x00,0x00,0x00,0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xf3, 0xc2, 0xe8, 0x23, 0x2c, 0x3a, 0x23, 0x74 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x75, 0x91, 0x12, 0xf7, 0xce, 0xd1, 0x23, 0x82, 0x1a, 0x06, 0xdb, 0x51, 0x2a, 0x90, 0x92, 0xd2 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


// payload to send to TTN gateway
static uint8_t payload[5];
static osjob_t sendjob;

// Pin mapping for Adafruit Feather M0 LoRa
// /!\ By default Adafruit Feather M0's pin 6 and DIO1 are not connected.
// Please ensure they are connected.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};


// init. DHT
DHT dht(DHTPIN, DHTTYPE);

// CayenneLPP testing stuff
#define MAX_CAYENNE_PAYLOAD_SIZE 51
CayenneLPP lpp(MAX_CAYENNE_PAYLOAD_SIZE);

// Declare real time clock
RTCZero rtc;

uint32_t userUTCTime; // Seconds since the UTC epoch

// Utility function for digital clock display: prints preceding colon and
// leading 0
void printDigits(int digits) {
    serial.print(':');
    if (digits < 10) serial.print('0');
    serial.print(digits);
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        serial.print('0');
    serial.print(v, HEX);
}

// A buffer for printing log messages.
static constexpr int MAX_MSG = 256;
static char msg[MAX_MSG];

// A printf-like function to print log messages prefixed by the current
// LMIC tick value. Don't call it before os_init();
//
// The RTC timestamps will start at 00:00:00, but will update to UTC
// if the DeviceTimeReq is answered.
void log_msg(const char *fmt, ...) {
#ifdef USE_SERIAL
    snprintf(msg, MAX_MSG, "%02d:%02d:%02d / ", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    serial.write(msg, strlen(msg));
    snprintf(msg, MAX_MSG, "% 012ld: ", os_getTime());
    serial.write(msg, strlen(msg));
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, MAX_MSG, fmt, args);
    va_end(args);
    serial.write(msg, strlen(msg));
    serial.println();
#endif
}

/*
 * This function is used to set the alarm to a relative time in the future, such as when
 * sleeping between LMIC tasks.
 */
void set_delta_alarm(uint32_t delta_seconds) {
    int32_t ss = (int32_t)rtc.getSeconds();
    int32_t mm = (int32_t)rtc.getMinutes();
    int32_t hh = (int32_t)rtc.getHours();

    // Sanity check.
    if (delta_seconds < 1) {
        delta_seconds = 1;
    }

    int32_t delta = delta_seconds;
    int32_t hh_delta = delta / 3600; delta -= (hh_delta * 3600);
    // Will always be less than 1 hour.
    int32_t mm_delta = delta / 60; delta -= (mm_delta * 60);
    // Will always be less than 1 minute.
    int32_t ss_delta = delta;

    ss += ss_delta;
    if (ss > 59) {
        ss = ss % 60;
        mm_delta++;
    }

    mm += mm_delta;
    if (mm > 59) {
        mm = mm % 60;
        hh_delta++;
    }

    hh = (hh + hh_delta) % 24;

    serial.print("Delta(s) = ");
    serial.print(delta_seconds);
    serial.print(F(", wake at "));
    serial.print(hh);
    printDigits(mm);
    printDigits(ss);
    serial.println();

    rtc.setAlarmTime((uint8_t)(hh & 0xff), (uint8_t)(mm & 0xff), (uint8_t)(ss & 0xff));
    rtc.enableAlarm(RTCZero::MATCH_HHMMSS);
}


void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess) {
    // Explicit conversion from void* to uint32_t* to avoid compiler errors
    uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;

    // A struct that will be populated by LMIC_getNetworkTimeReference.
    // It contains the following fields:
    //  - tLocal: the value returned by os_GetTime() when the time
    //            request was sent to the gateway, and
    //  - tNetwork: the seconds between the GPS epoch and the time
    //              the gateway received the time request
    lmic_time_reference_t lmicTimeReference;

    if (flagSuccess != 1) {
        serial.println(F("USER CALLBACK: Not a success"));
        return;
    }

    // Populate "lmic_time_reference"
    flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
    if (flagSuccess != 1) {
        serial.println(F("USER CALLBACK: LMIC_getNetworkTimeReference didn't succeed"));
        return;
    }

    // Update userUTCTime, considering the difference between the GPS and UTC
    // epoch, and the leap seconds
    *pUserUTCTime = lmicTimeReference.tNetwork + 315964800;

    // Add the delay between the instant the time was transmitted and
    // the current time

    // Current time, in ticks
    ostime_t ticksNow = os_getTime();
    // Time when the request was sent, in ticks
    ostime_t ticksRequestSent = lmicTimeReference.tLocal;
    uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
    *pUserUTCTime += requestDelaySec;

    // Add time zone offset
    *pUserUTCTime += (10*60*60);

    // Update the system time with the time read from the network
    rtc.setEpoch(*pUserUTCTime);

    serial.print(F("The current UTC time is: "));
    serial.print(rtc.getHours());
    printDigits(rtc.getMinutes());
    printDigits(rtc.getSeconds());
    serial.print(' ');
    serial.print(rtc.getDay());
    serial.print('/');
    serial.print(rtc.getMonth());
    serial.print('/');
    serial.print(rtc.getYear());
    serial.println();
}

void onEvent (ev_t ev) {
    serial.print(os_getTime());
    serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              serial.print("netid: ");
              serial.println(netid, DEC);
              serial.print("devaddr: ");
              serial.println(devaddr, HEX);
              serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  serial.print("-");
                printHex2(artKey[i]);
              }
              serial.println("");
              serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              serial.print("-");
                      printHex2(nwkKey[i]);
              }
              serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            state = STATE_DO_JOB;
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              serial.println(F("Received "));
              serial.println(LMIC.dataLen);
              serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            state = STATE_IDLE;
            break;
        case EV_LOST_TSYNC:
            serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            serial.print(F("Unknown event: "));
            serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // read the temperature from the DHT22
        float temperature = dht.readTemperature();
        serial.print("Temperature: "); serial.print(temperature);
        serial.println(" *C");
        // adjust for the f2sflt16 range (-1 to 1)
        // temperature = temperature / 100;

        // read the humidity from the DHT22
        float rHumidity = dht.readHumidity();
        serial.print("%RH ");
        serial.println(rHumidity);
        // adjust for the f2sflt16 range (-1 to 1)
        // rHumidity = rHumidity / 100;

        // float -> int
        // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
        // uint16_t payloadTemp = LMIC_f2sflt16(temperature);
        // // int -> bytes
        // byte tempLow = lowByte(payloadTemp);
        // byte tempHigh = highByte(payloadTemp);
        // // place the bytes into the payload
        // payload[0] = tempLow;
        // payload[1] = tempHigh;

        // // float -> int
        // uint16_t payloadHumid = LMIC_f2sflt16(rHumidity);
        // // int -> bytes
        // byte humidLow = lowByte(payloadHumid);
        // byte humidHigh = highByte(payloadHumid);
        // payload[2] = humidLow;
        // payload[3] = humidHigh;

        // prepare upstream data transmission at the next possible time.
        // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
        // LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);

        // Testing CayenneLPP payload structure
        lpp.reset();
        lpp.addTemperature(1, temperature);
        lpp.addRelativeHumidity(1, rHumidity);
        serial.print(F("Cayenne Packet Size: "));
        serial.println(lpp.getSize());

        lmic_tx_error_t txDataError;
        txDataError = LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);

        serial.println(F("Packet queued"));
        if (txDataError == LMIC_ERROR_SUCCESS) {
            serial.println(F("Packet will be sent"));
        } else if (txDataError == LMIC_ERROR_TX_BUSY) {
            serial.println(F("Packet not sent, LMIC busy sending other message"));
        } else if (txDataError == LMIC_ERROR_TX_TOO_LARGE) {
            serial.println(F("Packet too large for current datarate"));
        } else if (txDataError == LMIC_ERROR_TX_NOT_FEASIBLE) {
            serial.println(F("Packet unsuitable for current datarate"));
        } else {
            serial.println(F("Queued message failed to send for other reason than data len"));
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(5000);
    pinMode(LED_BUILTIN, OUTPUT);
    serial.begin(115200);
    while (!serial);
    serial.println(F("Starting"));

    dht.begin();
    rtc.begin(false);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Disable link-check mode and ADR, because ADR tends to complicate testing.
    LMIC_setLinkCheckMode(0);
    // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
    // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
    LMIC_setDrTxpow(DR_SF7,14);
    // in the US, with TTN, it saves join time if we start on subband 1 (channels 8-15). This will
    // get overridden after the join by parameters from the network. If working with other
    // networks or in other regions, this will need to be changed.
    LMIC_selectSubBand(1);
    // Define device as being powered by external power source
    LMIC_setBatteryLevel(MCMD_DEVS_EXT_POWER);

    serial.println(F("Joining"));
    LMIC_startJoining();
    // Start job (sending automatically starts OTAA too)
    // serial.println(F("Do First Job"));
    // do_send(&sendjob);
    LMIC_requestNetworkTime(user_request_network_time_callback, &userUTCTime);
}

bit_t have_deadline = 0;

void loop() {
    // we call the LMIC's runloop processor. This will cause things to happen based on events and time. One
    // of the things that will happen is callbacks for transmission complete or received messages. We also
    // use this loop to queue periodic data transmissions.  You can put other things here in the `loop()` routine,
    // but beware that LoRaWAN timing is pretty tight, so if you do more than a few milliseconds of work, you
    // will want to call `os_runloop_once()` every so often, to keep the radio running.
    os_runloop_once();

    // Let radio do its thing before consider doing anything else.
    if (!(LMIC.opmode & OP_TXRXPEND)) {
        switch (state) {
            case STATE_DO_JOB:
                digitalWrite(LED_BUILTIN, HIGH);
                serial.println(F("State: Do_JOB"));
                os_setCallback(&sendjob, do_send);
                // State update in radio_onevent
                state = STATE_IDLE;
                break;
            case STATE_STANDBY_READY:
                digitalWrite(LED_BUILTIN, LOW);
                serial.println(F("State: STANDBY_READY"));
                set_delta_alarm(TX_INTERVAL);
                // delay(TX_INTERVAL*1000);
                serial.flush();
                rtc.standbyMode();

                // Disable the alarm in case it was set to some short interval and LMIC
                // tasks will run for longer than that. It probably wouldn't cause
                // trouble but may as well be sure.
                rtc.disableAlarm();
                state = STATE_DO_JOB;
                break;
            case STATE_IDLE:
                have_deadline = os_queryTimeCriticalJobs(sec2osticks(TX_INTERVAL));
                // os_getNextDeadline(&have_deadline);
                if (!have_deadline) {
                    state = STATE_STANDBY_READY;
                }
                break;
        }
    }
}
