// Based originally off of code from:
// https://github.com/rickkas7/LIS3DH
//
#include "Serial5/Serial5.h"
#include "Particle.h"
#include "LIS3DH.h"
#include "TinyGPS++.h"

SYSTEM_THREAD(ENABLED);

FuelGauge batteryMonitor;
#ifdef MONITOR_MOTION
LIS3DHSPI accel(SPI, A2, WKP);
#endif
TinyGPSPlus gps;

// This is the name of the Particle event to publish for battery or movement detection events
const char *eventName = "gps";

// Various timing constants
const uint32_t PUBLISH_INTERVAL_MS = 22 * 60 * 1000;     // Only publish every fifteen minutes
const uint32_t PUBLISH_INTERVAL_SEC = PUBLISH_INTERVAL_MS / 1000;
const uint32_t MAX_TIME_TO_PUBLISH_MS = 5 * 1000;       // Only stay awake for 60 seconds trying to connect to the cloud and publish
const uint32_t MAX_TIME_FOR_GPS_FIX_MS = 3 * 60 * 1000;  // Only stay awake for 3 minutes trying to get a GPS fix
const uint32_t TIME_AFTER_PUBLISH_MS = 4 * 1000;         // After publish, wait 4 seconds for data to go out
const uint32_t TIME_AFTER_BOOT_MS = 5 * 1000;            // At boot, wait 5 seconds before going to sleep again (after coming online)
const uint32_t PUBLISH_TTL = 60;

// Stuff for the finite state machine
enum State {
    ONLINE_WAIT_STATE,
    RESET_STATE,
    RESET_WAIT_STATE,
    PUBLISH_STATE,
    SLEEP_STATE,
    SLEEP_WAIT_STATE,
    BOOT_WAIT_STATE,
    GPS_WAIT_STATE
};
State state = ONLINE_WAIT_STATE;
uint32_t stateTime = 0;
uint32_t startFix = 0;
#ifdef MONITOR_MOTION
bool moved = 0;
#endif

void flashLed(uint8_t times, uint32_t delayMs) {
    for (uint8_t i = 0; i < times; ++i) {
        digitalWrite(D7, HIGH);
        delay(delayMs);
        digitalWrite(D7, LOW);
        delay(delayMs);
    }
}

void flashRgb(uint8_t r, uint8_t g, uint8_t b, uint8_t times, uint32_t delayMs) {
    RGB.control(true);
    RGB.brightness(255);
    for (uint8_t i = 0; i < times; ++i) {
        RGB.color(r, g, b);
        delay(delayMs);
        RGB.color(0, 0, 0);
        delay(delayMs);
    }
    RGB.control(false);
}

#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PGCMD_ANTENNA "$PGCMD,33,1*6C"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial5.begin(9600);

    pinMode(D6, OUTPUT);
    digitalWrite(D6, HIGH);

    delay(500);

    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    /*
    delay(500);
    Serial1.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    delay(500);
    Serial1.println(PMTK_SET_NMEA_UPDATE_1HZ);
    delay(500);
    Serial1.println(PGCMD_NOANTENNA);
    delay(500);
    */

    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

    startFix = millis();
}

void loop() {
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        Serial5.print(c);
        if (gps.encode(c)) {
            Serial5.println();
        }
    }

    switch(state) {
    case ONLINE_WAIT_STATE:
        if (Particle.connected()) {
            state = RESET_STATE;
        }
        if (millis() - stateTime > 5000) {
            stateTime = millis();
        }
        break;

    case RESET_STATE: {
        #ifdef MONITOR_MOTION
        LIS3DHConfig config;
        config.setLowPowerWakeMode(16);

        if (!accel.setup(config)) {
            state = SLEEP_STATE;
            break;
        }
        #endif

        state = BOOT_WAIT_STATE;
        break;
    }
    case GPS_WAIT_STATE:
        if (gps.location.isValid()) {
            state = PUBLISH_STATE;
            break;
        }
        if (millis() - stateTime >= MAX_TIME_FOR_GPS_FIX_MS) {
            flashRgb(255, 0, 0, 1, 500);
            state = SLEEP_STATE; // SLEEP_STATE
            break;
        }
        break;

    case PUBLISH_STATE: {
        char data[128];
        float cellVoltage = batteryMonitor.getVCell();
        float stateOfCharge = batteryMonitor.getSoC();

        snprintf(data, sizeof(data), "%.02f,%.02f,%f,%f,%f,%ld,%ld,%ld",
                 cellVoltage,
                 stateOfCharge,
                 gps.location.lat(),
                 gps.location.lng(),
                 gps.altitude.meters(),
                 gps.satellites.value(),
                 gps.hdop.value(),
                 gps.location.isValid());

        Serial5.print(Time.now());
        Serial5.print(",");
        Serial5.println(data);

        if (Particle.connected()) {
            flashRgb(0, 255, 0, 1, 500);

            Particle.publish(eventName, data, PUBLISH_TTL, PRIVATE);

            stateTime = millis();
            state = SLEEP_WAIT_STATE;
        }
        else {
            if (millis() - stateTime >= MAX_TIME_TO_PUBLISH_MS) {
                flashRgb(255, 0, 0, 1, 500);
                state = SLEEP_STATE;
            }
        }
        break;
    }
    case SLEEP_WAIT_STATE:
        if (millis() - stateTime >= TIME_AFTER_PUBLISH_MS) {
            state = SLEEP_STATE;
        }
        break;

    case BOOT_WAIT_STATE:
        if (millis() - stateTime >= TIME_AFTER_BOOT_MS) {
            state = GPS_WAIT_STATE;
            stateTime = millis();
        }
        break;

    case SLEEP_STATE:
        #ifdef MONITOR_MOTION
        // Wait for Electron to stop moving for 2 seconds so we can recalibrate the accelerometer
        accel.calibrateFilter(2000);
        #endif

        delay(500);

        digitalWrite(D6, HIGH);

        // System.sleep(WKP, RISING, PUBLISH_INTERVAL_SEC, SLEEP_NETWORK_STANDBY);
        System.sleep(SLEEP_MODE_DEEP, PUBLISH_INTERVAL_SEC);

        // This delay should not be necessary, but sometimes things don't seem to work right
        // immediately coming out of sleep.
        delay(500);

        #ifdef MONITOR_MOTION
        moved = ((accel.clearInterrupt() & LIS3DH::INT1_SRC_IA) != 0);
        #endif

        digitalWrite(D6, LOW);
        startFix = millis();

        state = GPS_WAIT_STATE;
        stateTime = millis();
        break;
    }
}
