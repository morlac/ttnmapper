/**
 *
 */

#include <Arduino.h>

#include "ttnmapper-config.hpp"

#include <NeoSWSerial.h>
NeoSWSerial gpsPort(RX_PIN, TX_PIN); // RX_PIN, TX_PIN defined in ttnmapper-config.hpp

#include <TinyGPS++.h>
TinyGPSPlus gps;

//#define LAST_SENTENCE_IN_INTERVAL NMEAGPS::NMEA_GLL

// for c++-style cout and "<<" operator
#include <Streaming.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#ifdef _NWSKEY // #defined in ttnmapper-config.hpp
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM uint8_t NWKSKEY[16] = _NWSKEY;
#endif

#ifdef _APPSKEY // #defined in ttnmapper-config.hpp
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const uint8_t PROGMEM APPSKEY[16] = _APPSKEY;
#endif

#ifdef _DEVADDR // #defined in ttnmapper-config.hpp
// LoRaWAN end-device address (DevAddr)
static const uint32_t DEVADDR = _DEVADDR ; // <-- Change this address for every node!
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
		.nss = 10,
		.rxtx = LMIC_UNUSED_PIN,
		.rst = 5,
		.dio = {2, 3, LMIC_UNUSED_PIN},
};

static uint8_t TX_INTERVAL = 10;

static int32_t lastSpeed = 0;

static uint8_t output[52]; // 51 + \0

/*
char *ftoa(char *a, double f, int precision) {
	long p[] = {0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};

	char *ret = a;
	long heiltal = (long)f;
	itoa(heiltal, a, 10);
	while (*a != '\0') a++;
 *a++ = '.';
	long desimal = abs((long)((f - heiltal) * p[precision]));
	itoa(desimal, a, 10);

	return ret;
}
 */

/**
 *
 */
void do_send(osjob_t* j) {
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) {
		Serial << (F("OP_TXRXPEND, not sending")) << endl;
	} else {
		// Prepare upstream data transmission at the next possible time.

		char latitude[11] = {0};
		dtostrf(gps.location.lat(), 4, 5, latitude); // 4 Stellen inkl. Punkt, davon 4 hinter dem Punkt

		char longitude[11] = {0};
		dtostrf(gps.location.lng(), 5, 5, longitude); // 5 Stellen inkl. Punkt, davon 4 hinter dem Punkt

		char altitude[11] = {0};
		dtostrf(gps.altitude.meters(), 6, 1, altitude); // 6 Stellen inkl. Punkt, davon 1 hinter dem Punkt

		String alt_str = String(altitude);

		alt_str.trim();

		memset(altitude, 0 , 11);

		strncpy(altitude, alt_str.c_str(), 11);

		char hdop[6] = {0};
		dtostrf(gps.hdop.hdop(), 3, 1, hdop); // 3 Stellen inkl. Punkt, davon 1 hinter dem Punkt

		snprintf((char*)output, 51,
				"[%s,%s,%s,%s]",
				latitude, longitude, altitude, hdop);

		Serial << F("status: ") << gps.location.isValid() << endl;
		Serial << F("age   : ") << gps.location.age() << endl;
		Serial << F("date  : ") << gps.date.value() << endl;
		Serial << F("time  : ") << gps.time.value() << endl;
		Serial << F("lat   : ") << gps.location.lat() << endl;
		Serial << F("lon   : ") << gps.location.lng() << endl;
		Serial << F("alt   : ") << altitude << endl;
		Serial << F("speed : ") << gps.speed.kmph() << endl;
		Serial << F("-valid: ") << gps.speed.isValid() << endl;
		Serial << F("hdop  : ") << hdop << endl;

		Serial << F("                  1         2         3         4         5") << endl;
		Serial << F("         123456789012345678901234567890123456789012345678901") << endl;
		Serial << F("json  : [") << (char*)output << F("]") << endl;

		if (gps.speed.isValid()) {
			if (gps.speed.kmph() <= 10.0) {        // 10.000 m / 3.600 s == 10 m / 3.6 s = x / 60 s
				TX_INTERVAL = 60;                  // => 166.66 m
			} else if (gps.speed.kmph() <= 20.0) { // 20.000 m / 3.600 s == 20 m / 3.6 s = x / 30 s
				TX_INTERVAL = 30;                  // => 166.66 m
			} else if (gps.speed.kmph() <= 40.0) { // 40.000 m / 3.600 s == 20 m / 3.6 s = x / 15 s
				TX_INTERVAL = 15;                  // => 166.66 m
			} else {             //
				TX_INTERVAL = 10; //
			}
		} else {
			TX_INTERVAL = 30;
		}

		Serial << F("TX_INT: ") << TX_INTERVAL << endl;

		if (gps.location.isValid()) {
			LMIC_setTxData2(1, output, (size_t)strlen((char*)output), false);

			Serial << (F("Packet queued")) << endl;
		} else {
			// Schedule the next transmission
			os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

			Serial << F("rescheduled TX in: [") << TX_INTERVAL << F("]") << endl;

//			LMIC_sendAlive();

//			Serial << F("empty TX queued") << endl;
		}
	}

	// Next TX is scheduled after TX_COMPLETE event.
}

/**
 *
 */
static void GPSloop(void) {
	while (gpsPort.available() > 0) {
		gps.encode(gpsPort.read());
	}

	if (gps.speed.isValid() && ((abs(gps.speed.value() - lastSpeed) > 500))) {
		Serial << F("scheduling out of bands TX - speed [") << gps.speed.value() << F("] lastSpeed [") << lastSpeed << F("]") << endl;
		os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
	}

	lastSpeed = gps.speed.value();
}

/**
 *
 */
void onEvent(ev_t ev) {
	Serial << (os_getTime()) << F(": ");

	switch (ev) {

/*
	case EV_SCAN_TIMEOUT:
		Serial << (F("EV_SCAN_TIMEOUT")) << endl;
		break;
*/
/*
	case EV_BEACON_FOUND:
		Serial << (F("EV_BEACON_FOUND")) << endl;
		break;
*/
/*
	case EV_BEACON_MISSED:
		Serial << (F("EV_BEACON_MISSED")) << endl;
		break;
*/
/*
	case EV_BEACON_TRACKED:
		Serial << (F("EV_BEACON_TRACKED")) << endl;
		break;
*/
/*
	case EV_JOINING:
		Serial << (F("EV_JOINING")) << endl;
		break;
*/
/*
	case EV_JOINED:
		Serial << (F("EV_JOINED")) << endl;
		break;
*/
/*
	case EV_RFU1:
		Serial << (F("EV_RFU1")) << endl;
		break;
*/
/*
	case EV_JOIN_FAILED:
		Serial << (F("EV_JOIN_FAILED")) << endl;
		break;
*/
/*
	case EV_REJOIN_FAILED:
		Serial << (F("EV_REJOIN_FAILED")) << endl;
		break;
*/

	case EV_TXCOMPLETE:
		Serial << (F("EV_TXCOMPLETE (includes waiting for RX windows)")) << endl;

		Serial << F("TX-Channel: [") << LMIC.txChnl << F("]") << endl;
		Serial << F("TX-Count  : [") << LMIC.txCnt << F("]") << endl;

		if (LMIC.txrxFlags & TXRX_ACK) {
			Serial << (F("Received ack")) << endl;
		}

		if ((LMIC.dataLen != 0) || (LMIC.dataBeg != 0)) {
			// Serial << (F("#############################################")) << endl;
			Serial << (F("Received [")) << LMIC.dataLen << (F("] bytes of payload")) << endl;
			// Serial << (F("#############################################")) << endl;
		}

		// Schedule the next transmission
		os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

		break;

	case EV_LOST_TSYNC:
		Serial << (F("EV_LOST_TSYNC")) << endl;
		break;

	case EV_RESET:
		Serial << (F("EV_RESET")) << endl;
		break;

	case EV_RXCOMPLETE:
		// data received in ping slot
		Serial << (F("EV_RXCOMPLETE")) << endl;

		if ((LMIC.dataLen != 0) || (LMIC.dataBeg != 0)) {
			// Serial << (F("#############################################")) << endl;
			Serial << (F("Received [")) << LMIC.dataLen << (F("] bytes of payload")) << endl;
			//Serial << (F("#############################################")) << endl;
		}

		break;

	case EV_LINK_DEAD:
		Serial << (F("EV_LINK_DEAD")) << endl;
		break;

	case EV_LINK_ALIVE:
		Serial.println(F("EV_LINK_ALIVE"));
		break;
/*
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
*/
		/*
    case EV_RXSTART:
      Serial.println(F("EV_RXSTART"));
      break;
*/
		/*
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE"));
      break;
		 */

	default:
		//Serial << (F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")) << endl;
		Serial << (F("Unknown event: [")) << ev << F("]") << endl;
		//Serial << (F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")) << endl;

		break;
	}
}

/**
 *
 */
void setup(void) {
	Serial.begin(9600);

	Serial << F("ttnmapper") << endl;
	//	Serial << F("looking for gps on ");
	//	Serial <<  (GPS_PORT_NAME) << endl;

	gpsPort.begin(9600);

	/*
	  const char *msg = "PUBX,40,GSV,0,1,0,0";

	  // find checksum
	  int checksum = 0;
	  for (int i = 0; msg[i]; i++)
	      checksum ^= (unsigned char)msg[i];

	  // convert and create checksum HEX string
	  char checkTmp[8];
	  snprintf(checkTmp, sizeof(checkTmp)-1, "*%.2X", checksum);

	  // send to module
	  gpsPort.write("$");
	  gpsPort.write(msg);
	  gpsPort.write(checkTmp);
	 */
	// LMIC init
	os_init();

	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

#ifdef PROGMEM
	// On AVR, these values are stored in flash and only copied to RAM
	// once. Copy them to a temporary buffer here, LMIC_setSession will
	// copy them into a buffer of its own again.
	uint8_t appskey[sizeof(APPSKEY)];
	uint8_t nwkskey[sizeof(NWKSKEY)];

	memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
	memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));

	LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
#else
	// If not running an AVR with PROGMEM, just use the arrays directly
	LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

	// Set up the channels used by the Things Network, which corresponds
	// to the defaults of most gateways. Without this, only three base
	// channels from the LoRaWAN specification are used, which certainly
	// works, so it is good for debugging, but can overload those
	// frequencies, so be sure to configure the full frequency range of
	// your network here (unless your network autoconfigures them).
	// Setting up channels should happen after LMIC_setSession, as that
	// configures the minimal channel set.
	// NA-US channels 0-71 are configured automatically
	LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
	LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
	// TTN defines an additional channel at 869.525Mhz using SF9 for class B
	// devices' ping slots. LMIC does not have an easy way to define set this
	// frequency and support for class B is spotty and untested, so this
	// frequency is not configured here.

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF7, 14);

	// Use with Arduino Pro Mini ATmega328P 3.3V 8 MHz
	// Let LMIC compensate for +/- 1% clock error
	LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

	// Start job (sending automatically starts OTAA too)
	do_send(&sendjob);
}

/**
 *
 */
void loop(void) {
	GPSloop();

	os_runloop_once();
}
