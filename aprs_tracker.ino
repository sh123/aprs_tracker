//#define SERIAL_TX_BUFFER_SIZE 64
//#define SERIAL_RX_BUFFER_SIZE 64
#include <SPI.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <Rotary.h>

#include <LibAPRS.h>
#include <KISS.h>

#define ADAFRUIT_NO_BUTTON
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define _GPS_NO_STATS
#include <TinyGPS.h>

// use flags
#define USE_SCREEN
#define USE_RECEIVE
#define USE_RECEIVE_INFO
#define USE_GPS
#define USE_PRECISE_DISTANCE
#define USE_RX_ON_CALLBACK
#define USE_COMPRESSED_POSITION
//#define USE_SMART_BEACONING
//#define USE_SERIAL
//#define USE_KISS

#ifdef USE_KISS
  #undef USE_SCREEN
  #define USE_SERIAL
#endif

// common
#define ADC_REFERENCE REF_5V // _3V3
#define OPEN_SQUELCH false

#define GPS_POLL_DURATION_MS 1000 // for how long to poll gps
#define UPDATE_HEURISTICS_MAP_SIZE 9
#define TIMER_DISABLED -1

#define TX_PREAMBLE 350
#define TX_TAIL 50

// screen
#define SCREEN_TIMEOUT 30000UL // 30 seconds
#define SCREEN_CONTRAST 40
#define SCREEN_UPDATE_PERIOD 3000  // fow how often to update screen
#define GPS_UPDATE_PERIOD 3000  // fow how often to update screen

// aprs symbols
#define APRS_SYM_PHONE '$'
#define APRS_SYM_CAR '>'
#define APRS_SYM_BIKE 'b'
#define APRS_SYM_JEEP 'j'
#define APRS_SYM_JOGGER '['

// aprs ssids
#define APRS_SSID_GENERIC_2 2
#define APRS_SSID_GENERIC_3 3
#define APRS_SSID_WALKIE_TALKIE 7
#define APRS_SSID_PRIMARY_MOBILE 9
#define APRS_SSID_ADDITIONAL 15

// aprs config
#define APRS_MY_CALLSIGN "NOCALL"

// smart beaconing parameters
#define APRS_SB_LOW_SPEED 5
#define APRS_SB_HIGH_SPEED 100
#define APRS_SB_SLOW_RATE 1200
#define APRS_SB_FAST_RATE 60
#define APRS_SB_TURN_MIN 15
#define APRS_SB_TURN_TH 10
#define APRS_SB_TURN_SLOPE 240

// eeprom storage
#define EEPROM_ACTIVE_HEURISTIC_IDX 0
#define EEPROM_ACTIVE_SYMBOL_IDX 1

// pins
#define PIN_SCREEN_LIGHT  8
#define PIN_GPS_RX 2
#define PIN_GPS_TX 15
#define PIN_ROT_CLK 17
#define PIN_ROT_DT 18
#define PIN_ROT_BTN 19

// Screen state
enum t_screen_state {
  S_STATUS = 0,
  S_CONFIG_HEURISTICS,
  S_CONFIG_SYMBOL,
  S_CONFIG_SSID,
  S_RECEIVE
};

// APRS update heuristics
enum t_update_heuristics {
  H_OFF = 0,
  H_PERIODIC_1MIN,
  H_PERIODIC_5MIN,
  H_PERIODIC_10MIN,
  H_PERIODIC_30MIN,
  H_RANGE_500M,
  H_RANGE_1KM,
  H_RANGE_5KM,
  H_SMART_BEACONING,
  H_MAX_HEURISTICS = H_SMART_BEACONING 
};

// heuristic labels
const char label_x[] PROGMEM = "OFF";
const char label_t1[] PROGMEM = "T01";
const char label_t5[] PROGMEM = "T05";
const char label_t10[] PROGMEM = "T10";
const char label_t30[] PROGMEM = "T30";
const char label_r05[] PROGMEM = "R00";
const char label_r1[] PROGMEM = "R01";
const char label_r5[] PROGMEM = "R05";
const char label_b[] PROGMEM = "BKN";

// heuristic to string map
const PROGMEM char  * const update_heuristics_map[UPDATE_HEURISTICS_MAP_SIZE] PROGMEM = {
  label_x, label_t1, label_t5, label_t10, label_t30, 
  label_r05, label_r1, label_r5, label_b
};

// peripherals
SimpleTimer timer;

#ifdef USE_GPS
TinyGPS gps;
SoftwareSerial serial_gps(PIN_GPS_RX, PIN_GPS_TX);
#endif

Rotary rotary = Rotary(PIN_ROT_CLK, PIN_ROT_DT, PIN_ROT_BTN);

#ifdef USE_SCREEN
Adafruit_PCD8544 display = Adafruit_PCD8544(13, 11, 12, 10, 9);
#endif

// variables, program state
extern Afsk modem;
extern AX25Ctx AX25;
bool send_aprs_update = false;
long lat = 0;
long lon = 0;
long lat_prev = 0;
long lon_prev = 0;
long distance = 0;
//long course_prev = 0;
char cur_symbol;

// counters
unsigned long age = 0;
unsigned char cnt_tx = 0;
unsigned char cnt_rx = 0;
unsigned char cnt = 0;

// timers
char aprs_update_timer_idx = TIMER_DISABLED;
char screen_lights_off_timer_idx = TIMER_DISABLED;
char screen_update_timer_idx = TIMER_DISABLED;
char gps_update_timer_idx = TIMER_DISABLED;

// heuristics
char active_heuristic = H_OFF;
char selected_heuristic = H_OFF;

// buffer for conversions
#define CONV_BUF_SIZE 15
static char conv_buf[CONV_BUF_SIZE];

// incoming packet
#ifdef USE_RECEIVE
unsigned short incoming_pkt_length = 0;
#endif

/*
**  Initialization, serial, gps, screen, aprs
*/
void setup() {

  // serial
#ifdef USE_SERIAL
  Serial.begin(9600);
#endif

  // screen
#ifdef USE_SCREEN
  display.begin();
  display.setContrast(SCREEN_CONTRAST);
  pinMode(PIN_SCREEN_LIGHT, OUTPUT); 
  screenOn();
#endif

  // aprs
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
#ifdef USE_KISS
  kiss_init(&AX25, &modem, &Serial, 0);
#else
  // hardcoded in libaprs to save memory
  //APRS_setCallsign(APRS_MY_CALLSIGN, APRS_MY_SSID);
  //APRS_setDestination("APZMDM", 0);
  //APRS_setPath1("WIDE1", 1);
  //APRS_setPath2("WIDE2", 2);
  APRS_setPreamble(TX_PREAMBLE);
  APRS_setTail(TX_TAIL);
  APRS_useAlternateSymbolTable(false);
  //APRS_setSymbol(APRS_MY_SYMBOL);
  //APRS_printSettings();
  setSymbol(EEPROM[EEPROM_ACTIVE_SYMBOL_IDX]);

  // timers
  screen_update_timer_idx = timer.setInterval(SCREEN_UPDATE_PERIOD, updateScreen);
  gps_update_timer_idx = timer.setInterval(GPS_UPDATE_PERIOD, updateGpsData);
#if defined(USE_RECEIVE) && !defined(USE_RX_ON_CALLBACK)
  timer.setInterval(500, processPacket);
#endif

  // heuristics, load from eeprom
  activateAprsUpdateHeuristic(EEPROM[EEPROM_ACTIVE_HEURISTIC_IDX] > H_MAX_HEURISTICS ? H_PERIODIC_1MIN : EEPROM[EEPROM_ACTIVE_HEURISTIC_IDX]);

  updateScreenAndGps();
  screen_lights_off_timer_idx = timer.setTimeout(SCREEN_TIMEOUT, screenOff);
#endif
}

/*
**  Switch off the screen
*/
void screenOff() {
  digitalWrite(PIN_SCREEN_LIGHT, LOW);
  screen_lights_off_timer_idx = TIMER_DISABLED;
}

/*
**  Select aprs symbol and corresponding ssid
*/
void setSymbol(char sym) {
  char ssid = 0;
  switch (sym) {
    case APRS_SYM_JOGGER:
      ssid = APRS_SSID_WALKIE_TALKIE;
      break;
    case APRS_SYM_CAR:
      ssid = APRS_SSID_PRIMARY_MOBILE;
      break;
    case APRS_SYM_BIKE:
      ssid = APRS_SSID_GENERIC_3;
      break;
    default:
      ssid = APRS_SYM_JOGGER;
      sym = APRS_SSID_WALKIE_TALKIE;
      break;
  }
  if (ssid != 0) {
    APRS_setCallsign(APRS_MY_CALLSIGN, ssid);
    APRS_setSymbol(sym);
    cur_symbol = sym;
    EEPROM[EEPROM_ACTIVE_SYMBOL_IDX] = sym;
  }
}

/*
**  Switch on the screen
*/
void screenOn() {
  if (isScreenOn()) {
    if (screen_lights_off_timer_idx != TIMER_DISABLED)
      timer.restartTimer(screen_lights_off_timer_idx);
  } else {
    digitalWrite(PIN_SCREEN_LIGHT, HIGH);
    screen_lights_off_timer_idx = timer.setTimeout(SCREEN_TIMEOUT, screenOff);
  }
}

/*
**  Return true if screen is on
*/
bool isScreenOn() {
  return digitalRead(PIN_SCREEN_LIGHT);
}

/*
**  Delete active aprs update timer if it is already active
*/
void deleteActiveAprsUpdateTimer() {
  if (aprs_update_timer_idx != TIMER_DISABLED) {
    timer.deleteTimer(aprs_update_timer_idx);
    aprs_update_timer_idx = TIMER_DISABLED;
  }
}

/*
**  Activate given APRS update heuristic
*/
void activateAprsUpdateHeuristic(char heuristic) {
  
  unsigned long timeout = 0L;
  switch (heuristic) {
    case H_OFF:
      deleteActiveAprsUpdateTimer();
      break;
    case H_PERIODIC_1MIN:
      timeout = 1L*60L*1000L;
      break;
    case H_PERIODIC_5MIN:
      timeout = 5L*60L*1000L;
      break;
    case H_PERIODIC_10MIN:
      timeout = 10L*60L*1000L;
      break;
    case H_PERIODIC_30MIN:
      timeout = 30L*60L*1000L;
      break;
    case H_RANGE_500M:
    case H_RANGE_1KM:
    case H_RANGE_5KM:
      distance = 0;
      timeout = 30L*60L*1000L;
      break;
    case H_SMART_BEACONING:
      break;
    default:
      break;
  }
  deleteActiveAprsUpdateTimer();
  if (timeout != 0) {
    aprs_update_timer_idx = timer.setInterval(timeout, setAprsUpdateFlag);
  }
  active_heuristic = heuristic;
  selected_heuristic = heuristic;
  EEPROM[EEPROM_ACTIVE_HEURISTIC_IDX] = selected_heuristic;
}

/*
**  Sets flag, which will trigger aprs packet sending
*/
void setAprsUpdateFlag() {
  send_aprs_update = true;
}

/*
**  Process SmartBeaconing heuristic
*/
void heuristicProcessSmartBeaconing() {
#ifdef USE_SMART_BEACONING

// https://github.com/ge0rg/aprsdroid/blob/master/src/location/SmartBeaconing.scala

float speed = gps.f_speed_kmph();
float course_prev = gps.course();
float heading_change_since_beacon = abs(course_prev - gps.course());
long beacon_rate, secs_since_beacon, turn_time;

  if (speed < APRS_SB_LOW_SPEED)
  {
    beacon_rate = APRS_SB_SLOW_RATE;
  }
  else
  {
    // Adjust beacon rate according to speed
    if (speed > APRS_SB_HIGH_SPEED)
    {
      beacon_rate = APRS_SB_FAST_RATE;
    }
    else
    {
      beacon_rate = APRS_SB_FAST_RATE * APRS_SB_HIGH_SPEED / speed;
    }

    // Corner pegging - ALWAYS occurs if not "stopped"
    // Note turn threshold is speed-dependent

    // what is mph?  
    float turn_threshold = APRS_SB_TURN_MIN + APRS_SB_TURN_SLOPE / (speed * 2.23693629);

    if ((heading_change_since_beacon > APRS_SB_TURN_TH) && (secs_since_beacon > turn_time))
    {
      secs_since_beacon = beacon_rate;
    }
  }

  if (secs_since_beacon > beacon_rate) {
    send_aprs_update = true;
  }
#endif
}

/*
**  Process heuristics when distance changed
*/
void heuristicDistanceChanged() {
  bool needs_update = false;
  switch (active_heuristic) {
    case H_RANGE_500M:
      if (distance > 500) {
        needs_update = true;
      }
      break;
    case H_RANGE_1KM:
      if (distance > 1000) {
        needs_update = true;
      }
      break;
    case H_RANGE_5KM:
      if (distance > 5000) {
        needs_update = true;
      }
      break;
    default:
      heuristicProcessSmartBeaconing();
      break;
  }
  if (needs_update) {
    send_aprs_update = true;
#ifndef USE_PRECISE_DISTANCE
    lon_prev = lon;
    lat_prev = lat;
#endif
    distance = 0;
  }
}

/*
**  Update distance and heuristics
*/
void updateDistance() {
  if (lon_prev != 0 && lat_prev != 0 && lon != 0 && lat != 0) {
#ifdef USE_GPS
    distance += gps.distance_between2(lon_prev, lat_prev, lon, lat);
#endif
    heuristicDistanceChanged();
  }
#ifndef USE_PRECISE_DISTANCE
  else {
#endif
    lon_prev = lon;
    lat_prev = lat;
#ifndef USE_PRECISE_DISTANCE
  }
#endif
}

/*
**  Message callback, called on new message from APRS
*/
void aprs_msg_callback(struct AX25Ctx *ctx) {
  cnt_rx++;
#ifdef USE_KISS
  kiss_messageCallback(ctx);
#endif
#ifdef USE_RECEIVE
  incoming_pkt_length = ctx->frame_len;
#ifdef USE_RX_ON_CALLBACK
  processPacket();
  incoming_pkt_length = 0;
#endif
#endif
}

/*
**  Print string to display from program memory
*/
void displayPrintPgm(PGM_P str) {
#ifdef USE_SCREEN
  for (char c; (c = pgm_read_byte(str)); str++) display.print(c);
#endif
}

/*
**  Periodic screen update and gps data reading
*/
void updateScreenAndGps() {
  updateGpsData();
  updateScreen();
}

/*
**  Update screen
*/
void updateScreen() {
#ifdef USE_SCREEN
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);

  // active mode / selected mode / free memory
  displayPrintPgm((char*)pgm_read_word(&(update_heuristics_map[active_heuristic]))); 
    display.print(F("/"));
    displayPrintPgm((char*)pgm_read_word(&(update_heuristics_map[selected_heuristic]))); 
    display.print(F("/")); 
    display.print(freeMemory());
    display.print(F("/")); 
    display.println((char)cur_symbol);

  // count updates / count tx / count rx / satellites / age
  display.print(cnt); 
    display.print(F("/"));
    display.print(cnt_tx); 
    display.print(F("tx"));
    display.print(F("/")); 
    display.print(cnt_rx); 
    display.println(F("rx"));
#ifdef USE_GPS
  // coordinates
  display.print(deg_to_nmea(lat, true));
    display.print(F("/"));
    display.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    display.println((char)233);

  display.print(deg_to_nmea(lon, false));
    display.print(F("/"));
    display.println(age);

  // qth 
  display.print(deg_to_qth(lat, lon));
    display.print(F("/"));
    display.print(gps.course() == TinyGPS::GPS_INVALID_ANGLE ? 0 : gps.course() / 100);
    display.println("'");

  // altitude / satellites / age
  display.print(gps.altitude() == TinyGPS::GPS_INVALID_ALTITUDE ? 0 : gps.altitude() / 100); 
    display.print(F("m"));
    display.print(F("/"));
    display.print(gps.speed() == TinyGPS::GPS_INVALID_SPEED ? 0 : gps.speed() / 200); 
    display.print(F("/"));
    display.print(distance);
#endif
  display.display();
#endif
  cnt++;
}

/*
**  Convert degrees in long format to NMEA string format
**  DDMM.hhN for latitude and DDDMM.hhW for longitude
*/
char* deg_to_nmea(long deg, boolean is_lat) {
  unsigned long b = (deg % 1000000UL) * 60UL;
  unsigned long a = (deg / 1000000UL) * 100UL + b / 1000000UL;
  b = (b % 1000000UL) / 10000UL;
  // DDMM.hhN/DDDMM.hhW
  // http://www.aprs.net/vm/DOS/PROTOCOL.HTM
  conv_buf[0] = '0';
  snprintf(conv_buf + 1, 5, "%04u", a);
  conv_buf[5] = '.';
  snprintf(conv_buf + 6, 3, "%02u", b);
  conv_buf[9] = '\0';
  if (is_lat) {
    conv_buf[8] = deg > 0 ? 'N': 'S';
    return conv_buf + 1;
  } else {
     conv_buf[8] = deg > 0 ? 'E': 'W';
     return conv_buf;
  }
}

/*
**  Convert degrees in long format to APRS compressed format
**  http://www.aprs.org/doc/APRS101.PDF page 36
*/
char* deg_to_compressed(long deg, boolean is_lat) {
  conv_buf[0] = '\0';
}

/*
**  Convert latidude and longitude to QTH maidenhead locator
*/
char *deg_to_qth(long lat, long lon) {
  // https://en.wikipedia.org/wiki/Maidenhead_Locator_System
  float lat_ = ((float)lat) / 1000000.0 + 90.0;
  float lon_ = ((float)lon) / 1000000.0 + 180.0;
  conv_buf[0] = 'A' + int(lon_ / 20.0);
  conv_buf[1] = 'A' + int(lat_ / 10.0);
  conv_buf[2] = '0' + int(fmod(lon_, 20) / 2.0);
  conv_buf[3] = '0' + int(fmod(lat_, 10) / 1.0);
  conv_buf[4] = 'A' + int((lon_- (int(lon_ / 2.0) * 2)) / (5.0/60.0));
  conv_buf[5] = 'A' + int((lat_ - (int(lat_ / 1.0) * 1)) / (2.5/60.0));
  conv_buf[6] = '\0';
  return conv_buf;
}

/*
**  Returns a comment string with encoded angle, speed, altitude and comment text itself
*/
char *get_comment() {
  // CRS/SPD/A=ALTITD <comment>
  // 208/007/A=000045 <comment>
#ifdef USE_GPS
  sprintf(conv_buf + 0, "%03u", gps.course() == TinyGPS::GPS_INVALID_ANGLE ? 0 : gps.course() / 100);
  conv_buf[3] = '/';
  // speed is in knots
  sprintf(conv_buf + 4, "%03u", gps.speed() == TinyGPS::GPS_INVALID_SPEED ? 0 : gps.speed() / 100);
  conv_buf[7] = '/';
  conv_buf[8] = 'A';
  conv_buf[9] = '=';
  // altitude is in feet, feet = cm / 30.48
  sprintf(conv_buf + 10, "%06u", gps.altitude() == TinyGPS::GPS_INVALID_ALTITUDE || gps.altitude() < 0 ? 0 : gps.altitude() / 30);
  conv_buf[16] = '\0';
#else
  conv_buf[0] = ' ';
  conv_buf[1] = '.';
  conv_buf[2] = '\0';
#endif
  return conv_buf;
}

/*
**  Poll for data from GPS module
*/
void updateGpsData() {
#ifdef USE_GPS
  serial_gps.begin(9600);

  for (unsigned long start = millis(); millis() - start < GPS_POLL_DURATION_MS;)
  {
    while (serial_gps.available())
    {
      char c = serial_gps.read();
      if (gps.encode(c)) {
        gps.get_position(&lat, &lon, &age);
        updateDistance();
        serial_gps.end();
      }
    }
  }
  serial_gps.end();
#endif
}

/*
**  Send APRS location update
*/
bool sendAprsLocationUpdate() {
#ifdef USE_GPS
  if (lat != 0 && lon != 0)
  {
#endif
    cnt_tx++;
#ifdef USE_COMPRESSED_POSITION
    APRS_setLat((char*)deg_to_compressed(lat, true));
    APRS_setLon((char*)deg_to_compressed(lon, false));
#else
    APRS_setLat((char*)deg_to_nmea(lat, true));
    APRS_setLon((char*)deg_to_nmea(lon, false));
#endif
    char * comment = "";  // get_comment()
    APRS_sendLoc(comment, strlen(comment));
    return true;
#ifdef USE_GPS
  }
  return false;
#endif
}

/*
**  Process incoming packet
*/
#ifdef USE_RECEIVE
void processPacket() {
  if (incoming_pkt_length != 0) {
#ifdef USE_SCREEN
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0, 0);
    display.print(cnt_rx);
    display.print(' ');

    uint8_t *buf = (uint8_t*)AX25.buf;

    DECODE_CALL2(buf, conv_buf);
    conv_buf[6] = 0;
    display.print(conv_buf);
    uint8_t ssid = (uint8_t)(*buf++ >> 1) & 0x0F;
    if (ssid > 0) {
      display.print('-');
      display.print((int)ssid);
    }
    display.print(' ');

    DECODE_CALL2(buf, conv_buf);
    conv_buf[6] = 0;
    display.print(conv_buf);
    ssid = (uint8_t)(*buf >> 1) & 0x0F;
    if (ssid > 0) {
      display.print('-');
      display.print((int)ssid);
    }
    display.print(' ');

    for (char rpt_count = 0; !(*buf++ & 0x01) && (rpt_count < AX25_MAX_RPT); rpt_count++) {
      DECODE_CALL2(buf, conv_buf);
      conv_buf[6] = 0;
      display.print(conv_buf);
      display.print('-');
      display.print((uint8_t)(*buf >> 1) & 0x0F);
      display.print(' ');
     }

    buf += 2;
    for (int i = 0; i < incoming_pkt_length - 2 - (buf - AX25.buf); i++) {
      uint8_t c = buf[i];
      if (isprint((char)c)) {
        display.print((char)c);
      } else {
        display.print(' ');
      }
    }
    display.display();
#endif
    incoming_pkt_length = 0;
    timer.disable(screen_update_timer_idx);
  }
}
#endif

/*
**  Rotary is rotated left
*/
void onRotaryLeft() {
  if (--selected_heuristic < 0) {
    selected_heuristic = UPDATE_HEURISTICS_MAP_SIZE - 1;
  }
}

/*
**  Rotary is rotated right
*/
void onRotaryRight() {
  if (++selected_heuristic >= UPDATE_HEURISTICS_MAP_SIZE) {
    selected_heuristic = 0;
  }
}

/*
**  Short press button release
*/
void onBtnReleased() {
  if (!timer.isEnabled(screen_update_timer_idx)) {
      timer.enable(screen_update_timer_idx);
  }
  else if (isScreenOn()) {
    // activate selected heuristic
    if (selected_heuristic == active_heuristic) {
      selectNextSymbol();
    } else { 
      activateAprsUpdateHeuristic(selected_heuristic);
    }
  // switch on the screen
  } else {
    screenOn();
  }
}

/*
**  Iterate to next selection for the symbol + ssid
*/
void selectNextSymbol() {
  switch (cur_symbol) {
    case APRS_SYM_PHONE:
      setSymbol(APRS_SYM_CAR);
      break;
    case APRS_SYM_CAR:
      setSymbol(APRS_SYM_BIKE);
      break;
    case APRS_SYM_BIKE:
      setSymbol(APRS_SYM_JOGGER);
      break;
    default:
      setSymbol(APRS_SYM_CAR);
      break;
  }
}

/*
**  Long button press
*/
void onLongBtnPress() {
  screenOn();
  send_aprs_update = true;
}

/*
**  Process rotation and button press-release
*/
bool processRotary() {

  bool update_screen = false;
  unsigned char rotary_state = rotary.process();
  unsigned char rotary_btn_state = rotary.process_button();

  // rotation right-left
  if (rotary_state) {
     screenOn();
     if (rotary_state == DIR_CW) {
       onRotaryLeft();
     } else {
       onRotaryRight();
     }
    update_screen = true;
  }

  // button state
  switch (rotary_btn_state) {
  case BTN_NONE:
    break;
  case BTN_PRESSED:
    break;
  case BTN_RELEASED:
    onBtnReleased();
    update_screen = true;
    break;
  case BTN_PRESSED_LONG:
    onLongBtnPress();
    update_screen = true;
    break;
  case BTN_RELEASED_LONG:
    break;
  default:
    break;
  }
  return update_screen;
}

/*
**  Message loop
*/
void loop() {
#ifdef USE_KISS
  char incomingByte;
  while (Serial.available() > 0) {
    incomingByte = Serial.read();
    kiss_serialCallback(incomingByte);
  }
#else
  bool update_screen = processRotary();
  if (send_aprs_update) {
    update_screen |= sendAprsLocationUpdate();
    send_aprs_update = false;
  }
  if (update_screen && timer.isEnabled(screen_update_timer_idx)) {
    updateScreen();
  }
  timer.run();
#endif
}
