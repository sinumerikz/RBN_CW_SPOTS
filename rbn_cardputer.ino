/*
  RBN Checker for M5Cardputer
  ============================
  - On first boot (or whenever no saved Wi-Fi credentials exist): the
    device scans for nearby Wi-Fi networks, you pick one and enter the
    password. On success, SSID + password are stored in flash (NVS, via
    the Preferences library) - next boot it reconnects automatically.
  - Optional: set up a QRZ.com login (username + password of a QRZ XML
    subscription) in Settings. If configured, the app looks up the real
    coordinates of your callsign and of every spotting station via the
    QRZ XML Data API and shows an accurate min/max range in km. If NO
    QRZ login is configured, the range display is simply left out
    entirely (no rough estimate is shown).
  - Settings menu (Wi-Fi / QRZ login) is reachable any time from the
    callsign screen by holding G0 for 1.5s.
  - Enter your callsign -> live connection to
    telnet.reversebeacon.net:7000 (the official RBN telnet feed for
    CW/RTTY spots). NOTE: this is a LIVE feed, not a lookup of past
    spots! After entering your callsign you (or another station) must
    actually call CQ so an RBN skimmer can hear and spot you.
  - The live screen updates instantly whenever a new spot for your
    callsign comes in: it shows the 5 most recent spotting stations
    (callsign, band, SNR) and the running spot count. A short beep
    plays on every new spot.
  - Press G0 to stop; there's also a generous safety timeout so the
    connection doesn't stay open forever if you forget about it.

  Required libraries (Arduino IDE Library/Board manager):
    - M5Cardputer (m5stack/M5Cardputer)
    - WiFi, WiFiClientSecure, HTTPClient and Preferences (all included
      in the ESP32 Arduino core, nothing extra to install)

  Note on QRZ:
    The QRZ XML Data service (paid subscription) is what actually
    returns a station's coordinates. It authenticates with your QRZ
    username + password (not a fixed API key) and hands back a
    temporary session key, which this sketch reuses until it expires.
    Lookups are cached in memory for the running session so the same
    spotting station isn't queried twice. TLS certificate checking is
    disabled (WiFiClientSecure::setInsecure()) to keep things simple on
    the ESP32 - fine for a hobby project, just be aware of that
    trade-off.
*/

#include <M5Cardputer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <SPI.h>
#include <SD.h>

// microSD SPI pins for the M5Cardputer (per official M5Stack docs)
#define SD_SPI_SCK_PIN  40
#define SD_SPI_MISO_PIN 39
#define SD_SPI_MOSI_PIN 14
#define SD_SPI_CS_PIN   12

// Declared this early on purpose: the Arduino IDE auto-generates
// function prototypes near the top of the file, before it has seen
// custom types defined further down. QrzResult must exist before that
// point or those prototypes fail to compile.
enum QrzResult { QRZ_OK, QRZ_NOT_FOUND, QRZ_TEMP_ERROR };

// ---------------------------------------------------------------------
// CONFIGURATION
// ---------------------------------------------------------------------
const char* RBN_HOST = "telnet.reversebeacon.net";
const uint16_t RBN_PORT = 7000;            // 7000 = CW/RTTY, 7001 = FT8
const uint32_t MAX_RUNTIME_SECONDS = 1800; // safety timeout (30 min) in
                                            // case you forget to stop;
                                            // normally you stop via G0.
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
const int MAX_NETWORKS_SHOWN = 6;
const uint32_t G0_HOLD_MS = 1500;          // hold G0 = open settings menu
const int RECENT_COUNT = 5;                // how many recent spotters to keep
const uint16_t BEEP_FREQ_HZ = 1500;
const uint32_t BEEP_MS = 150;
const uint32_t QRZ_SESSION_MAX_AGE_MS = 50UL * 60UL * 1000UL; // refresh after ~50 min
const int LOC_CACHE_SIZE = 20;             // cached QRZ lookups per session

Preferences prefs;

// ---------------------------------------------------------------------
// Band table (kHz)
// ---------------------------------------------------------------------
struct BandDef {
  uint32_t fmin, fmax;
  const char* name;
};

const BandDef BANDS[] = {
  {1800,   2000,   "160m"},
  {3500,   4000,   "80m"},
  {5330,   5410,   "60m"},
  {7000,   7300,   "40m"},
  {10100,  10150,  "30m"},
  {14000,  14350,  "20m"},
  {18068,  18168,  "17m"},
  {21000,  21450,  "15m"},
  {24890,  24990,  "12m"},
  {28000,  29700,  "10m"},
  {50000,  54000,  "6m"},
};
const int NUM_BANDS = sizeof(BANDS) / sizeof(BANDS[0]);

int bandIndexForFreq(float freqKHz) {
  for (int i = 0; i < NUM_BANDS; i++) {
    if (freqKHz >= BANDS[i].fmin && freqKHz <= BANDS[i].fmax) return i;
  }
  return -1;
}

// Great-circle distance in km
float distanceKm(float lat1, float lon1, float lat2, float lon2) {
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sinf(dLat / 2) * sinf(dLat / 2) +
            cosf(radians(lat1)) * cosf(radians(lat2)) *
            sinf(dLon / 2) * sinf(dLon / 2);
  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return 6371.0f * c;
}

// ---------------------------------------------------------------------
// Small string helpers (URL-encoding, XML tag extraction)
// ---------------------------------------------------------------------
String urlEncode(const String &s) {
  String out = "";
  char buf[4];
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += c;
    } else {
      snprintf(buf, sizeof(buf), "%%%02X", (unsigned char)c);
      out += buf;
    }
  }
  return out;
}

String extractXmlTag(const String &xml, const char* tag) {
  String openTag = String("<") + tag + ">";
  String closeTag = String("</") + tag + ">";
  int start = xml.indexOf(openTag);
  if (start < 0) return "";
  start += openTag.length();
  int end = xml.indexOf(closeTag, start);
  if (end < 0) return "";
  return xml.substring(start, end);
}

// ---------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------
enum AppState {
  ST_WIFI_SCANNING, ST_WIFI_LIST, ST_WIFI_SSID_INPUT, ST_WIFI_PASS_INPUT,
  ST_WIFI_CONNECTING, ST_WIFI_ERROR,
  ST_SETTINGS_MENU, ST_QRZ_USER_INPUT, ST_QRZ_PASS_INPUT, ST_HIGHSCORE_VIEW,
  ST_QRZ_TESTING, ST_QRZ_TEST_RESULT,
  ST_INPUT, ST_RBN_CONNECTING, ST_LISTENING, ST_RESULT, ST_RBN_ERROR, ST_MAP_VIEW
};
AppState state = ST_WIFI_SCANNING;

String myCall = "";
String statusMsg = "";
float myLat = 0, myLon = 0;
bool myLocKnown = false;

WiFiClient client;
uint32_t listenStartMs = 0;
char lineBuf[160];
uint8_t lineLen = 0;

int spotCount = 0;
int bandCounts[NUM_BANDS];
float minDist = 1e9f, maxDist = -1.0f;
bool anyDistKnown = false;

// -- last few received spots, most recent first, for the live list --
struct SpotEntry {
  String spotter;
  int bandIdx;
  int snr;
};
SpotEntry recentSpots[RECENT_COUNT];
int recentSpotCount = 0;
bool displayDirty = true; // true = live screen needs an immediate redraw

// -- Wi-Fi management --
struct WifiNet {
  String ssid;
  int32_t rssi;
  bool secure;
};
WifiNet networks[MAX_NETWORKS_SHOWN];
int networkCount = 0;
String textInputBuffer = ""; // reused for SSID / Wi-Fi password / QRZ user / QRZ password
String pendingSsid = "";
String pendingPass = "";
String pendingQrzUser = "";
bool g0HoldTriggered = false;
bool soundMuted = false;

void toggleMute() {
  soundMuted = !soundMuted;
  prefs.begin("rbncfg", false);
  prefs.putBool("muted", soundMuted);
  prefs.end();
}

// -- QRZ.com XML lookup --
String qrzUser = "";
String qrzPass = "";
bool qrzConfigured = false;
String qrzSessionKey = "";
uint32_t qrzSessionObtainedMs = 0;

struct LocCacheEntry {
  String call;
  float lat, lon;
  bool valid; // slot in use
  bool found; // true = QRZ returned coordinates; false = confirmed "not on QRZ"
};
LocCacheEntry locCache[LOC_CACHE_SIZE];
int locCacheNext = 0;

// -- Highscore: farthest logged contact per band --
// Stored on the microSD card when one is present (much better write
// endurance, and swappable for a few euros if it ever wears out);
// falls back to internal flash (NVS) when no card is inserted.
struct HighscoreEntry {
  bool valid;
  float km;
  String spotter;
  String mycall;
};
HighscoreEntry highscores[NUM_BANDS];
bool sdAvailable = false;

void initSdCard() {
  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);
  sdAvailable = SD.begin(SD_SPI_CS_PIN, SPI, 25000000);
}

void loadHighscoresFromSd() {
  File f = SD.open("/highscore.csv", FILE_READ);
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    int p3 = line.indexOf(',', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0) continue;
    int idx = line.substring(0, p1).toInt();
    if (idx < 0 || idx >= NUM_BANDS) continue;
    highscores[idx].valid = true;
    highscores[idx].km = line.substring(p1 + 1, p2).toFloat();
    highscores[idx].spotter = line.substring(p2 + 1, p3);
    highscores[idx].mycall = line.substring(p3 + 1);
  }
  f.close();
}

void saveHighscoresToSd() {
  SD.remove("/highscore.csv");
  File f = SD.open("/highscore.csv", FILE_WRITE);
  if (!f) return;
  for (int i = 0; i < NUM_BANDS; i++) {
    if (!highscores[i].valid) continue;
    f.print(i); f.print(',');
    f.print(highscores[i].km, 1); f.print(',');
    f.print(highscores[i].spotter); f.print(',');
    f.println(highscores[i].mycall);
  }
  f.close();
}

void loadHighscores() {
  // read whatever is in flash first (this doubles as the fallback when
  // there's no SD card, and as the source for a one-time migration)
  prefs.begin("rbnhs", true);
  for (int i = 0; i < NUM_BANDS; i++) {
    String base = "b" + String(i);
    float km = prefs.getFloat((base + "km").c_str(), -1.0f);
    if (km >= 0.0f) {
      highscores[i].valid = true;
      highscores[i].km = km;
      highscores[i].spotter = prefs.getString((base + "sp").c_str(), "");
      highscores[i].mycall = prefs.getString((base + "mc").c_str(), "");
    } else {
      highscores[i].valid = false;
    }
  }
  prefs.end();

  if (sdAvailable) {
    if (SD.exists("/highscore.csv")) {
      for (int i = 0; i < NUM_BANDS; i++) highscores[i].valid = false;
      loadHighscoresFromSd(); // SD data wins once a card file exists
    } else {
      saveHighscoresToSd(); // first time with this card: migrate flash data over
    }
  }
}

void saveHighscore(int bandIdx) {
  if (sdAvailable) {
    saveHighscoresToSd();
    return;
  }
  String base = "b" + String(bandIdx);
  prefs.begin("rbnhs", false);
  prefs.putFloat((base + "km").c_str(), highscores[bandIdx].km);
  prefs.putString((base + "sp").c_str(), highscores[bandIdx].spotter);
  prefs.putString((base + "mc").c_str(), highscores[bandIdx].mycall);
  prefs.end();
}

void checkAndUpdateHighscore(int bandIdx, float km, const String &spotter) {
  if (bandIdx < 0) return;
  if (!highscores[bandIdx].valid || km > highscores[bandIdx].km) {
    highscores[bandIdx].valid = true;
    highscores[bandIdx].km = km;
    highscores[bandIdx].spotter = spotter;
    highscores[bandIdx].mycall = myCall;
    saveHighscore(bandIdx); // written right away, survives power loss
  }
}

void clearHighscores() {
  if (sdAvailable) {
    SD.remove("/highscore.csv");
  } else {
    prefs.begin("rbnhs", false);
    prefs.clear();
    prefs.end();
  }
  for (int i = 0; i < NUM_BANDS; i++) highscores[i].valid = false;
}

// -- last-used callsign: same SD-first, flash-fallback treatment --
String loadLastCall() {
  if (sdAvailable && SD.exists("/lastcall.txt")) {
    File f = SD.open("/lastcall.txt", FILE_READ);
    if (f) {
      String s = f.readStringUntil('\n');
      f.close();
      s.trim();
      return s;
    }
  }
  prefs.begin("rbncfg", true);
  String s = prefs.getString("lastcall", "");
  prefs.end();
  if (sdAvailable && s.length() > 0) {
    SD.remove("/lastcall.txt");
    File f = SD.open("/lastcall.txt", FILE_WRITE);
    if (f) { f.println(s); f.close(); } // migrate to SD
  }
  return s;
}

void saveLastCall(const String &call) {
  if (sdAvailable) {
    SD.remove("/lastcall.txt");
    File f = SD.open("/lastcall.txt", FILE_WRITE);
    if (f) {
      f.println(call);
      f.close();
      return;
    }
  }
  prefs.begin("rbncfg", false);
  prefs.putString("lastcall", call);
  prefs.end();
}

// -- coordinates of every distinct spotter heard this session, for the map --
const int MAX_MAP_POINTS = 30;
struct MapPoint { float lat, lon; };
MapPoint sessionSpotterCoords[MAX_MAP_POINTS];
int sessionSpotterCoordCount = 0;

void addMapPoint(float lat, float lon) {
  for (int i = 0; i < sessionSpotterCoordCount; i++) {
    if (fabsf(sessionSpotterCoords[i].lat - lat) < 0.01f &&
        fabsf(sessionSpotterCoords[i].lon - lon) < 0.01f) return; // already have it
  }
  if (sessionSpotterCoordCount < MAX_MAP_POINTS) {
    sessionSpotterCoords[sessionSpotterCoordCount].lat = lat;
    sessionSpotterCoords[sessionSpotterCoordCount].lon = lon;
    sessionSpotterCoordCount++;
  }
}

void resetResults() {
  spotCount = 0;
  for (int i = 0; i < NUM_BANDS; i++) bandCounts[i] = 0;
  minDist = 1e9f; maxDist = -1.0f; anyDistKnown = false;
  lineLen = 0;
  recentSpotCount = 0;
  sessionSpotterCoordCount = 0;
  displayDirty = true;
}

void pushRecentSpot(const String &spotter, int bandIdx, int snr) {
  for (int i = RECENT_COUNT - 1; i > 0; i--) {
    recentSpots[i] = recentSpots[i - 1];
  }
  recentSpots[0].spotter = spotter;
  recentSpots[0].bandIdx = bandIdx;
  recentSpots[0].snr = snr;
  if (recentSpotCount < RECENT_COUNT) recentSpotCount++;
}

// ---------------------------------------------------------------------
// QRZ.com XML Data API
// ---------------------------------------------------------------------
const uint32_t HTTP_TIMEOUT_MS = 6000; // hard cap so a stuck request can't freeze the app

String httpsGet(const String &url) {
  if (WiFi.status() != WL_CONNECTED) return ""; // fail fast, don't even try a dead link

  WiFiClientSecure sclient;
  sclient.setInsecure(); // skip TLS cert validation - simplest option on ESP32
  sclient.setTimeout(HTTP_TIMEOUT_MS / 1000); // socket read/write timeout, in seconds

  HTTPClient https;
  https.setConnectTimeout(HTTP_TIMEOUT_MS); // cap the TLS/TCP connect phase
  https.setTimeout(HTTP_TIMEOUT_MS);        // cap the request/response phase

  String payload = "";
  if (https.begin(sclient, url)) {
    int code = https.GET();
    if (code == HTTP_CODE_OK) payload = https.getString();
    https.end();
  }
  return payload;
}

String qrzLastError = "";

bool qrzLogin() {
  String url = "https://xmldata.qrz.com/xml/current/?username=" + urlEncode(qrzUser) +
               ";password=" + urlEncode(qrzPass) + ";agent=RBNCardputer1.0";
  String resp = httpsGet(url);
  if (resp.length() == 0) {
    qrzLastError = "No response from QRZ (network?)";
    return false;
  }
  String key = extractXmlTag(resp, "Key");
  if (key.length() == 0) {
    String err = extractXmlTag(resp, "Error");
    qrzLastError = (err.length() > 0) ? err : "Login failed (no session key)";
    return false;
  }
  qrzSessionKey = key;
  qrzSessionObtainedMs = millis();
  qrzLastError = "";
  return true;
}

// Strictly checks whether a string is a valid decimal number (optional
// leading '-', digits, optional single '.'). Arduino's String::toFloat()
// silently returns 0.0 for garbage input, which could otherwise turn an
// incomplete/corrupt parse into a bogus "valid" coordinate of 0.0.
bool isPlausibleNumber(const String &s) {
  if (s.length() == 0) return false;
  bool seenDigit = false, seenDot = false;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '-' && i == 0) continue;
    if (c == '.') {
      if (seenDot) return false;
      seenDot = true;
      continue;
    }
    if (!isDigit(c)) return false;
    seenDigit = true;
  }
  return seenDigit;
}

// Rejects anything outside valid Earth coordinates, and "null island"
// (0,0) which usually means a parse failure rather than a real place.
bool isPlausibleCoord(float lat, float lon) {
  if (isnan(lat) || isnan(lon)) return false;
  if (lat < -90.0f || lat > 90.0f) return false;
  if (lon < -180.0f || lon > 180.0f) return false;
  if (fabsf(lat) < 0.0001f && fabsf(lon) < 0.0001f) return false; // null island
  return true;
}

// One HTTP round trip. gotResponse=false means we couldn't reach QRZ at
// all (network hiccup, timeout) - that's different from QRZ answering
// but not having coordinates for this callsign.
void testQrzLogin() {
  if (!qrzConfigured) {
    qrzLastError = "No QRZ login set.";
    state = ST_QRZ_TEST_RESULT;
    drawQrzTestResultScreen();
    return;
  }
  state = ST_QRZ_TESTING;
  drawQrzTestingScreen();
  qrzSessionKey = ""; // force a real fresh login, not a cached session
  qrzLogin();         // sets qrzLastError on failure, clears it on success
  state = ST_QRZ_TEST_RESULT;
  drawQrzTestResultScreen();
}

QrzResult qrzLookupOnce(const String &call, float &lat, float &lon) {
  String url = "https://xmldata.qrz.com/xml/current/?s=" + qrzSessionKey +
               ";callsign=" + urlEncode(call);
  String resp = httpsGet(url);
  if (resp.length() == 0) return QRZ_TEMP_ERROR;
  String latS = extractXmlTag(resp, "lat");
  String lonS = extractXmlTag(resp, "lon");
  if (!isPlausibleNumber(latS) || !isPlausibleNumber(lonS)) return QRZ_NOT_FOUND;
  float parsedLat = latS.toFloat();
  float parsedLon = lonS.toFloat();
  if (!isPlausibleCoord(parsedLat, parsedLon)) return QRZ_NOT_FOUND;
  lat = parsedLat;
  lon = parsedLon;
  return QRZ_OK;
}

QrzResult qrzLookup(const String &call, float &lat, float &lon) {
  if (!qrzConfigured) return QRZ_TEMP_ERROR;
  if (qrzSessionKey.length() == 0 || millis() - qrzSessionObtainedMs > QRZ_SESSION_MAX_AGE_MS) {
    if (!qrzLogin()) return QRZ_TEMP_ERROR;
  }
  QrzResult r = qrzLookupOnce(call, lat, lon);
  if (r == QRZ_OK) return QRZ_OK;
  // could be an expired/invalid session - log in fresh and try once more
  if (!qrzLogin()) return QRZ_TEMP_ERROR;
  return qrzLookupOnce(call, lat, lon);
}

bool cacheLookup(const String &call, bool &found, float &lat, float &lon) {
  for (int i = 0; i < LOC_CACHE_SIZE; i++) {
    if (locCache[i].valid && locCache[i].call == call) {
      found = locCache[i].found;
      lat = locCache[i].lat; lon = locCache[i].lon;
      return true;
    }
  }
  return false;
}

void cacheStore(const String &call, bool found, float lat, float lon) {
  locCache[locCacheNext].call = call;
  locCache[locCacheNext].found = found;
  locCache[locCacheNext].lat = found ? lat : 0;
  locCache[locCacheNext].lon = found ? lon : 0;
  locCache[locCacheNext].valid = true;
  locCacheNext = (locCacheNext + 1) % LOC_CACHE_SIZE;
}

// Only resolves a location when QRZ is configured; otherwise always
// returns false (no distance is ever shown without QRZ credentials).
// Callsigns confirmed as "not on QRZ" are cached too, so we don't ask
// again every time the same station spots us. Temporary/network errors
// are NOT cached, so those get retried on the next spot.
bool resolveLocation(const String &callIn, float &lat, float &lon) {
  if (!qrzConfigured) return false;
  String call = callIn;
  call.toUpperCase();
  int slashIdx = call.indexOf('/');
  if (slashIdx > 0) call = call.substring(0, slashIdx); // strip /P, /QRP etc.

  bool cachedFound;
  if (cacheLookup(call, cachedFound, lat, lon)) {
    return cachedFound; // hit - either real coords or a known "not found"
  }

  QrzResult r = qrzLookup(call, lat, lon);
  if (r == QRZ_OK) {
    cacheStore(call, true, lat, lon);
    return true;
  }
  if (r == QRZ_NOT_FOUND) {
    cacheStore(call, false, 0, 0); // remember: don't ask again this session
  }
  // QRZ_TEMP_ERROR falls through uncached, will simply try again next time
  return false;
}

// ---------------------------------------------------------------------
// Real land-mask points, derived from a pixelated world map (amCharts
// Pixel Map Generator SVG) the user provided, converted from that SVG's
// own pixel space into approximate lat/lon via a least-squares fit
// against several countries' known real-world coordinates. Values are
// stored as tenths of a degree (int16_t) to keep flash usage tiny.
// Rendered as a dot cloud - matches the source image's own pixel style
// and needs no polygon/fill logic.
const int16_t LAND_POINTS[][2] = {
  {752,-642},{752,-614},{752,-559},{752,-531},{752,-503},{729,-836},{729,-614},{729,-586},
  {707,-919},{684,-947},{684,-891},{684,-864},{684,-836},{684,-808},{684,-753},{684,-725},
  {684,-670},{684,-642},{684,-614},{684,-586},{684,-559},{661,-1085},{661,-1058},{661,-1030},
  {661,-1002},{661,-975},{661,-947},{661,-919},{661,-864},{661,-836},{661,-753},{661,-725},
  {661,-697},{661,-670},{661,-642},{661,-559},{661,-531},{638,-1113},{638,-1085},{638,-1058},
  {638,-1030},{638,-1002},{638,-975},{638,-947},{638,-919},{638,-891},{638,-864},{638,-836},
  {638,-808},{638,-780},{638,-753},{638,-725},{638,-697},{638,-586},{638,-559},{638,-503},
  {616,-1141},{616,-1113},{616,-1085},{616,-1058},{616,-1030},{616,-1002},{616,-975},{616,-947},
  {616,-919},{616,-891},{616,-864},{616,-836},{616,-808},{616,-780},{616,-753},{616,-697},
  {616,-586},{616,-559},{616,-531},{593,-1169},{593,-1141},{593,-1113},{593,-1085},{593,-1058},
  {593,-1030},{593,-1002},{593,-975},{593,-947},{593,-919},{593,-891},{593,-864},{593,-836},
  {593,-808},{593,-642},{593,-614},{593,-586},{570,-1141},{570,-1113},{570,-1085},{570,-1058},
  {570,-1030},{570,-1002},{570,-975},{570,-947},{570,-919},{570,-891},{570,-864},{570,-836},
  {570,-808},{570,-642},{570,-614},{570,-586},{570,-559},{548,-1113},{548,-1085},{548,-1058},
  {548,-1030},{548,-1002},{548,-975},{548,-947},{548,-919},{548,-891},{548,-864},{548,-836},
  {548,-808},{548,-780},{548,-642},{548,-614},{548,-586},{548,-559},{548,-531},{525,-1141},
  {525,-1113},{525,-1085},{525,-1058},{525,-1030},{525,-1002},{525,-975},{525,-947},{525,-919},
  {525,-891},{525,-864},{525,-836},{525,-808},{525,-780},{525,-753},{525,-697},{525,-670},
  {525,-642},{525,-614},{525,-586},{525,-559},{525,-531},{525,-503},{502,-1141},{502,-1113},
  {502,-1085},{502,-1058},{502,-1030},{502,-1002},{502,-975},{502,-947},{502,-919},{502,-891},
  {502,-864},{502,-836},{502,-808},{502,-780},{502,-753},{502,-725},{502,-697},{502,-670},
  {502,-642},{502,-614},{502,-586},{502,-559},{502,-531},{502,-503},{480,-1141},{480,-836},
  {480,-808},{480,-780},{480,-753},{480,-725},{480,-697},{480,-670},{480,-642},{480,-614},
  {480,-586},{480,-503},{457,-753},{457,-725},{457,-697},{457,-670},{457,-614},{457,-559},
  {434,-753},{434,-725},{434,-614},{752,-448},{752,-420},{752,-392},{752,-365},{752,-337},
  {752,-309},{752,-281},{752,-254},{752,-226},{752,-198},{752,-171},{752,-143},{752,-115},
  {729,-503},{729,-475},{729,-448},{729,-420},{729,-392},{729,-365},{729,-337},{729,-309},
  {729,-281},{729,-254},{729,-226},{729,-198},{729,-171},{729,-143},{707,-392},{707,-365},
  {707,-337},{707,-309},{707,-281},{707,-254},{707,-226},{707,-198},{707,-171},{707,-143},
  {684,-392},{684,-365},{684,-337},{684,-309},{684,-281},{684,-254},{684,-226},{684,-198},
  {684,-171},{661,-392},{661,-365},{661,-337},{661,-309},{661,-281},{661,-254},{661,-226},
  {638,-420},{638,-392},{638,-365},{638,-337},{638,-309},{638,-281},{616,-392},{616,-365},
  {616,-337},{593,-392},{593,-365},{661,-1307},{661,-1280},{661,-1252},{661,-1224},{661,-1196},
  {661,-1169},{661,-1141},{661,-1113},{638,-1335},{638,-1280},{638,-1252},{638,-1224},{638,-1196},
  {638,-1169},{638,-1141},{616,-1335},{616,-1307},{616,-1280},{616,-1252},{616,-1224},{616,-1196},
  {616,-1169},{593,-1363},{593,-1335},{593,-1307},{593,-1280},{593,-1252},{593,-1224},{593,-1196},
  {570,-1335},{570,-1169},{548,-1141},{480,-1113},{480,-1085},{480,-1058},{480,-1030},{480,-1002},
  {480,-975},{480,-947},{480,-919},{480,-891},{480,-864},{457,-1141},{457,-1113},{457,-1085},
  {457,-1058},{457,-1030},{457,-1002},{457,-975},{457,-947},{457,-919},{457,-891},{457,-864},
  {457,-836},{457,-808},{457,-780},{457,-642},{434,-1141},{434,-1113},{434,-1085},{434,-1058},
  {434,-1030},{434,-1002},{434,-975},{434,-947},{434,-919},{434,-891},{434,-864},{434,-836},
  {434,-808},{434,-780},{434,-697},{434,-670},{411,-1169},{411,-1141},{411,-1113},{411,-1085},
  {411,-1058},{411,-1030},{411,-1002},{411,-975},{411,-947},{411,-919},{411,-891},{411,-864},
  {411,-836},{411,-808},{411,-780},{411,-753},{411,-725},{411,-697},{411,-670},{389,-1169},
  {389,-1141},{389,-1113},{389,-1085},{389,-1058},{389,-1030},{389,-1002},{389,-975},{389,-947},
  {389,-919},{389,-891},{389,-864},{389,-836},{389,-808},{389,-780},{389,-753},{389,-725},
  {366,-1169},{366,-1141},{366,-1113},{366,-1085},{366,-1058},{366,-1030},{366,-1002},{366,-975},
  {366,-947},{366,-919},{366,-891},{366,-864},{366,-836},{366,-808},{366,-780},{366,-753},
  {343,-1169},{343,-1141},{343,-1113},{343,-1085},{343,-1058},{343,-1030},{343,-1002},{343,-975},
  {343,-947},{343,-919},{343,-891},{343,-864},{343,-836},{343,-808},{343,-780},{343,-753},
  {321,-1113},{321,-1085},{321,-1058},{321,-1030},{321,-1002},{321,-975},{321,-947},{321,-919},
  {321,-891},{321,-864},{321,-836},{321,-808},{298,-1030},{298,-1002},{298,-975},{298,-947},
  {298,-919},{298,-836},{298,-808},{275,-975},{275,-808},{253,-808},{661,162},{661,190},
  {616,107},{593,79},{593,107},{570,79},{661,218},{638,218},{638,245},{616,218},
  {616,245},{593,218},{593,245},{638,-171},{638,-143},{638,-115},{638,134},{638,162},
  {638,190},{616,134},{616,162},{593,134},{570,134},{548,134},{570,-32},{548,-32},
  {525,-4},{502,-4},{525,-60},{525,79},{525,107},{525,134},{502,79},{502,107},
  {502,134},{480,107},{480,134},{480,24},{480,51},{480,79},{457,24},{457,51},
  {434,24},{434,51},{434,79},{457,107},{457,134},{434,107},{411,134},{411,162},
  {411,-32},{411,-4},{411,24},{389,-60},{389,-32},{389,-4},{366,-32},{752,661},
  {752,689},{729,772},{707,439},{707,689},{707,717},{707,744},{707,772},{707,800},
  {707,828},{707,855},{684,412},{684,439},{684,550},{684,578},{684,606},{684,634},
  {684,661},{684,689},{684,717},{684,744},{684,772},{684,800},{684,828},{684,855},
  {684,883},{684,911},{684,939},{684,966},{684,994},{684,1105},{684,1133},{684,1160},
  {661,245},{661,273},{661,301},{661,495},{661,523},{661,578},{661,606},{661,634},
  {661,661},{661,689},{661,717},{661,744},{661,772},{661,800},{661,828},{661,855},
  {661,883},{661,911},{661,939},{661,966},{661,994},{661,1022},{661,1049},{661,1077},
  {661,1105},{661,1133},{661,1160},{661,1188},{661,1216},{661,1243},{661,1271},{661,1299},
  {661,1327},{661,1354},{661,1382},{661,1410},{661,1438},{638,273},{638,356},{638,384},
  {638,412},{638,439},{638,467},{638,495},{638,523},{638,550},{638,578},{638,606},
  {638,634},{638,661},{638,689},{638,717},{638,744},{638,772},{638,800},{638,828},
  {638,855},{638,883},{638,911},{638,939},{638,966},{638,994},{638,1022},{638,1049},
  {638,1077},{638,1105},{638,1133},{638,1160},{638,1188},{638,1216},{638,1243},{638,1271},
  {638,1299},{638,1327},{638,1354},{638,1382},{638,1410},{638,1438},{638,1465},{616,273},
  {616,301},{616,329},{616,356},{616,384},{616,412},{616,439},{616,467},{616,495},
  {616,523},{616,550},{616,578},{616,606},{616,634},{616,661},{616,689},{616,717},
  {616,744},{616,772},{616,800},{616,828},{616,855},{616,883},{616,911},{616,939},
  {616,966},{616,994},{616,1022},{616,1049},{616,1077},{616,1105},{616,1133},{616,1160},
  {616,1188},{616,1216},{616,1243},{616,1271},{616,1299},{616,1327},{616,1354},{616,1382},
  {616,1410},{616,1438},{616,1465},{616,1493},{593,273},{593,301},{593,329},{593,356},
  {593,384},{593,412},{593,439},{593,467},{593,495},{593,523},{593,550},{593,578},
  {593,606},{593,634},{593,661},{593,689},{593,717},{593,744},{593,772},{593,800},
  {593,828},{593,855},{593,883},{593,911},{593,939},{593,966},{593,994},{593,1022},
  {593,1049},{593,1077},{593,1105},{593,1133},{593,1160},{593,1188},{593,1216},{593,1243},
  {593,1271},{593,1299},{593,1327},{593,1410},{593,1438},{593,1465},{570,273},{570,301},
  {570,329},{570,356},{570,384},{570,412},{570,439},{570,467},{570,495},{570,523},
  {570,550},{570,578},{570,606},{570,634},{570,661},{570,689},{570,717},{570,744},
  {570,772},{570,800},{570,828},{570,855},{570,883},{570,911},{570,939},{570,966},
  {570,994},{570,1022},{570,1049},{570,1077},{570,1105},{570,1133},{570,1160},{570,1188},
  {570,1216},{570,1410},{548,273},{548,301},{548,329},{548,356},{548,384},{548,412},
  {548,439},{548,467},{548,495},{548,523},{548,550},{548,578},{548,606},{548,634},
  {548,661},{548,689},{548,717},{548,744},{548,772},{548,800},{548,828},{548,855},
  {548,883},{548,911},{548,939},{548,966},{548,994},{548,1022},{548,1049},{548,1077},
  {548,1105},{548,1133},{548,1160},{548,1188},{548,1216},{548,1382},{548,1410},{525,301},
  {525,329},{525,356},{525,384},{525,412},{525,439},{525,467},{525,495},{525,523},
  {525,550},{525,717},{525,744},{525,772},{525,800},{525,828},{525,855},{525,883},
  {525,911},{525,939},{525,966},{525,994},{525,1022},{525,1049},{525,1077},{525,1105},
  {525,1133},{525,1160},{525,1188},{525,1216},{525,1243},{525,1410},{525,1438},{502,329},
  {502,356},{502,384},{502,412},{502,439},{502,523},{502,550},{502,772},{502,800},
  {502,828},{502,855},{502,883},{502,939},{502,966},{502,994},{502,1022},{502,1049},
  {502,1077},{502,1188},{502,1216},{502,1243},{502,1271},{480,384},{480,412},{480,439},
  {480,1216},{480,1243},{480,1271},{480,1299},{457,384},{457,412},{457,439},{457,467},
  {457,1271},{457,1299},{434,384},{434,412},{434,1271},{411,467},{-133,1382},{-133,1410},
  {-133,1493},{-156,1299},{-156,1327},{-156,1354},{-156,1382},{-156,1410},{-156,1493},{-179,1271},
  {-179,1299},{-179,1327},{-179,1354},{-179,1382},{-179,1410},{-179,1438},{-179,1465},{-179,1493},
  {-202,1243},{-202,1271},{-202,1299},{-202,1327},{-202,1354},{-202,1382},{-202,1410},{-202,1438},
  {-202,1465},{-202,1493},{-202,1521},{-224,1188},{-224,1216},{-224,1243},{-224,1271},{-224,1299},
  {-224,1327},{-224,1354},{-224,1382},{-224,1410},{-224,1438},{-224,1465},{-224,1493},{-224,1521},
  {-247,1188},{-247,1216},{-247,1243},{-247,1271},{-247,1299},{-247,1327},{-247,1354},{-247,1382},
  {-247,1410},{-247,1438},{-247,1465},{-247,1493},{-247,1521},{-247,1548},{-270,1160},{-270,1188},
  {-270,1216},{-270,1243},{-270,1271},{-270,1299},{-270,1327},{-270,1354},{-270,1382},{-270,1410},
  {-270,1438},{-270,1465},{-270,1493},{-270,1521},{-270,1548},{-292,1160},{-292,1188},{-292,1216},
  {-292,1243},{-292,1271},{-292,1299},{-292,1327},{-292,1354},{-292,1382},{-292,1410},{-292,1438},
  {-292,1465},{-292,1493},{-292,1521},{-315,1160},{-315,1188},{-315,1216},{-315,1243},{-315,1271},
  {-315,1299},{-315,1327},{-315,1354},{-315,1382},{-315,1410},{-315,1438},{-315,1465},{-315,1493},
  {-315,1521},{-338,1160},{-338,1188},{-338,1216},{-338,1354},{-338,1382},{-338,1410},{-338,1438},
  {-338,1465},{-338,1493},{-360,1354},{-360,1382},{-360,1410},{-360,1438},{-360,1465},{-383,1382},
  {-383,1410},{-383,1438},{25,-642},{25,-531},{3,-697},{3,-670},{3,-642},{3,-614},
  {3,-586},{3,-559},{3,-531},{-20,-697},{-20,-670},{-20,-642},{-20,-614},{-20,-586},
  {-20,-559},{-20,-531},{-20,-475},{-43,-697},{-43,-670},{-43,-642},{-43,-614},{-43,-586},
  {-43,-559},{-43,-531},{-43,-503},{-43,-475},{-43,-448},{-43,-420},{-43,-392},{-65,-725},
  {-65,-697},{-65,-670},{-65,-642},{-65,-614},{-65,-586},{-65,-559},{-65,-531},{-65,-503},
  {-65,-475},{-65,-448},{-65,-420},{-65,-392},{-65,-365},{-88,-753},{-88,-725},{-88,-697},
  {-88,-670},{-88,-642},{-88,-614},{-88,-586},{-88,-559},{-88,-531},{-88,-503},{-88,-475},
  {-88,-448},{-88,-420},{-88,-392},{-88,-365},{-111,-642},{-111,-614},{-111,-586},{-111,-559},
  {-111,-531},{-111,-503},{-111,-475},{-111,-448},{-111,-420},{-111,-392},{-133,-614},{-133,-586},
  {-133,-559},{-133,-531},{-133,-503},{-133,-475},{-133,-448},{-133,-420},{-156,-586},{-156,-559},
  {-156,-531},{-156,-503},{-156,-475},{-156,-448},{-156,-420},{-156,-392},{-179,-559},{-179,-531},
  {-179,-503},{-179,-475},{-179,-448},{-179,-420},{-202,-559},{-202,-531},{-202,-503},{-202,-475},
  {-202,-448},{-202,-420},{-224,-559},{-224,-531},{-224,-503},{-224,-475},{-224,-448},{-224,-420},
  {-247,-531},{-247,-503},{-247,-475},{-270,-531},{-270,-503},{-292,-531},{-292,-503},{-315,-531},
  {-224,-670},{-224,-642},{-247,-670},{-247,-642},{-247,-614},{-270,-670},{-270,-642},{-270,-614},
  {-270,-586},{-292,-670},{-292,-642},{-292,-614},{-292,-586},{-292,-559},{-315,-670},{-315,-642},
  {-315,-614},{-315,-586},{-338,-670},{-338,-642},{-338,-614},{-338,-586},{-360,-670},{-360,-642},
  {-360,-614},{-360,-586},{-360,-559},{-383,-670},{-383,-642},{-383,-614},{-383,-586},{-383,-559},
  {-406,-670},{-406,-642},{-406,-614},{-429,-670},{-429,-642},{-429,-614},{-451,-642},{-474,-642},
  {-474,-614},{-497,-642},{-497,-614},{-542,-586},{-202,-697},{-224,-697},{-247,-697},{-270,-697},
  {-292,-697},{-315,-697},{-338,-697},{-360,-697},{-383,-697},{-406,-697},{-451,-670},{-474,-670},
  {-497,-670},{-519,-642},{-542,-614},{-247,301},{-247,329},{-270,218},{-270,245},{-270,273},
  {-270,301},{-292,190},{-292,218},{-292,245},{-292,273},{-315,190},{-315,218},{-315,245},
  {-315,273},{-315,301},{-338,218},{-338,245},{298,273},{298,301},{298,329},{275,273},
  {275,301},{275,329},{253,273},{253,301},{253,329},{253,356},{230,273},{230,301},
  {230,329},{230,356},{25,218},{25,245},{25,273},{25,301},{25,329},{3,218},
  {3,245},{3,273},{3,301},{-20,190},{-20,218},{-20,245},{-20,273},{-20,301},
  {-43,190},{-43,218},{-43,245},{-43,273},{-43,301},{-65,190},{-65,218},{-65,245},
  {-65,273},{-65,301},{-88,245},{-88,273},{-88,301},{-111,245},{-111,273},{-111,301},
  {298,412},{298,439},{275,384},{275,412},{275,439},{275,467},{275,495},{253,384},
  {253,412},{253,439},{253,467},{253,495},{230,412},{230,439},{230,467},{230,495},
  {230,523},{207,412},{207,439},{207,467},{207,495},{207,523},{207,550},{184,439},
  {184,467},{184,495},{184,523},{502,1105},{502,1133},{502,1160},{480,1105},{480,1133},
  {480,1160},{480,1188},{457,800},{457,828},{457,855},{457,1133},{457,1160},{457,1188},
  {457,1216},{457,1243},{434,772},{434,800},{434,828},{434,855},{434,883},{434,911},
  {434,1077},{434,1105},{434,1133},{434,1160},{434,1188},{434,1216},{434,1243},{411,772},
  {411,800},{411,828},{411,855},{411,883},{411,911},{411,939},{411,966},{411,994},
  {411,1022},{411,1049},{411,1077},{411,1105},{411,1133},{411,1160},{411,1188},{411,1216},
  {389,744},{389,772},{389,800},{389,828},{389,855},{389,883},{389,911},{389,939},
  {389,966},{389,994},{389,1022},{389,1049},{389,1077},{389,1105},{389,1133},{389,1188},
  {366,744},{366,772},{366,800},{366,828},{366,855},{366,883},{366,911},{366,939},
  {366,966},{366,994},{366,1022},{366,1049},{366,1077},{366,1105},{366,1133},{366,1160},
  {366,1188},{343,800},{343,828},{343,855},{343,883},{343,911},{343,939},{343,966},
  {343,994},{343,1022},{343,1049},{343,1077},{343,1105},{343,1133},{343,1160},{321,800},
  {321,828},{321,855},{321,883},{321,911},{321,939},{321,966},{321,994},{321,1022},
  {321,1049},{321,1077},{321,1105},{321,1133},{321,1160},{321,1188},{298,855},{298,883},
  {298,911},{298,939},{298,966},{298,994},{298,1022},{298,1049},{298,1077},{298,1105},
  {298,1133},{298,1160},{298,1188},{298,1216},{275,883},{275,939},{275,1022},{275,1049},
  {275,1077},{275,1105},{275,1133},{275,1160},{275,1188},{275,1216},{253,1022},{253,1049},
  {253,1077},{253,1105},{253,1133},{253,1160},{253,1188},{253,1216},{230,1022},{230,1049},
  {230,1077},{230,1105},{230,1133},{230,1160},{230,1188},{207,1133},{184,1133},{343,772},
  {321,772},{298,772},{298,800},{275,744},{275,772},{275,800},{275,828},{275,966},
  {253,744},{253,772},{253,800},{253,828},{253,855},{253,883},{253,939},{253,966},
  {230,717},{230,744},{230,772},{230,800},{230,828},{230,855},{230,883},{230,911},
  {230,939},{207,744},{207,772},{207,800},{207,828},{207,855},{207,883},{184,772},
  {184,800},{184,828},{184,855},{162,772},{162,800},{162,828},{162,855},{139,800},
  {139,828},{116,800},{116,828},{94,828},{434,1354},{366,1382},{343,1327},{343,1354},
  {321,-1141},{298,-1141},{298,-1113},{298,-1085},{298,-1058},{275,-1085},{275,-1058},{275,-1030},
  {275,-1002},{253,-1085},{253,-1058},{253,-1030},{253,-1002},{253,-975},{230,-1058},{230,-1030},
  {230,-1002},{207,-1058},{207,-1030},{207,-1002},{207,-891},{184,-1058},{184,-1030},{184,-1002},
  {184,-919},{184,-891},{162,-1002},{162,-975},{162,-947},{48,1022},{25,1049},{25,1216},
  {3,1049},{3,1077},{3,1160},{3,1188},{3,1216},{3,1271},{3,1299},{-20,1077},
  {-20,1160},{-20,1188},{-20,1216},{-20,1271},{-43,1105},{-43,1438},{-43,1465},{-65,1133},
  {-65,1465},{-88,1243},{-88,1271},{-360,1715},{-383,1715},{-429,1632},{-451,1604},{-43,1493},
  {-65,1493},{-65,1521},{-65,1548},{-65,1632},{-88,1493},{-88,1548},{411,273},{411,329},
  {389,273},{389,301},{389,329},{389,356},{389,384},{389,412},{366,301},{366,329},
  {366,356},{366,384},{366,412},{343,-32},{321,-60},{321,-32},{321,-4},{298,-60},
  {275,-115},{275,-87},{343,-4},{343,24},{343,51},{343,79},{321,24},{321,51},
  {321,79},{298,-32},{298,-4},{298,24},{298,51},{298,79},{275,-60},{275,-32},
  {275,-4},{275,24},{275,51},{275,79},{275,107},{253,-32},{253,-4},{253,24},
  {253,51},{253,79},{253,107},{230,-4},{230,24},{230,51},{230,79},{230,107},
  {207,24},{207,51},{207,79},{207,273},{207,301},{207,329},{207,356},{207,384},
  {184,273},{184,301},{184,329},{184,356},{184,384},{162,273},{162,301},{162,329},
  {162,356},{162,384},{139,245},{139,273},{139,301},{139,329},{139,356},{139,384},
  {116,245},{116,273},{116,301},{116,329},{116,356},{94,301},{116,51},{116,79},
  {116,107},{116,134},{94,51},{94,79},{94,107},{94,134},{71,51},{71,79},
  {71,107},{48,79},{139,412},{116,384},{116,412},{116,439},{94,384},{94,412},
  {94,439},{71,384},{71,412},{71,439},{71,467},{71,495},{48,384},{48,412},
  {48,439},{48,467},{525,578},{525,606},{525,634},{525,661},{525,689},{502,467},
  {502,495},{502,578},{502,606},{502,634},{502,661},{502,689},{502,717},{502,744},
  {480,467},{480,495},{480,523},{480,550},{480,578},{480,606},{480,634},{480,661},
  {480,689},{480,717},{480,744},{480,772},{480,800},{457,523},{457,550},{457,578},
  {457,606},{457,634},{457,661},{457,689},{457,717},{457,744},{457,772},{434,495},
  {434,523},{434,606},{434,634},{434,661},{434,689},{434,717},{434,744},{411,661},
  {502,911},{480,828},{480,855},{480,883},{480,911},{480,939},{480,966},{480,994},
  {480,1022},{480,1049},{480,1077},{457,883},{457,911},{457,939},{457,966},{457,994},
  {457,1022},{457,1049},{457,1077},{457,1105},{434,939},{434,966},{434,994},{434,1022},
  {434,1049},{321,134},{321,218},{298,107},{298,134},{298,162},{298,190},{298,218},
  {298,245},{275,134},{275,162},{275,190},{275,218},{275,245},{253,134},{253,162},
  {253,190},{253,218},{253,245},{230,134},{230,162},{230,190},{230,218},{230,245},
  {207,245},{366,107},{343,107},{321,107},{253,-115},{230,-143},{230,-115},{253,-87},
  {253,-60},{230,-87},{230,-60},{207,-143},{207,-115},{207,-87},{207,-60},{184,-143},
  {184,-115},{184,-87},{184,-60},{162,-115},{162,-87},{162,-60},{230,-32},{207,-32},
  {207,-4},{184,-32},{184,-4},{184,24},{184,51},{162,-32},{162,-4},{162,24},
  {162,51},{139,-115},{139,-87},{139,-60},{139,-32},{116,-60},{207,107},{207,134},
  {207,162},{184,79},{184,107},{184,134},{184,162},{162,79},{162,107},{162,134},
  {162,162},{139,24},{139,51},{139,79},{139,107},{139,134},{207,190},{207,218},
  {184,190},{184,218},{184,245},{162,190},{162,218},{162,245},{139,162},{139,190},
  {139,218},{116,190},{116,218},{94,162},{94,190},{94,218},{162,-143},{139,-143},
  {116,-143},{116,-115},{116,-87},{94,-87},{71,-87},{94,-115},{71,-115},{94,-60},
  {94,-32},{71,-60},{71,-32},{48,-60},{94,-4},{71,-4},{48,-4},{94,24},
  {71,24},{139,-4},{116,-32},{116,-4},{116,24},{94,245},{71,190},{71,218},
  {71,245},{71,273},{48,190},{48,218},{48,245},{48,273},{94,273},{94,329},
  {94,356},{71,301},{71,329},{71,356},{48,301},{48,329},{48,356},{94,467},
  {94,495},{94,523},{71,523},{48,495},{25,439},{25,467},{3,439},{162,412},
  {25,384},{25,412},{3,384},{3,412},{-20,384},{-20,412},{-20,439},{-43,412},
  {25,356},{3,329},{3,356},{-20,329},{-20,356},{-43,329},{-43,356},{-43,384},
  {-65,329},{-65,356},{-65,384},{-65,412},{-88,356},{-88,384},{-88,412},{-111,384},
  {-111,412},{25,190},{3,162},{3,190},{-20,162},{-43,134},{-43,162},{3,134},
  {-20,107},{-20,134},{116,162},{71,134},{71,162},{48,107},{48,134},{48,162},
  {25,134},{25,162},{-65,162},{-88,162},{-88,190},{-88,218},{-111,162},{-111,190},
  {-111,218},{-133,162},{-133,190},{-133,218},{-156,134},{-156,162},{-156,190},{-156,218},
  {-179,218},{-88,329},{-111,329},{-133,245},{-133,273},{-133,301},{-133,329},{-156,245},
  {-156,273},{-156,301},{-179,273},{-111,356},{-133,356},{-133,384},{-133,412},{-156,329},
  {-156,356},{-156,384},{-156,412},{-179,356},{-179,384},{-202,356},{-224,356},{-247,356},
  {-133,523},{-156,495},{-156,523},{-179,467},{-179,495},{-202,467},{-202,495},{-224,467},
  {-224,495},{-247,467},{-179,134},{-179,162},{-179,190},{-179,245},{-202,162},{-202,190},
  {-202,218},{-224,162},{-224,190},{-247,162},{-247,190},{-270,190},{-179,301},{-179,329},
  {-202,301},{-202,329},{-224,329},{-202,245},{-202,273},{-224,218},{-224,245},{-224,273},
  {-224,301},{-247,218},{-247,245},{-247,273},{-270,329},{-292,301},{94,-753},{71,-780},
  {71,-753},{48,-780},{48,-753},{48,-725},{25,-780},{25,-753},{25,-725},{3,-780},
  {3,-753},{3,-725},{-20,-753},{-20,-725},{94,-725},{94,-697},{94,-670},{94,-642},
  {71,-725},{71,-697},{71,-670},{71,-642},{48,-697},{48,-670},{48,-642},{25,-697},
  {25,-670},{71,-614},{48,-614},{25,-614},{48,-586},{48,-559},{25,-586},{25,-559},
  {3,-808},{-20,-808},{-20,-780},{-43,-808},{-43,-780},{-43,-753},{-43,-725},{-65,-808},
  {-65,-780},{-65,-753},{-88,-808},{-88,-780},{-111,-780},{-111,-753},{-111,-725},{-133,-780},
  {-133,-753},{-133,-725},{-156,-753},{-156,-725},{-111,-697},{-111,-670},{-133,-697},{-133,-670},
  {-133,-642},{-156,-697},{-156,-670},{-156,-642},{-156,-614},{-179,-697},{-179,-670},{-179,-642},
  {-179,-614},{-179,-586},{-202,-670},{-202,-642},{-202,-614},{-202,-586},{-224,-614},{-224,-586},
  {-247,-586},{-247,-559},{-270,-559},{-315,-559},{-338,-559},{-338,-531},{570,218},{570,245},
  {548,218},{548,245},{502,51},{525,162},{525,190},{525,218},{502,162},{502,190},
  {502,218},{525,245},{525,273},{502,245},{502,273},{502,301},{480,218},{480,245},
  {480,273},{480,301},{480,329},{480,356},{457,329},{480,162},{480,190},{457,79},
  {457,162},{434,162},{457,190},{457,218},{457,245},{457,273},{434,245},{434,273},
  {434,190},{434,218},{411,-60},{411,218},{411,245},{389,218},{343,329},{411,412},
  {411,439},{389,467},
};
const int LAND_POINT_COUNT = 2018;

// Latitude range shown on the map - cropped to where hams actually are,
// so Arctic/Antarctic emptiness doesn't waste screen space.
const float MAP_LAT_MIN = -58.0f;
const float MAP_LAT_MAX = 78.0f;

void projectLatLon(float lat, float lon, int mapX, int mapY, int mapW, int mapH,
                    int &px, int &py) {
  px = mapX + (int)((lon + 180.0f) / 360.0f * mapW);
  float latSpan = MAP_LAT_MAX - MAP_LAT_MIN;
  py = mapY + (int)((MAP_LAT_MAX - lat) / latSpan * mapH);
  py = constrain(py, mapY, mapY + mapH - 1);
}

// ---------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------
void drawHeader(const char* title) {
  M5Cardputer.Display.fillScreen(BLACK);
  M5Cardputer.Display.setTextSize(2);
  M5Cardputer.Display.setTextColor(GREEN, BLACK);
  M5Cardputer.Display.setCursor(4, 4);
  M5Cardputer.Display.print(title);

  // battery indicator, top right corner
  M5Cardputer.Display.setTextSize(1);
  int batLevel = M5.Power.getBatteryLevel(); // 0-100, -1 if unknown
  bool charging = M5.Power.isCharging();
  char batStr[8];
  if (batLevel >= 0) {
    snprintf(batStr, sizeof(batStr), "%s%d%%", charging ? "+" : "", batLevel);
  } else {
    snprintf(batStr, sizeof(batStr), "--");
  }
  bool lowBattery = (batLevel >= 0 && batLevel <= 15 && !charging);
  M5Cardputer.Display.setTextColor(lowBattery ? RED : WHITE, BLACK);
  int batW = M5Cardputer.Display.textWidth(batStr);
  M5Cardputer.Display.setCursor(M5Cardputer.Display.width() - batW - 4, 8);
  M5Cardputer.Display.print(batStr);

  M5Cardputer.Display.drawFastHLine(0, 24, M5Cardputer.Display.width(), DARKGREY);
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
  M5Cardputer.Display.setTextSize(1);
}

// -- Wi-Fi screens --
void drawWifiScanningScreen() {
  drawHeader("Wi-Fi Scan");
  M5Cardputer.Display.setCursor(4, 34);
  M5Cardputer.Display.println("Scanning for networks...");
}

void drawWifiListScreen() {
  drawHeader("Select Wi-Fi");
  int y = 30;
  if (networkCount == 0) {
    M5Cardputer.Display.setCursor(4, y);
    M5Cardputer.Display.println("No networks found.");
    y += 12;
  } else {
    for (int i = 0; i < networkCount; i++) {
      M5Cardputer.Display.setCursor(4, y);
      M5Cardputer.Display.printf("%d %s%s\n", i + 1,
        networks[i].secure ? "* " : "  ",
        networks[i].ssid.c_str());
      y += 11;
    }
  }
  M5Cardputer.Display.setCursor(4, y + 2);
  M5Cardputer.Display.println("m = enter network manually");
  M5Cardputer.Display.setCursor(4, y + 13);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.println("G0 = rescan  (* = encrypted)");
}

void drawTextInputScreen(const char* title, const String &prompt,
                          const String &value, bool masked) {
  drawHeader(title);
  M5Cardputer.Display.setCursor(4, 34);
  M5Cardputer.Display.println(prompt);
  M5Cardputer.Display.setTextSize(2);
  M5Cardputer.Display.setCursor(4, 58);
  M5Cardputer.Display.setTextColor(YELLOW, BLACK);
  if (masked) {
    for (unsigned int i = 0; i < value.length(); i++) M5Cardputer.Display.print("*");
  } else {
    M5Cardputer.Display.print(value);
  }
  M5Cardputer.Display.print("_");
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.setCursor(4, 100);
  M5Cardputer.Display.println("ENTER = confirm");
  M5Cardputer.Display.println("G0 = back");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

void drawWifiConnectingScreen(const String &ssid) {
  drawHeader("Connecting...");
  M5Cardputer.Display.setCursor(4, 34);
  M5Cardputer.Display.print("Wi-Fi: ");
  M5Cardputer.Display.println(ssid);
}

void drawWifiErrorScreen(const String &ssid) {
  drawHeader("Wi-Fi Error");
  M5Cardputer.Display.setCursor(4, 34);
  M5Cardputer.Display.println(statusMsg);
  M5Cardputer.Display.setCursor(4, 50);
  M5Cardputer.Display.print("Network: ");
  M5Cardputer.Display.println(ssid);
  M5Cardputer.Display.setCursor(4, 80);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.println("ENTER = try again");
  M5Cardputer.Display.println("G0 = network list");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

// -- Settings screen --
void drawSettingsMenuScreen() {
  drawHeader("Settings");
  M5Cardputer.Display.setCursor(4, 26);
  M5Cardputer.Display.println("1  Wi-Fi setup");
  M5Cardputer.Display.setCursor(4, 37);
  if (qrzConfigured) {
    M5Cardputer.Display.print("2  QRZ login (on: ");
    M5Cardputer.Display.print(qrzUser);
    M5Cardputer.Display.println(")");
  } else {
    M5Cardputer.Display.println("2  QRZ login (off)");
  }
  M5Cardputer.Display.setCursor(4, 48);
  M5Cardputer.Display.println("3  Remove QRZ login");
  M5Cardputer.Display.setCursor(4, 59);
  M5Cardputer.Display.println("4  Highscore");
  M5Cardputer.Display.setCursor(4, 70);
  M5Cardputer.Display.print("5  Sound: ");
  M5Cardputer.Display.println(soundMuted ? "muted" : "on");
  M5Cardputer.Display.setCursor(4, 81);
  M5Cardputer.Display.println("6  Test QRZ login");
  M5Cardputer.Display.setCursor(4, 98);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.print("SD card: ");
  M5Cardputer.Display.println(sdAvailable ? "found (used for data)" : "not found (using flash)");
  M5Cardputer.Display.setCursor(4, 109);
  M5Cardputer.Display.println("No QRZ login = no range shown.");
  M5Cardputer.Display.setCursor(4, 122);
  M5Cardputer.Display.println("G0 = back");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

void drawQrzTestingScreen() {
  drawHeader("QRZ Test");
  M5Cardputer.Display.setCursor(4, 34);
  M5Cardputer.Display.println("Logging in to QRZ...");
}

void drawQrzTestResultScreen() {
  drawHeader("QRZ Test");
  M5Cardputer.Display.setCursor(4, 34);
  M5Cardputer.Display.setTextColor(qrzLastError.length() == 0 ? GREEN : RED, BLACK);
  M5Cardputer.Display.println(qrzLastError.length() == 0 ? "Login successful!" : "Login failed:");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
  if (qrzLastError.length() > 0) {
    M5Cardputer.Display.setCursor(4, 48);
    M5Cardputer.Display.println(qrzLastError);
  }
  M5Cardputer.Display.setCursor(4, 80);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.println("ENTER = back to settings");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

void drawHighscoreScreen() {
  drawHeader("Highscore");
  int y = 26;
  for (int i = 0; i < NUM_BANDS; i++) {
    M5Cardputer.Display.setCursor(4, y);
    if (highscores[i].valid) {
      M5Cardputer.Display.printf("%-5s %5dkm %.10s\n", BANDS[i].name,
        (int)highscores[i].km, highscores[i].spotter.c_str());
    } else {
      M5Cardputer.Display.printf("%-5s --\n", BANDS[i].name);
    }
    y += 8;
  }
  M5Cardputer.Display.setCursor(4, y + 2);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.println("G0=back  c=clear all");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

// -- RBN screens --
void drawInputScreen() {
  drawHeader("RBN Check");
  M5Cardputer.Display.setCursor(4, 34);
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.println("Enter your callsign,");
  M5Cardputer.Display.println("confirm with ENTER:");
  M5Cardputer.Display.setTextSize(2);
  M5Cardputer.Display.setCursor(4, 58);
  M5Cardputer.Display.setTextColor(YELLOW, BLACK);
  M5Cardputer.Display.print(myCall);
  M5Cardputer.Display.print("_");
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.setCursor(4, 96);
  M5Cardputer.Display.print("Wi-Fi: ");
  M5Cardputer.Display.println(WiFi.SSID());
  M5Cardputer.Display.setCursor(4, 106);
  M5Cardputer.Display.print("QRZ: ");
  M5Cardputer.Display.println(qrzConfigured ? "on" : "off");
  M5Cardputer.Display.setCursor(4, 118);
  M5Cardputer.Display.println("Hold G0 = settings");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

void drawConnectingScreen() {
  drawHeader("Connecting...");
  M5Cardputer.Display.setCursor(4, 34);
  M5Cardputer.Display.println(statusMsg);
}

void drawListeningScreen(uint32_t elapsedSec) {
  drawHeader("LIVE");
  M5Cardputer.Display.setCursor(4, 28);
  M5Cardputer.Display.print(myCall);
  M5Cardputer.Display.print("  running ");
  M5Cardputer.Display.print(elapsedSec);
  M5Cardputer.Display.println("s");

  M5Cardputer.Display.setCursor(4, 40);
  M5Cardputer.Display.print("Spots: ");
  M5Cardputer.Display.print(spotCount);
  if (qrzConfigured) {
    M5Cardputer.Display.print("   Range: ");
    if (anyDistKnown) {
      M5Cardputer.Display.print((int)minDist);
      M5Cardputer.Display.print("-");
      M5Cardputer.Display.print((int)maxDist);
      M5Cardputer.Display.print("km");
    } else {
      M5Cardputer.Display.print("n/a");
    }
  }
  M5Cardputer.Display.println();

  M5Cardputer.Display.drawFastHLine(0, 52, M5Cardputer.Display.width(), DARKGREY);

  M5Cardputer.Display.setCursor(4, 56);
  M5Cardputer.Display.setTextColor(GREEN, BLACK);
  M5Cardputer.Display.println("Recent spotters:");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);

  int y = 68;
  if (recentSpotCount == 0) {
    M5Cardputer.Display.setCursor(4, y);
    M5Cardputer.Display.println("Waiting for first spot...");
  } else {
    for (int i = 0; i < recentSpotCount; i++) {
      M5Cardputer.Display.setCursor(4, y);
      M5Cardputer.Display.print(recentSpots[i].spotter);
      M5Cardputer.Display.print("  ");
      if (recentSpots[i].bandIdx >= 0) {
        M5Cardputer.Display.print(BANDS[recentSpots[i].bandIdx].name);
        M5Cardputer.Display.print("  ");
      }
      M5Cardputer.Display.print(recentSpots[i].snr);
      M5Cardputer.Display.println("dB");
      y += 10;
    }
  }

  M5Cardputer.Display.setCursor(4, 122);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.println("Call CQ now!  G0 = Stop");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

void drawResultScreen() {
  drawHeader("Result");
  int y = 30;
  M5Cardputer.Display.setCursor(4, y); y += 12;
  M5Cardputer.Display.print(myCall);
  M5Cardputer.Display.print("  Spots: ");
  M5Cardputer.Display.println(spotCount);

  M5Cardputer.Display.setCursor(4, y); y += 12;
  if (spotCount == 0) {
    M5Cardputer.Display.println("No spots on RBN.");
  } else {
    M5Cardputer.Display.println("Bands:");
    M5Cardputer.Display.setCursor(4, y); y += 12;
    String line = "";
    for (int i = 0; i < NUM_BANDS; i++) {
      if (bandCounts[i] > 0) {
        line += BANDS[i].name;
        line += "(";
        line += bandCounts[i];
        line += ") ";
        if (line.length() > 26) {
          M5Cardputer.Display.println(line);
          M5Cardputer.Display.setCursor(4, (M5Cardputer.Display.getCursorY()));
          line = "";
        }
      }
    }
    if (line.length() > 0) M5Cardputer.Display.println(line);

    if (qrzConfigured) {
      y = M5Cardputer.Display.getCursorY() + 4;
      M5Cardputer.Display.setCursor(4, y); y += 12;
      if (anyDistKnown) {
        M5Cardputer.Display.print("Range: ");
        M5Cardputer.Display.print((int)minDist);
        M5Cardputer.Display.print(" - ");
        M5Cardputer.Display.print((int)maxDist);
        M5Cardputer.Display.println(" km");
      } else {
        M5Cardputer.Display.println("Range: n/a");
      }
    }
  }
  M5Cardputer.Display.setCursor(4, 122);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  if (sessionSpotterCoordCount > 0) {
    M5Cardputer.Display.println("ENTER = new check  G0 = map");
  } else {
    M5Cardputer.Display.println("ENTER = new check");
  }
}

void drawWorldMap() {
  drawHeader("World Map");
  int mapX = 2, mapY = 26;
  int mapW = M5Cardputer.Display.width() - 4;
  int mapH = M5Cardputer.Display.height() - 26 - 12;

  // land points from the real pixel-map data, drawn as a dot cloud
  uint16_t landColor = M5Cardputer.Display.color565(90, 130, 100);
  for (int i = 0; i < LAND_POINT_COUNT; i++) {
    float lat = LAND_POINTS[i][0] / 10.0f;
    float lon = LAND_POINTS[i][1] / 10.0f;
    int x, y;
    projectLatLon(lat, lon, mapX, mapY, mapW, mapH, x, y);
    M5Cardputer.Display.drawPixel(x, y, landColor);
  }

  // your own location, green with a black outline for contrast
  if (myLocKnown) {
    int x, y;
    projectLatLon(myLat, myLon, mapX, mapY, mapW, mapH, x, y);
    M5Cardputer.Display.fillCircle(x, y, 3, BLACK);
    M5Cardputer.Display.fillCircle(x, y, 2, GREEN);
  }

  // every spotter heard this session, red with a black outline
  for (int i = 0; i < sessionSpotterCoordCount; i++) {
    int x, y;
    projectLatLon(sessionSpotterCoords[i].lat, sessionSpotterCoords[i].lon,
                  mapX, mapY, mapW, mapH, x, y);
    M5Cardputer.Display.fillCircle(x, y, 3, BLACK);
    M5Cardputer.Display.fillCircle(x, y, 2, RED);
  }

  M5Cardputer.Display.setCursor(4, M5Cardputer.Display.height() - 10);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.println("G0 = back to result");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

void drawRbnErrorScreen() {
  drawHeader("Error");
  M5Cardputer.Display.setCursor(4, 34);
  M5Cardputer.Display.println(statusMsg);
  M5Cardputer.Display.setCursor(4, 70);
  M5Cardputer.Display.setTextColor(DARKGREY, BLACK);
  M5Cardputer.Display.println("ENTER = back");
}

// ---------------------------------------------------------------------
// Wi-Fi logic
// ---------------------------------------------------------------------
void scanWifiNetworks() {
  state = ST_WIFI_SCANNING;
  drawWifiScanningScreen();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();

  networkCount = 0;
  if (n > 0) {
    int limit = (n > 64) ? 64 : n;
    bool used[64];
    for (int i = 0; i < limit; i++) used[i] = false;

    while (networkCount < MAX_NETWORKS_SHOWN) {
      int bestIdx = -1;
      int32_t bestRssi = -1000;
      for (int i = 0; i < limit; i++) {
        if (used[i]) continue;
        if (WiFi.RSSI(i) > bestRssi) { bestRssi = WiFi.RSSI(i); bestIdx = i; }
      }
      if (bestIdx < 0) break;
      used[bestIdx] = true;

      String s = WiFi.SSID(bestIdx);
      if (s.length() == 0) continue; // hidden SSID -> use manual entry instead

      bool dup = false;
      for (int k = 0; k < networkCount; k++) {
        if (networks[k].ssid == s) { dup = true; break; }
      }
      if (dup) continue;

      networks[networkCount].ssid = s;
      networks[networkCount].rssi = bestRssi;
      networks[networkCount].secure = (WiFi.encryptionType(bestIdx) != WIFI_AUTH_OPEN);
      networkCount++;
    }
  }
  WiFi.scanDelete();
  state = ST_WIFI_LIST;
  drawWifiListScreen();
}

void startWifiConnect(const String &ssid, const String &pass) {
  pendingSsid = ssid;
  pendingPass = pass;
  state = ST_WIFI_CONNECTING;
  drawWifiConnectingScreen(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  uint32_t t0 = millis();
  wl_status_t st;
  while (true) {
    st = WiFi.status();
    if (st == WL_CONNECTED) break;
    if (st == WL_CONNECT_FAILED) break;
    if (st == WL_NO_SSID_AVAIL) break;
    if (millis() - t0 > WIFI_CONNECT_TIMEOUT_MS) break;
    delay(200);
  }

  if (st == WL_CONNECTED) {
    // successful credentials, store persistently (NVS/flash)
    prefs.begin("rbncfg", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();

    state = ST_INPUT;
    drawInputScreen();
  } else {
    if (st == WL_CONNECT_FAILED) {
      statusMsg = "Wrong password?";
    } else if (st == WL_NO_SSID_AVAIL) {
      statusMsg = "Network not found.";
    } else {
      statusMsg = "Connection timed out.";
    }
    state = ST_WIFI_ERROR;
    drawWifiErrorScreen(ssid);
  }
}

// ---------------------------------------------------------------------
// Parse an RBN spot line
// Format: "DX de SPOTTER-#:   FREQ  DXCALL  MODE  SNR dB  SPD WPM  TYPE  TIMEZ"
// ---------------------------------------------------------------------
void handleSpotLine(char* line) {
  if (strncmp(line, "DX de", 5) != 0) return;

  char* tokens[16];
  int n = 0;
  char* p = strtok(line, " \t");
  while (p != NULL && n < 16) {
    tokens[n++] = p;
    p = strtok(NULL, " \t");
  }
  if (n < 5) return;

  String spotter = String(tokens[2]);
  int dash = spotter.indexOf('-');
  if (dash > 0) spotter = spotter.substring(0, dash);
  else {
    int colon = spotter.indexOf(':');
    if (colon > 0) spotter = spotter.substring(0, colon);
  }

  float freq = atof(tokens[3]);
  String dxcall = String(tokens[4]);
  dxcall.toUpperCase();

  String want = myCall;
  want.toUpperCase();

  if (dxcall != want) return; // not our callsign

  spotCount++;
  int bIdx = bandIndexForFreq(freq);
  if (bIdx >= 0) bandCounts[bIdx]++;

  int snr = 0;
  if (n > 6) snr = atoi(tokens[6]); // tokens: ... MODE SNR "dB" ...

  if (myLocKnown) {
    float sLat, sLon;
    if (resolveLocation(spotter, sLat, sLon)) {
      addMapPoint(sLat, sLon);
      float d = distanceKm(myLat, myLon, sLat, sLon);
      if (d >= 0.0f && d <= 20020.0f) { // sanity check: max possible distance on Earth
        if (d < minDist) minDist = d;
        if (d > maxDist) maxDist = d;
        anyDistKnown = true;
        checkAndUpdateHighscore(bIdx, d, spotter);
      }
    }
  }

  pushRecentSpot(spotter, bIdx, snr);
  displayDirty = true;

  // short beep to signal a new spot (unless muted in settings)
  if (!soundMuted) M5Cardputer.Speaker.tone(BEEP_FREQ_HZ, BEEP_MS);
}

void pollTelnetData() {
  while (client.available()) {
    char c = client.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) handleSpotLine(lineBuf);
      lineLen = 0;
    } else {
      if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
    }
  }
}

void startRbnConnection() {
  resetResults();

  saveLastCall(myCall);

  myLocKnown = false;
  if (qrzConfigured) {
    statusMsg = "Looking up your location...";
    state = ST_RBN_CONNECTING;
    drawConnectingScreen();
    myLocKnown = resolveLocation(myCall, myLat, myLon);
  }

  state = ST_RBN_CONNECTING;
  statusMsg = "Connecting to RBN...";
  drawConnectingScreen();

  if (!client.connect(RBN_HOST, RBN_PORT, HTTP_TIMEOUT_MS)) {
    statusMsg = "RBN not reachable.";
    state = ST_RBN_ERROR;
    drawRbnErrorScreen();
    return;
  }
  client.setTimeout(HTTP_TIMEOUT_MS / 1000); // cap any blocking read on this socket too

  // briefly drain the banner/login prompt, then send the callsign as login
  uint32_t tb = millis();
  while (millis() - tb < 1500) {
    while (client.available()) client.read();
    delay(50);
  }
  client.print(myCall);
  client.print("\r\n");

  listenStartMs = millis();
  lineLen = 0;
  state = ST_LISTENING;
}

// ---------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setTextSize(1);

  initSdCard();

  prefs.begin("rbncfg", true);
  String savedSsid = prefs.getString("ssid", "");
  String savedPass = prefs.getString("pass", "");
  qrzUser = prefs.getString("qrzuser", "");
  qrzPass = prefs.getString("qrzpass", "");
  soundMuted = prefs.getBool("muted", false);
  prefs.end();
  myCall = loadLastCall();
  loadHighscores();
  qrzConfigured = (qrzUser.length() > 0 && qrzPass.length() > 0);

  if (savedSsid.length() > 0) {
    startWifiConnect(savedSsid, savedPass); // try the saved credentials first
  } else {
    scanWifiNetworks(); // no saved credentials -> scan right away
  }
}

void loop() {
  M5Cardputer.update();

  // Physical button (G0 / BtnA): short press = back/cancel,
  // hold 1.5s (only on the callsign screen) = open the settings menu
  bool escPressed = M5Cardputer.BtnA.wasPressed();
  bool g0Held = M5Cardputer.BtnA.pressedFor(G0_HOLD_MS);
  if (!M5Cardputer.BtnA.isPressed()) g0HoldTriggered = false;

  bool kbChanged = M5Cardputer.Keyboard.isChange();
  bool enterPressed = false;
  String typed = "";
  bool delPressed = false;

  if (kbChanged && M5Cardputer.Keyboard.isPressed()) {
    Keyboard_Class::KeysState st = M5Cardputer.Keyboard.keysState();
    for (auto ch : st.word) typed += ch;
    enterPressed = st.enter;
    delPressed   = st.del;
  }

  switch (state) {

    // ---------------- Wi-Fi ----------------
    case ST_WIFI_SCANNING: {
      // handled synchronously inside scanWifiNetworks()
      break;
    }

    case ST_WIFI_LIST: {
      if (escPressed) { scanWifiNetworks(); break; }
      if (typed.length() > 0) {
        char c = typed[0];
        if (c == 'm' || c == 'M') {
          textInputBuffer = "";
          state = ST_WIFI_SSID_INPUT;
          drawTextInputScreen("Wi-Fi Name", "Enter SSID:", textInputBuffer, false);
        } else if (c >= '1' && c <= '9' && (c - '1') < networkCount) {
          int idx = c - '1';
          pendingSsid = networks[idx].ssid;
          if (networks[idx].secure) {
            textInputBuffer = "";
            state = ST_WIFI_PASS_INPUT;
            drawTextInputScreen("Wi-Fi Password", pendingSsid, textInputBuffer, true);
          } else {
            startWifiConnect(pendingSsid, "");
          }
        }
      }
      break;
    }

    case ST_WIFI_SSID_INPUT: {
      if (typed.length() > 0) textInputBuffer += typed;
      if (delPressed && textInputBuffer.length() > 0) {
        textInputBuffer.remove(textInputBuffer.length() - 1);
      }
      if (escPressed) {
        state = ST_WIFI_LIST;
        drawWifiListScreen();
        break;
      }
      if (enterPressed && textInputBuffer.length() > 0) {
        pendingSsid = textInputBuffer;
        textInputBuffer = "";
        state = ST_WIFI_PASS_INPUT;
        drawTextInputScreen("Wi-Fi Password", pendingSsid, textInputBuffer, true);
      } else if (kbChanged) {
        drawTextInputScreen("Wi-Fi Name", "Enter SSID:", textInputBuffer, false);
      }
      break;
    }

    case ST_WIFI_PASS_INPUT: {
      if (typed.length() > 0) textInputBuffer += typed;
      if (delPressed && textInputBuffer.length() > 0) {
        textInputBuffer.remove(textInputBuffer.length() - 1);
      }
      if (escPressed) {
        state = ST_WIFI_LIST;
        drawWifiListScreen();
        break;
      }
      if (enterPressed) {
        startWifiConnect(pendingSsid, textInputBuffer);
      } else if (kbChanged) {
        drawTextInputScreen("Wi-Fi Password", pendingSsid, textInputBuffer, true);
      }
      break;
    }

    case ST_WIFI_CONNECTING: {
      // handled synchronously inside startWifiConnect()
      break;
    }

    case ST_WIFI_ERROR: {
      if (enterPressed) {
        startWifiConnect(pendingSsid, pendingPass); // retry
      } else if (escPressed) {
        scanWifiNetworks();
      }
      break;
    }

    // ---------------- Settings ----------------
    case ST_SETTINGS_MENU: {
      if (escPressed) {
        state = ST_INPUT;
        drawInputScreen();
        break;
      }
      if (typed.length() > 0) {
        char c = typed[0];
        if (c == '1') {
          scanWifiNetworks();
        } else if (c == '2') {
          textInputBuffer = qrzUser; // prefill so ENTER alone keeps it
          state = ST_QRZ_USER_INPUT;
          drawTextInputScreen("QRZ Login", "QRZ username:", textInputBuffer, false);
        } else if (c == '3') {
          qrzUser = "";
          qrzPass = "";
          qrzConfigured = false;
          qrzSessionKey = "";
          prefs.begin("rbncfg", false);
          prefs.putString("qrzuser", "");
          prefs.putString("qrzpass", "");
          prefs.end();
          state = ST_INPUT;
          drawInputScreen();
        } else if (c == '4') {
          state = ST_HIGHSCORE_VIEW;
          drawHighscoreScreen();
        } else if (c == '5') {
          toggleMute();
          drawSettingsMenuScreen();
        } else if (c == '6') {
          testQrzLogin();
        }
      }
      break;
    }

    case ST_QRZ_TESTING: {
      // handled synchronously inside testQrzLogin()
      break;
    }

    case ST_QRZ_TEST_RESULT: {
      if (enterPressed) {
        state = ST_SETTINGS_MENU;
        drawSettingsMenuScreen();
      }
      break;
    }

    case ST_HIGHSCORE_VIEW: {
      if (escPressed) {
        state = ST_SETTINGS_MENU;
        drawSettingsMenuScreen();
        break;
      }
      if (typed.length() > 0 && (typed[0] == 'c' || typed[0] == 'C')) {
        clearHighscores();
        drawHighscoreScreen();
      }
      break;
    }

    case ST_QRZ_USER_INPUT: {
      if (typed.length() > 0) textInputBuffer += typed;
      if (delPressed && textInputBuffer.length() > 0) {
        textInputBuffer.remove(textInputBuffer.length() - 1);
      }
      if (escPressed) {
        state = ST_SETTINGS_MENU;
        drawSettingsMenuScreen();
        break;
      }
      if (enterPressed) {
        pendingQrzUser = textInputBuffer;
        textInputBuffer = "";
        state = ST_QRZ_PASS_INPUT;
        drawTextInputScreen("QRZ Login", "QRZ password:", textInputBuffer, false);
      } else if (kbChanged) {
        drawTextInputScreen("QRZ Login", "QRZ username:", textInputBuffer, false);
      }
      break;
    }

    case ST_QRZ_PASS_INPUT: {
      if (typed.length() > 0) textInputBuffer += typed;
      if (delPressed && textInputBuffer.length() > 0) {
        textInputBuffer.remove(textInputBuffer.length() - 1);
      }
      if (escPressed) {
        state = ST_SETTINGS_MENU;
        drawSettingsMenuScreen();
        break;
      }
      if (enterPressed) {
        qrzUser = pendingQrzUser;
        qrzPass = textInputBuffer;
        qrzConfigured = (qrzUser.length() > 0 && qrzPass.length() > 0);
        qrzSessionKey = ""; // force a fresh login next time it's needed
        prefs.begin("rbncfg", false);
        prefs.putString("qrzuser", qrzUser);
        prefs.putString("qrzpass", qrzPass);
        prefs.end();
        if (qrzConfigured) {
          testQrzLogin();
        } else {
          state = ST_SETTINGS_MENU;
          drawSettingsMenuScreen();
        }
      } else if (kbChanged) {
        drawTextInputScreen("QRZ Login", "QRZ password:", textInputBuffer, false);
      }
      break;
    }

    // ---------------- Callsign / RBN ----------------
    case ST_INPUT: {
      if (g0Held && !g0HoldTriggered) {
        g0HoldTriggered = true;
        state = ST_SETTINGS_MENU;
        drawSettingsMenuScreen();
        break;
      }
      if (typed.length() > 0) {
        typed.toUpperCase();
        myCall += typed;
      }
      if (delPressed && myCall.length() > 0) myCall.remove(myCall.length() - 1);
      if (enterPressed && myCall.length() > 0) {
        startRbnConnection();
      } else if (kbChanged) {
        drawInputScreen();
      }
      break;
    }

    case ST_RBN_CONNECTING: {
      // handled synchronously inside startRbnConnection()
      break;
    }

    case ST_LISTENING: {
      pollTelnetData(); // sets displayDirty=true as soon as a matching spot arrives

      uint32_t elapsed = (millis() - listenStartMs) / 1000;
      static uint32_t lastDraw = 0;
      // redraw instantly on a new spot, otherwise once a second (clock ticks)
      if (displayDirty || millis() - lastDraw > 1000) {
        drawListeningScreen(elapsed);
        lastDraw = millis();
        displayDirty = false;
      }

      bool timeoutReached = (elapsed >= MAX_RUNTIME_SECONDS);
      if (escPressed || timeoutReached) {
        client.stop();
        state = ST_RESULT;
        drawResultScreen();
      }
      break;
    }

    case ST_RESULT: {
      if (enterPressed) {
        state = ST_INPUT;
        drawInputScreen();
      } else if (escPressed && sessionSpotterCoordCount > 0) {
        state = ST_MAP_VIEW;
        drawWorldMap();
      }
      break;
    }

    case ST_MAP_VIEW: {
      if (escPressed || enterPressed) {
        state = ST_RESULT;
        drawResultScreen();
      }
      break;
    }

    case ST_RBN_ERROR: {
      if (enterPressed) {
        state = ST_INPUT;
        drawInputScreen();
      }
      break;
    }
  }

  delay(10);
}
