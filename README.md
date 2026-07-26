# RBN CW Checker for M5Cardputer

A ham radio gadget for the M5Stack Cardputer (ESP32-S3) that checks
whether your callsign is being heard on air right now, using the
[Reverse Beacon Network](https://www.reversebeacon.net) (RBN) live
telnet feed. Optionally uses your QRZ.com XML subscription to compute
real distances, log per-band DX records, and plot every spotter on a
tiny world map.

## What it does

1. Enter your callsign on the Cardputer's keyboard.
2. The device connects live to `telnet.reversebeacon.net:7000` (the
   official RBN feed for CW/RTTY skimmer spots).
3. Call CQ. Every time a skimmer spots you, the screen updates
   instantly: running spot count, current band breakdown, and the 5
   most recent spotting stations (callsign, band, SNR). A short beep
   plays on every new spot.
4. Stop anytime with the physical **G0** button to see a summary
   (spots, bands, range) — and, if QRZ is configured, a small dot map
   of every station that heard you.

**Important:** this is a *live* check, not a lookup of past spots. RBN
only streams spots from the moment you connect onward, so you need to
actually be on the air (or have someone else call CQ using your
callsign check) for anything to show up.

## Features

- **Wi-Fi setup built in** — scans for networks, lets you pick one and
  enter the password (or enter an SSID manually for hidden networks).
  Credentials are saved to flash (NVS) and reused automatically on the
  next boot. Clear error messages for wrong password / network not
  found / timeout, with retry.
- **Optional QRZ.com integration** — enter your QRZ XML subscription
  username + password once (Settings menu) to get real coordinates for
  your callsign and every spotting station, giving an accurate min/max
  range in km. Without QRZ configured, the range line is simply left
  out (no rough guessing). A built-in connection test confirms your
  QRZ login works, with QRZ's own error message shown on failure.
  Lookups are cached in memory per session so repeat spotters (a small,
  recurring set of skimmer stations) aren't re-queried.
- **Live spotter list** — the 5 most recent stations that heard you,
  updated the instant a new spot arrives, not on a timer.
- **Per-band highscore** — the farthest contact ever logged per band is
  saved automatically (to a **microSD card** if one is inserted, for
  much better write endurance than internal flash; falls back to
  internal flash otherwise, with automatic one-time migration when a
  card is later added). Viewable and clearable from the Settings menu.
- **World map** — plots every spotter from the current session as a
  dot over a small world map, built from ~1,600 real land-mask points
  (derived from a pixel map SVG, converted to lat/lon and calibrated
  against several countries' known coordinates).
- **Battery indicator** — percentage shown top-right on every screen,
  turns red when low and not charging.
- **Sound mute toggle**, **last-used callsign remembered** (and
  restored on next boot), automatic **uppercasing** while typing your
  callsign.
- **Safety timeout** (30 minutes) so a live session doesn't stay open
  forever if you forget to stop it.

## Hardware

- M5Stack **Cardputer** or **Cardputer-Adv** (ESP32-S3, built-in
  keyboard, speaker, battery, and microSD slot)
- microSD card (optional, FAT32-formatted) for highscore/last-callsign
  storage with better write endurance than internal flash
- Wi-Fi network with internet access
- Optional: a QRZ.com XML Data subscription, for real distances

## Required libraries

Install via the Arduino IDE Library Manager / Board Manager:

- **M5Cardputer** (`m5stack/M5Cardputer`)
- `WiFi`, `WiFiClientSecure`, `HTTPClient`, `Preferences`, `SPI`, `SD` —
  all included in the ESP32 Arduino core, nothing extra to install

## Flashing

1. Install the Arduino IDE and the M5Stack board package, then install
   the M5Cardputer library (see M5Stack's own setup docs for details).
2. Open `rbn_cardputer.ino`.
3. Select the Cardputer / Cardputer-Adv board and the correct port.
4. Compile and upload.

No credentials need to be edited in the source before flashing — both
Wi-Fi and QRZ login are entered on the device itself and saved to
flash.

## Controls

| Action | Effect |
|---|---|
| Type on keyboard | Enter callsign / SSID / password / etc. |
| **ENTER** | Confirm the current screen |
| **G0** (short press) | Back / cancel / stop the live session |
| **G0** (hold ~1.5s, on the callsign screen) | Open Settings |

### Settings menu

1. Wi-Fi setup (rescan and reconnect / switch networks)
2. QRZ login (enter username + password; runs a connection test
   immediately after saving)
3. Remove QRZ login
4. Highscore (view per-band DX records; press `c` to clear all)
5. Sound (toggle mute on/off)
6. Test QRZ login (re-run the connection test anytime)

## Notes and limitations

- **Range calculation** requires a QRZ.com XML subscription
  (username + password, not a static API key — that's a different QRZ
  product used for logbook access). Coordinates are validated
  (plausible lat/lon range, rejecting "null island" 0,0) before being
  used, to avoid bogus distances from a corrupted lookup.
- **TLS certificate checking is disabled**
  (`WiFiClientSecure::setInsecure()`) for QRZ HTTPS requests, to keep
  things simple on the ESP32. This is a reasonable trade-off for a
  hobby project but is technically less secure than full certificate
  validation.
- **World map** land points are an approximation: real pixel-map data
  converted through a calibrated linear projection, not surveyed
  cartographic data. Good enough to recognize continents at
  240×135 resolution; not measurement-grade.
- **microSD pin mapping** (`SCK=40, MISO=39, MOSI=14, CS=12`) is per
  M5Stack's official documentation, listed as applicable to both
  Cardputer and Cardputer-Adv. If your specific unit doesn't detect the
  card, double-check it's FAT32-formatted before assuming a pin issue.
- Several M5Unified/M5GFX API calls (`M5Cardputer.BtnA.pressedFor()`,
  `M5Cardputer.Speaker.tone()`, `M5.Power.getBatteryLevel()`) are used
  as documented for recent library versions; if your installed version
  differs slightly, the compiler will point to the exact line.

## Credits

- [Reverse Beacon Network](https://www.reversebeacon.net) for the live
  spot feed.
- [QRZ.com](https://www.qrz.com) XML Data API for callsign coordinates.
- World map land points derived from a pixel map SVG generated with
  [amCharts' Pixel Map Generator](https://pixelmap.amcharts.com).
