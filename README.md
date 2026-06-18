# Weather display on ESP32 + ST7789 (narodmon.ru data)

A standalone ESP32 device that pulls readings from the nearest public sensors on the [narodmon.ru](https://narodmon.ru) project (temperature, humidity, pressure, wind, radiation, dust, precipitation), averages them with distance-based weighting, and displays them on a TFT screen as a carousel of large, easy-to-read screens.

Author: **xumbax**

![Temperature screen](images/screen_temperature.png)

## Why

I couldn't find a ready-made weather station that aggregates nearby public sensor data this way, and building my own full sensor set (thermometer, barometer, anemometer, dosimeter) felt redundant when dozens of public devices already report to narodmon nearby. The idea is to take several of the closest sensors of each type and average them with distance-based weighting, which is more robust to outliers than relying on a single random device.

## Features

- **8 parameters**: temperature, humidity, pressure, wind speed and direction, radiation level, air dust concentration, precipitation
- **Distance-weighted averaging (IDW)** — `w = 1 / (d + ε)^p`, closer sensors carry more weight; the number of sensors used, the decay exponent, and the search radius are all runtime settings, not hardcoded constants
- **9-screen carousel** — 8 parameter screens plus a diagnostic screen listing the IDs, distances, and data age of every sensor currently in use
- **Trend indicator** on every screen — 4 bars (3h / 1h / 20min / now), bar height shows the relative level of the value at that point in time; history is kept in RAM from the device's own measurements, with no extra requests to the server
- **Multi-device request scheduling** — period/offset based on minute-of-day (UTC), so several of these displays can run side by side without colliding on the 1 request/minute API limit
- **Automatic sensor type code discovery** via `appInit` — narodmon doesn't publish an official table of numeric `type` codes, so the firmware looks them up by name at startup instead of hardcoding guessed numbers
- **Alarms** when a parameter crosses a threshold (extreme cold/heat, storm winds, radiation, dust storm) — a dedicated alert screen, the LED stays solidly lit, and a buzzer beeps once every 15 minutes
- **LED-based connectivity diagnostics** — a single priority hierarchy: no WiFi → blinks once a second, WiFi up but API unreachable → once every 3 seconds, API reachable but no sensors found → once every 5 seconds, everything fine → LED off
- **Self-healing reboot** — if the device has been stuck in a problem state (WiFi/API/sensors) for 30+ minutes straight, it reboots itself

## Screens

| Parameter screen | Diagnostic screen | Alarm |
|---|---|---|
| ![temperature](images/screen_temperature.png) | ![diagnostic](images/screen_service.png) | ![alarm](images/screen_alarm.png) |

*These images are illustrative mockups of the screen layout, not photos of an actual unit.*

## Hardware

- ESP32 DevKit (any WROOM-32 based board)
- 2.0" ST7789 TFT display, 240×320, SPI
- LED + 220 Ω resistor
- 5V active piezo buzzer

Full wiring diagram, shopping list, and `TFT_eSPI` library setup are in [docs/setup_guide.html](docs/setup_guide.html).

## Setup

1. Arduino IDE + ESP32 board package
2. Libraries: `TFT_eSPI` (Bodmer), `ArduinoJson` v6 (Benoit Blanchon), `NTPClient` (Fabrice Weinberg)
3. Configure `User_Setup.h` in the `TFT_eSPI` library for your display's pinout (details in the guide)
4. Open `weather_display.ino` and fill in:
   - `WIFI_SSID`, `WIFI_PASS`
   - `NM_API_KEY` — obtained from narodmon.ru → Profile → My applications
   - `MY_LAT`, `MY_LON` — coordinates of your observation point
5. Flash, then open Serial Monitor at 115200 baud for diagnostics

## narodmon API limits

This project uses the `sensorsNearby` method to request several of the nearest public sensors at once. Under narodmon's rules, accessing data from **more than 3 other people's public sensors** requires approval from their technical support team (a request describing how the data is used). By default this device uses up to 5 nearest sensors per parameter across 8 parameter types — the `NEAREST_COUNT` setting in the code can be lowered to 3 to run without separate approval, or you can contact narodmon support with a description of the project (the diagnostic screen is a handy illustration of exactly how the data is used).

Requests are sent no more often than once per minute per key — the API rate limit is respected via the `REQUEST_PERIOD_MIN` setting.

## License

MIT — use, modify, and distribute freely.

## Acknowledgments

Data provided by the [narodmon.ru](https://narodmon.ru) project — a crowdsourced network of weather and environmental sensors.
