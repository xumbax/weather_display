/*
 * ============================================================
 *  WEATHER DISPLAY — ESP32 + ST7789 (240x320)
 *  v4 — narodmon (oldest-first rotation) + Яндекс.Погода (REST API v2) +
 *       веб-интерфейс (настройки/датчики/логи) + WiFi AP fallback
 *
 *  DATA SOURCES:
 *    1. narodmon.ru sensorsNearby  — discovers sensor IDs, individual
 *       per-type radius (10..100 km, +15km hops, stops once 2 sensors
 *       found per type; a value is shown on screen already at 1 sensor)
 *    2. narodmon.ru sensorsValues  — picks 3 OLDEST-data sensors among
 *       all known (up to 24 = 3 per type × 8 types), with a
 *       5-minute-per-sensor requery guard
 *    3. Яндекс.Погода REST API v2 (free "smart home" tier) — current
 *       weather (fact) + nearest hourly forecast to "+2h from request
 *       time" + 2-day forecast (today + tomorrow only, per the tier's
 *       documented limit)
 *
 *  CAROUSEL (6 screens):
 *    0   Temp/Humidity (narodmon)       — large T block + H block
 *    1   Wind/Direction/Pressure        — 3 blocks (narodmon); direction
 *        shows current + 1h vector average side by side, plus
 *        24h/12h/6h prevailing-direction text instead of a trend bar
 *    2   Air Quality/Radiation/Precip   — 3 blocks (narodmon)
 *    3   Yandex: at request time + nearest hour to +2h from then
 *    4   Yandex: 2-day forecast (Today/Tomorrow)
 *    5   Diagnostics screen
 *  (no separate full-screen alarm screen — on threshold breach, only
 *  that parameter's number turns red, LED lights solid while an alarm
 *  screen is shown, buzzer beeps once per screen display, throttled
 *  to once every ALARM_BEEP_INTERVAL_SEC across all alarms combined)
 *
 *  TOUCH BUTTON (TTP223): forces screen 0, opens a 30s manual-nav
 *  window; further taps step through screens and reset the window;
 *  after 30s of inactivity the normal carousel resumes from wherever
 *  it was left.
 *
 *  NOTE on TFT_eSPI's built-in font: it has no Cyrillic glyphs, so all
 *  on-screen text is English. Serial Monitor logs stay in Russian.
 *
 *  Libraries: TFT_eSPI (Bodmer), ArduinoJson v6, NTPClient
 * ============================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUDP.h>
#include <MD5Builder.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <stdarg.h>

// ============================================================
//  SETTINGS
// ============================================================
// ВСЕ настройки в этом блоке — только значения "по умолчанию": реальные
// значения после первого сохранения через веб-интерфейс живут в NVS
// (флеш-память ESP32, переживает перепрошивку) и подгружаются в эти же
// переменные функцией loadConfig() в начале setup(). Если в NVS ещё
// ничего не сохранено (первая прошивка "с нуля") — используются именно
// эти значения. Поэтому они больше не const — веб-интерфейс их меняет
// в рантайме.
String WIFI_SSID       = "YOUR_WIFI";
String WIFI_PASS       = "YOUR_PASSWORD";
String NM_UUID_SRC     = "weather-display-01";
String NM_API_KEY      = "YOUR_API_KEY";

// Яндекс.Погода — бесплатный тариф "для умного дома"
String YANDEX_WEATHER_KEY = "YOUR_YANDEX_WEATHER_KEY";

double MY_LAT = 0.0;   // your latitude  (e.g. 55.7558)
double MY_LON = 0.0;   // your longitude (e.g. 37.6173)

// Display-only timezone offset for the on-screen clock (hours). Does NOT
// affect any internal calculations — those stay in UTC to match narodmon's
// "time" field and the request scheduling logic.
int TZ_DISPLAY_OFFSET_HOURS = 3; // e.g. 3 for Moscow (UTC+3)

// Narodmon sensor discovery
int    BASE_SEARCH_RADIUS = 10;   // km, starting radius for every type
int    MAX_SEARCH_RADIUS  = 100;  // km, hard ceiling
int    RADIUS_STEP_KM     = 15;   // km, шаг расширения за один "хоп"
int    MIN_SENSORS_TO_STOP_EXPAND = 2; // расширяемся, пока не наберём это число
int    MIN_SENSORS_NEEDED = 1;    // минимум датчиков для показа значения; N/D только при 0
int    NEAREST_COUNT      = 3;    // sensors per type to track (max for display)

// Narodmon rotation scheduling (minute-of-day modulo). Period=1/offset=0
// means "every minute" — these only matter if you run several devices
// under one IP and want to stagger their requests to respect the
// 1-request-per-minute-per-client API limit (e.g. period=2,offset=1 for
// a second device would make it request on odd minutes only).
int    REQUEST_PERIOD_MIN = 1;
int    REQUEST_OFFSET_MIN = 0;

// Sensor freshness
int    FRESH_MINUTES    = 60;   // discard narodmon readings older than this — поднято с 30,
// так как некоторые соседские датчики (особенно ветер/направление/осадки)
// сами по себе обновляются редко, раз в 25-30+ минут на своей стороне —
// при пороге 30 минут это давало систематические N/D даже при честной
// попытке переопроса с нашей стороны (см. разбор в чате 24.06.2026)

// Минимальный интервал повторного запроса ОДНОГО И ТОГО ЖЕ датчика
int MIN_REQUERY_INTERVAL_MIN = 5;

// IDW averaging
float  IDW_POWER        = 1.0;
float  IDW_EPS          = 0.1;  // km, prevents division by zero

// Проверка на "сломанный нулевой" датчик: если в группе 2+ датчика, и один
// показывает РОВНО 0, а другой отличается от него больше чем на этот порог —
// нулевой считается сломанным и отбрасывается из усреднения
float  ZERO_SENSOR_DIFF_THRESHOLD = 1.0;

// Carousel timing
int DELAY_SERVICE_SEC  = 4;  // служебный экран — короче остальных
int DELAY_NORMAL_SEC   = 8;  // обычные экраны без алерта
int DELAY_ALARM_SEC    = 12; // экран с активным алертом — дольше обычного

// Alarms (по отдельным параметрам — красная цифра, не весь экран)
// ВАЖНО про радиацию: датчики narodmon отдают значение в mR/h (милли-
// рентген/час), не в мкЗв/ч — используем число как есть, без пересчёта,
// поэтому порог тоже в этих же единицах (был 0.30 для мкЗв/ч, теперь
// умножен на 100, как и сами показания датчиков).
float ALARM_RADIATION_MRH  = 30.0;   // mR/h
float ALARM_WIND_MS        = 20.0;
float ALARM_TEMP_COLD_C    = -30.0;
float ALARM_TEMP_HOT_C     = 35.0;
float ALARM_DUST_UGM3      = 150.0;
float ALARM_PRESS_LOW_MMHG = 660.0;
float ALARM_PRESS_HIGH_MMHG= 800.0;
float ALARM_HUM_HIGH_PCT   = 99.0;

// LED + buzzer
int PIN_LED    = 26;
int PIN_BUZZER = 25;
int PIN_TOUCH_BUTTON = 27; // TTP223, цифровой выход HIGH при касании
unsigned long TOUCH_OVERRIDE_TIMEOUT_MS = 30UL * 1000; // 30 секунд
int ALARM_BEEP_INTERVAL_SEC = 15 * 60;
int ALARM_BEEP_DURATION_MS  = 300;

// Watchdog: reboot if stuck in problem state > 30 min
unsigned long REBOOT_AFTER_STUCK_SEC = 30 * 60;

// Тренд: см. TREND_SLOT_MINUTES/TREND_SLOT_COUNT в блоке тренд-буфера ниже
// (это ФИКСИРОВАННЫЕ размеры массивов на этапе компиляции — их нельзя
// сделать редактируемыми через веб без перехода на динамическое
// выделение памяти, поэтому в веб-интерфейсе их нет)

// ============================================================
//  WIFI FALLBACK / ВЕБ-ИНТЕРФЕЙС
//  Если за WIFI_CONNECT_TIMEOUT_MIN минут ПОСЛЕ ВКЛЮЧЕНИЯ устройство ни
//  разу не смогло подключиться к сохранённой сети — поднимаем собственную
//  точку доступа (AP_SSID/AP_PASSWORD), на экране показываем инструкцию.
//  Это СТРОГО одноразовый сценарий начальной настройки: как только
//  устройство хоть раз успешно подключилось после старта — дальнейшие
//  обрывы связи (даже надолго) обрабатываются как обычный сбой сети,
//  бесконечным ретраем, БЕЗ повторного ухода в AP — до следующей
//  физической перезагрузки. WEB_ADMIN_PASSWORD, если не пустой, включает
//  HTTP Basic Auth (логин "admin") на всех страницах веб-интерфейса —
//  рекомендуется задать, т.к. на страницах видны WiFi-пароль и API-ключи.
// ============================================================
int    WIFI_CONNECT_TIMEOUT_MIN = 10;
String AP_SSID            = "WeatherDisplay-Setup";
String AP_PASSWORD        = "12345678"; // мин. 8 символов для WPA2 (WiFi.softAP)
String WEB_ADMIN_PASSWORD = "";         // пусто = без пароля на веб-интерфейс

// ============================================================
//  SENSOR TYPE DEFINITIONS (narodmon)
// ============================================================
enum SensorIndex {
  S_TEMP=0, S_HUM, S_PRESS, S_WIND_SPEED, S_WIND_DIR,
  S_RADIATION, S_DUST, S_PRECIP, S_COUNT
};

struct SensorTypeConfig {
  const char* searchName;   // name to match in appInit types[]
  const char* displayName;  // shown on screen (service screen, logs)
  const char* shortLabel;   // короткая подпись параметра возле крупной цифры
  const char* unit;
  int   typeCode;           // filled by loadSensorTypeCodes()
};

SensorTypeConfig sensorTypes[S_COUNT] = {
  { "t воздуха",    "TEMP",     "Temp",  "C"     },
  { "RH влажность", "HUMIDITY", "Wet",   "%"     },
  { "атм.давление", "PRESSURE", "Press", "mmHg"  },
  { "скорость",     "WIND",     "Wind",  "m/s"   },
  { "направление",  "DIR",      "Dir",   ""      }, // направление выводится словами (N/NE/E...), без размерности
  { "радиация",     "RADIATION","Rad",   "mR/h"  }, // датчики narodmon отдают mR/h, не uSv/h — используем как есть, без пересчёта
  { "запыленность", "AIR QUAL", "Air",   "ug/m3" },
  { "осадки",       "PRECIP",   "Rain",  "mm"    },
};

// ============================================================
//  NARODMON SENSOR ROSTER
// ============================================================
const int MAX_ROSTER = 5;   // >= NEAREST_COUNT

struct RosterEntry {
  long  id;
  float distance;
};

RosterEntry sensorRoster[S_COUNT][MAX_ROSTER];
int         rosterCount[S_COUNT] = {0};
int         searchRadiusByType[S_COUNT];

// ============================================================
//  NARODMON READING CACHE
// ============================================================
struct NmReading {
  float value;
  long  time;          // unix UTC показания от сервера; 0 = ещё не получали
  float dist;           // km, for IDW weight
  long  lastQueriedTs;  // unix UTC момента, когда МЫ последний раз запросили
  bool  excludedAsZero; // true если этот датчик отброшен как "сломанный ноль"
};

NmReading nmCache[S_COUNT][MAX_ROSTER];

// ============================================================
//  COMPUTED RESULTS (народмон только, IDW-усреднение)
// ============================================================
struct SensorResult {
  bool  valid       = false;
  float value       = 0;
  int   nmCount     = 0;
  float nearestDist = 0;
  bool  inAlarm     = false; // этот конкретный параметр сейчас в алерте?
};

SensorResult results[S_COUNT];

// Forward declarations (определены ниже в файле, но используются раньше)
void checkPerParameterAlarms();
void drawCarouselDots(int activeIdx);
bool screenHasVisibleAlarm(int idx);
void logPrintln(const String& s);
void logPrintf(const char* fmt, ...);
// ============================================================
//  ТРЕНД-СЛОТЫ — фиксированные временные метки кратные 45 минутам от
//  начала суток UTC (00:00, 00:45, 01:30, ...), а не "скользящее окно
//  от текущего момента". Это устраняет рассинхрон между реальным
//  аптаймом устройства и числом видимых столбиков: раньше тренд искал
//  точку, ближайшую к "сейчас минус N×15 минут", и если устройство
//  работало меньше расчётной глубины (7.5ч на 30 слотов), часть
//  слотов слева физически не могла быть покрыта реальными данными —
//  то есть число "живых" столбиков на экране оказывалось МЕНЬШЕ
//  ожидаемого, и тем более могло заметно отставать от количества
//  реально полученных от narodmon показаний.
//
//  Новая схема: каждому параметру соответствует кольцевой буфер на
//  TREND_SLOT_COUNT=32 слота. При получении нового значения вычисляем
//  номер слота = (unix_время / (45*60)) — если это новый слот (не тот
//  же, что был последним записанным) — двигаем "голову" буфера и
//  записываем туда. Если тот же слот — обновляем значение в нём
//  (берём последнее известное значение за эти 45 минут).
// ============================================================
const int TREND_SLOT_MINUTES = 45;
const int TREND_SLOT_COUNT   = 32; // 32 * 45 мин = 24 часа охвата

struct TrendSlot {
  long  slotIndex = -1; // unix_время/(45*60); -1 = слот ещё не использован
  float value = 0;
  bool  valid = false;
};

struct TrendBuffer {
  TrendSlot slots[TREND_SLOT_COUNT];
  int head = 0; // индекс САМОГО НОВОГО заполненного слота
  int filledCount = 0; // сколько слотов реально когда-либо заполнено (для лога/диагностики)
};

TrendBuffer trendBuf[S_COUNT];

// Добавить новое значение в тренд-буфер параметра idx. Сама определяет,
// попадает ли новое измерение в уже открытый текущий слот (обновляет
// его) или нужно открыть новый слот (сдвигает head по кругу).
void trendPush(int idx, long nowTs, float v) {
  long slotIdx = nowTs / ((long)TREND_SLOT_MINUTES * 60);
  TrendBuffer& buf = trendBuf[idx];

  if (buf.filledCount > 0 && buf.slots[buf.head].slotIndex == slotIdx) {
    // Тот же 45-минутный слот, что и в прошлый раз — просто обновляем значение
    buf.slots[buf.head].value = v;
    return;
  }

  // Новый слот — двигаем голову по кругу и заполняем
  buf.head = (buf.head + 1) % TREND_SLOT_COUNT;
  buf.slots[buf.head] = {slotIdx, v, true};
  if (buf.filledCount < TREND_SLOT_COUNT) buf.filledCount++;

  logPrintf("  [trend] %s: new slot opened, %d/%d filled\n",
    sensorTypes[idx].shortLabel, buf.filledCount, TREND_SLOT_COUNT);
}

// Для совместимости с остальным кодом (вызывается как раньше из computeResults)
void historyPush(int idx, long t, float v) {
  trendPush(idx, t, v);
}


// ============================================================
//  НАПРАВЛЕНИЕ ВЕТРА — почасовые векторы (вместо тренд-бара, который
//  для категориальной величины визуально бессмысленен и "дёргался").
//  Векторная сумма направлений за период вместо обычного среднего —
//  иначе 350° и 10° усреднились бы в 180° (юг), что физически неверно.
//  Оптимизация: считаем не по всей сырой истории каждый раз, а
//  накапливаем ОДИН вектор на ТЕКУЩИЙ час, и при переходе на новый
//  час "закрываем" его в кольцевой буфер на 24 часа. Запрос за
//  1ч/6ч/12ч/24ч — это просто сумма последних N закрытых часов
//  (+ текущий открытый для 1ч), O(N) вместо пересчёта всей истории.
// ============================================================
struct WindDirHourVector {
  float sumX = 0, sumY = 0; // сумма единичных векторов направления за этот час
  int   count = 0;          // сколько измерений вошло (для информации/отладки)
  bool  valid = false;
};

const int WIND_DIR_HOURS = 24;
WindDirHourVector windDirHourly[WIND_DIR_HOURS]; // кольцевой буфер закрытых часов
int  windDirHourlyHead = 0; // индекс следующего слота для записи (циклически)
long windDirCurrentHourStart = 0; // unix UTC начала ТЕКУЩЕГО (открытого) часа
WindDirHourVector windDirCurrentHour; // накопитель текущего часа, ещё не закрытого

// Добавить новое измерение направления ветра (вызывается из computeResults,
// синхронно с обновлением trendBuf[S_WIND_DIR]). Сама решает, нужно ли
// "закрыть" предыдущий час и начать новый.
void windDirAddSample(long nowTs, float degrees) {
  long hourStart = (nowTs / 3600) * 3600;

  if (windDirCurrentHourStart == 0) {
    windDirCurrentHourStart = hourStart; // первый запуск
  } else if (hourStart != windDirCurrentHourStart) {
    // Перешли в новый час — закрываем предыдущий накопитель в буфер
    windDirHourly[windDirHourlyHead] = windDirCurrentHour;
    windDirHourlyHead = (windDirHourlyHead + 1) % WIND_DIR_HOURS;
    windDirCurrentHour = WindDirHourVector(); // сброс накопителя
    windDirCurrentHourStart = hourStart;
  }

  float rad = radians(degrees);
  windDirCurrentHour.sumX += cos(rad);
  windDirCurrentHour.sumY += sin(rad);
  windDirCurrentHour.count++;
  windDirCurrentHour.valid = true;
}

// Преобладающее направление за последние N часов (включая текущий открытый
// час). Возвращает false если вообще нет данных за этот период.
bool windDirPrevailing(int hoursBack, float& outDegrees) {
  float sumX = windDirCurrentHour.sumX, sumY = windDirCurrentHour.sumY;
  bool any = windDirCurrentHour.valid;

  // Закрытые часы, от самого свежего назад: индекс (head-1), (head-2), ...
  for (int k = 0; k < hoursBack && k < WIND_DIR_HOURS; k++) {
    int idx = (windDirHourlyHead - 1 - k + WIND_DIR_HOURS) % WIND_DIR_HOURS;
    if (!windDirHourly[idx].valid) continue;
    sumX += windDirHourly[idx].sumX;
    sumY += windDirHourly[idx].sumY;
    any = true;
  }

  if (!any || (fabs(sumX) < 0.0001f && fabs(sumY) < 0.0001f)) return false;

  float deg = degrees(atan2(sumY, sumX));
  if (deg < 0) deg += 360.0f;
  outDegrees = deg;
  return true;
}

// ============================================================
//  YANDEX WEATHER — REST API v2, бесплатный тариф "для умного дома".
//  Подтверждённые рабочим логом поля: temp, feels_like, wind_speed,
//  condition (fact); почасовой forecasts[].hours[] для +2h-слота;
//  parts.day/night.temp_avg для 2-дневного прогноза.
// ============================================================
struct YandexNow {
  bool  valid = false;
  float temperature;
  float feelsLike;
  float windSpeed;
  String condition; // приходит от сервера на русском — на экране не выводится (нет кириллицы в шрифте)
  long  fetchedAtTs = 0; // unix UTC момента запроса — используется для подписи "AT HH:MM" на экране
};
YandexNow yandexNow;

struct YandexHour {
  bool  valid = false;
  long  time;       // unix UTC этого часового слота
  float temperature;
  float windSpeed;
  String condition;
};
// Слот "через 2 часа от текущего момента" — один, не массив,
// перевыбирается из ответа при каждом обновлении
YandexHour yandexPlus2h;

struct YandexDay {
  bool  valid = false;
  float tempDay;        // средняя дневная (parts.day.temp_avg)
  float tempNight;      // средняя ночная (parts.night.temp_avg)
};
YandexDay yandexDays[2]; // [0] = сегодня, [1] = завтра

unsigned long bootMillis = 0;
bool firstYandexFetchDone = false;
// Первый запрос к Яндексу — не сразу при включении, а через 5 минут
// (даёт время устройству спокойно подключиться к WiFi/NTP и не плодить
// лишний сетевой трафик в первые секунды после старта)
const unsigned long YANDEX_STARTUP_DELAY_MS = 5UL * 60 * 1000;
bool yandexReachable = false;

// ============================================================
//  АДАПТИВНОЕ РАСПИСАНИЕ запросов к Яндексу: вместо фиксированного
//  интервала — следующий запрос ровно за 30 минут до того, как
//  истекает актуальность текущего прогнозного слота "+2 часа"
//  (yandexPlus2h.time). Пример: запрос в 1:18 -> слот на 3:00 ->
//  следующий запрос в 2:30 -> новый слот на 4:00 -> запрос в 3:30...
//  Шаг между запросами получается ~1.5 часа (с учётом округления до
//  целого часа), это ~16 запросов/сутки — в пределах лимита 30/сутки.
//  При неудачном запросе (нет сети/ошибка) — повтор через 30 минут,
//  а не по обычному расписанию.
// ============================================================
const int YANDEX_PRE_EXPIRY_MIN = 30; // запрашиваем заранее, за столько минут
const int YANDEX_RETRY_AFTER_FAIL_MIN = 30; // повтор при неудаче — через сколько минут
long nextYandexFetchTs = 0; // unix UTC момента следующего запроса (0 = ещё не назначен)

// ============================================================
//  GLOBALS
// ============================================================
TFT_eSPI tft = TFT_eSPI();
WiFiUDP  ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);
String   nmUUID;

// Scheduling
long  lastRequestMinute   = -1;
long  lastNearbyHour      = -1;  // hour-of-day of last sensorsNearby call (0/6/12/18)

unsigned long lastFastExpandCheck = 0;
const unsigned long FAST_EXPAND_INTERVAL_MS = 2UL * 60 * 1000; // 2 минуты

// Carousel
int  carouselIndex = 0;
unsigned long lastCarouselSwitch = 0;

// TTP223 сенсорная кнопка: принудительное управление каруселью.
// При касании — переход на следующий экран (или экран 0, если override
// ещё не активен) и 30-секундный таймер; новое касание внутри этого окна
// двигает дальше и сбрасывает таймер. По истечении 30с без касаний —
// обычная автокарусель продолжает с текущего экрана.
bool touchOverrideActive = false;
unsigned long touchOverrideUntil = 0;
bool lastTouchPinState = false; // для детектирования фронта (debounce)
unsigned long lastTouchChangeMs = 0;
const unsigned long TOUCH_DEBOUNCE_MS = 50;
const int TOTAL_SCREENS = 6; // см. список экранов в шапке файла

// Alarm + LED (теперь алерт — это свойство КОНКРЕТНОГО параметра, не всего экрана)
bool anyAlarmActive = false; // хоть один параметр сейчас в алерте?
unsigned long lastAlarmBeep = 0;
bool ledBlinkState = false;
unsigned long lastLedBlink = 0;
unsigned long problemStateSince = 0;

// Status flags
bool apiReachable    = false;
bool anySensorFound  = false;
bool typeCodesLoaded = false;
bool apModeActive    = false; // true пока активна собственная точка доступа (WiFi fallback) —
                               // объявлено здесь, т.к. используется в updateStatusLed() ниже,
                               // остальная AP-fallback инфраструктура — перед setup()/loop()
bool wifiEverConnectedSinceBoot = false; // как только один раз стало true — AP fallback больше
                                          // не включается до следующей физической перезагрузки;
                                          // без этого флага любой ПОЗДНИЙ обрыв связи (после
                                          // многих часов работы) тоже бы уходил в AP почти сразу,
                                          // т.к. таймер сравнивается с millis() от старта устройства
// ============================================================
//  UTILITIES
// ============================================================
String md5String(const String& s) {
  MD5Builder m; m.begin(); m.add(s); m.calculate(); return m.toString();
}

bool isScheduledMinuteNow() {
  long nowTs = (long)timeClient.getEpochTime();
  long mn = (nowTs / 60) % 1440;
  if (mn == lastRequestMinute) return false;
  int p = max(1, REQUEST_PERIOD_MIN);
  if ((mn % p) == (REQUEST_OFFSET_MIN % p)) { lastRequestMinute = mn; return true; }
  return false;
}

// True if sensorsNearby should run now (every 6h at 0,6,12,18 UTC)
bool isNearbyRefreshDue() {
  long nowTs = (long)timeClient.getEpochTime();
  long hourOfDay = (nowTs / 3600) % 24;
  long nearbySlot = (hourOfDay / 6) * 6;
  if (nearbySlot == lastNearbyHour) return false;
  lastNearbyHour = nearbySlot; return true;
}

// True if any sensor type still has too few sensors AND hasn't hit the
// radius ceiling yet — used to drive the fast (2-min) expansion cycle.
bool anyTypeNeedsExpansion() {
  for (int i = 0; i < S_COUNT; i++) {
    if (rosterCount[i] < MIN_SENSORS_TO_STOP_EXPAND && searchRadiusByType[i] < MAX_SEARCH_RADIUS)
      return true;
  }
  return false;
}

// ============================================================
//  WIFI
// ============================================================
// Общий хелпер: активен ли алерт именно на ТЕКУЩЕМ показанном экране
// (не глобально по anyAlarmActive). Экран 5 — служебный, алертов не бывает.
// Используется и здесь (жёсткий запрет мигания LED при переподключении
// во время алерта), и в updateStatusLed() — единая точка правды.
bool currentScreenHasAlarm() {
  return (carouselIndex != 5) && screenHasVisibleAlarm(carouselIndex);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());
  unsigned long start = millis();
  unsigned long lastBlink = 0; bool blinkSt = false;
  while (WiFi.status() != WL_CONNECTED && millis()-start < 15000) {
    // Жёсткий запрет: если на экране активен алерт, LED обязан гореть
    // ровным светом — попытка переподключения к WiFi не должна её перебивать.
    if (currentScreenHasAlarm()) {
      digitalWrite(PIN_LED, HIGH);
    } else if (millis()-lastBlink > 500) {
      blinkSt=!blinkSt; digitalWrite(PIN_LED,blinkSt); lastBlink=millis();
    }
    delay(50);
  }
  if (WiFi.status() == WL_CONNECTED)
    logPrintln("WiFi OK  " + WiFi.localIP().toString());
  else
    logPrintln("WiFi FAIL — will retry");
}

// ============================================================
//  HTTP helpers — поддерживают и http:// (narodmon), и https://
//  (Яндекс). На ESP32 Arduino Core 3.x статический переживающий
//  WiFiClientSecure + явные таймауты + задержка после неудачи —
//  обходит известную проблему CONNECTION_REFUSED (-1).
// ============================================================
String httpGet(const String& url, const char* extraHeaderName = nullptr, const char* extraHeaderValue = nullptr) {
  HTTPClient http;
  int code = -1;
  String r;

  if (url.startsWith("https://")) {
    static WiFiClientSecure secureClient;
    secureClient.setInsecure();
    secureClient.setTimeout(15000);
    http.setConnectTimeout(15000);
    http.setTimeout(15000);
    if (http.begin(secureClient, url)) {
      http.addHeader("User-Agent","ESP32WeatherDisplay");
      if (extraHeaderName) http.addHeader(extraHeaderName, extraHeaderValue);
      code = http.GET();
      r = (code == 200) ? http.getString() : "";
    }
    if (code <= 0) delay(200);
  } else {
    WiFiClient client;
    http.setConnectTimeout(10000);
    http.begin(client, url);
    http.addHeader("User-Agent","ESP32WeatherDisplay");
    if (extraHeaderName) http.addHeader(extraHeaderName, extraHeaderValue);
    code = http.GET();
    r = (code == 200) ? http.getString() : "";
  }

  http.end();
  if (code != 200) logPrintf("HTTP GET %d: %s\n", code, url.c_str());
  return r;
}

String nmPost(const String& body) {
  WiFiClient client; HTTPClient http;
  http.begin(client, "http://narodmon.ru/api");
  http.addHeader("Content-Type","application/json");
  http.addHeader("User-Agent","ESP32WeatherDisplay");
  int code = http.POST(body);
  String r = (code == 200) ? http.getString() : "";
  http.end();
  return r;
}
// ============================================================
//  appInit — get sensor type codes by name
// ============================================================
bool loadSensorTypeCodes() {
  String body = "{\"cmd\":\"appInit\",\"version\":\"1.0\",\"uuid\":\"" + nmUUID +
                "\",\"api_key\":\"" + NM_API_KEY + "\",\"lang\":\"ru\"}";
  String resp = nmPost(body);
  if (!resp.length()) return false;
  DynamicJsonDocument doc(8192);
  if (deserializeJson(doc, resp) || doc.containsKey("error")) return false;
  JsonArray types = doc["types"].as<JsonArray>();
  for (int i = 0; i < S_COUNT; i++) {
    sensorTypes[i].typeCode = -1;
    for (JsonObject t : types) {
      if (String(t["name"].as<const char*>()) == String(sensorTypes[i].searchName)) {
        sensorTypes[i].typeCode = t["type"].as<int>();
        logPrintf("  %s -> type=%d\n", sensorTypes[i].displayName, sensorTypes[i].typeCode);
        break;
      }
    }
  }
  return true;
}

// ============================================================
//  sensorsNearby — discover sensor IDs and build roster.
//
//  expandHungryOnly=true (быстрый цикл, 2 мин): запрашиваем ТОЛЬКО типы,
//    у которых ещё меньше MIN_SENSORS_TO_STOP_EXPAND датчиков, с радиусом
//    +RADIUS_STEP_KM от текущего. Останавливаемся, как только наберётся
//    MIN_SENSORS_TO_STOP_EXPAND (по умолчанию 2) — дальше не расширяем,
//    хотя НА ЭКРАНЕ значение показывается уже при MIN_SENSORS_NEEDED=1.
//  expandHungryOnly=false (обычный цикл, 6 часов): все 8 типов, текущие
//    радиусы; если у типа >=4 датчика — сужаем радиус обратно к базовому.
// ============================================================

void refreshSensorRoster(bool expandHungryOnly) {
  String typesList = "[";
  bool first = true;
  int requestRadius = BASE_SEARCH_RADIUS;

  for (int i = 0; i < S_COUNT; i++) {
    if (sensorTypes[i].typeCode < 0) continue;
    bool hungry = (rosterCount[i] < MIN_SENSORS_TO_STOP_EXPAND);

    if (expandHungryOnly && !hungry) continue;

    if (!first) typesList += ",";
    typesList += String(sensorTypes[i].typeCode);
    first = false;

    int r = searchRadiusByType[i];
    if (expandHungryOnly && hungry) r = min(r + RADIUS_STEP_KM, MAX_SEARCH_RADIUS);
    if (r > requestRadius) requestRadius = r;
  }

  if (first) {
    logPrintln("refreshSensorRoster: no types to query (nothing hungry), skipping");
    return;
  }
  typesList += "]";

  logPrintf("sensorsNearby: %s mode, radius %d km, types=%s\n",
    expandHungryOnly ? "FAST-EXPAND" : "FULL-REVIEW", requestRadius, typesList.c_str());

  String body = "{\"cmd\":\"sensorsNearby\","
    "\"lat\":" + String(MY_LAT,6) + ",\"lon\":" + String(MY_LON,6) + ","
    "\"radius\":" + String(requestRadius) + ",\"pub\":1,"
    "\"types\":" + typesList + ","
    "\"uuid\":\"" + nmUUID + "\",\"api_key\":\"" + NM_API_KEY + "\","
    "\"lang\":\"ru\"}";

  String resp = nmPost(body);
  if (!resp.length()) { logPrintln("sensorsNearby: no response"); return; }

  DynamicJsonDocument doc(32768);
  if (deserializeJson(doc, resp) || doc.containsKey("error")) {
    logPrintln("sensorsNearby: error"); return;
  }

  struct Cand { long id; float dist; };
  Cand cands[S_COUNT][40]; int candCnt[S_COUNT] = {0};

  for (JsonObject dev : doc["devices"].as<JsonArray>()) {
    float dist = dev["distance"].as<float>();
    for (JsonObject s : dev["sensors"].as<JsonArray>()) {
      int type = s["type"].as<int>();
      long sid  = s["id"].as<long>();
      for (int i = 0; i < S_COUNT; i++) {
        if (sensorTypes[i].typeCode == type) {
          bool hungry = (rosterCount[i] < MIN_SENSORS_TO_STOP_EXPAND);
          int effectiveRadius = searchRadiusByType[i];
          if (expandHungryOnly && hungry) effectiveRadius = min(effectiveRadius + RADIUS_STEP_KM, MAX_SEARCH_RADIUS);
          if (dist <= effectiveRadius && candCnt[i] < 40) {
            cands[i][candCnt[i]++] = {sid, dist};
          }
          break;
        }
      }
    }
  }

  for (int i = 0; i < S_COUNT; i++) {
    if (sensorTypes[i].typeCode < 0) continue;
    bool wasQueried = !expandHungryOnly || (rosterCount[i] < MIN_SENSORS_TO_STOP_EXPAND);
    if (!wasQueried) continue;

    for (int a=0;a<candCnt[i]-1;a++) for (int b=0;b<candCnt[i]-1-a;b++)
      if (cands[i][b].dist > cands[i][b+1].dist) { Cand tmp=cands[i][b]; cands[i][b]=cands[i][b+1]; cands[i][b+1]=tmp; }

    int newCount = min(candCnt[i], NEAREST_COUNT);
    rosterCount[i] = newCount;
    for (int k=0; k<newCount; k++) {
      sensorRoster[i][k] = {cands[i][k].id, cands[i][k].dist};
      bool found = false;
      for (int p=0; p<MAX_ROSTER; p++)
        if (nmCache[i][p].time > 0 && sensorRoster[i][k].id == sensorRoster[i][k].id) { found=true; break; }
      if (!found) nmCache[i][k] = {0, 0, cands[i][k].dist, 0, false};
    }

    if (expandHungryOnly) {
      if (newCount < MIN_SENSORS_TO_STOP_EXPAND && searchRadiusByType[i] < MAX_SEARCH_RADIUS) {
        int oldR = searchRadiusByType[i];
        searchRadiusByType[i] = min(searchRadiusByType[i] + RADIUS_STEP_KM, MAX_SEARCH_RADIUS);
        logPrintf("  %s: %d sensors (< %d needed to stop), radius %d -> %d km\n",
          sensorTypes[i].displayName, newCount, MIN_SENSORS_TO_STOP_EXPAND, oldR, searchRadiusByType[i]);
      } else {
        logPrintf("  %s: %d sensors, radius stays %d km\n",
          sensorTypes[i].displayName, newCount, searchRadiusByType[i]);
      }
    } else {
      if (newCount >= 4 && searchRadiusByType[i] > BASE_SEARCH_RADIUS) {
        int oldR = searchRadiusByType[i];
        searchRadiusByType[i] = BASE_SEARCH_RADIUS;
        logPrintf("  %s: %d sensors (>=4, запас есть), radius %d -> %d km (сужаем)\n",
          sensorTypes[i].displayName, newCount, oldR, searchRadiusByType[i]);
      } else {
        logPrintf("  %s: %d sensors, radius stays %d km\n",
          sensorTypes[i].displayName, newCount, searchRadiusByType[i]);
      }
    }
  }

  int totalKnown = 0;
  for (int i=0; i<S_COUNT; i++) totalKnown += rosterCount[i];
  logPrintf("Total known sensors: %d\n", totalKnown);
}

// ============================================================
//  sensorsValues — выбираем 3 датчика с самыми старыми показаниями
//  (датчики без данных вообще — "максимально старые", приоритет выше всех).
//  Защита: не переспрашиваем один датчик чаще раза в MIN_REQUERY_INTERVAL_MIN.
// ============================================================
void fetchOldestSensors() {
  long nowTs = (long)timeClient.getEpochTime();

  struct Candidate { int typeIdx; int rosterIdx; long sortAge; };
  Candidate all[S_COUNT * MAX_ROSTER];
  int allCount = 0;

  for (int i = 0; i < S_COUNT; i++) {
    for (int k = 0; k < rosterCount[i]; k++) {
      NmReading& r = nmCache[i][k];
      if (r.lastQueriedTs > 0 && (nowTs - r.lastQueriedTs) < MIN_REQUERY_INTERVAL_MIN * 60L)
        continue;
      long sortAge = (r.time == 0) ? LONG_MAX : (nowTs - r.time);
      all[allCount++] = {i, k, sortAge};
    }
  }

  if (allCount == 0) {
    logPrintln("fetchOldestSensors: nothing eligible right now (all queried <5 min ago)");
    return;
  }

  for (int a=1; a<allCount; a++) {
    Candidate key = all[a]; int b = a-1;
    while (b>=0 && all[b].sortAge < key.sortAge) { all[b+1]=all[b]; b--; }
    all[b+1] = key;
  }

  int n = min(allCount, 3);
  long ids[3]; int typeIdxs[3]; int rosterIdxs[3];
  for (int k=0; k<n; k++) {
    typeIdxs[k] = all[k].typeIdx;
    rosterIdxs[k] = all[k].rosterIdx;
    ids[k] = sensorRoster[typeIdxs[k]][rosterIdxs[k]].id;
    nmCache[typeIdxs[k]][rosterIdxs[k]].lastQueriedTs = nowTs;
  }

  String idsStr = "[";
  for (int k=0; k<n; k++) { if (k) idsStr+=","; idsStr += String(ids[k]); }
  idsStr += "]";

  String body = "{\"cmd\":\"sensorsValues\",\"sensors\":" + idsStr +
    ",\"uuid\":\"" + nmUUID + "\",\"api_key\":\"" + NM_API_KEY + "\",\"lang\":\"ru\"}";

  String resp = nmPost(body);
  if (!resp.length()) { apiReachable = false; return; }

  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, resp) || doc.containsKey("error")) { apiReachable = false; return; }
  apiReachable = true;

  JsonArray sensors = doc["sensors"].as<JsonArray>();
  for (JsonObject s : sensors) {
    long sid = s["id"].as<long>();
    float val = s["value"].as<float>();
    long  t   = s["time"].as<long>();
    long ageMin = (nowTs - t) / 60;
    if (ageMin > FRESH_MINUTES || ageMin < -2) continue;

    for (int k=0; k<n; k++) {
      if (sensorRoster[typeIdxs[k]][rosterIdxs[k]].id == sid) {
        nmCache[typeIdxs[k]][rosterIdxs[k]].value = val;
        nmCache[typeIdxs[k]][rosterIdxs[k]].time = t;
        logPrintf("  cached %s[%d] = %.2f (age %ldm)\n",
          sensorTypes[typeIdxs[k]].displayName, rosterIdxs[k], val, ageMin);
        break;
      }
    }
  }
}
// ============================================================
//  YANDEX WEATHER — REST API v2 (тариф "для умного дома" / "Оптимальный")
//  Эндпоинт: GET https://api.weather.yandex.ru/v2/forecast?lat=..&lon=..&hours=true
//  Заголовок: X-Yandex-Weather-Key: <ключ>
//
//  Структура ответа (подтверждена официальной документацией):
//    { "now": unixTs, "now_dt": "ISO8601",
//      "info": {...},
//      "fact": { temp, feels_like, icon, condition, wind_speed,
//                wind_gust, wind_dir, pressure_mm, pressure_pa, humidity },
//      "forecasts": [ { date, date_ts, hours: [ {hour, hour_ts, temp,
//                        wind_speed, wind_dir, pressure_mm, condition}, ... ],
//                        parts: { day: {temp_avg,...}, night: {...} } }, ... ] }
//
//  ВНИМАНИЕ: на бесплатном тарифе "для умного дома" набор полей может
//  быть урезан (например давление/влажность могут отсутствовать) —
//  код проверяет наличие каждого поля отдельно (isNull()), пропуская
//  то, чего нет, вместо падения. Если структура всё же отличается от
//  документации — Serial Monitor покажет сырой ответ при ошибке парсинга.
// ============================================================
void fetchYandexWeather() {
  if (MY_LAT == 0.0 && MY_LON == 0.0) return;
  logPrintln("Fetching Yandex Weather...");

  String url = "https://api.weather.yandex.ru/v2/forecast"
    "?lat=" + String(MY_LAT, 6) + "&lon=" + String(MY_LON, 6) +
    "&lang=ru_RU&limit=2&hours=true&extra=false";

  String resp = httpGet(url, "X-Yandex-Weather-Key", YANDEX_WEATHER_KEY.c_str());

  if (!resp.length()) {
    logPrintln("Yandex Weather: no response");
    yandexReachable = false;
    return;
  }

  DynamicJsonDocument doc(24576);
  DeserializationError err = deserializeJson(doc, resp);
  if (err) {
    logPrintln("Yandex Weather: JSON parse error: " + String(err.c_str()));
    logPrintln("Raw response (first 600 chars): " + resp.substring(0, 600));
    yandexReachable = false;
    return;
  }

  if (doc.containsKey("errors") || doc.containsKey("error")) {
    logPrintln("Yandex Weather: error in response:");
    logPrintln(resp.substring(0, 400));
    yandexReachable = false;
    return;
  }

  JsonObject fact = doc["fact"];
  if (fact.isNull()) {
    logPrintln("Yandex Weather: no 'fact' in response — структура ответа другая,");
    logPrintln("проверьте заголовок (X-Yandex-Weather-Key vs X-Yandex-API-Key) и тариф.");
    logPrintln("Raw response (first 600 chars): " + resp.substring(0, 600));
    yandexReachable = false;
    return;
  }

  yandexReachable = true;
  long nowTs = doc["now"].as<long>();
  if (nowTs == 0) nowTs = (long)timeClient.getEpochTime(); // fallback

  // --- Текущая погода (fact) ---
  yandexNow.valid = true;
  yandexNow.fetchedAtTs = nowTs;
  yandexNow.temperature = fact["temp"].as<float>();
  yandexNow.feelsLike    = fact.containsKey("feels_like") ? fact["feels_like"].as<float>() : NAN;
  yandexNow.windSpeed    = fact.containsKey("wind_speed") ? fact["wind_speed"].as<float>() : NAN;
  yandexNow.condition    = fact.containsKey("condition") ? String(fact["condition"].as<const char*>()) : "";
  logPrintf("  Yandex fact: T=%.1f feels=%.1f wind=%.1f cond=%s\n",
    yandexNow.temperature, yandexNow.feelsLike, yandexNow.windSpeed, yandexNow.condition.c_str());

  // --- Почасовой прогноз: ищем час, ближайший к "сейчас + 2 часа" ---
  long targetTs = nowTs + 2*3600;
  long bestDiff = -1;
  yandexPlus2h.valid = false;

  JsonArray forecasts = doc["forecasts"].as<JsonArray>();
  int dayIdx = 0;
  for (JsonObject fday : forecasts) {
    if (fday.containsKey("hours")) {
      for (JsonObject h : fday["hours"].as<JsonArray>()) {
        long hourTs = h["hour_ts"].as<long>();
        if (hourTs == 0) continue;
        long diff = abs(hourTs - targetTs);
        if (bestDiff < 0 || diff < bestDiff) {
          bestDiff = diff;
          yandexPlus2h.valid = true;
          yandexPlus2h.time = hourTs;
          yandexPlus2h.temperature = h["temp"].as<float>();
          yandexPlus2h.windSpeed = h.containsKey("wind_speed") ? h["wind_speed"].as<float>() : NAN;
          yandexPlus2h.condition = h.containsKey("condition") ? String(h["condition"].as<const char*>()) : "";
        }
      }
    }

    // --- Дневной прогноз (parts.day / parts.night) для первых 2 дней ---
    if (dayIdx < 2 && fday.containsKey("parts")) {
      JsonObject parts = fday["parts"];
      yandexDays[dayIdx].valid = true;
      yandexDays[dayIdx].tempDay   = parts.containsKey("day")   ? parts["day"]["temp_avg"].as<float>()   : NAN;
      yandexDays[dayIdx].tempNight = parts.containsKey("night") ? parts["night"]["temp_avg"].as<float>() : NAN;
    }
    dayIdx++;
  }

  if (yandexPlus2h.valid) {
    logPrintf("  Yandex +2h: T=%.1f wind=%.1f cond=%s\n",
      yandexPlus2h.temperature, yandexPlus2h.windSpeed, yandexPlus2h.condition.c_str());
  } else {
    logPrintln("  Yandex +2h: подходящий часовой слот не найден (возможно hours=false сработал не так, или почасовых данных нет на этом тарифе)");
  }
  for (int d=0; d<2; d++) {
    if (yandexDays[d].valid) {
      logPrintf("  Yandex day[%d]: day=%.1f night=%.1f\n", d, yandexDays[d].tempDay, yandexDays[d].tempNight);
    }
  }
}
// ============================================================
//  computeResults — IDW-усреднение по narodmon (взвешено по расстоянию).
//  Проверка "сломанного нуля": если в группе 2+ датчика, один показывает
//  РОВНО 0.0, а другой(ие) отличаются от него больше чем на
//  ZERO_SENSOR_DIFF_THRESHOLD — нулевой исключается из усреднения.
// ============================================================
void computeResults() {
  long nowTs = (long)timeClient.getEpochTime();
  anySensorFound = false;

  for (int i = 0; i < S_COUNT; i++) {
    // Собираем свежие показания этого типа
    float vals[MAX_ROSTER]; float dists[MAX_ROSTER]; int freshCount = 0;

    for (int k = 0; k < rosterCount[i]; k++) {
      NmReading& r = nmCache[i][k];
      r.excludedAsZero = false;
      if (r.time == 0) continue;
      long ageMin = (nowTs - r.time) / 60;
      if (ageMin > FRESH_MINUTES || ageMin < -2) continue;
      vals[freshCount] = r.value;
      dists[freshCount] = r.dist;
      freshCount++;
    }

    // Проверка "сломанного нуля" — только если есть 2+ свежих показания
    bool isZero[MAX_ROSTER] = {false};
    if (freshCount >= 2) {
      for (int a = 0; a < freshCount; a++) {
        if (fabsf(vals[a]) > 0.0001f) continue; // не ноль — пропускаем
        // Этот датчик показывает ровно 0 — сравниваем с остальными
        for (int b = 0; b < freshCount; b++) {
          if (a == b || fabsf(vals[b]) <= 0.0001f) continue; // не сравниваем с другим нулём
          if (fabsf(vals[a] - vals[b]) > ZERO_SENSOR_DIFF_THRESHOLD) {
            isZero[a] = true; // помечаем как "сломанный", отбросим ниже
            break;
          }
        }
      }
    }

    // IDW-усреднение, пропуская помеченные как "сломанный ноль"
    float wSum = 0, vSum = 0;
    int   nmCount = 0;
    float nearestDist = 1e9f;
    for (int a = 0; a < freshCount; a++) {
      if (isZero[a]) continue;
      float w = 1.0f / powf(dists[a] + IDW_EPS, IDW_POWER);
      wSum += w; vSum += w * vals[a];
      nmCount++;
      if (dists[a] < nearestDist) nearestDist = dists[a];
    }

    if (nmCount < MIN_SENSORS_NEEDED) {
      results[i].valid = false;
      continue;
    }

    results[i].valid = true;
    results[i].value = vSum / wSum;
    results[i].nmCount = nmCount;
    results[i].nearestDist = nearestDist;
    historyPush(i, nowTs, results[i].value);
    if (i == S_WIND_DIR) windDirAddSample(nowTs, results[i].value);
    anySensorFound = true;
  }

  checkPerParameterAlarms();
}

// ============================================================
//  Алерты — теперь ПО КАЖДОМУ ПАРАМЕТРУ отдельно (results[i].inAlarm),
//  а не общий полноэкранный алерт. anyAlarmActive — общий флаг для LED.
// ============================================================
void checkPerParameterAlarms() {
  anyAlarmActive = false;

  results[S_RADIATION].inAlarm = results[S_RADIATION].valid && results[S_RADIATION].value >= ALARM_RADIATION_MRH;
  results[S_WIND_SPEED].inAlarm = results[S_WIND_SPEED].valid && results[S_WIND_SPEED].value >= ALARM_WIND_MS;
  results[S_DUST].inAlarm = results[S_DUST].valid && results[S_DUST].value >= ALARM_DUST_UGM3;
  results[S_TEMP].inAlarm = results[S_TEMP].valid &&
    (results[S_TEMP].value <= ALARM_TEMP_COLD_C || results[S_TEMP].value >= ALARM_TEMP_HOT_C);
  results[S_PRESS].inAlarm = results[S_PRESS].valid &&
    (results[S_PRESS].value < ALARM_PRESS_LOW_MMHG || results[S_PRESS].value > ALARM_PRESS_HIGH_MMHG);
  results[S_HUM].inAlarm = results[S_HUM].valid && results[S_HUM].value >= ALARM_HUM_HIGH_PCT;

  for (int i = 0; i < S_COUNT; i++) {
    if (results[i].inAlarm) anyAlarmActive = true;
  }
}
// ============================================================
//  LED / Buzzer status — горит непрерывно при любом активном алерте
//  (anyAlarmActive), иначе обычная иерархия мигания по связи — но не
//  мгновенно: чтобы кратковременные сбои (доли секунды/пара секунд) не
//  вызывали дёрганье индикатора, состояние сэмплируется раз в
//  CONN_CHECK_INTERVAL_MS и мигание начинается только после
//  CONN_FAIL_STREAK_THRESHOLD проверок подряд с проблемой (по умолчанию
//  3 проверки × 10 сек = минимум 30 секунд непрерывной проблемы).
// ============================================================
const unsigned long CONN_CHECK_INTERVAL_MS = 10000UL;
const int CONN_FAIL_STREAK_THRESHOLD = 3;
unsigned long lastConnCheckMs = 0;
int  connFailStreak = 0;
bool connProblemConfirmed = false;

void updateStatusLed() {
  // Пока активен AP fallback — отдельный, отличимый от прочих, паттерн
  // мигания (быстрое 2 Гц), и НЕ считаем это "проблемным состоянием" для
  // watchdog'а: устройство намеренно ждёт, пока его настроят через веб —
  // это не зависание, перезагрузка каждые 30 минут только мешала бы.
  if (apModeActive) {
    problemStateSince = 0;
    if (millis() - lastLedBlink > 250) {
      ledBlinkState = !ledBlinkState;
      digitalWrite(PIN_LED, ledBlinkState ? HIGH : LOW);
      lastLedBlink = millis();
    }
    return;
  }

  // Алерт на текущем экране — абсолютный приоритет: жёсткий запрет на
  // проверки связи и мигание, LED обязан гореть ровным светом. Заодно
  // сбрасываем счётчик неудач, чтобы после алерта индикация связи не
  // "выстрелила" мгновенно из уже накопленного до алерта счётчика.
  if (currentScreenHasAlarm()) {
    problemStateSince = 0;
    connFailStreak = 0;
    connProblemConfirmed = false;
    digitalWrite(PIN_LED, HIGH);
    return;
  }

  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  bool rawProblem = !wifiOk || !apiReachable || !anySensorFound;

  if (millis() - lastConnCheckMs > CONN_CHECK_INTERVAL_MS) {
    lastConnCheckMs = millis();
    if (rawProblem) {
      if (connFailStreak < 1000) connFailStreak++; // защита от переполнения при долгой проблеме
      if (connFailStreak >= CONN_FAIL_STREAK_THRESHOLD) connProblemConfirmed = true;
    } else {
      connFailStreak = 0;
      connProblemConfirmed = false;
    }
  }

  if (!connProblemConfirmed) {
    problemStateSince = 0;
    digitalWrite(PIN_LED, LOW);
    return;
  }

  // Проблема подтверждена (держится минимум ~30 сек) — приоритет причины
  // мигания берём по текущему живому состоянию, не по сэмплу
  unsigned long blinkMs;
  if (!wifiOk)              blinkMs = 1000;
  else if (!apiReachable)   blinkMs = 3000;
  else                      blinkMs = 5000; // !anySensorFound

  if (!problemStateSince) problemStateSince = millis();
  else if (millis() - problemStateSince > REBOOT_AFTER_STUCK_SEC * 1000UL) {
    logPrintln("Stuck 30min — reboot"); delay(500); ESP.restart();
  }
  if (millis() - lastLedBlink > blinkMs/2) {
    ledBlinkState = !ledBlinkState;
    digitalWrite(PIN_LED, ledBlinkState ? HIGH : LOW);
    lastLedBlink = millis();
  }
}

// ============================================================
//  Пищалка по алерту — вызывается ИМЕННО в момент отрисовки экрана
//  (не каждую итерацию loop()). Один общий таймер lastAlarmBeep на
//  ВСЕ алерты вместе: если запикали на экране температуры, на экране
//  радиации не запикаем снова, пока не пройдёт ALARM_BEEP_INTERVAL_SEC
//  с того момента — независимо от того, какой именно параметр сработал.
// ============================================================
void beepIfScreenHasAlarm(int idx) {
  if (idx == 5) return; // служебный экран алертов не показывает
  if (!screenHasVisibleAlarm(idx)) return;
  if (millis() - lastAlarmBeep < (unsigned long)ALARM_BEEP_INTERVAL_SEC * 1000UL) return;

  tone(PIN_BUZZER, 2000, ALARM_BEEP_DURATION_MS);
  lastAlarmBeep = millis();
}
// ============================================================
//  HEADER BAR (день недели, дата, время, температура — на каждом экране)
//  Высота и шрифт увеличены в 2 раза по сравнению с первой версией —
//  места на экране достаточно, а текст так гораздо легче читать.
// ============================================================
const int HEADER_H = 44;

void drawHeaderBar(uint16_t bg) {
  tft.fillRect(0, 0, 240, HEADER_H, bg);
  tft.drawFastHLine(0, HEADER_H, 240, TFT_DARKGREY);
  long nowTs = (long)timeClient.getEpochTime();
  long displayTs = nowTs + (long)TZ_DISPLAY_OFFSET_HOURS * 3600L;
  int hh = (displayTs%86400L)/3600, mm = (displayTs%3600L)/60;

  long days=displayTs/86400L,z=days+719468L;
  long era=(z>=0?z:z-146096)/146097;
  long doe=z-era*146097;
  long yoe=(doe-doe/1460+doe/36524-doe/146096)/365;
  long yr=yoe+era*400;
  long doy=doe-(365*yoe+yoe/4-yoe/100);
  long mp=(5*doy+2)/153;
  int  dd=doy-(153*mp+2)/5+1;
  int  mo=mp+(mp<10?3:-9);
  long y=yr+(mo<=2?1:0);

  // День недели: 1 янв 1970 (days=0) был четвергом. Сокращено до 2 символов,
  // чтобы оставить место для температуры справа без перекрытия.
  const char* dowNames[7] = {"Su","Mo","Tu","We","Th","Fr","Sa"};
  int dow = ((days % 7) + 4 + 7) % 7; // +7 защищает от отрицательного остатка
  const char* dowStr = dowNames[dow];

  char buf[40];
  snprintf(buf,sizeof(buf),"%s %02d.%02d %02d:%02d",dowStr,dd,mo,hh,mm);

  tft.setTextSize(2); tft.setTextColor(TFT_GREENYELLOW, bg);
  tft.setCursor(2,14); tft.print(buf);

  char tbuf[12];
  if (results[S_TEMP].valid) {
    float t = results[S_TEMP].value;
    // Двузначная и более отрицательная температура ("-10" и холоднее) —
    // округляем до целого без десятых, иначе строка слишком длинная и
    // налезает на дату/время слева (проверено: "-28.8C" пересекается
    // с "Mo 23.06 20:32" при текущей раскладке шапки).
    if (t <= -10.0f) snprintf(tbuf,sizeof(tbuf),"%.0fC",t);
    else snprintf(tbuf,sizeof(tbuf),"%.1fC",t);
  }
  else snprintf(tbuf,sizeof(tbuf),"--C");
  tft.setTextColor(TFT_WHITE, bg);
  tft.setCursor(240-strlen(tbuf)*12-2, 14); tft.print(tbuf);
}

// ============================================================
//  ТРЕНД-БАР — во всю ширину экрана, без подписей. Слоты теперь
//  фиксированы по времени суток (кратны TREND_SLOT_MINUTES=45 минутам),
//  а не "скользящее окно от текущего момента" — см. TrendBuffer выше.
//  32 столбика * 45 минут = 24 часа охвата. Ширина 6px + промежуток 1px:
//  32*(6+1) = 224px из 240, с запасом по краям.
// ============================================================
const int TREND_BAR_W   = 6;
const int TREND_BAR_GAP = 1;
const int TREND_BARS_COUNT = TREND_SLOT_COUNT; // 32, видимых столбиков = размер буфера
const int TREND_LEVELS = 7;

int trendLevel(float v, float vMin, float vMax) {
  if (vMax-vMin < 0.0001f) return (TREND_LEVELS+1)/2;
  int lvl = 1 + (int)roundf((v-vMin)/(vMax-vMin)*(TREND_LEVELS-1));
  return constrain(lvl, 1, TREND_LEVELS);
}

// Рисует тренд-бар на всю ширину экрана (x=0..240) с базовой линией на y,
// высотой столбиков вверх до maxBarHeight. idx — индекс параметра в trendBuf[].
void drawFullWidthTrend(int idx, int baseY, int maxBarHeight, uint16_t bg, bool alarmColor) {
  if (!results[idx].valid) return;

  TrendBuffer& buf = trendBuf[idx];
  float vals[TREND_BARS_COUNT];
  bool  has[TREND_BARS_COUNT];
  float vMin = results[idx].value, vMax = results[idx].value;

  // k=0 — самый старый видимый слот (слева), k=TREND_BARS_COUNT-1 — самый
  // новый (справа). Буфер кольцевой, buf.head — индекс самого нового слота,
  // поэтому слот для позиции k на экране — это (head - (COUNT-1-k)) по кругу.
  for (int k = 0; k < TREND_BARS_COUNT; k++) {
    int stepsBack = TREND_BARS_COUNT - 1 - k;
    int slotPos = (buf.head - stepsBack + TREND_BARS_COUNT) % TREND_BARS_COUNT;
    has[k] = buf.slots[slotPos].valid;
    if (has[k]) {
      vals[k] = buf.slots[slotPos].value;
      if (vals[k] < vMin) vMin = vals[k];
      if (vals[k] > vMax) vMax = vals[k];
    }
  }

  uint16_t normalColor = TFT_GREENYELLOW;
  uint16_t lastColor = alarmColor ? TFT_RED : TFT_CYAN;

  for (int k = 0; k < TREND_BARS_COUNT; k++) {
    int bx = k * (TREND_BAR_W + TREND_BAR_GAP);
    if (!has[k]) { tft.drawFastHLine(bx, baseY, TREND_BAR_W, TFT_DARKGREY); continue; }
    int lvl = trendLevel(vals[k], vMin, vMax);
    int bh = max(2, maxBarHeight * lvl / TREND_LEVELS);
    uint16_t color = (k == TREND_BARS_COUNT - 1) ? lastColor : normalColor;
    tft.fillRect(bx, baseY - bh, TREND_BAR_W, bh, color);
  }
}

// ============================================================
//  Перевод направления ветра (градусы) в 8-секторную аббревиатуру
// ============================================================
const char* windDirStr8(float deg) {
  const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  int sector = ((int)((deg + 22.5f) / 45.0f)) % 8;
  if (sector < 0) sector += 8;
  return dirs[sector];
}

// ============================================================
//  Замена тренд-бара для НАПРАВЛЕНИЯ ВЕТРА: вместо столбиков (которые для
//  категориальной величины визуально не имеют смысла и "дёргаются") —
//  3 текстовые метки преобладающего направления слева-направо: 24h, 12h, 6h.
//  Размер и положение строки — там же, где была бы полоса тренда; цвет —
//  тот же зелёный, что у обычных столбиков (TFT_GREENYELLOW).
// ============================================================
void drawWindDirSummary(int baseY, uint16_t bg) {
  struct Period { int hours; const char* label; };
  Period periods[3] = { {24,"24h"}, {12,"12h"}, {6,"6h"} };

  tft.setTextSize(2); // крупнее обычного текста подписи, но мельче основной цифры — занимает место тренда
  int colW = 240 / 3;

  for (int k = 0; k < 3; k++) {
    float deg;
    bool ok = windDirPrevailing(periods[k].hours, deg);
    char buf[16];
    if (ok) snprintf(buf,sizeof(buf),"%s %s",windDirStr8(deg),periods[k].label);
    else snprintf(buf,sizeof(buf),"-- %s",periods[k].label);

    tft.setTextColor(TFT_GREENYELLOW, bg);
    int textW = strlen(buf) * 12; // 6px база * textSize2
    int x = k*colW + (colW-textW)/2; // центрируем в своей колонке
    if (x < 2) x = 2;
    tft.setCursor(x, baseY);
    tft.print(buf);
  }
}

// ============================================================
//  УНИВЕРСАЛЬНЫЙ БЛОК ПАРАМЕТРА: цифра слева (крупно), размерность
//  справа от цифры (мелко), тренд-бар на всю ширину под ними.
//  При алерте — цифра алого цвета.
//  Возвращает Y-координату ПОСЛЕ блока (для размещения следующего).
// ============================================================
int drawParamBlock(int idx, int topY, int blockHeight, bool bigNumber, uint16_t bg) {
  uint16_t numColor = results[idx].inAlarm ? TFT_RED : TFT_WHITE;

  // Максимально увеличенный шрифт, который ещё помещается по высоте в зазор
  // до тренд-бара (проверено на экстремальных значениях: "-9.5"/"35.0" для
  // температуры, "100.0"/"800.0"/"999.9" для остальных — см. расчёт в чате):
  // обычные блоки 4->5, крупный блок (температура) 6->7.
  int numTextSize = bigNumber ? 7 : 5;
  int trendH = bigNumber ? 36 : 26; // высота тренд-бара, побольше для крупного блока
  int numY = topY + 4;

  tft.setTextColor(numColor, bg);
  tft.setTextSize(numTextSize);
  tft.setCursor(4, numY);

  char vbuf[16];
  if (!results[idx].valid) {
    tft.setTextColor(TFT_DARKGREY, bg);
    tft.print("N/D");
  } else {
    if (idx == S_WIND_DIR) {
      // Направление — словами (8 секторов), не числом градусов
      snprintf(vbuf,sizeof(vbuf),"%s",windDirStr8(results[idx].value));
    } else {
      snprintf(vbuf,sizeof(vbuf),"%.1f",results[idx].value);
    }
    tft.print(vbuf);
  }

  // Размерность — мелким шрифтом справа от цифры, на той же визуальной строке.
  // Для направления ветра вместо размерности — второе крупное значение
  // (преобладающее направление за последний час), см. ниже отдельным блоком.
  int charWidthPx = bigNumber ? 42 : 30; // 6px база * numTextSize (7 и 5 соответственно)
  int labelX = 4;
  if (idx != S_WIND_DIR) {
    int numWidthPx = charWidthPx * strlen(results[idx].valid ? vbuf : "N/D");
    labelX = min(numWidthPx + 8, 200);
    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY, bg);
    tft.setCursor(labelX, numY + (bigNumber ? 8 : 4));
    tft.print(sensorTypes[idx].unit);
  } else {
    int numWidthPx = charWidthPx * strlen(results[idx].valid ? vbuf : "N/D");
    labelX = min(numWidthPx + 8, 200);
  }

  // Короткая подпись параметра (Temp/Wet/Press/...) — справа от цифры,
  // в зазоре между низом цифры и верхом тренда, тем же цветом что "no sensors".
  // Считаем середину зазора так, чтобы не задеть ни цифру, ни тренд.
  // Высота глифа = 8px * numTextSize (56px для крупного, 40px для обычного).
  int numBottomY = numY + 8 * numTextSize;
  int trendTopY  = topY + blockHeight - 4 - trendH;
  int labelY = numBottomY + (trendTopY - numBottomY) / 2 - 4; // -4 центрирует текст высотой ~8px

  if (idx == S_WIND_DIR) {
    // Подпись "Dir" под текущим направлением (как у остальных параметров)
    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY, bg);
    tft.setCursor(4, labelY);
    tft.print("Dir");

    // Второе крупное значение: преобладающее направление за последний час —
    // тот же размер шрифта (numTextSize), что и текущее, своя подпись "Dir 1h"
    float avgDeg;
    bool haveAvg = windDirPrevailing(1, avgDeg);
    char avgBuf[8];
    snprintf(avgBuf,sizeof(avgBuf),"%s", haveAvg ? windDirStr8(avgDeg) : "--");

    int avgX = 130; // правее середины экрана (120px), с запасом от края при максимальной длине "NW"
    tft.setTextColor(TFT_WHITE, bg);
    tft.setTextSize(numTextSize);
    tft.setCursor(avgX, numY);
    tft.print(avgBuf);

    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY, bg);
    tft.setCursor(avgX, labelY);
    tft.print("Dir 1h");
  } else {
    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY, bg);
    tft.setCursor(labelX, labelY);
    tft.print(sensorTypes[idx].shortLabel);
  }

  int trendBaseY = topY + blockHeight - 4;
  if (idx == S_WIND_DIR) {
    // Направление — вместо столбиков тренда показываем преобладающее
    // направление за 24h/12h/6h текстом (см. drawWindDirSummary)
    drawWindDirSummary(trendBaseY - 16, bg);
  } else {
    drawFullWidthTrend(idx, trendBaseY, trendH, bg, results[idx].inAlarm);
  }


  return topY + blockHeight;
}
// ============================================================
//  ЭКРАН 0: Температура (крупно) + Влажность (обычно)
// ============================================================
void drawScreenTempHum() {
  uint16_t bg = TFT_BLACK;
  tft.fillScreen(bg);
  drawHeaderBar(bg);

  int y = HEADER_H + 4;
  drawParamBlock(S_TEMP, y, 110, true, bg);   // крупный блок ~110px высоты

  // Влажность размещена на той же высоте, где на экранах 1/2 находится
  // третий блок (давление/осадки) — для единообразной разметки по экранам.
  int blockH = (320 - HEADER_H - 10) / 3;
  int humY = HEADER_H + 2 + 2 * blockH;
  drawParamBlock(S_HUM, humY, blockH, false, bg);

  drawCarouselDots(0);
}

// ============================================================
//  ЭКРАН 1: Ветер / Направление / Давление
// ============================================================
void drawScreenWindPress() {
  uint16_t bg = TFT_BLACK;
  tft.fillScreen(bg);
  drawHeaderBar(bg);

  int y = HEADER_H + 2;
  int blockH = (320 - HEADER_H - 10) / 3; // делим оставшееся место на 3 равных блока
  y = drawParamBlock(S_WIND_SPEED, y, blockH, false, bg);
  y = drawParamBlock(S_WIND_DIR,   y, blockH, false, bg);
  y = drawParamBlock(S_PRESS,      y, blockH, false, bg);

  drawCarouselDots(1);
}

// ============================================================
//  ЭКРАН 2: Качество воздуха (запылённость) / Радиация / Осадки
// ============================================================
void drawScreenAirRadPrecip() {
  uint16_t bg = TFT_BLACK;
  tft.fillScreen(bg);
  drawHeaderBar(bg);

  int y = HEADER_H + 2;
  int blockH = (320 - HEADER_H - 10) / 3;
  y = drawParamBlock(S_DUST,      y, blockH, false, bg);
  y = drawParamBlock(S_RADIATION, y, blockH, false, bg);
  y = drawParamBlock(S_PRECIP,    y, blockH, false, bg);

  drawCarouselDots(2);
}

// ============================================================
//  Точки-индикатор карусели снизу экрана (общие для всех 6 экранов)
// ============================================================
void drawCarouselDots(int activeIdx) {
  int total = TOTAL_SCREENS;
  int sx = 120 - (total*8)/2;
  for (int k=0;k<total;k++)
    tft.fillCircle(sx+k*8, 314, 2, k==activeIdx?TFT_WHITE:TFT_DARKGREY);
}
// ============================================================
//  ЭКРАН 3: Яндекс — данные на момент запроса (верх) + через 2 часа от
//  того же момента (низ). Подписи — точное время ИМЕННО ЭТИХ данных
//  (yandexNow.fetchedAtTs), а не текущее время дисплея — это важно,
//  потому что обновление раз в 2 часа, и к моменту просмотра экрана
//  "сейчас" уже не совпадает с реальным текущим временем.
// ============================================================
void drawScreenYandexNow() {
  uint16_t bg = TFT_BLACK;
  tft.fillScreen(bg);
  drawHeaderBar(bg);

  tft.setTextColor(TFT_CYAN, bg); tft.setTextSize(2);
  tft.setCursor(4, HEADER_H+8); tft.print("YANDEX WEATHER");
  tft.drawFastHLine(0, HEADER_H+26, 240, TFT_DARKGREY);

  int y = HEADER_H + 36;

  // --- Блок данных "на момент запроса" ---
  tft.setTextColor(TFT_DARKGREY, bg); tft.setTextSize(2);
  tft.setCursor(4, y);
  if (yandexNow.valid) {
    long displayTs = yandexNow.fetchedAtTs + (long)TZ_DISPLAY_OFFSET_HOURS*3600L;
    int hh = (displayTs%86400)/3600, mm = (displayTs%3600)/60;
    char lbuf[16]; snprintf(lbuf,sizeof(lbuf),"AT %02d:%02d", hh, mm);
    tft.print(lbuf);
  } else {
    tft.print("LATEST");
  }
  y += 24;

  if (!yandexReachable || !yandexNow.valid) {
    tft.setTextColor(TFT_DARKGREY, bg); tft.setTextSize(2);
    tft.setCursor(8, y+20); tft.print("N/D");
    y += 90;
  } else {
    tft.setTextColor(TFT_WHITE, bg); tft.setTextSize(7);
    tft.setCursor(8, y);
    char tbuf[10]; snprintf(tbuf,sizeof(tbuf),"%.0f",yandexNow.temperature);
    tft.print(tbuf);
    tft.setTextSize(2); tft.print("C");

    tft.setTextSize(1); tft.setTextColor(TFT_DARKGREY, bg);
    int rx = 150;
    int ry = y;
    if (!isnan(yandexNow.feelsLike)) { tft.setCursor(rx, ry); tft.printf("Feels: %.1fC", yandexNow.feelsLike); ry += 14; }
    if (!isnan(yandexNow.windSpeed)) { tft.setCursor(rx, ry); tft.printf("Wind: %.1f m/s", yandexNow.windSpeed); ry += 14; }

    y += 80;
  }

  tft.drawFastHLine(0, y, 240, tft.color565(30,30,30));
  y += 20;

  // --- Блок "через 2 часа от момента запроса" ---
  tft.setTextColor(TFT_DARKGREY, bg); tft.setTextSize(2);
  tft.setCursor(4, y);
  if (yandexPlus2h.valid) {
    long displayTs = yandexPlus2h.time + (long)TZ_DISPLAY_OFFSET_HOURS*3600L;
    int hh = (displayTs%86400)/3600, mm = (displayTs%3600)/60;
    char lbuf[16]; snprintf(lbuf,sizeof(lbuf),"AT %02d:%02d", hh, mm);
    tft.print(lbuf);
  } else {
    tft.print("+2H");
  }
  y += 24;

  if (!yandexReachable || !yandexPlus2h.valid) {
    tft.setTextColor(TFT_DARKGREY, bg); tft.setTextSize(2);
    tft.setCursor(8, y+20); tft.print("N/D");
  } else {
    tft.setTextColor(TFT_WHITE, bg); tft.setTextSize(7);
    tft.setCursor(8, y);
    char tbuf[10]; snprintf(tbuf,sizeof(tbuf),"%.0f",yandexPlus2h.temperature);
    tft.print(tbuf);
    tft.setTextSize(2); tft.print("C");

    tft.setTextSize(1); tft.setTextColor(TFT_DARKGREY, bg);
    int rx = 150;
    if (!isnan(yandexPlus2h.windSpeed)) { tft.setCursor(rx, y); tft.printf("Wind: %.1f m/s", yandexPlus2h.windSpeed); }
  }

  drawCarouselDots(3);
}

// ============================================================
//  ЭКРАН 4: Яндекс — прогноз на 2 дня (today + tomorrow).
//  Бесплатный тариф отдаёт только эти 2 дня (forecasts[0]=today,
//  forecasts[1]=tomorrow) — подписываем явно словами, без претензии
//  на "послезавтра", которого на этом тарифе физически нет.
// ============================================================
// ============================================================
void drawScreenYandexForecast() {
  uint16_t bg = TFT_BLACK;
  tft.fillScreen(bg);
  drawHeaderBar(bg);

  tft.setTextColor(TFT_CYAN, bg); tft.setTextSize(2);
  tft.setCursor(4, HEADER_H+8); tft.print("YANDEX FORECAST");
  tft.drawFastHLine(0, HEADER_H+26, 240, TFT_DARKGREY);

  int y = HEADER_H + 36;

  // Бесплатный тариф отдаёт только 2 дня — сегодня и завтра (forecasts[0]/[1]),
  // подписываем явно словами, без претензии на "послезавтра"
  const char* dayLabels[2] = {"TODAY", "TOMORROW"};

  for (int d = 0; d < 2; d++) {
    tft.setTextColor(TFT_DARKGREY, bg); tft.setTextSize(2);
    tft.setCursor(4, y);
    tft.print(dayLabels[d]);
    y += 24;

    if (!yandexReachable || !yandexDays[d].valid) {
      tft.setTextColor(TFT_DARKGREY, bg); tft.setTextSize(2);
      tft.setCursor(8, y+20); tft.print("N/D");
      y += 100;
      continue;
    }

    tft.setTextColor(TFT_WHITE, bg); tft.setTextSize(5);
    int dayX = 8;
    tft.setCursor(dayX, y);
    char dbuf[10]; snprintf(dbuf,sizeof(dbuf),"%.0f",yandexDays[d].tempDay);
    tft.print(dbuf);
    tft.setTextSize(1); tft.print("C");

    // Подпись "day" точно под цифрой (тот же X, что начало числа), под низом глифа
    tft.setTextColor(TFT_DARKGREY, bg);
    tft.setCursor(dayX, y + 8*5 + 2); // 8*textSize = высота глифа цифры, +2 небольшой зазор
    tft.print("day");

    if (!isnan(yandexDays[d].tempNight)) {
      // Фиксированная позиция правее середины экрана (120px), с запасом от
      // края при экстремальных значениях (двузначный минус "-15" и т.п.) —
      // не зависит от длины дневной температуры, чтобы не "скакать" по X.
      int nightX = 130;
      tft.setTextSize(5); tft.setTextColor(TFT_WHITE, bg);
      tft.setCursor(nightX, y);
      char nbuf[10]; snprintf(nbuf,sizeof(nbuf),"%.0f",yandexDays[d].tempNight);
      tft.print(nbuf);
      tft.setTextSize(1); tft.print("C");
      tft.setTextColor(TFT_DARKGREY, bg);
      tft.setCursor(nightX, y + 8*5 + 2); // под цифрой ночной температуры, тот же принцип
      tft.print("nt");
    }

    y += 56;
    y += 20;
    tft.drawFastHLine(0, y, 240, tft.color565(30,30,30));
    y += 10;
  }

  drawCarouselDots(4);
}

// ============================================================
//  ЭКРАН 5: Служебный экран — список ID/расстояний/возраста датчиков narodmon
// ============================================================
void drawScreenService() {
  uint16_t bg = TFT_BLACK;
  tft.fillScreen(bg);
  drawHeaderBar(bg);
  tft.setTextColor(TFT_CYAN,bg); tft.setTextSize(1);
  tft.setCursor(4,HEADER_H+8); tft.print("DIAGNOSTICS");
  tft.drawFastHLine(0,HEADER_H+20,240,TFT_DARKGREY);

  // Адрес веб-интерфейса (настройки/датчики/логи) — чтобы не лезть в Serial Monitor
  tft.setTextColor(TFT_GREENYELLOW,bg);
  tft.setCursor(4,HEADER_H+24);
  if (WiFi.status()==WL_CONNECTED) tft.print("Web: " + WiFi.localIP().toString());
  else tft.print("Web: not connected");
  tft.drawFastHLine(0,HEADER_H+34,240,TFT_DARKGREY);

  long nowTs = (long)timeClient.getEpochTime();
  int y = HEADER_H + 42;
  for (int i=0; i<S_COUNT; i++) {
    if (y>300) break;
    tft.setTextColor(TFT_WHITE,bg);
    tft.setCursor(4,y); tft.print(sensorTypes[i].displayName); tft.print(":");
    tft.setCursor(130,y); tft.printf("r=%dkm t=%d/%d", searchRadiusByType[i], trendBuf[i].filledCount, TREND_SLOT_COUNT);
    y+=10;
    for (int k=0; k<rosterCount[i]; k++) {
      if (y>300) break;
      NmReading& r = nmCache[i][k];
      long ageMin = r.time>0 ? (nowTs-r.time)/60 : -1;
      char buf[56];
      if (r.time==0) snprintf(buf,sizeof(buf)," S%ld  %.1fkm  waiting",
        sensorRoster[i][k].id, sensorRoster[i][k].distance);
      else snprintf(buf,sizeof(buf)," S%ld  %.1fkm  %ldm%s",
        sensorRoster[i][k].id, sensorRoster[i][k].distance, ageMin,
        r.excludedAsZero ? " [0!]" : "");
      tft.setTextColor(r.excludedAsZero ? TFT_RED : (r.time>0?TFT_GREENYELLOW:TFT_DARKGREY), bg);
      tft.setCursor(4,y); tft.print(buf);
      y+=10;
    }
    if (rosterCount[i]==0) { tft.setTextColor(TFT_DARKGREY,bg); tft.setCursor(4,y); tft.print(" no sensors"); y+=10; }
    y+=2;
  }

  drawCarouselDots(5);
}
// ============================================================
//  Проверка "есть ли смысл показывать этот экран карусели".
//  Экран 0 (темп/влажность) — всегда показывается, температура
//  является якорем карусели даже с N/D.
// ============================================================
bool carouselScreenHasData(int idx) {
  if (idx == 0) return true; // якорь карусели

  if (idx == 1) return results[S_WIND_SPEED].valid || results[S_WIND_DIR].valid || results[S_PRESS].valid;
  if (idx == 2) return results[S_DUST].valid || results[S_RADIATION].valid || results[S_PRECIP].valid;
  if (idx == 3) return yandexNow.valid || yandexPlus2h.valid;
  if (idx == 4) return yandexDays[0].valid || yandexDays[1].valid;

  // idx == 5, служебный экран — показываем только если есть хоть один датчик
  for (int i=0; i<S_COUNT; i++) if (rosterCount[i] > 0) return true;
  return false;
}

// Есть ли алерт хоть по одному параметру, видимому НА ЭТОМ конкретном экране?
// (используется для удвоения времени показа — не общий anyAlarmActive,
// а именно то, что показано на текущем экране)
bool screenHasVisibleAlarm(int idx) {
  if (idx == 0) return results[S_TEMP].inAlarm || results[S_HUM].inAlarm;
  if (idx == 1) return results[S_WIND_SPEED].inAlarm || results[S_PRESS].inAlarm;
  if (idx == 2) return results[S_DUST].inAlarm || results[S_RADIATION].inAlarm;
  return false; // экраны Яндекса и служебный — алертов не показывают
}

// Общая функция отрисовки экрана по индексу — используется и обычной
// каруселью, и принудительным переключением кнопкой TTP223.
void drawScreenByIndex(int idx) {
  switch (idx) {
    case 0: drawScreenTempHum(); break;
    case 1: drawScreenWindPress(); break;
    case 2: drawScreenAirRadPrecip(); break;
    case 3: drawScreenYandexNow(); break;
    case 4: drawScreenYandexForecast(); break;
    case 5: drawScreenService(); break;
  }
  beepIfScreenHasAlarm(idx); // писк только в момент показа экрана с алертом, не постоянно
}

// ============================================================
//  TTP223 сенсорная кнопка — принудительное управление каруселью.
//  Логика: первое касание -> экран 0, активируется 30-секундное окно.
//  Повторное касание в этом окне -> следующий экран, окно сбрасывается
//  на новые 30 секунд. Без новых касаний 30 секунд -> override снимается,
//  обычная автокарусель продолжает с ТЕКУЩЕГО экрана (не сбрасывается).
// ============================================================
void handleTouchButton() {
  bool pinState = (digitalRead(PIN_TOUCH_BUTTON) == HIGH);

  // Простой debounce по времени: реагируем на изменение состояния
  // только если оно устойчиво держится TOUCH_DEBOUNCE_MS
  if (pinState != lastTouchPinState) {
    lastTouchChangeMs = millis();
    lastTouchPinState = pinState;
    return; // ждём следующего вызова для подтверждения
  }

  static bool touchHandledThisPress = false;
  bool stable = (millis() - lastTouchChangeMs) > TOUCH_DEBOUNCE_MS;

  if (pinState && stable && !touchHandledThisPress) {
    // Зафиксировано новое нажатие (фронт, подтверждённый debounce)
    touchHandledThisPress = true;

    if (!touchOverrideActive) {
      // Первое нажатие — принудительно показываем экран 0
      touchOverrideActive = true;
      carouselIndex = 0;
    } else {
      // Уже в режиме override — двигаемся на следующий экран
      do {
        carouselIndex = (carouselIndex + 1) % TOTAL_SCREENS;
      } while (!carouselScreenHasData(carouselIndex));
    }

    drawScreenByIndex(carouselIndex);
    lastCarouselSwitch = millis();
    touchOverrideUntil = millis() + TOUCH_OVERRIDE_TIMEOUT_MS;
  }

  if (!pinState) {
    touchHandledThisPress = false; // кнопка отпущена — готовы к следующему нажатию
  }

  // Истекло 30-секундное окно — снимаем override, дальше обычная карусель
  // продолжает с ТЕКУЩЕГО экрана (carouselIndex не трогаем)
  if (touchOverrideActive && millis() > touchOverrideUntil) {
    touchOverrideActive = false;
    lastCarouselSwitch = millis(); // даём текущему экрану его обычное время показа с нуля
  }
}

void updateCarousel() {
  if (touchOverrideActive) return; // карусель не трогает экран, пока активен override от кнопки

  int delayS;
  if (carouselIndex == 5) {
    delayS = DELAY_SERVICE_SEC; // служебный экран — всегда короче, алертов не показывает
  } else {
    delayS = screenHasVisibleAlarm(carouselIndex) ? DELAY_ALARM_SEC : DELAY_NORMAL_SEC;
  }
  if (millis()-lastCarouselSwitch < (unsigned long)delayS*1000UL) return;
  lastCarouselSwitch = millis();

  drawScreenByIndex(carouselIndex);

  do {
    carouselIndex = (carouselIndex+1) % TOTAL_SCREENS;
  } while (!carouselScreenHasData(carouselIndex));
}
// ============================================================
//  ЛОГ-БУФЕР (для веб-вкладки "Логи") — кольцевой буфер строк с
//  таймстампом millis(). logPrintln/logPrintf — замена Serial.println/
//  Serial.printf по всему файлу: пишут и в Serial (как раньше), и сюда.
//  /api/logs отдаёт только записи не старше 5 минут.
// ============================================================
struct LogEntry {
  unsigned long ms;
  String line;
};
const int LOG_BUFFER_SIZE = 400; // с запасом на 5 минут при обычной частоте событий
LogEntry logBuffer[LOG_BUFFER_SIZE];
int logHead  = 0; // индекс СЛЕДУЮЩЕЙ свободной ячейки (циклически)
int logCount = 0; // сколько реально заполнено (<= LOG_BUFFER_SIZE)

void logRingPush(const String& s) {
  logBuffer[logHead] = { millis(), s };
  logHead = (logHead + 1) % LOG_BUFFER_SIZE;
  if (logCount < LOG_BUFFER_SIZE) logCount++;
}

void logPrintln(const String& s) {
  Serial.println(s);
  logRingPush(s);
}

void logPrintf(const char* fmt, ...) {
  char buf[200];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  String s(buf);
  s.trim(); // убираем завершающий \n — в буфере одна запись = одна строка
  logRingPush(s);
}

// HH:MM:SS в дисплейном часовом поясе (TZ_DISPLAY_OFFSET_HOURS), как в шапке экрана
String formatHHMMSS(long ts) {
  if (ts <= 0) return "--:--:--";
  long displayTs = ts + (long)TZ_DISPLAY_OFFSET_HOURS * 3600L;
  long secOfDay = ((displayTs % 86400L) + 86400L) % 86400L;
  int hh = secOfDay/3600, mm = (secOfDay%3600)/60, ss = secOfDay%60;
  char buf[10];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hh, mm, ss);
  return String(buf);
}

// ============================================================
//  НАСТРОЙКИ — ХРАНЕНИЕ В NVS (Preferences)
//  loadConfig() читает сохранённые значения поверх дефолтов из блока
//  SETTINGS выше (если ключа ещё нет в NVS — остаётся дефолт).
//  handleSettingsPost() пишет то, что пришло из веб-формы, сразу и в
//  переменные, и в NVS, затем перезагружает устройство, чтобы все
//  зависящие структуры (радиусы поиска, ростер и т.п.) переинициализи-
//  ровались с нуля без риска рассинхрона.
// ============================================================
Preferences prefs;

void loadConfig() {
  prefs.begin("wdcfg", true);
  WIFI_SSID   = prefs.getString("ssid", WIFI_SSID);
  WIFI_PASS   = prefs.getString("pass", WIFI_PASS);
  NM_UUID_SRC = prefs.getString("nm_uuid_src", NM_UUID_SRC);
  NM_API_KEY  = prefs.getString("nm_key", NM_API_KEY);
  YANDEX_WEATHER_KEY = prefs.getString("ya_key", YANDEX_WEATHER_KEY);

  MY_LAT = prefs.getDouble("lat", MY_LAT);
  MY_LON = prefs.getDouble("lon", MY_LON);
  TZ_DISPLAY_OFFSET_HOURS = prefs.getInt("tz_off", TZ_DISPLAY_OFFSET_HOURS);

  BASE_SEARCH_RADIUS         = prefs.getInt("base_r", BASE_SEARCH_RADIUS);
  MAX_SEARCH_RADIUS          = prefs.getInt("max_r", MAX_SEARCH_RADIUS);
  RADIUS_STEP_KM             = prefs.getInt("step_r", RADIUS_STEP_KM);
  MIN_SENSORS_TO_STOP_EXPAND = prefs.getInt("min_stop", MIN_SENSORS_TO_STOP_EXPAND);
  MIN_SENSORS_NEEDED         = prefs.getInt("min_need", MIN_SENSORS_NEEDED);
  NEAREST_COUNT              = prefs.getInt("nearest_n", NEAREST_COUNT);
  REQUEST_PERIOD_MIN         = prefs.getInt("req_period", REQUEST_PERIOD_MIN);
  REQUEST_OFFSET_MIN         = prefs.getInt("req_offset", REQUEST_OFFSET_MIN);

  FRESH_MINUTES            = prefs.getInt("fresh_min", FRESH_MINUTES);
  MIN_REQUERY_INTERVAL_MIN = prefs.getInt("requery_min", MIN_REQUERY_INTERVAL_MIN);

  IDW_POWER = prefs.getFloat("idw_pow", IDW_POWER);
  IDW_EPS   = prefs.getFloat("idw_eps", IDW_EPS);
  ZERO_SENSOR_DIFF_THRESHOLD = prefs.getFloat("zero_diff", ZERO_SENSOR_DIFF_THRESHOLD);

  DELAY_SERVICE_SEC = prefs.getInt("d_service", DELAY_SERVICE_SEC);
  DELAY_NORMAL_SEC  = prefs.getInt("d_normal", DELAY_NORMAL_SEC);
  DELAY_ALARM_SEC   = prefs.getInt("d_alarm", DELAY_ALARM_SEC);

  ALARM_RADIATION_MRH   = prefs.getFloat("al_rad", ALARM_RADIATION_MRH);
  ALARM_WIND_MS         = prefs.getFloat("al_wind", ALARM_WIND_MS);
  ALARM_TEMP_COLD_C     = prefs.getFloat("al_cold", ALARM_TEMP_COLD_C);
  ALARM_TEMP_HOT_C      = prefs.getFloat("al_hot", ALARM_TEMP_HOT_C);
  ALARM_DUST_UGM3       = prefs.getFloat("al_dust", ALARM_DUST_UGM3);
  ALARM_PRESS_LOW_MMHG  = prefs.getFloat("al_pl", ALARM_PRESS_LOW_MMHG);
  ALARM_PRESS_HIGH_MMHG = prefs.getFloat("al_ph", ALARM_PRESS_HIGH_MMHG);
  ALARM_HUM_HIGH_PCT    = prefs.getFloat("al_hum", ALARM_HUM_HIGH_PCT);

  PIN_LED          = prefs.getInt("pin_led", PIN_LED);
  PIN_BUZZER       = prefs.getInt("pin_buzz", PIN_BUZZER);
  PIN_TOUCH_BUTTON = prefs.getInt("pin_touch", PIN_TOUCH_BUTTON);
  TOUCH_OVERRIDE_TIMEOUT_MS = prefs.getULong("touch_ms", TOUCH_OVERRIDE_TIMEOUT_MS);
  ALARM_BEEP_INTERVAL_SEC   = prefs.getInt("beep_int", ALARM_BEEP_INTERVAL_SEC);
  ALARM_BEEP_DURATION_MS    = prefs.getInt("beep_dur", ALARM_BEEP_DURATION_MS);

  REBOOT_AFTER_STUCK_SEC = prefs.getULong("reboot_sec", REBOOT_AFTER_STUCK_SEC);

  WIFI_CONNECT_TIMEOUT_MIN = prefs.getInt("wifi_to", WIFI_CONNECT_TIMEOUT_MIN);
  AP_SSID            = prefs.getString("ap_ssid", AP_SSID);
  AP_PASSWORD        = prefs.getString("ap_pass", AP_PASSWORD);
  WEB_ADMIN_PASSWORD = prefs.getString("web_pass", WEB_ADMIN_PASSWORD);
  prefs.end();

  logPrintln("Config loaded from NVS (defaults kept for any key not yet saved)");
}

// ============================================================
//  ВЕБ-ИНТЕРФЕЙС
// ============================================================
WebServer server(80);
DNSServer  dnsServer;
unsigned long wifiConnectStartMs = 0;
unsigned long lastApStaRetry     = 0;
unsigned long lastApScreenRefresh= 0;

bool checkAuth() {
  if (WEB_ADMIN_PASSWORD.length() == 0) return true; // пароль не задан — без авторизации
  if (!server.authenticate("admin", WEB_ADMIN_PASSWORD.c_str())) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

String htmlEscape(const String& in) {
  String out; out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c=='&') out += "&amp;";
    else if (c=='<') out += "&lt;";
    else if (c=='>') out += "&gt;";
    else if (c=='"') out += "&quot;";
    else if (c=='\'') out += "&#39;";
    else out += c;
  }
  return out;
}

String pageStyle() {
  return "<style>"
    "body{font-family:sans-serif;background:#111;color:#eee;margin:0}"
    ".wrap{max-width:640px;margin:0 auto;padding:12px}"
    "nav{display:flex;gap:4px;margin-bottom:12px}"
    "nav a{flex:1;text-align:center;padding:10px;background:#222;color:#9cf;text-decoration:none;border-radius:6px;font-size:14px}"
    "nav a.active{background:#356;color:#fff;font-weight:bold}"
    ".card{background:#1c1c1c;border-radius:8px;padding:12px 16px;margin-bottom:12px}"
    ".card h2{margin-top:0;font-size:16px;color:#9cf}"
    ".row{display:flex;flex-direction:column;margin-bottom:10px}"
    ".row label{font-size:13px;color:#aaa;margin-bottom:3px}"
    ".row input{padding:8px;border-radius:5px;border:1px solid #444;background:#0c0c0c;color:#eee;font-size:14px}"
    ".hint{font-size:11px;color:#777;margin-top:2px}"
    "button{width:100%;padding:12px;background:#2a6;color:#fff;border:none;border-radius:6px;font-size:15px;margin-top:4px}"
    "table{width:100%;border-collapse:collapse;font-size:13px}"
    "th,td{padding:5px 6px;border-bottom:1px solid #333;text-align:left}"
    ".ok{color:#7d7}.bad{color:#e77}.mid{color:#dd7}"
    "</style>";
}

String pageHeader(const char* active) {
  String cls_settings = (strcmp(active,"settings")==0) ? "active" : "";
  String cls_sensors  = (strcmp(active,"sensors")==0)  ? "active" : "";
  String cls_logs     = (strcmp(active,"logs")==0)     ? "active" : "";
  return String("<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Weather Display</title>") + pageStyle() +
    "</head><body><div class='wrap'><nav>"
    "<a class='" + cls_settings + "' href='/settings'>Настройки</a>"
    "<a class='" + cls_sensors  + "' href='/sensors'>Датчики</a>"
    "<a class='" + cls_logs     + "' href='/logs'>Логи</a>"
    "</nav>";
}

String row(const String& label, const String& name, const String& value, const String& hint = "") {
  String h = "<div class='row'><label>" + label + "</label>"
    "<input type='text' name='" + name + "' value='" + htmlEscape(value) + "'>";
  if (hint.length()) h += "<span class='hint'>" + hint + "</span>";
  h += "</div>";
  return h;
}

void handleRoot() {
  server.sendHeader("Location", "/settings", true);
  server.send(302, "text/plain", "");
}

void handleSettingsGet() {
  if (!checkAuth()) return;
  String h = pageHeader("settings");
  h += "<form method='POST' action='/settings'>";

  h += "<div class='card'><h2>WiFi</h2>";
  h += row("SSID (имя домашней сети)", "ssid", WIFI_SSID);
  h += row("Пароль", "pass", WIFI_PASS);
  h += row("Таймаут до режима точки доступа при включении, мин", "wifi_to", String(WIFI_CONNECT_TIMEOUT_MIN),
           "только для ПЕРВОГО подключения после включения; если WiFi обрывается позже (уже после успешного старта) — устройство просто продолжает переподключаться, в AP не уходит");
  h += "</div>";

  h += "<div class='card'><h2>Точка доступа (fallback) и веб-интерфейс</h2>";
  h += row("SSID точки доступа", "ap_ssid", AP_SSID);
  h += row("Пароль точки доступа", "ap_pass", AP_PASSWORD, "минимум 8 символов");
  h += row("Пароль веб-интерфейса", "web_pass", WEB_ADMIN_PASSWORD, "пусто = без пароля; логин всегда admin");
  h += "</div>";

  h += "<div class='card'><h2>narodmon.ru</h2>";
  h += row("UUID seed приложения", "nm_uuid_src", NM_UUID_SRC);
  h += row("API key", "nm_key", NM_API_KEY);
  h += "</div>";

  h += "<div class='card'><h2>Яндекс.Погода</h2>";
  h += row("API key", "ya_key", YANDEX_WEATHER_KEY);
  h += "</div>";

  h += "<div class='card'><h2>Координаты и часовой пояс</h2>";
  h += row("Широта (lat)", "lat", String(MY_LAT, 6));
  h += row("Долгота (lon)", "lon", String(MY_LON, 6));
  h += row("Смещение часового пояса на экране, ч", "tz_off", String(TZ_DISPLAY_OFFSET_HOURS));
  h += "</div>";

  h += "<div class='card'><h2>Поиск датчиков narodmon</h2>";
  h += row("Начальный радиус, км", "base_r", String(BASE_SEARCH_RADIUS));
  h += row("Максимальный радиус, км", "max_r", String(MAX_SEARCH_RADIUS));
  h += row("Шаг расширения, км", "step_r", String(RADIUS_STEP_KM));
  h += row("Мин. датчиков, чтобы прекратить расширение", "min_stop", String(MIN_SENSORS_TO_STOP_EXPAND));
  h += row("Мин. датчиков для показа значения", "min_need", String(MIN_SENSORS_NEEDED));
  h += row("Датчиков на тип (макс. " + String(MAX_ROSTER) + ")", "nearest_n", String(NEAREST_COUNT));
  h += row("Период опроса, мин (для нескольких устройств)", "req_period", String(REQUEST_PERIOD_MIN));
  h += row("Смещение опроса, мин", "req_offset", String(REQUEST_OFFSET_MIN));
  h += "</div>";

  h += "<div class='card'><h2>Свежесть и усреднение</h2>";
  h += row("Данные считаются устаревшими через, мин", "fresh_min", String(FRESH_MINUTES));
  h += row("Мин. интервал переопроса одного датчика, мин", "requery_min", String(MIN_REQUERY_INTERVAL_MIN));
  h += row("IDW степень", "idw_pow", String(IDW_POWER, 2));
  h += row("IDW эпсилон, км", "idw_eps", String(IDW_EPS, 2));
  h += row("Порог отбраковки нулевого датчика", "zero_diff", String(ZERO_SENSOR_DIFF_THRESHOLD, 2));
  h += "</div>";

  h += "<div class='card'><h2>Время показа экранов, сек</h2>";
  h += row("Служебный экран", "d_service", String(DELAY_SERVICE_SEC));
  h += row("Обычный экран", "d_normal", String(DELAY_NORMAL_SEC));
  h += row("Экран с алертом", "d_alarm", String(DELAY_ALARM_SEC));
  h += "</div>";

  h += "<div class='card'><h2>Пороги алертов</h2>";
  h += row("Радиация, mR/h", "al_rad", String(ALARM_RADIATION_MRH,1));
  h += row("Ветер, m/s", "al_wind", String(ALARM_WIND_MS,1));
  h += row("Холод, °C", "al_cold", String(ALARM_TEMP_COLD_C,1));
  h += row("Жара, °C", "al_hot", String(ALARM_TEMP_HOT_C,1));
  h += row("Запылённость, ug/m3", "al_dust", String(ALARM_DUST_UGM3,1));
  h += row("Давление, нижний порог, mmHg", "al_pl", String(ALARM_PRESS_LOW_MMHG,1));
  h += row("Давление, верхний порог, mmHg", "al_ph", String(ALARM_PRESS_HIGH_MMHG,1));
  h += row("Влажность, верхний порог, %", "al_hum", String(ALARM_HUM_HIGH_PCT,1));
  h += "</div>";

  h += "<div class='card'><h2>LED / зуммер / кнопка</h2>";
  h += row("Пин LED", "pin_led", String(PIN_LED), "требует перепайки и перезагрузки");
  h += row("Пин зуммера", "pin_buzz", String(PIN_BUZZER), "требует перепайки и перезагрузки");
  h += row("Пин сенсорной кнопки", "pin_touch", String(PIN_TOUCH_BUTTON), "требует перепайки и перезагрузки");
  h += row("Окно ручного управления кнопкой, мс", "touch_ms", String((unsigned long)TOUCH_OVERRIDE_TIMEOUT_MS));
  h += row("Интервал писка при алерте, сек", "beep_int", String(ALARM_BEEP_INTERVAL_SEC));
  h += row("Длительность писка, мс", "beep_dur", String(ALARM_BEEP_DURATION_MS));
  h += "</div>";

  h += "<div class='card'><h2>Watchdog</h2>";
  h += row("Перезагрузка, если проблема держится дольше, сек", "reboot_sec", String((unsigned long)REBOOT_AFTER_STUCK_SEC));
  h += "</div>";

  h += "<div class='card'><button type='submit'>Сохранить и перезагрузить</button></div>";
  h += "</form></div></body></html>";
  server.send(200, "text/html; charset=utf-8", h);
}

// Небольшие хелперы для handleSettingsPost — читают поле формы (если оно
// пришло), обновляют переменную в памяти И тут же пишут в NVS.
void setStr(const char* argName, const char* nvsKey, String& target) {
  if (server.hasArg(argName)) { target = server.arg(argName); prefs.putString(nvsKey, target); }
}
void setInt(const char* argName, const char* nvsKey, int& target) {
  if (server.hasArg(argName)) { target = server.arg(argName).toInt(); prefs.putInt(nvsKey, target); }
}
void setFloat(const char* argName, const char* nvsKey, float& target) {
  if (server.hasArg(argName)) { target = server.arg(argName).toFloat(); prefs.putFloat(nvsKey, target); }
}
void setDouble(const char* argName, const char* nvsKey, double& target) {
  // ESP32 String не имеет toDouble() на всех версиях ядра — toFloat()
  // достаточно точен для широты/долготы (см. комментарий у MY_LAT/MY_LON)
  if (server.hasArg(argName)) { target = (double)server.arg(argName).toFloat(); prefs.putDouble(nvsKey, target); }
}
void setULong(const char* argName, const char* nvsKey, unsigned long& target) {
  // toInt() достаточно — оба текущих unsigned long поля (таймауты в мс/сек)
  // далеко в пределах диапазона long, toDouble() есть не во всех версиях ядра
  if (server.hasArg(argName)) { target = (unsigned long)server.arg(argName).toInt(); prefs.putULong(nvsKey, target); }
}

void handleSettingsPost() {
  if (!checkAuth()) return;
  prefs.begin("wdcfg", false);

  setStr("ssid", "ssid", WIFI_SSID);
  setStr("pass", "pass", WIFI_PASS);
  setStr("ap_ssid", "ap_ssid", AP_SSID);
  setStr("ap_pass", "ap_pass", AP_PASSWORD);
  setStr("web_pass", "web_pass", WEB_ADMIN_PASSWORD);
  setInt("wifi_to", "wifi_to", WIFI_CONNECT_TIMEOUT_MIN);

  setStr("nm_uuid_src", "nm_uuid_src", NM_UUID_SRC);
  setStr("nm_key", "nm_key", NM_API_KEY);
  setStr("ya_key", "ya_key", YANDEX_WEATHER_KEY);

  setDouble("lat", "lat", MY_LAT);
  setDouble("lon", "lon", MY_LON);
  setInt("tz_off", "tz_off", TZ_DISPLAY_OFFSET_HOURS);

  setInt("base_r", "base_r", BASE_SEARCH_RADIUS);
  setInt("max_r", "max_r", MAX_SEARCH_RADIUS);
  setInt("step_r", "step_r", RADIUS_STEP_KM);
  setInt("min_stop", "min_stop", MIN_SENSORS_TO_STOP_EXPAND);
  setInt("min_need", "min_need", MIN_SENSORS_NEEDED);
  setInt("nearest_n", "nearest_n", NEAREST_COUNT);
  setInt("req_period", "req_period", REQUEST_PERIOD_MIN);
  setInt("req_offset", "req_offset", REQUEST_OFFSET_MIN);

  setInt("fresh_min", "fresh_min", FRESH_MINUTES);
  setInt("requery_min", "requery_min", MIN_REQUERY_INTERVAL_MIN);
  setFloat("idw_pow", "idw_pow", IDW_POWER);
  setFloat("idw_eps", "idw_eps", IDW_EPS);
  setFloat("zero_diff", "zero_diff", ZERO_SENSOR_DIFF_THRESHOLD);

  setInt("d_service", "d_service", DELAY_SERVICE_SEC);
  setInt("d_normal", "d_normal", DELAY_NORMAL_SEC);
  setInt("d_alarm", "d_alarm", DELAY_ALARM_SEC);

  setFloat("al_rad", "al_rad", ALARM_RADIATION_MRH);
  setFloat("al_wind", "al_wind", ALARM_WIND_MS);
  setFloat("al_cold", "al_cold", ALARM_TEMP_COLD_C);
  setFloat("al_hot", "al_hot", ALARM_TEMP_HOT_C);
  setFloat("al_dust", "al_dust", ALARM_DUST_UGM3);
  setFloat("al_pl", "al_pl", ALARM_PRESS_LOW_MMHG);
  setFloat("al_ph", "al_ph", ALARM_PRESS_HIGH_MMHG);
  setFloat("al_hum", "al_hum", ALARM_HUM_HIGH_PCT);

  setInt("pin_led", "pin_led", PIN_LED);
  setInt("pin_buzz", "pin_buzz", PIN_BUZZER);
  setInt("pin_touch", "pin_touch", PIN_TOUCH_BUTTON);
  setULong("touch_ms", "touch_ms", TOUCH_OVERRIDE_TIMEOUT_MS);
  setInt("beep_int", "beep_int", ALARM_BEEP_INTERVAL_SEC);
  setInt("beep_dur", "beep_dur", ALARM_BEEP_DURATION_MS);

  setULong("reboot_sec", "reboot_sec", REBOOT_AFTER_STUCK_SEC);

  prefs.end();
  logPrintln("Settings saved via web UI, rebooting to apply");

  String html = String("<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Сохранено</title>") + pageStyle() + "</head><body><div class='wrap'>"
    "<div class='card'><h2>Сохранено</h2>"
    "<p>Настройки записаны, устройство перезагружается.</p>"
    "<p>Если менялись SSID/пароль домашней WiFi — эта страница станет недоступна "
    "по старому адресу. Если новая сеть недоступна, через " + String(WIFI_CONNECT_TIMEOUT_MIN) +
    " мин. устройство снова само поднимет точку доступа.</p></div></div></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
  delay(600);
  ESP.restart();
}

void handleSensorsPage() {
  if (!checkAuth()) return;
  String h = pageHeader("sensors");
  h += "<div class='card'><button onclick='load()'>Обновить</button></div>"
       "<div id='content' class='card'>Загрузка...</div></div>"
       "<script>"
       "function esc(s){return s==null?'':String(s);}"
       "function load(){"
       "fetch('/api/sensors').then(r=>r.json()).then(d=>{"
       "var h='<p>Обновлено: '+esc(d.now)+'</p><table><tr><th>Параметр</th><th>Значение</th><th>Датчики</th></tr>';"
       "d.params.forEach(function(p){"
       "var v=p.valid?(Math.round(p.value*100)/100)+' '+p.unit:'N/D';"
       "var cls=p.alarm?'bad':(p.valid?'ok':'mid');"
       "var sensors=p.sensors.map(function(s){"
       "var age=s.ageMin<0?'ждём':s.ageMin+'м';"
       "return 'S'+s.id+' ('+s.distanceKm.toFixed(1)+'км, '+age+')'+(s.excludedAsZero?' [0!]':'');"
       "}).join('<br>');"
       "h+='<tr><td>'+p.name+'</td><td class=\"'+cls+'\">'+v+'</td><td>'+(sensors||'—')+'</td></tr>';"
       "});"
       "h+='</table><h2 style=\"color:#9cf;margin-top:14px\">Яндекс.Погода</h2>';"
       "if(d.yandex.now.valid) h+='<p>Сейчас ('+d.yandex.now.fetchedAt+'): '+d.yandex.now.temp+'&deg;C, ощущается '+d.yandex.now.feelsLike+'&deg;C, ветер '+d.yandex.now.wind+' м/с</p>';"
       "else h+='<p>Нет данных</p>';"
       "if(d.yandex.plus2h.valid) h+='<p>Прогноз на '+d.yandex.plus2h.time+': '+d.yandex.plus2h.temp+'&deg;C, ветер '+d.yandex.plus2h.wind+' м/с</p>';"
       "d.yandex.days.forEach(function(dd){ if(dd.valid) h+='<p>'+(dd.label==\"today\"?\"Сегодня\":\"Завтра\")+': день '+dd.tempDay+'&deg;C / ночь '+dd.tempNight+'&deg;C</p>'; });"
       "document.getElementById('content').innerHTML=h;"
       "}).catch(function(e){document.getElementById('content').innerText='Ошибка загрузки: '+e;});"
       "}"
       "load(); setInterval(load, 15000);"
       "</script></body></html>";
  server.send(200, "text/html; charset=utf-8", h);
}

void handleApiSensors() {
  if (!checkAuth()) return;
  DynamicJsonDocument doc(6144);
  long nowTs = (long)timeClient.getEpochTime();
  doc["now"] = formatHHMMSS(nowTs);

  JsonArray params = doc.createNestedArray("params");
  for (int i = 0; i < S_COUNT; i++) {
    JsonObject p = params.createNestedObject();
    p["name"]  = sensorTypes[i].displayName;
    p["unit"]  = sensorTypes[i].unit;
    p["valid"] = results[i].valid;
    p["value"] = results[i].value;
    p["alarm"] = results[i].inAlarm;
    JsonArray sensors = p.createNestedArray("sensors");
    for (int k = 0; k < rosterCount[i]; k++) {
      JsonObject s = sensors.createNestedObject();
      s["id"] = sensorRoster[i][k].id;
      s["distanceKm"] = sensorRoster[i][k].distance;
      NmReading& r = nmCache[i][k];
      s["value"] = r.value;
      s["ageMin"] = r.time > 0 ? (int)((nowTs - r.time) / 60) : -1;
      s["excludedAsZero"] = r.excludedAsZero;
    }
  }

  JsonObject y = doc.createNestedObject("yandex");
  JsonObject yn = y.createNestedObject("now");
  yn["valid"] = yandexNow.valid;
  if (yandexNow.valid) {
    yn["temp"] = yandexNow.temperature;
    yn["feelsLike"] = yandexNow.feelsLike;
    yn["wind"] = yandexNow.windSpeed;
    yn["fetchedAt"] = formatHHMMSS(yandexNow.fetchedAtTs);
  }
  JsonObject yp = y.createNestedObject("plus2h");
  yp["valid"] = yandexPlus2h.valid;
  if (yandexPlus2h.valid) {
    yp["time"] = formatHHMMSS(yandexPlus2h.time);
    yp["temp"] = yandexPlus2h.temperature;
    yp["wind"] = yandexPlus2h.windSpeed;
  }
  JsonArray days = y.createNestedArray("days");
  const char* dayLabels[2] = {"today","tomorrow"};
  for (int d = 0; d < 2; d++) {
    JsonObject jd = days.createNestedObject();
    jd["label"] = dayLabels[d];
    jd["valid"] = yandexDays[d].valid;
    if (yandexDays[d].valid) {
      jd["tempDay"]   = yandexDays[d].tempDay;
      jd["tempNight"] = yandexDays[d].tempNight;
    }
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleLogsPage() {
  if (!checkAuth()) return;
  String h = pageHeader("logs");
  h += "<div class='card'><button onclick='load()'>Обновить</button></div>"
       "<div id='content' class='card'><table id='t'></table></div></div>"
       "<script>"
       "function load(){"
       "fetch('/api/logs').then(r=>r.json()).then(d=>{"
       "var h='<tr><th style=\"width:70px\">Время</th><th>Строка</th></tr>';"
       "d.lines.forEach(function(l){h+='<tr><td>'+l.t+'</td><td>'+l.line+'</td></tr>';});"
       "if(d.lines.length===0) h+='<tr><td colspan=2>Нет записей за последние 5 минут</td></tr>';"
       "document.getElementById('t').innerHTML=h;"
       "}).catch(function(e){document.getElementById('content').innerText='Ошибка загрузки: '+e;});"
       "}"
       "load();"
       "</script></body></html>";
  server.send(200, "text/html; charset=utf-8", h);
}

void handleApiLogs() {
  if (!checkAuth()) return;
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.createNestedArray("lines");
  unsigned long nowMs = millis();
  long nowTs = (long)timeClient.getEpochTime();
  int n = logCount;
  int startIdx = (logHead - n + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE; // индекс самой старой записи
  for (int k = 0; k < n; k++) {
    int idx = (startIdx + k) % LOG_BUFFER_SIZE;
    LogEntry& e = logBuffer[idx];
    unsigned long ageMs = nowMs - e.ms; // корректно даже через переполнение millis()
    if (ageMs > 5UL * 60 * 1000) continue; // старше 5 минут — пропускаем
    long entryTs = nowTs - (long)(ageMs / 1000);
    JsonObject o = arr.createNestedObject();
    o["t"] = formatHHMMSS(entryTs);
    o["line"] = e.line;
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// Стандартные пути, по которым разные ОС проверяют наличие интернета за
// точкой доступа — редиректим их на страницу настроек, чтобы у телефона/
// ноутбука сама открылось окно captive portal
void handleCaptiveRedirect() {
  server.sendHeader("Location", "http://192.168.4.1/settings", true);
  server.send(302, "text/plain", "");
}

void handleNotFound() {
  if (apModeActive) { handleCaptiveRedirect(); return; }
  server.send(404, "text/plain", "Not found");
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.on("/sensors", HTTP_GET, handleSensorsPage);
  server.on("/api/sensors", HTTP_GET, handleApiSensors);
  server.on("/logs", HTTP_GET, handleLogsPage);
  server.on("/api/logs", HTTP_GET, handleApiLogs);
  server.on("/generate_204", handleCaptiveRedirect);        // Android
  server.on("/gen_204", handleCaptiveRedirect);
  server.on("/hotspot-detect.html", handleCaptiveRedirect); // Apple
  server.on("/library/test/success.html", handleCaptiveRedirect);
  server.on("/connecttest.txt", handleCaptiveRedirect);     // Windows
  server.on("/ncsi.txt", handleCaptiveRedirect);
  server.onNotFound(handleNotFound);
  server.begin();
  logPrintln("Web server started");
}

// ============================================================
//  AP FALLBACK — если за WIFI_CONNECT_TIMEOUT_MIN минут не удалось
//  подключиться к сохранённой сети, поднимаем свою точку доступа и
//  показываем инструкцию на экране. В фоне продолжаем раз в 30 секунд
//  пробовать подключиться к домашней сети — если получилось (например,
//  роутер просто перезагружался) — сами выходим из AP-режима без
//  участия пользователя.
// ============================================================
void drawScreenApSetup() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK); tft.setTextSize(2);
  tft.setCursor(4,8); tft.println("WIFI NOT FOUND");
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(1);
  tft.setCursor(4,32); tft.println("Setup mode. To configure:");

  tft.setCursor(4,50); tft.println("1. Connect to WiFi network:");
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK); tft.setTextSize(2);
  tft.setCursor(4,62); tft.println(AP_SSID);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(1);
  tft.setCursor(4,84); tft.print("Password: "); tft.println(AP_PASSWORD);

  tft.setCursor(4,104); tft.println("2. Open in browser:");
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK); tft.setTextSize(2);
  tft.setCursor(4,116); tft.println("192.168.4.1");
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(1);

  tft.setCursor(4,142); tft.println("3. Enter your home WiFi");
  tft.setCursor(4,152); tft.println("   name + password, Save.");
  tft.setCursor(4,166); tft.println("Device reboots and retries");
  tft.setCursor(4,176); tft.println("automatically afterwards.");

  unsigned long waitMin = (millis() - wifiConnectStartMs) / 60000UL;
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(4,200); tft.printf("Waiting %lum. Retrying WiFi", waitMin);
  tft.setCursor(4,210); tft.println("in background every 30s.");
}

void startApFallback() {
  logPrintln("WiFi: " + String(WIFI_CONNECT_TIMEOUT_MIN) + " min elapsed, no connection -> starting AP fallback (" + AP_SSID + ")");
  apModeActive = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID.c_str(), AP_PASSWORD.c_str());
  delay(100);
  dnsServer.start(53, "*", WiFi.softAPIP());
  drawScreenApSetup();
  lastApScreenRefresh = millis();
  lastApStaRetry = millis();
}

void exitApFallback() {
  logPrintln("WiFi: connection restored (" + WiFi.localIP().toString() + "), leaving AP fallback mode");
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apModeActive = false;
  wifiEverConnectedSinceBoot = true;
  wifiConnectStartMs = 0;
  drawScreenByIndex(carouselIndex);
  lastCarouselSwitch = millis();
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200); delay(300);
  wifiConnectStartMs = millis(); // отсюда отсчитываем WIFI_CONNECT_TIMEOUT_MIN до AP fallback
  logPrintln("\n=== Weather Display v4.0 (narodmon + Яндекс.Погода + web) ===");

  loadConfig(); // подтягиваем сохранённые в NVS настройки поверх дефолтов из блока SETTINGS

  for (int i = 0; i < S_COUNT; i++) searchRadiusByType[i] = BASE_SEARCH_RADIUS;

  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_TOUCH_BUTTON, INPUT); // TTP223 имеет собственный выходной драйвер, внешняя подтяжка не нужна

  // Один писк + одно мигание светодиодом при загрузке — проверка подключения
  digitalWrite(PIN_LED, HIGH);
  tone(PIN_BUZZER, 2000, 200);
  delay(250);
  digitalWrite(PIN_LED, LOW);
  noTone(PIN_BUZZER);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(4,4); tft.println("Starting...");

  nmUUID = md5String(String(NM_UUID_SRC));
  if (NEAREST_COUNT > MAX_ROSTER) NEAREST_COUNT = MAX_ROSTER;

  connectWiFi();
  tft.setCursor(4,14); tft.print("WiFi... ");
  tft.println(WiFi.status()==WL_CONNECTED?"OK":"FAIL");

  setupWebServer(); // поднимаем независимо от результата WiFi — станет доступен,
                     // как только появится STA-подключение или (позже) AP fallback

  timeClient.begin(); timeClient.forceUpdate();

  tft.setCursor(4,24); tft.println("Sensor types...");
  if (WiFi.status()==WL_CONNECTED) typeCodesLoaded = loadSensorTypeCodes();
  tft.setCursor(4,34); tft.println(typeCodesLoaded?"Types: OK":"Types: FAIL");

  // Яндекс.Погода — НЕ запрашиваем сразу при старте, первый запрос только
  // через YANDEX_STARTUP_DELAY_MS (5 минут) после включения, см. loop().
  tft.setCursor(4,44); tft.println("Yandex Weather: waiting 5 min...");
  bootMillis = millis();

  if (typeCodesLoaded && WiFi.status()==WL_CONNECTED) {
    tft.setCursor(4,54); tft.println("Searching sensors...");
    refreshSensorRoster(false);
    lastNearbyHour = ((long)timeClient.getEpochTime()/3600)%24;
    lastNearbyHour = (lastNearbyHour/6)*6;
  }

  computeResults();
  lastRequestMinute = ((long)timeClient.getEpochTime()/60)%1440;

  drawScreenTempHum();
  lastCarouselSwitch = millis();
}

void loop() {
  // --- WiFi: обычное подключение, либо AP fallback — ТОЛЬКО если устройство
  // ещё НИ РАЗУ не подключалось с момента включения (см. wifiEverConnectedSinceBoot).
  // Обрыв связи ПОСЛЕ хотя бы одного успешного подключения — это уже не "не
  // настроено", а обычный временный сбой сети: просто продолжаем ретраить
  // connectWiFi() бесконечно, без ухода в AP и без прерывания карусели экранов.
  if (!apModeActive) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiEverConnectedSinceBoot = true;
    } else if (!wifiEverConnectedSinceBoot) {
      unsigned long timeoutMs = (unsigned long)WIFI_CONNECT_TIMEOUT_MIN * 60UL * 1000UL;
      if (millis() - wifiConnectStartMs > timeoutMs) {
        startApFallback();
      } else {
        connectWiFi(); // блокирующая попытка ~15 сек, как раньше
      }
    } else {
      connectWiFi(); // уже подключались раньше — обычный бесконечный ретрай, без AP
    }
  } else {
    // В AP-режиме раз в 30 секунд незаметно пробуем восстановить основную
    // сеть в фоне (вдруг роутер вернулся) — пока это не мешает работе
    // веб-интерфейса на самой точке доступа (WIFI_AP_STA — оба режима сразу).
    if (millis() - lastApStaRetry > 30000UL) {
      lastApStaRetry = millis();
      if (WiFi.status() != WL_CONNECTED) WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());
    }
    if (WiFi.status() == WL_CONNECTED) {
      exitApFallback();
    } else if (millis() - lastApScreenRefresh > 5000UL) {
      lastApScreenRefresh = millis();
      drawScreenApSetup(); // обновляем "waiting Xm" на экране
    }
  }

  server.handleClient();
  if (apModeActive) dnsServer.processNextRequest();

  timeClient.update();

  if (!typeCodesLoaded && WiFi.status()==WL_CONNECTED)
    typeCodesLoaded = loadSensorTypeCodes();

  bool rosterJustRefreshed = false;

  // Обычный полный обзор раз в 6 часов (все 8 типов, попытка сузить радиус)
  if (typeCodesLoaded && isNearbyRefreshDue() && WiFi.status()==WL_CONNECTED) {
    refreshSensorRoster(false);
    rosterJustRefreshed = true;
  }

  // Быстрый цикл расширения радиуса (каждые 2 минуты, до 2 датчиков на тип)
  if (typeCodesLoaded && WiFi.status()==WL_CONNECTED &&
      millis()-lastFastExpandCheck > FAST_EXPAND_INTERVAL_MS) {
    if (anyTypeNeedsExpansion()) {
      logPrintln("Fast expand: some types still short on sensors, retrying sooner");
      refreshSensorRoster(true);
      rosterJustRefreshed = true;
    }
    lastFastExpandCheck = millis();
  }

  // Запрос показаний по самым старым датчикам
  bool anyKnownSensor = false;
  for (int i=0; i<S_COUNT; i++) if (rosterCount[i] > 0) { anyKnownSensor = true; break; }

  if (!rosterJustRefreshed && typeCodesLoaded && anyKnownSensor &&
      isScheduledMinuteNow() && WiFi.status()==WL_CONNECTED) {
    fetchOldestSensors();
    computeResults();
  }

  // Яндекс.Погода: первый запрос — через 5 минут после старта. Далее —
  // адаптивно: за 30 минут до истечения актуальности текущего слота
  // "+2 часа" (nextYandexFetchTs считается внутри после каждого успешного
  // запроса). При неудаче — повтор через YANDEX_RETRY_AFTER_FAIL_MIN.
  if (WiFi.status()==WL_CONNECTED) {
    bool shouldFetchNow = false;

    if (!firstYandexFetchDone) {
      shouldFetchNow = (millis() - bootMillis >= YANDEX_STARTUP_DELAY_MS);
    } else {
      long nowTs = (long)timeClient.getEpochTime();
      shouldFetchNow = (nextYandexFetchTs > 0 && nowTs >= nextYandexFetchTs);
    }

    if (shouldFetchNow) {
      fetchYandexWeather();
      firstYandexFetchDone = true;

      long nowTs = (long)timeClient.getEpochTime();
      if (yandexReachable && yandexPlus2h.valid) {
        // Следующий запрос — за 30 минут до истечения актуальности этого слота
        nextYandexFetchTs = yandexPlus2h.time - (long)YANDEX_PRE_EXPIRY_MIN * 60;
        // Защита: если по какой-то причине это время уже в прошлом
        // (например, слот оказался слишком близко к "сейчас") — не зависаем
        // в цикле непрерывных запросов, даём хотя бы небольшую паузу.
        if (nextYandexFetchTs <= nowTs) nextYandexFetchTs = nowTs + (long)YANDEX_RETRY_AFTER_FAIL_MIN * 60;
      } else {
        // Неудача — пробуем снова через 30 минут, не ждём расчётного времени
        nextYandexFetchTs = nowTs + (long)YANDEX_RETRY_AFTER_FAIL_MIN * 60;
      }
    }
  }

  if (!apModeActive) {
    handleTouchButton();
    updateCarousel();
  }
  updateStatusLed();
  delay(200);
}
