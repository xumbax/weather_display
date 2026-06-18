/*
 * ============================================================
 *  ПОГОДНЫЙ ДИСПЛЕЙ — ESP32 + ST7789 (240x320)
 *  Берёт N ближайших публичных датчиков каждого типа с narodmon.ru,
 *  усредняет с весами по расстоянию (IDW), показывает карусель.
 *
 *  Типы датчиков: температура, влажность, давление, скорость ветра,
 *                 направление ветра, радиация, запылённость, осадки
 *
 *  ЛОГИКА:
 *    1. При старте — запрос appInit, получаем справочник типов
 *       (код type для каждого названия датчика). НЕ зашиваем числа
 *       руками — народмон не публикует их официально, видим только
 *       из реального ответа.
 *    2. По расписанию (см. ниже) — запрос sensorsNearby с фильтром
 *       по types, получаем все датчики этих типов в радиусе.
 *    3. Группируем по типу, берём NEAREST_COUNT ближайших СВЕЖИХ
 *       (<15 мин) датчиков каждого типа.
 *    4. Взвешенное усреднение: w_i = 1 / (d_i + EPS)^IDW_POWER
 *    5. Карусель из 9 экранов на TFT: температура — всегда первая,
 *       6 обычных параметров, 1 служебный (список ID датчиков).
 *    6. Светодиод — единая иерархия статуса (от грубого к точному):
 *       нет WiFi -> 1с, WiFi есть/нет API -> 3с, API есть/нет датчиков -> 5с,
 *       всё хорошо -> не горит. Погодная тревога перекрывает всё это:
 *       горит непрерывно + пищалка раз в 15 мин (не пищит при проблемах связи).
 *    7. На каждом экране сверху всегда: дата, время, температура.
 *    8. Тренд-индикатор на каждом из 8 параметров: 4 столбика (3ч/1ч/20мин/
 *       сейчас), высота = относительный уровень значения (7 уровней),
 *       не направление изменения. История хранится в RAM на основе
 *       собственных измерений устройства — без лишних запросов к серверу.
 *    9. Защитная перезагрузка: если индикатор непрерывно "мигает"
 *       (проблема WiFi/API/датчиков) дольше 30 минут — ESP.restart().
 *       Тревога погоды и нормальная работа НЕ считаются проблемой.
 *
 *  РАСПИСАНИЕ ЗАПРОСОВ (для нескольких устройств без коллизий):
 *    Запрос выполняется на минуте M от начала суток (UTC), когда
 *    (M % REQUEST_PERIOD_MIN) == REQUEST_OFFSET_MIN.
 *    Примеры для трёх устройств без пересечений:
 *      Device 1: period=1, offset=0  -> каждую минуту
 *      Device 2: period=2, offset=0  -> минуты 0,2,4,6...
 *      Device 3: period=3, offset=1  -> минуты 1,4,7,10...
 *    Работает одинаково каждые сутки автоматически — это просто
 *    арифметика по модулю от текущего UTC-времени, без необходимости
 *    отдельно отслеживать смену суток.
 *
 *  Библиотеки (Library Manager):
 *    - TFT_eSPI         by Bodmer
 *    - ArduinoJson      by Benoit Blanchon (v6)
 *    - NTPClient        by Fabrice Weinberg
 *
 *  ВАЖНО: TFT_eSPI настраивается через User_Setup.h в папке библиотеки,
 *  либо через User_Setup_Select.h — см. файл TFT_eSPI_User_Setup.h
 *  из комплекта, его нужно скопировать в библиотеку.
 *
 *  Лимит API народмона: не чаще 1 запроса/мин с одного ключа.
 *  REQUEST_PERIOD_MIN=1 — это и есть верхний предел частоты.
 * ============================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <MD5Builder.h>
#include <TFT_eSPI.h>

// ============================================================
//  НАСТРОЙКИ — ИЗМЕНИТЕ ПОД СЕБЯ
// ============================================================

const char* WIFI_SSID   = "ВАШ_WIFI";
const char* WIFI_PASS   = "ВАШ_ПАРОЛЬ";

const char* NM_API_KEY  = "ВАШ_API_KEY";       // narodmon.ru → Профиль → Мои приложения
const char* NM_UUID_SRC = "weather-display-01"; // любая строка → станет MD5

// Координаты точки наблюдения — ОБЯЗАТЕЛЬНО заменить на свои!
// Узнать свои координаты можно через Google Maps (клик правой кнопкой на точке)
double MY_LAT = 0.0;   // широта, например 55.7558
double MY_LON = 0.0;   // долгота, например 37.6173

// === НАСТРОЙКИ УСРЕДНЕНИЯ (можно менять без перепрошивки логики) ===
int    NEAREST_COUNT  = 5;     // сколько ближайших датчиков каждого типа брать
float  IDW_POWER      = 1.0;   // степень затухания веса от расстояния (p)
float  IDW_EPS        = 0.1;   // защита от деления на 0 при d→0 (км)
int    SEARCH_RADIUS  = 50;    // радиус поиска датчиков, км
int    FRESH_MINUTES  = 15;    // датчик считается активным если новее N минут

// === РАСПИСАНИЕ ЗАПРОСОВ (для разводки нескольких устройств) ===
// Запрос выполняется на минуте M (от начала суток UTC), когда
// (M % REQUEST_PERIOD_MIN) == REQUEST_OFFSET_MIN.
// Устройство 1: period=1, offset=0 (каждую минуту)
// Устройство 2: period=2, offset=0 (минуты 0,2,4,6...)
// Устройство 3: period=3, offset=1 (минуты 1,4,7,10...)
int REQUEST_PERIOD_MIN = 1;   // раз в сколько минут запрашивать (1, 2 или 3)
int REQUEST_OFFSET_MIN = 0;   // сдвиг, чтобы не совпадать с другими устройствами

// Карусель экранов
const int CAROUSEL_INTERVAL_SEC = 6;   // сколько секунд держим экран перед сменой
// Экран 0 = температура (всегда первый), экраны 1..7 = остальные параметры,
// экран 8 = служебный (список использованных датчиков)

// === ПОРОГИ ТРЕВОГ ===
const float ALARM_RADIATION_USVH   = 0.30;  // мкЗв/ч, выше — тревога радиации
const float ALARM_WIND_MS          = 20.0;  // м/с, шторм
const float ALARM_TEMP_COLD_C      = -30.0; // сильный мороз
const float ALARM_TEMP_HOT_C       = 35.0;  // сильная жара
const float ALARM_DUST_UGM3        = 150.0; // мкг/м3 PM2.5/PM10, пылевая буря

// === ИНДИКАЦИЯ ===
const int PIN_BUZZER = 25;   // пищалка (активный пьезо или транзисторный ключ)
const int PIN_LED    = 26;   // светодиод статуса связи + тревоги (см. updateStatusLed)
const int ALARM_BEEP_INTERVAL_SEC = 15 * 60;  // пищать раз в 15 минут при тревоге
const int ALARM_BEEP_DURATION_MS  = 300;

// TFT пины задаются в TFT_eSPI User_Setup.h, здесь не нужны

// ============================================================
//  Типы датчиков, которые ищем (название должно точно совпадать
//  с тем, что вернёт appInit в поле name; регистр важен)
// ============================================================
struct SensorTypeConfig {
  const char* searchName;   // как ищем в справочнике appInit
  const char* displayName;  // как показываем на экране
  const char* unit;         // единица для отображения (если не пришла с сервера)
  int   typeCode;           // код типа — заполняется после appInit, -1 если не найден
};

enum SensorIndex {
  S_TEMP = 0, S_HUM, S_PRESS, S_WIND_SPEED, S_WIND_DIR,
  S_RADIATION, S_DUST, S_PRECIP, S_COUNT
};

SensorTypeConfig sensorTypes[S_COUNT] = {
  { "t воздуха",    "Температура",     "°C",    -1 },
  { "RH влажность", "Влажность",       "%",     -1 },
  { "атм.давление", "Давление",        "мм рт.ст.", -1 },
  { "скорость",     "Скорость ветра",  "м/с",   -1 },
  { "направление",  "Направление ветра","°",    -1 },
  { "радиация",     "Радиация",        "мкЗв/ч",-1 },
  { "запыленность", "Запылённость",    "мкг/м3",-1 },
  { "осадки",       "Осадки",          "мм",    -1 },
};

// ============================================================
//  Результат усреднения для одного типа датчика
// ============================================================
const int MAX_USED_SENSORS = 5; // должно быть >= максимально возможного NEAREST_COUNT

struct UsedSensorInfo {
  long  id;       // ID датчика в проекте народмон
  float distance; // км
  long  time;     // unix time последнего показания
};

struct SensorResult {
  bool  valid       = false;  // удалось ли получить хоть один свежий датчик
  float value       = 0;      // взвешенное среднее
  int   usedCount    = 0;     // сколько датчиков участвовало
  float nearestDist  = 0;     // расстояние до самого близкого использованного
  UsedSensorInfo used[MAX_USED_SENSORS]; // детали по каждому использованному датчику
};

SensorResult results[S_COUNT];

// ============================================================
//  История значений для тренд-индикатора (3ч / 1ч / 20мин)
//  Кольцевой буфер в RAM на основе СОБСТВЕННЫХ измерений устройства —
//  не запрашиваем sensorsHistory у народмона (это умножило бы кол-во
//  запросов в разы и упёрлось бы в лимит 1/мин).
//  Глубина буфера рассчитана на худший случай (период=1 мин, нужно
//  покрыть 3 часа = 180 точек), с запасом.
// ============================================================
const int HISTORY_CAPACITY = 200;

struct HistoryPoint {
  long  time;   // unix time (UTC) измерения
  float value;  // значение в этот момент
};

struct HistoryBuffer {
  HistoryPoint points[HISTORY_CAPACITY];
  int   head  = 0;   // индекс следующей свободной ячейки (циклически)
  int   count = 0;   // сколько точек реально записано (<= CAPACITY)
};

HistoryBuffer history[S_COUNT];

// Добавить новую точку в историю типа idx (вызывается после каждого успешного fetchAndAverage)
void historyPush(int idx, long time, float value) {
  HistoryBuffer& h = history[idx];
  h.points[h.head] = { time, value };
  h.head = (h.head + 1) % HISTORY_CAPACITY;
  if (h.count < HISTORY_CAPACITY) h.count++;
}

// Найти точку из истории, ближайшую по времени к "targetTime".
// Возвращает false если в буфере вообще нет точек.
bool historyFindClosest(int idx, long targetTime, float& outValue) {
  HistoryBuffer& h = history[idx];
  if (h.count == 0) return false;

  long bestDiff = -1;
  float bestValue = 0;
  for (int k = 0; k < h.count; k++) {
    // head указывает на следующую свободную ячейку, т.е. реальные записи лежат
    // по индексам (head - count) .. (head - 1) по модулю CAPACITY
    int realIndex = (h.head - 1 - k + HISTORY_CAPACITY) % HISTORY_CAPACITY;
    long diff = abs((long)(h.points[realIndex].time - targetTime));
    if (bestDiff < 0 || diff < bestDiff) {
      bestDiff = diff;
      bestValue = h.points[realIndex].value;
    }
  }
  outValue = bestValue;
  return true;
}

// ============================================================
//  Глобальные объекты
// ============================================================
TFT_eSPI tft = TFT_eSPI();
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // чистый UTC
String nmUUID;

int  carouselIndex = 0;            // 0 = температура, дальше остальные, последний = служебный
unsigned long lastCarouselSwitch = 0;
long lastRequestMinute = -1;       // минута суток (UTC) последнего выполненного запроса
unsigned long lastAlarmBeep      = 0;
bool alarmActive  = false;
bool ledBlinkState = false;
unsigned long lastLedBlink = 0;

// === Статус связи для индикации светодиодом (заполняется в fetchAndAverage) ===
bool apiReachable   = false;  // последний запрос к народмону дошёл и распарсился без ошибки
bool anySensorFound  = false;  // хотя бы один датчик хотя бы одного типа дал валидный результат
bool typeCodesLoaded = false;  // справочник типов (appInit) успешно загружен хотя бы раз

// === Защитный таймаут: если индикатор непрерывно "мигает" (проблема связи/
//    API/датчиков, не "всё хорошо" и не "тревога") дольше REBOOT_AFTER_STUCK_SEC —
//    перезагружаемся. Лечит зависания WiFi-стека/памяти лучше внутренних ретраев.
const unsigned long REBOOT_AFTER_STUCK_SEC = 30 * 60; // 30 минут
unsigned long problemStateSince = 0;  // millis() момента, когда впервые увидели проблему; 0 = сейчас всё ок

// Список активных тревог (для отображения текста на экране)
String activeAlarmText = "";

// ============================================================
//  Проверка расписания: пора ли делать запрос на текущей минуте?
//  Минута считается от начала суток UTC: 0..1439
// ============================================================
bool isScheduledMinuteNow() {
  long nowTs = (long)timeClient.getEpochTime();
  long minuteOfDay = (nowTs / 60) % 1440;

  if (minuteOfDay == lastRequestMinute) return false; // уже делали запрос в эту минуту

  int period = max(1, REQUEST_PERIOD_MIN); // защита от 0
  int offset = REQUEST_OFFSET_MIN % period;

  if ((minuteOfDay % period) == offset) {
    lastRequestMinute = minuteOfDay;
    return true;
  }
  return false;
}

// ============================================================
//  MD5
// ============================================================
String md5String(const String& s) {
  MD5Builder md5;
  md5.begin();
  md5.add(s);
  md5.calculate();
  return md5.toString();
}

// ============================================================
//  WiFi
//  Не перезагружает плату при неудаче — возвращает управление в loop(),
//  чтобы продолжали работать индикация и карусель (с "нет данных").
//  Мигает светодиодом раз в секунду во время попытки подключения.
// ============================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi подключение");
  unsigned long start = millis();
  unsigned long lastBlink = 0;
  bool blinkState = false;
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    if (millis() - lastBlink > 500) {
      blinkState = !blinkState;
      digitalWrite(PIN_LED, blinkState ? HIGH : LOW);
      lastBlink = millis();
    }
    delay(50);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK  IP: " + WiFi.localIP().toString());
  } else {
    Serial.println(" пока не подключено, попробуем позже");
  }
}

// ============================================================
//  HTTP POST запрос к narodmon API, возвращает тело ответа
// ============================================================
String narodmonRequest(const String& body) {
  WiFiClient client;
  HTTPClient http;
  http.begin(client, "http://narodmon.ru/api");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("User-Agent", "ESP32WeatherDisplay");
  int code = http.POST(body);
  String resp = "";
  if (code == 200) {
    resp = http.getString();
  } else {
    Serial.printf("HTTP код: %d\n", code);
  }
  http.end();
  return resp;
}

// ============================================================
//  appInit — получить справочник типов датчиков, заполнить
//  sensorTypes[].typeCode по совпадению имени
// ============================================================
bool loadSensorTypeCodes() {
  String body = "{\"cmd\":\"appInit\",\"version\":\"1.0\",\"uuid\":\"" + nmUUID +
                "\",\"api_key\":\"" + String(NM_API_KEY) + "\",\"lang\":\"ru\"}";

  Serial.println("Запрос appInit (справочник типов)...");
  String resp = narodmonRequest(body);
  if (resp.length() == 0) {
    Serial.println("appInit: пустой ответ");
    return false;
  }

  DynamicJsonDocument doc(8192);
  DeserializationError err = deserializeJson(doc, resp);
  if (err) {
    Serial.println("appInit JSON ошибка: " + String(err.c_str()));
    return false;
  }
  if (doc.containsKey("error")) {
    Serial.println("appInit ошибка сервера: " + String(doc["error"].as<const char*>()));
    return false;
  }

  JsonArray types = doc["types"].as<JsonArray>();
  Serial.printf("Справочник типов получен: %d записей\n", types.size());

  // Для каждого нашего искомого типа ищем совпадение по имени
  for (int i = 0; i < S_COUNT; i++) {
    sensorTypes[i].typeCode = -1;
    for (JsonObject t : types) {
      const char* name = t["name"];
      if (name && String(name) == String(sensorTypes[i].searchName)) {
        sensorTypes[i].typeCode = t["type"].as<int>();
        Serial.printf("  %s -> type=%d\n", sensorTypes[i].displayName, sensorTypes[i].typeCode);
        break;
      }
    }
    if (sensorTypes[i].typeCode == -1) {
      Serial.printf("  ВНИМАНИЕ: тип \"%s\" не найден в справочнике!\n", sensorTypes[i].searchName);
    }
  }
  return true;
}

// ============================================================
//  Гаверсинус — расстояние между двумя точками на сфере (км)
//  (используется как запасной вариант, если сервер не вернул distance)
// ============================================================
float haversineKm(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return (float)(R * c);
}

// ============================================================
//  Структура для сбора кандидатов одного типа перед усреднением
// ============================================================
struct Candidate {
  long  id;
  float distance;
  float value;
  long  time;
};

// ============================================================
//  sensorsNearby — запросить все типы сразу, распарсить,
//  для каждого типа найти NEAREST_COUNT ближайших свежих,
//  посчитать взвешенное среднее (IDW)
// ============================================================
void fetchAndAverage() {
  // Собираем список найденных типов в массив для фильтра
  String typesList = "[";
  bool first = true;
  for (int i = 0; i < S_COUNT; i++) {
    if (sensorTypes[i].typeCode < 0) continue;
    if (!first) typesList += ",";
    typesList += String(sensorTypes[i].typeCode);
    first = false;
  }
  typesList += "]";

  if (typesList == "[]") {
    Serial.println("Нет ни одного известного типа датчика — пропускаем запрос");
    return;
  }

  String body = "{\"cmd\":\"sensorsNearby\","
                "\"lat\":" + String(MY_LAT, 6) + ","
                "\"lon\":" + String(MY_LON, 6) + ","
                "\"radius\":" + String(SEARCH_RADIUS) + ","
                "\"pub\":1,"
                "\"types\":" + typesList + ","
                "\"uuid\":\"" + nmUUID + "\","
                "\"api_key\":\"" + String(NM_API_KEY) + "\","
                "\"lang\":\"ru\"}";

  Serial.println("Запрос sensorsNearby...");
  String resp = narodmonRequest(body);
  if (resp.length() == 0) {
    Serial.println("sensorsNearby: пустой ответ (нет сети?)");
    apiReachable = false;
    return;
  }

  DynamicJsonDocument doc(32768);
  DeserializationError err = deserializeJson(doc, resp);
  if (err) {
    Serial.println("sensorsNearby JSON ошибка: " + String(err.c_str()));
    apiReachable = false;
    return;
  }
  if (doc.containsKey("error")) {
    Serial.println("sensorsNearby ошибка сервера: " + String(doc["error"].as<const char*>()));
    apiReachable = false;
    return;
  }

  // Дошли до сюда — народмон ответил и тело распарсилось без ошибок
  apiReachable = true;

  JsonArray devices = doc["devices"].as<JsonArray>();
  Serial.printf("Устройств в ответе: %d\n", devices.size());

  long nowTs = (long)timeClient.getEpochTime();

  // Для каждого нашего типа — список кандидатов
  static Candidate candidates[S_COUNT][40]; // максимум 40 кандидатов на тип, более чем достаточно
  int candidateCount[S_COUNT] = {0};

  for (JsonObject dev : devices) {
    float devDistance = dev["distance"].as<float>();
    JsonArray sensors = dev["sensors"].as<JsonArray>();
    for (JsonObject s : sensors) {
      int type = s["type"].as<int>();
      long sTime = s["time"].as<long>();
      float sValue = s["value"].as<float>();
      long sId = s["id"].as<long>();

      // Свежесть
      long ageMin = (nowTs - sTime) / 60;
      if (ageMin > FRESH_MINUTES || ageMin < -2) continue; // -2 допуск на дрейф часов

      // Найти к какому из наших индексов относится этот type
      for (int i = 0; i < S_COUNT; i++) {
        if (sensorTypes[i].typeCode == type) {
          if (candidateCount[i] < 40) {
            candidates[i][candidateCount[i]].id       = sId;
            candidates[i][candidateCount[i]].distance = devDistance;
            candidates[i][candidateCount[i]].value    = sValue;
            candidates[i][candidateCount[i]].time     = sTime;
            candidateCount[i]++;
          }
          break;
        }
      }
    }
  }

  // Для каждого типа: отсортировать по расстоянию, взять NEAREST_COUNT, усреднить с IDW
  for (int i = 0; i < S_COUNT; i++) {
    int n = candidateCount[i];
    if (n == 0) {
      results[i].valid = false;
      Serial.printf("%s: нет свежих публичных датчиков в радиусе\n", sensorTypes[i].displayName);
      continue;
    }

    // Простая сортировка по расстоянию (n мало, пузырьком достаточно)
    for (int a = 0; a < n - 1; a++) {
      for (int b = 0; b < n - 1 - a; b++) {
        if (candidates[i][b].distance > candidates[i][b+1].distance) {
          Candidate tmp = candidates[i][b];
          candidates[i][b] = candidates[i][b+1];
          candidates[i][b+1] = tmp;
        }
      }
    }

    int useCount = min(n, NEAREST_COUNT);
    float weightSum = 0;
    float valueSum = 0;
    for (int k = 0; k < useCount; k++) {
      float d = candidates[i][k].distance;
      float w = 1.0 / pow(d + IDW_EPS, IDW_POWER);
      weightSum += w;
      valueSum += w * candidates[i][k].value;

      if (k < MAX_USED_SENSORS) {
        results[i].used[k].id       = candidates[i][k].id;
        results[i].used[k].distance = candidates[i][k].distance;
        results[i].used[k].time     = candidates[i][k].time;
      }
    }

    results[i].valid = true;
    results[i].value = valueSum / weightSum;
    results[i].usedCount = useCount;
    results[i].nearestDist = candidates[i][0].distance;

    // Сохраняем точку в историю для тренд-индикатора (3ч/1ч/20мин)
    historyPush(i, nowTs, results[i].value);

    Serial.printf("%s: %.2f %s (из %d датчиков, ближний %.1f км)\n",
      sensorTypes[i].displayName, results[i].value, sensorTypes[i].unit,
      useCount, results[i].nearestDist);
  }

  // Обновить общий флаг: хотя бы один тип дал валидный результат?
  anySensorFound = false;
  for (int i = 0; i < S_COUNT; i++) {
    if (results[i].valid) { anySensorFound = true; break; }
  }
}

// ============================================================
//  Проверка тревожных условий
// ============================================================
void checkAlarms() {
  String alarms = "";
  bool any = false;

  if (results[S_RADIATION].valid && results[S_RADIATION].value >= ALARM_RADIATION_USVH) {
    alarms += "РАДИАЦИЯ ";
    any = true;
  }
  if (results[S_WIND_SPEED].valid && results[S_WIND_SPEED].value >= ALARM_WIND_MS) {
    alarms += "ШТОРМ ";
    any = true;
  }
  if (results[S_TEMP].valid && results[S_TEMP].value <= ALARM_TEMP_COLD_C) {
    alarms += "МОРОЗ ";
    any = true;
  }
  if (results[S_TEMP].valid && results[S_TEMP].value >= ALARM_TEMP_HOT_C) {
    alarms += "ЖАРА ";
    any = true;
  }
  if (results[S_DUST].valid && results[S_DUST].value >= ALARM_DUST_UGM3) {
    alarms += "ПЫЛЬ ";
    any = true;
  }

  alarmActive = any;
  activeAlarmText = alarms;

  if (any) {
    Serial.println("ТРЕВОГА: " + alarms);
  }
}

// ============================================================
//  Индикация светодиодом — иерархия приоритетов (от худшего к лучшему):
//    1. Нет WiFi              -> мигает раз в 1 сек
//    2. WiFi есть, нет API    -> мигает раз в 3 сек
//    3. API есть, нет датчиков -> мигает раз в 5 сек
//    4. Всё хорошо, без тревог -> не горит
//    5. Погодная тревога активна -> горит непрерывно + пищит раз в 15 мин
//  Тревога имеет наивысший приоритет и перекрывает статусную индикацию.
// ============================================================
void updateStatusLed() {
  bool wifiOk = (WiFi.status() == WL_CONNECTED);

  // Погодная тревога — наивысший приоритет, и это НЕ "проблемное" состояние
  // связи — таймер защитной перезагрузки сбрасываем.
  if (alarmActive) {
    problemStateSince = 0;
    digitalWrite(PIN_LED, HIGH); // горит непрерывно

    if (millis() - lastAlarmBeep > (unsigned long)ALARM_BEEP_INTERVAL_SEC * 1000UL) {
      tone(PIN_BUZZER, 2000, ALARM_BEEP_DURATION_MS);
      lastAlarmBeep = millis();
    }
    return;
  }

  // Без тревоги пищалка всегда молчит
  noTone(PIN_BUZZER);

  // Статус связи — мигание с разной частотой, приоритет по тяжести проблемы
  unsigned long blinkPeriodMs;
  bool isProblem = true;
  if (!wifiOk) {
    blinkPeriodMs = 1000;       // нет WiFi — мигаем раз в секунду
  } else if (!apiReachable) {
    blinkPeriodMs = 3000;       // WiFi есть, API не отвечает — раз в 3 сек
  } else if (!anySensorFound) {
    blinkPeriodMs = 5000;       // API отвечает, но датчиков нет — раз в 5 сек
  } else {
    isProblem = false;
  }

  if (!isProblem) {
    problemStateSince = 0;      // всё хорошо — сбрасываем таймер защитной перезагрузки
    digitalWrite(PIN_LED, LOW); // и не горит
    return;
  }

  // Защитная перезагрузка: если "проблемное" мигание держится непрерывно
  // дольше REBOOT_AFTER_STUCK_SEC — перезагружаемся. Лечит зависания
  // WiFi-стека/памяти, которые внутренние ретраи не лечат.
  if (problemStateSince == 0) {
    problemStateSince = millis();
  } else if (millis() - problemStateSince > REBOOT_AFTER_STUCK_SEC * 1000UL) {
    Serial.println("Застряли в проблемном состоянии 30+ минут — перезагрузка");
    delay(500);
    ESP.restart();
  }

  // Мигание: половину периода горит, половину не горит
  unsigned long halfPeriod = blinkPeriodMs / 2;
  if (millis() - lastLedBlink > halfPeriod) {
    ledBlinkState = !ledBlinkState;
    digitalWrite(PIN_LED, ledBlinkState ? HIGH : LOW);
    lastLedBlink = millis();
  }
}

// ============================================================
//  Верхняя строка: дата, время, температура — на КАЖДОМ экране,
//  включая служебный. Высота полосы — 22px.
// ============================================================
const int HEADER_HEIGHT = 22;

void drawHeaderBar(uint16_t bgColor) {
  tft.fillRect(0, 0, 240, HEADER_HEIGHT, bgColor);
  tft.drawFastHLine(0, HEADER_HEIGHT, 240, TFT_DARKGREY);

  long nowTs = (long)timeClient.getEpochTime();
  int hh = (nowTs % 86400L) / 3600;
  int mm = (nowTs % 3600L) / 60;
  int ss = nowTs % 60;

  // Дата считается от epoch (дни с 1970-01-01), переводим в Y-M-D
  long days = nowTs / 86400L;
  int y, mo, d;
  // Алгоритм civil_from_days (Howard Hinnant), без зависимостей
  long z = days + 719468L;
  long era = (z >= 0 ? z : z - 146096) / 146097;
  long doe = z - era * 146097;
  long yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;
  long yr = yoe + era * 400;
  long doy = doe - (365*yoe + yoe/4 - yoe/100);
  long mp = (5*doy + 2)/153;
  d = doy - (153*mp+2)/5 + 1;
  mo = mp + (mp < 10 ? 3 : -9);
  y = yr + (mo <= 2 ? 1 : 0);

  char dateBuf[16];
  snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%04d", d, mo, y);
  char timeBuf[10];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d", hh, mm, ss);

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, bgColor);
  tft.setCursor(4, 7);
  tft.print(dateBuf);
  tft.print("  ");
  tft.print(timeBuf);

  // Температура справа
  char tBuf[16];
  if (results[S_TEMP].valid) {
    snprintf(tBuf, sizeof(tBuf), "%.1fC", results[S_TEMP].value);
  } else {
    snprintf(tBuf, sizeof(tBuf), "--.-C");
  }
  int tw = strlen(tBuf) * 6; // примерная ширина при textsize=1
  tft.setCursor(236 - tw, 7);
  tft.print(tBuf);
}

// Высота в уровнях (1..LEVELS) для значения v относительно [vMin,vMax].
// Если диапазон нулевой (все точки идентичны) — средний уровень.
int trendLevelOf(float v, float vMin, float vMax, int levels) {
  if (vMax - vMin < 0.0001f) return (levels + 1) / 2;
  float frac = (v - vMin) / (vMax - vMin);
  int lvl = 1 + (int)round(frac * (levels - 1));
  if (lvl < 1) lvl = 1;
  if (lvl > levels) lvl = levels;
  return lvl;
}

// ============================================================
//  Тренд-индикатор: 4 столбика (3ч, 1ч, 20мин, сейчас) показывающие
//  ОТНОСИТЕЛЬНЫЙ уровень значения в каждой точке (не направление —
//  величину), 7 уровней высоты. Текущая точка справа всегда служит
//  визуальным якорем посередине шкалы.
//  История хранится в RAM на основе собственных измерений устройства,
//  без дополнительных запросов sensorsHistory к серверу.
// ============================================================
void drawTrendBars(int idx, int x, int y, uint16_t bgColor) {
  if (!results[idx].valid) return; // нет текущего значения — индикатор не рисуем

  long nowTs = (long)timeClient.getEpochTime();
  float v3h, v1h, v20m;
  bool has3h  = historyFindClosest(idx, nowTs - 3*3600, v3h);
  bool has1h  = historyFindClosest(idx, nowTs - 1*3600, v1h);
  bool has20m = historyFindClosest(idx, nowTs - 20*60,  v20m);
  float vNow = results[idx].value;

  // Собираем доступные точки для расчёта диапазона (всегда включаем "сейчас")
  float vals[4];
  bool  has[4]  = { has3h, has1h, has20m, true };
  vals[0] = v3h; vals[1] = v1h; vals[2] = v20m; vals[3] = vNow;

  float vMin = vNow, vMax = vNow;
  for (int k = 0; k < 4; k++) {
    if (!has[k]) continue;
    if (vals[k] < vMin) vMin = vals[k];
    if (vals[k] > vMax) vMax = vals[k];
  }

  const int LEVELS    = 7;     // 7 уровней высоты, как договорились
  const int BAR_W      = 8;    // ширина одного столбика
  const int BAR_GAP    = 6;    // промежуток между столбиками
  const int BAR_MAX_H  = 28;   // высота, соответствующая верхнему уровню
  const int BAR_BASE_Y = y;    // базовая линия (низ столбиков)

  int levels[4];
  for (int k = 0; k < 4; k++) {
    levels[k] = has[k] ? trendLevelOf(vals[k], vMin, vMax, LEVELS) : 0; // 0 = нет данных
  }

  for (int k = 0; k < 4; k++) {
    int bx = x + k * (BAR_W + BAR_GAP);
    if (levels[k] == 0) {
      // Нет данных для этой точки — рисуем тонкую серую черту-заглушку у базы
      tft.drawFastHLine(bx, BAR_BASE_Y, BAR_W, TFT_DARKGREY);
      continue;
    }
    int barH = (BAR_MAX_H * levels[k]) / LEVELS;
    if (barH < 2) barH = 2;
    uint16_t color = (k == 3) ? TFT_CYAN : TFT_GREENYELLOW; // текущая точка выделена цветом
    tft.fillRect(bx, BAR_BASE_Y - barH, BAR_W, barH, color);
  }
}


void drawSensorScreen(int idx, bool isAlarmFrame) {
  uint16_t bgColor = isAlarmFrame ? tft.color565(120, 0, 0) : TFT_BLACK;
  tft.fillScreen(bgColor);

  drawHeaderBar(bgColor);

  // Заголовок — название параметра
  tft.setTextColor(TFT_WHITE, bgColor);
  tft.setTextSize(2);
  tft.setCursor(10, 32);
  tft.println(sensorTypes[idx].displayName);

  // Линия-разделитель
  tft.drawFastHLine(10, 58, 220, TFT_DARKGREY);

  if (!results[idx].valid) {
    tft.setTextSize(2);
    tft.setCursor(10, 110);
    tft.setTextColor(TFT_DARKGREY, bgColor);
    tft.println("Нет данных");
    tft.setTextSize(1);
    tft.setCursor(10, 140);
    tft.println("Нет свежих публичных");
    tft.setCursor(10, 155);
    tft.println("датчиков рядом");
    return;
  }

  // Крупное значение
  tft.setTextColor(TFT_WHITE, bgColor);
  tft.setTextSize(5);
  tft.setCursor(10, 85);
  char buf[16];
  if (idx == S_WIND_DIR) {
    // направление — округляем до целого градуса
    snprintf(buf, sizeof(buf), "%d", (int)round(results[idx].value));
  } else {
    snprintf(buf, sizeof(buf), "%.1f", results[idx].value);
  }
  tft.print(buf);
  tft.setTextSize(2);
  tft.print(" ");
  tft.println(sensorTypes[idx].unit);

  // Детали — сколько датчиков, расстояние до ближнего
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, bgColor);
  tft.setCursor(10, 175);
  char detailBuf[64];
  snprintf(detailBuf, sizeof(detailBuf), "Усреднено по %d датчикам", results[idx].usedCount);
  tft.println(detailBuf);
  tft.setCursor(10, 190);
  snprintf(detailBuf, sizeof(detailBuf), "Ближайший: %.1f км", results[idx].nearestDist);
  tft.println(detailBuf);

  // Тренд-индикатор: 4 столбика (3ч / 1ч / 20мин / сейчас), не мешает крупной цифре
  tft.setTextColor(TFT_DARKGREY, bgColor);
  tft.setCursor(10, 208);
  tft.print("3ч  1ч  20м  сейчас");
  drawTrendBars(idx, 10, 248, bgColor);

  // Тревожный баннер
  if (isAlarmFrame && activeAlarmText.length() > 0) {
    tft.fillRect(0, 290, 240, 30, TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(5, 297);
    tft.println(activeAlarmText);
  }

  // Точки-индикатор карусели снизу (только если не тревога)
  if (!isAlarmFrame) {
    int dotsY = 305;
    int totalDots = S_COUNT + 1; // +1 за служебный экран
    int startX = 120 - (totalDots * 10) / 2;
    for (int i = 0; i < totalDots; i++) {
      uint16_t c = (i == idx) ? TFT_WHITE : TFT_DARKGREY;
      tft.fillCircle(startX + i * 10, dotsY, 3, c);
    }
  }
}

// ============================================================
//  Служебный экран: список ID датчиков, которые сейчас
//  используются в усреднении каждого типа, мелким шрифтом.
//  Дата/время/температура — в той же шапке, что у всех экранов.
// ============================================================
void drawServiceScreen() {
  uint16_t bgColor = TFT_BLACK;
  tft.fillScreen(bgColor);
  drawHeaderBar(bgColor);

  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, bgColor);
  tft.setCursor(4, 28);
  tft.println("СЛУЖЕБНЫЙ ЭКРАН: датчики");

  int y = 42;
  const int lineH = 9;

  long nowTs = (long)timeClient.getEpochTime();

  for (int i = 0; i < S_COUNT; i++) {
    if (y > 295) break; // не вылезаем за пределы экрана

    tft.setTextColor(TFT_WHITE, bgColor);
    tft.setCursor(4, y);
    tft.print(sensorTypes[i].displayName);
    tft.print(":");
    y += lineH;

    if (!results[i].valid) {
      tft.setTextColor(TFT_DARKGREY, bgColor);
      tft.setCursor(8, y);
      tft.print("нет данных");
      y += lineH;
      continue;
    }

    for (int k = 0; k < results[i].usedCount && k < MAX_USED_SENSORS; k++) {
      if (y > 295) break;
      long ageMin = (nowTs - results[i].used[k].time) / 60;
      tft.setTextColor(TFT_GREENYELLOW, bgColor);
      tft.setCursor(8, y);
      char buf[48];
      snprintf(buf, sizeof(buf), "S%ld  %.1fкм  %ldмин назад",
        results[i].used[k].id, results[i].used[k].distance, ageMin);
      tft.print(buf);
      y += lineH;
    }
    y += 2; // небольшой отступ между группами
  }
}

// ============================================================
//  Карусель: решает какой экран показывать сейчас
//  Индексы 0..7 — обычные параметры, индекс 8 — служебный экран
// ============================================================
const int TOTAL_SCREENS = S_COUNT + 1; // +1 служебный

void updateCarousel() {
  if (millis() - lastCarouselSwitch < (unsigned long)CAROUSEL_INTERVAL_SEC * 1000UL) {
    return; // ещё не время менять экран
  }
  lastCarouselSwitch = millis();

  // Если тревога активна — через раз показываем тревожный экран
  static bool showAlarmNext = false;
  if (alarmActive) {
    showAlarmNext = !showAlarmNext;
    if (showAlarmNext) {
      drawSensorScreen(S_TEMP, true); // тревожный кадр — крупно температура + баннер
      return;
    }
  }

  // Обычная карусель: temp всегда первый (carouselIndex==0),
  // потом остальные параметры, последним — служебный экран
  if (carouselIndex == S_COUNT) {
    drawServiceScreen();
  } else {
    drawSensorScreen(carouselIndex, false);
  }

  carouselIndex++;
  if (carouselIndex >= TOTAL_SCREENS) carouselIndex = 0;
}

// ============================================================
//  Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Погодный дисплей ESP32 + ST7789 ===");

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUZZER, OUTPUT);

  tft.init();
  tft.setRotation(0); // 240x320 портретная ориентация; 2 = landscape если нужно
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Загрузка...");

  nmUUID = md5String(String(NM_UUID_SRC));

  // Защита: служебный экран хранит детали только по MAX_USED_SENSORS датчикам
  if (NEAREST_COUNT > MAX_USED_SENSORS) {
    Serial.printf("NEAREST_COUNT=%d больше MAX_USED_SENSORS=%d, ограничиваю\n", NEAREST_COUNT, MAX_USED_SENSORS);
    NEAREST_COUNT = MAX_USED_SENSORS;
  }

  connectWiFi();

  tft.setCursor(10, 40);
  if (WiFi.status() == WL_CONNECTED) {
    tft.println("WiFi OK");
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("WiFi не подключён, продолжаем...");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
  }

  timeClient.begin();
  timeClient.forceUpdate();

  tft.setCursor(10, 70);
  tft.println("Справочник типов...");
  bool typesOk = false;
  if (WiFi.status() == WL_CONNECTED) {
    typesOk = loadSensorTypeCodes();
  }
  if (!typesOk) {
    tft.setCursor(10, 100);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("Справочник не получен, попробуем в фоне");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    delay(1500);
  }
  typeCodesLoaded = typesOk;

  tft.setCursor(10, 130);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (typeCodesLoaded) {
    tft.println("Первые данные...");
    fetchAndAverage();
    checkAlarms();
  } else {
    tft.println("Ждём подключения...");
  }
  lastRequestMinute = ((long)timeClient.getEpochTime() / 60) % 1440;

  drawSensorScreen(S_TEMP, false);
  lastCarouselSwitch = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  timeClient.update();

  // Если справочник типов ещё не загружен (например WiFi не было при старте) —
  // пробуем подгрузить его при первой возможности, не дожидаясь расписания
  if (!typeCodesLoaded && WiFi.status() == WL_CONNECTED) {
    typeCodesLoaded = loadSensorTypeCodes();
  }

  // Обновление данных с сервера по расписанию (минута от начала суток UTC)
  if (typeCodesLoaded && isScheduledMinuteNow()) {
    fetchAndAverage();
    checkAlarms();
  }

  // Карусель экранов
  updateCarousel();

  // Индикация светодиодом: статус связи (мигание) или тревога (горит + пищалка)
  updateStatusLed();

  delay(200); // частота проверки расписания — раз в ~200мс достаточно
}
