// Зайчик Alilo 3.0.

#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <arduinoFFT.h>
#include <OneButton.h>

#define USB_POWER_DETECT 3
#define DETECT_CHARGE 7

#define BUTTON_MODE 2
#define BUTTON_DETECT_COLOR 4

#define LED_DETECT_ANODE 5
#define LED_HEAD_ANODE 6
#define LED_B 9
#define LED_G 10
#define LED_R 11

#define PLAYER_POWER 8
#define PLAYER_BUSY 12
#define PLAYER_TX 13
#define PLAYER_DAC_L A2   // Пин аудио линейного входа для светомузыки
#define PLAYER_VOLUME A1  // Пин регулировки громкости

#define SENSOR_PIN A0  // Пин фототранзистора
#define VIBRO_PIN A3   // Пин датчика вибрации для погремушки.
#define VIBRO_TIMEOUT 2000

#define COLOR_COUNT 9  // максимум цветов
#define KFACTOR 1.7    // чувствительность корректировки (0.0 ... 2.0)
#define COLOR_BLACK 0
#define COLOR_BLUE 1
#define COLOR_GREEN 2
#define COLOR_PURPLE 3
#define COLOR_RED 4
#define COLOR_WHITE 5
#define COLOR_YALOW 6
#define COLOR_GRAY 7
#define COLOR_ORANGE 8

#define EEPROM_ADDR_MODE 0
#define EEPROM_ADDR_TRACK_01 1
#define EEPROM_ADDR_TRACK_02 2
#define EEPROM_ADDR_TRACK_03 3
#define EEPROM_ADDR_NIGHT_EFFECT 4

#define SAMPLES 64
#define SAMPLING_FREQUENCY 4000

// Детектор цвета
const float TOLERANCE = 0.20;  // ±10%
unsigned long pressStart = 0;
bool lastState = HIGH;
enum PressType { NONE,
                 SHORT_PRESS,
                 LONG_PRESS };
struct ColorRef {
  int red;
  int green;
  int blue;
  int voltage;
};
ColorRef refs[COLOR_COUNT];
bool colorDetected = false;
// Детектор цвета

// Погремушка
bool vibroActive = false;
unsigned long vibroStart = 0;
unsigned long vibroLastPulse = 0;
int vibroCurrentTrack = 1;
int vibroRepeatCount = 0;
unsigned long vibroBusyLowSince = 0;
bool vibroBusyWasLow = false;
// Погремушка

//Регулятор громкости
int volume = 0;
volatile bool offVolume = false;
//Регулятор громкости

//Контроль питания и сна
unsigned long lastActiveTime = 0;
bool sleeping = false;
bool low_voltage = false;
unsigned long lastPowPrint = 0;
bool FTTOff = false;
unsigned long lastBlinkTime = 0;
bool ledState = false;
//Контроль питания и сна

// Player
bool repeatTrack = false;
uint8_t currentMode = 0;
uint8_t currentTrackMem[3] = { 1, 1, 1 };
uint8_t nightEffect = 0;
const int NUM_FOLDERS = 3;
const int tracksInFolder[NUM_FOLDERS + 1] = { 0, 30, 33, 32 };  // Индекс 1 и 2 = папки 01 и 02
int currentTrack = 1;
int currentFolder = 1;
bool trackManuallyChanged = false;
unsigned long busyLowStartTime = 0;
bool wasBusyLow = false;
double vReal[SAMPLES];
double vImag[SAMPLES];
OneButton buttonMode(BUTTON_MODE, true);  // true = кнопка подключена к GND и использует INPUT_PULLUP
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
SoftwareSerial mp3Serial(-1, PLAYER_TX);  // Подключи RX/TX DFPlayer сюда
DFRobotDFPlayerMini mp3;
// Player

void setup() {

  Serial.begin(9600);
  Serial.println("--------init---------");

  analogReadResolution(12);
  analogReference(INTERNAL1V024);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_DETECT_ANODE, OUTPUT);
  pinMode(LED_HEAD_ANODE, OUTPUT);
  pinMode(PLAYER_POWER, OUTPUT);
  pinMode(PLAYER_TX, OUTPUT);
  pinMode(DETECT_CHARGE, INPUT_PULLUP);
  pinMode(VIBRO_PIN, INPUT_PULLUP);
  pinMode(PLAYER_BUSY, INPUT_PULLUP);
  pinMode(BUTTON_DETECT_COLOR, INPUT_PULLUP);
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(USB_POWER_DETECT, INPUT);

  digitalWrite(PLAYER_POWER, HIGH);
  digitalWrite(LED_DETECT_ANODE, LOW);
  digitalWrite(LED_HEAD_ANODE, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  //



  detachInterrupt(digitalPinToInterrupt(USB_POWER_DETECT));
  loadState();

  if (digitalRead(BUTTON_DETECT_COLOR) == LOW) {
    unsigned long t0 = millis();
    digitalWrite(PLAYER_TX, LOW);

    // Ждем 2 секунды удержания
    while (digitalRead(BUTTON_DETECT_COLOR) == LOW) {
      if (millis() - t0 > 2000) {
        if (digitalRead(BUTTON_MODE) == LOW) calibrate();
        break;  // выходим из if, чтобы не зависнуть
      }
    }
  }

  loadRefs();

  buttonMode.attachClick(modeSingleClick);
  buttonMode.attachDoubleClick(modeDoubleClick);
  buttonMode.attachLongPressStart(modeLongPress);

  mp3Serial.begin(9600);

  delay(500);
  playerOn();
  if (currentMode < 3) {
    mp3.playFolder(currentFolder, currentTrack);
  } else if (currentMode == 3) {
    mp3.playFolder(7, 2);
  } else {
    mp3.playFolder(7, 3);
  }
  Serial.print("Setup-");
  Serial.print(getVsupply());
  Serial.print(",");
  Serial.print(currentMode);
  Serial.print(",");
  Serial.println(currentTrack);

  //attachInterrupt(digitalPinToInterrupt(BUTTON_MODE), handleButtonInterrupt, CHANGE);
  lastActiveTime = millis();
  lastPowPrint = millis();
  delay(500);
}

void loop() {

  handleSleepControl();
  handleVolumeControl();
  buttonMode.tick();
  
  if (currentMode < 3) {
    handleMainPlayer();
  } else if (currentMode == 3) {
    handleVibro();
    handleVibroPlayer();
  } else {
    handleColorDetect();
  }
  handleBatteryControl();
}


void modeSingleClick() {
  repeatTrack = false;
  lastActiveTime = millis();
  if (!offVolume && currentMode < 3) {
    low_voltage = false;
    FTTOff = false;
    colorDetected = false;
    currentTrack++;
    if (currentTrack > tracksInFolder[currentFolder]) currentTrack = 1;
    currentTrackMem[currentMode] = currentTrack;
    saveState();
    trackManuallyChanged = true;
    mp3.playFolder(currentFolder, currentTrack);
    Serial.print("Short-");
    Serial.print(getVsupply());
    Serial.print(",");
    Serial.print(currentMode);
    Serial.print(",");
    Serial.println(currentTrack);
  } else if (offVolume) {
    trackManuallyChanged = true;
    nightEffect++;
    if (nightEffect > 4) nightEffect = 0;
    Serial.print("Night-");
    Serial.print(getVsupply());
    Serial.print(",");
    Serial.print(currentMode);
    Serial.print(",");
    Serial.println(nightEffect);
    saveState();
  }
}

void modeDoubleClick() {
  Serial.println("Двойное нажатие");
  lastActiveTime = millis();
  repeatTrack = true;
}

void modeLongPress() {
  repeatTrack = false;
  lastActiveTime = millis();
  if (!offVolume) {
    low_voltage = false;
    FTTOff = false;
    colorDetected = false;
    currentMode = (currentMode + 1) % 5;

    currentFolder = (currentMode < 3) ? currentMode + 1 : currentFolder;
    currentTrack = (currentMode < 3) ? currentTrackMem[currentMode] : currentTrack;
    saveState();

    trackManuallyChanged = true;

    if (currentMode < 3) {
      mp3.playFolder(currentFolder, currentTrack);
    } else if (currentMode == 3) {
      mp3.playFolder(7, 2);
    } else {
      mp3.playFolder(7, 3);
    }

    Serial.print("Long-");
    Serial.print(getVsupply());
    Serial.print(",");
    Serial.print(currentMode);
    Serial.print(",");
    Serial.println(currentTrack);
  }
}

void handleMainPlayer() {
  bool busyNow = (digitalRead(PLAYER_BUSY) == LOW);

  if (!offVolume && busyNow) {
    // Музыка играет
    if (!wasBusyLow) {
      busyLowStartTime = millis();  // старт отсчёта BUSY == LOW
      wasBusyLow = true;
    }

    if (!FTTOff) {
      runFFT();  // визуализация
    }
  } else {
    // Музыка не играет
    if (!offVolume && wasBusyLow) {
      if (!trackManuallyChanged && millis() - busyLowStartTime > 5000) {
        // Если BUSY был LOW дольше 5 секунд → трек завершён
        if (!repeatTrack) {
          currentTrack++;
          if (currentTrack > tracksInFolder[currentFolder]) currentTrack = 1;
          currentTrackMem[currentMode] = currentTrack;
          saveState();
        } 
        mp3.playFolder(currentFolder, currentTrack);
        Serial.print("Next-");
        Serial.print(getVsupply());
        Serial.print(",");
        Serial.print(currentMode);
        Serial.print(",");
        Serial.println(currentTrack);
      }
      trackManuallyChanged = false;
      wasBusyLow = false;
    }
    if (!FTTOff) {
      if (nightEffect == 0) {
        animateIdleColor();  // "ночной режим 0"
      } else if (nightEffect == 1) {
        setHeadRGBVal(255, 255, 255);  // "ночной режим 1"
      } else if (nightEffect == 2) {
        setHeadRGBVal(255, 0, 0);  // "ночной режим 2"
      } else if (nightEffect == 3) {
        setHeadRGBVal(0, 255, 0);  // "ночной режим 3"
      } else if (nightEffect == 4) {
        setHeadRGBVal(0, 0, 255);  // "ночной режим 4"
      }
    }
  }
}

void runFFT() {
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(PLAYER_DAC_L);
    vImag[i] = 0;
    delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
  }

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double low = 0, mid = 0, high = 0;

  for (int i = 2; i < SAMPLES / 2; i++) {
    double freq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;

    if (freq < 250) low += vReal[i];
    else if (freq < 1000) mid += vReal[i];
    else high += vReal[i];

  }

  if (low < 100) low = 0;
  if (mid < 300) mid = 0;
  if (high < 500) high = 0;

  low = constrain(low / 50, 0, 255);
  mid = constrain(mid / 50, 0, 255);
  high = constrain(high / 150, 0, 255);

  setHeadRGBVal((int)low, (int)mid, (int)high);
}

void animateIdleColor() {
  static uint8_t phase = 0;
  static unsigned long lastUpdate = 0;

  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();
    phase++;

    // Простое переливание через фазу
    int r = (sin((phase + 0) * 0.1) + 1.0) * 127.5;
    int g = (sin((phase + 42) * 0.1) + 1.0) * 127.5;
    int b = (sin((phase + 85) * 0.1) + 1.0) * 127.5;

    setHeadRGBVal(r, g, b);
  }
}

void wakeUp() {
  // ничего не делаем — просто выходим из сна
}

void goToSleep() {
  saveState();            // Сохраняем всё
  playerOff();            // Выключаем плеер
  setHeadRGB(0, 0, 0);    // Гасим свет
  setDetectRGB(0, 0, 0);  // Гасим свет
  sleeping = true;
  offVolume = false;
  Serial.println("Sleep");
  //detachInterrupt(digitalPinToInterrupt(BUTTON_MODE));  // Отвязываем ISR
  delay(50);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();

  attachInterrupt(digitalPinToInterrupt(BUTTON_MODE), wakeUp, LOW);        // Пробуждение по нажатию
  attachInterrupt(digitalPinToInterrupt(USB_POWER_DETECT), wakeUp, HIGH);  // Пробуждение по зарядке
  delay(1000);
  sleep_mode();  // Засыпаем...

  // Проснулись
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(USB_POWER_DETECT));
  detachInterrupt(digitalPinToInterrupt(BUTTON_MODE));                                 // убираем пробуждающее прерывание
  //attachInterrupt(digitalPinToInterrupt(BUTTON_MODE), handleButtonInterrupt, CHANGE);  // восстанавливаем обычную ISR

  if (digitalRead(USB_POWER_DETECT) == LOW) {
    unsigned long sleepPressStart = millis();
    bool sleepLongPress = false;

    while (digitalRead(BUTTON_MODE) == LOW) {   // пока держат кнопку
      if (millis() - sleepPressStart > 2000) {  // дольше 1.5 сек
        sleepLongPress = true;
        break;
      }
    }

    if (sleepLongPress) {
      Serial.println("Wakeup long press OK");
    } else {
      Serial.println("Wake too short → back to sleep");
      goToSleep();  // снова засыпаем
      return;
    }
  }

  Serial.println("Wakup");
  delay(1000);
  int raw = analogRead(PLAYER_VOLUME);
  volume = map(raw, 0, 4000, 0, 30);
  if (volume > 25) volume = 25;
  if (volume > 0) {
    playerOn();  // Включаем плеер
    trackManuallyChanged = true;
    if (currentMode < 3) {
      mp3.playFolder(currentFolder, currentTrack);
    } else if (currentMode == 3) {
      mp3.playFolder(7, 2);
    } else {
      mp3.playFolder(7, 3);
    }
    Serial.print("Wakup-");
    Serial.print(getVsupply());
    Serial.print(",");
    Serial.print(currentMode);
    Serial.print(",");
    Serial.println(currentTrack);
  } else {
    playerOff();
    offVolume = true;
  }
  sleeping = false;
  lastActiveTime = millis();
}

void handleSleepControl() {
  if (!sleeping && millis() - lastActiveTime > 2400000) {
    goToSleep();
    return;
  }
}

void handleBatteryControl() {
  static uint32_t lowVoltStart = 0;  // момент первого обнаружения < 3.1 В
  uint32_t Vpow = getVsupply();
  bool Vusb = digitalRead(USB_POWER_DETECT);
  bool Icharge = digitalRead(DETECT_CHARGE);

  //if (millis() - lastPowPrint > 2000) {
  //  Serial.print("Vpow-");
  //  Serial.println(Vpow);
  //  lastPowPrint = millis();
  //}

  // Логика задержки при низком напряжении
  if (Vpow < 3300) {
    if (lowVoltStart == 0) {
      lowVoltStart = millis();  // запоминаем время первого падения
    } else if (millis() - lowVoltStart >= 10000 && !low_voltage) {
      // прошло 10 секунд ниже порога
      low_voltage = true;
      FTTOff = true;
      if (!offVolume) mp3.playFolder(7, 1);
    }
  } else {
    lowVoltStart = 0;  // сброс, если напряжение снова выше
  }

  if (!low_voltage) {
    if (!Icharge && Vusb) {
      lastActiveTime = millis();
      FTTOff = true;
      blinkChannel(LED_G, 500);
    } else if (Icharge && Vusb) {
      lastActiveTime = millis();
      FTTOff = true;
      setHeadRGB(0, 1, 0);
    } else {
      if (!colorDetected) FTTOff = false;
    }
  }

  if (low_voltage) {
    blinkChannel(LED_R, 500);
    return;  // ничего больше не делаем при низком напряжении
  }
}

void blinkChannel(int colorPin, unsigned long interval) {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;
  unsigned long currentMillis = millis();

  if (currentMillis - lastBlinkTime >= interval) {
    lastBlinkTime = currentMillis;
    ledState = !ledState;

    // всегда включён общий анод головы
    digitalWrite(LED_HEAD_ANODE, HIGH);

    // гасим все каналы
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);

    // если "вкл", зажигаем только выбранный канал
    if (ledState) {
      digitalWrite(colorPin, HIGH);
    }
  }
}

void loadState() {
  currentMode = constrain(EEPROM.read(EEPROM_ADDR_MODE), 0, 4);
  currentTrackMem[0] = constrain(EEPROM.read(EEPROM_ADDR_TRACK_01), 1, tracksInFolder[1]);
  currentTrackMem[1] = constrain(EEPROM.read(EEPROM_ADDR_TRACK_02), 1, tracksInFolder[2]);
  currentTrackMem[2] = constrain(EEPROM.read(EEPROM_ADDR_TRACK_03), 1, tracksInFolder[3]);
  currentFolder = currentMode + 1;
  if (currentFolder > 3) {
    currentTrack = currentTrackMem[0];
    currentFolder = 1;
  } else {
    currentTrack = currentTrackMem[currentMode];
  }
  nightEffect = constrain(EEPROM.read(EEPROM_ADDR_NIGHT_EFFECT), 0, 4);
}

void saveState() {
  EEPROM.update(EEPROM_ADDR_MODE, currentMode);
  EEPROM.update(EEPROM_ADDR_TRACK_01, currentTrackMem[0]);
  EEPROM.update(EEPROM_ADDR_TRACK_02, currentTrackMem[1]);
  EEPROM.update(EEPROM_ADDR_TRACK_03, currentTrackMem[2]);
  EEPROM.update(EEPROM_ADDR_NIGHT_EFFECT, nightEffect);
}

uint32_t getVsupply() {
  uint32_t v_supply = analogRead(VCCM);
  v_supply = (v_supply * 5) / 4;
  return v_supply;
}

int detectColor() {
  ColorRef cur = measureColor();
  for (int i = 0; i < COLOR_COUNT; i++) {
    if (isSimilar(cur, refs[i], 0)) {
      return i;
    }
  }
  return -1;
}

void blinkHeadRGB(int r, int g, int b, int times, int delayMs = 500) {
  Serial.println("Мигаем");
  for (int i = 0; i < times; i++) {
    setHeadRGB(r, g, b);
    delay(delayMs);
    setHeadRGB(0, 0, 0);
    delay(delayMs);
  }
  digitalWrite(LED_HEAD_ANODE, LOW);
}

// ================= Калибровка =================
void calibrate() {

  blinkHeadRGB(0, 0, 1, 3);  // вход в режим калибровки (синий мигает)

  clearColorMemory();  //Очистка эталонов в EEPROM
  loadRefs();

  for (int idx = 0; idx < COLOR_COUNT; idx++) {
    Serial.print("Ожидание кнопки для калибровки цвета #");
    Serial.println(idx);

    if (idx == COLOR_BLACK) {
      setHeadRGBVal(0, 0, 0);
    } else if (idx == COLOR_BLUE) {
      setHeadRGBVal(0, 0, 255);
    } else if (idx == COLOR_GREEN) {
      setHeadRGBVal(0, 255, 0);
    } else if (idx == COLOR_PURPLE) {
      setHeadRGBVal(255, 0, 100);
    } else if (idx == COLOR_RED) {
      setHeadRGBVal(255, 0, 0);
    } else if (idx == COLOR_WHITE) {
      setHeadRGBVal(255, 255, 255);
    } else if (idx == COLOR_YALOW) {
      setHeadRGBVal(150, 255, 0);
    } else if (idx == COLOR_GRAY) {
      setHeadRGBVal(20, 20, 20);
    } else if (idx == COLOR_ORANGE) {
      setHeadRGBVal(255, 100, 0);
    } else {
      setHeadRGBVal(255, 255, 255);
    }

    // Ждём нажатие
    PressType t = NONE;
    while (t == NONE) {
      t = readColorButton();
    }

    if (t == SHORT_PRESS) {
      ColorRef newC = measureColor();

      // Проверяем совпадения
      bool found = false;
      for (int j = 0; j < COLOR_COUNT; j++) {
        if (isSimilar(newC, refs[j], 1)) {
          found = true;
          break;
        }
      }

      if (found) {
        blinkHeadRGB(1, 0, 0, 3);  // 3x красным
        idx--;
        Serial.println("Такой цвет уже записан");
      } else {
        saveRef(idx, newC);
        blinkHeadRGB(0, 1, 0, 3);  // 3x зелёным
      }
    } else {
      idx--;
    }
  }
}

PressType readColorButton() {
  static unsigned long pressStart = 0;
  static bool waitingRelease = false;

  int state = digitalRead(BUTTON_MODE);

  // если ждём отпускания — пока не вернём NONE
  if (waitingRelease) {
    if (state == HIGH) {
      waitingRelease = false;  // кнопка отпущена
    }
    return NONE;
  }

  // фиксируем момент нажатия
  if (state == LOW && lastState == HIGH) {
    pressStart = millis();
  }

  // обработка отпускания
  if (state == HIGH && lastState == LOW) {
    unsigned long pressDuration = millis() - pressStart;
    waitingRelease = true;  // теперь ждём отпускания перед новым событием

    if (pressDuration >= 2000) {
      lastState = state;
      return LONG_PRESS;
    } else if (pressDuration >= 50) {
      lastState = state;
      return SHORT_PRESS;
    }
  }

  lastState = state;
  return NONE;
}

bool isSimilar(ColorRef a, ColorRef b, bool calibrate) {

  float rawK = (float)b.voltage / (float)getVsupply();
  // чувствительность
  float k = (1.0 + (rawK - 1.0) * KFACTOR);

  if (calibrate) {
    k = 1;
  }

  float ared = a.red * k;
  float agreen = a.green * k;
  float ablue = a.blue * k;

  bool mr = (ared >= b.red * (1 - TOLERANCE)) && (ared <= b.red * (1 + TOLERANCE));
  bool mg = (agreen >= b.green * (1 - TOLERANCE)) && (agreen <= b.green * (1 + TOLERANCE));
  bool mb = (ablue >= b.blue * (1 - TOLERANCE)) && (ablue <= b.blue * (1 + TOLERANCE));
  return mr && mg && mb;
}

void clearColorMemory() {
  const int base = 32;
  ColorRef zero = { 0, 0, 0, 0 };
  for (int i = 0; i < COLOR_COUNT; i++) {
    EEPROM.put(base + i * sizeof(ColorRef), zero);
  }
}

void saveRef(int idx, ColorRef c) {
  int offset = 0;
  offset = idx * sizeof(ColorRef);
  offset = offset + 32;  //Смещение в памяти

  Serial.print("Запись в память:");
  Serial.print(offset);
  Serial.print("|");
  Serial.print(c.red);
  Serial.print("|");
  Serial.print(c.green);
  Serial.print("|");
  Serial.print(c.blue);
  Serial.println("|");
  EEPROM.put(offset, c);
  refs[idx] = c;
}

void loadRefs() {
  for (int i = 0; i < COLOR_COUNT; i++) {
    EEPROM.get((i * sizeof(ColorRef)) + 32, refs[i]);  // 32 - Смещение в памяти.
  }
}

ColorRef measureColor() {
  ColorRef c;
  c.red = measureChannel(1, 0, 0);
  c.green = measureChannel(0, 1, 0);
  c.blue = measureChannel(0, 0, 1);

  c.voltage = getVsupply();

  Serial.print("Измерение цвета:");
  Serial.print(c.red);
  Serial.print("|");
  Serial.print(c.green);
  Serial.print("|");
  Serial.print(c.blue);
  Serial.print("|");
  Serial.print(c.voltage);
  Serial.println("|");
  return c;
}

int measureChannel(int r, int g, int b) {
  analogReference(INTERNAL4V096);
  setDetectRGB(r, g, b);
  delay(50);
  int v = analogRead(SENSOR_PIN);
  setDetectRGB(0, 0, 0);
  analogReference(INTERNAL1V024);
  return v;
}

void setDetectRGBVal(int r, int g, int b) {
  digitalWrite(LED_DETECT_ANODE, HIGH);
  digitalWrite(LED_HEAD_ANODE, LOW);
  analogWrite(LED_R, r);
  analogWrite(LED_G, g);
  analogWrite(LED_B, b);
  if (r == 0 && g == 0 && b == 0) {
    digitalWrite(LED_DETECT_ANODE, LOW);
  }
}

void setDetectRGB(bool r, bool g, bool b) {
  digitalWrite(LED_DETECT_ANODE, HIGH);
  digitalWrite(LED_HEAD_ANODE, LOW);
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
  if (r == 0 && g == 0 && b == 0) {
    digitalWrite(LED_DETECT_ANODE, LOW);
  }
}

void setHeadRGBVal(int r, int g, int b) {
  digitalWrite(LED_DETECT_ANODE, LOW);
  digitalWrite(LED_HEAD_ANODE, HIGH);
  analogWrite(LED_R, r);
  analogWrite(LED_G, g);
  analogWrite(LED_B, b);
  if (r == 0 && g == 0 && b == 0) {
    digitalWrite(LED_HEAD_ANODE, LOW);
  }
}

void setHeadRGB(bool r, bool g, bool b) {
  digitalWrite(LED_DETECT_ANODE, LOW);
  digitalWrite(LED_HEAD_ANODE, HIGH);
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
  if (r == 0 && g == 0 && b == 0) {
    digitalWrite(LED_HEAD_ANODE, LOW);
  }
}

void handleColorDetect() {

  static unsigned long pressStart = 0;    // время начала нажатия
  static bool buttonWasPressed = false;   // флаг, что кнопка уже нажата
  static bool ClongPressHandled = false;  // флаг, что длинное нажатие обработано
  
  if (offVolume) {
    if (!FTTOff) {
      if (nightEffect == 0) {
        animateIdleColor();  // "ночной режим 0"
      } else if (nightEffect == 1) {
        setHeadRGBVal(255, 255, 255);  // "ночной режим 1"
      } else if (nightEffect == 2) {
        setHeadRGBVal(255, 0, 0);  // "ночной режим 2"
      } else if (nightEffect == 3) {
        setHeadRGBVal(0, 255, 0);  // "ночной режим 3"
      } else if (nightEffect == 4) {
        setHeadRGBVal(0, 0, 255);  // "ночной режим 4"
      }
    }
    return;
  }

  bool state = digitalRead(BUTTON_DETECT_COLOR);

  if (state == LOW && !buttonWasPressed) {
    // кнопка только что нажата
    buttonWasPressed = true;
    pressStart = millis();
    ClongPressHandled = false;
  }

  if (state == LOW && buttonWasPressed) {
    // кнопка удерживается
    if (!ClongPressHandled && millis() - pressStart >= 1000) {
      // длинное нажатие
      lastActiveTime = millis();
      int color = detectColor();
      switch (color) {
        case COLOR_BLACK:
          FTTOff = true;
          colorDetected = true;
          setHeadRGBVal(0, 0, 0);
          Serial.println("Черный");
          mp3.playFolder(5, 10);
          delay(2000);
          mp3.playFolder(5, 1);
          break;
        case COLOR_BLUE:
          FTTOff = true;
          colorDetected = true;
          setHeadRGBVal(0, 0, 255);
          Serial.println("Синий");
          mp3.playFolder(5, 11);
          delay(2000);
          mp3.playFolder(5, 2);
          break;
        case COLOR_GREEN:
          FTTOff = true;
          colorDetected = true;
          setHeadRGBVal(0, 255, 0);
          Serial.println("Зеленый");
          mp3.playFolder(5, 12);
          delay(2000);
          mp3.playFolder(5, 3);
          break;
        case COLOR_PURPLE:
          FTTOff = true;
          colorDetected = true;
          setHeadRGBVal(255, 0, 100);
          Serial.println("Фиолетовый");
          mp3.playFolder(5, 13);
          delay(2500);
          mp3.playFolder(5, 4);
          break;
        case COLOR_RED:
          FTTOff = true;
          colorDetected = true;
          setHeadRGBVal(255, 0, 0);
          Serial.println("Красный");
          mp3.playFolder(5, 14);
          delay(2000);
          mp3.playFolder(5, 5);
          break;
        case COLOR_WHITE:
          FTTOff = true;
          colorDetected = true;
          setHeadRGBVal(255, 255, 255);
          Serial.println("Белый");
          mp3.playFolder(5, 15);
          delay(2000);
          mp3.playFolder(5, 6);
          break;
        case COLOR_YALOW:
          FTTOff = true;
          colorDetected = true;
          setHeadRGBVal(150, 255, 0);
          Serial.println("Желтый");
          mp3.playFolder(5, 16);
          delay(2000);
          mp3.playFolder(5, 7);
          break;
        case COLOR_GRAY:
          FTTOff = true;
          colorDetected = true;
          setHeadRGBVal(20, 20, 20);
          Serial.println("Серый");
          mp3.playFolder(5, 17);
          delay(2000);
          mp3.playFolder(5, 8);
          break;
        case COLOR_ORANGE:
          FTTOff = true;
          colorDetected = true;
          setHeadRGBVal(255, 100, 0);
          Serial.println("Оранжевый");
          mp3.playFolder(5, 18);
          delay(2500);
          mp3.playFolder(5, 9);
          break;
        default:
          Serial.println("Неизвестный цвет");
          break;
      }
      ClongPressHandled = true;  // чтобы не повторялось пока не отпустим
    }
  }

  if (state == HIGH && buttonWasPressed) {
    // кнопка отпущена
    buttonWasPressed = false;
  }
}

// ====== Громкость с потенциометра ======
void handleVolumeControl() {
  static int lastVolume = -1;
  static unsigned long lastUpdate = 0;

  if (millis() - lastUpdate > 500) {
    lastUpdate = millis();
    int raw = analogRead(PLAYER_VOLUME);
    volume = map(raw, 0, 4000, 0, 30);
    if (volume > 25) volume = 25;
    if (volume != lastVolume) {
      if (volume == 0) {
        Serial.println("VolOff");
        playerOff();
        offVolume = true;
      } else {
        if (offVolume) {
          Serial.println("VolOn");
          playerOn();
          setHeadRGBVal(0, 0, 0);
          if (currentMode < 3) {
            mp3.playFolder(currentFolder, currentTrack);
          } else if (currentMode == 3) {
            mp3.playFolder(7, 2);
          } else {
            mp3.playFolder(7, 3);
          }
          Serial.print("VolOn-");
          Serial.print(getVsupply());
          Serial.print(",");
          Serial.print(currentMode);
          Serial.print(",");
          Serial.println(currentTrack);
          trackManuallyChanged = true;
        }
        offVolume = false;
      }
      Serial.print(raw);
      Serial.print(",");
      Serial.println(volume);
      if (!offVolume) mp3.volume(volume);
      lastVolume = volume;
    }
  }
}

void playerOff() {
  digitalWrite(PLAYER_POWER, HIGH);
  digitalWrite(PLAYER_TX, LOW);
}

void playerOn() {
  digitalWrite(PLAYER_TX, HIGH);
  digitalWrite(PLAYER_POWER, LOW);
  delay(500);
  mp3.begin(mp3Serial, false, false);
  int raw = analogRead(PLAYER_VOLUME);
  volume = map(raw, 0, 4000, 0, 30);
  if (volume > 25) volume = 25;
  mp3.volume(volume);
}

void handleVibro() {
  static int vibrolastState = HIGH;  // хранит предыдущее состояние пина
  if (offVolume) {
    if (!FTTOff) {
      if (nightEffect == 0) {
        animateIdleColor();  // "ночной режим 0"
      } else if (nightEffect == 1) {
        setHeadRGBVal(255, 255, 255);  // "ночной режим 1"
      } else if (nightEffect == 2) {
        setHeadRGBVal(255, 0, 0);  // "ночной режим 2"
      } else if (nightEffect == 3) {
        setHeadRGBVal(0, 255, 0);  // "ночной режим 3"
      } else if (nightEffect == 4) {
        setHeadRGBVal(0, 0, 255);  // "ночной режим 4"
      }
    }
    return;
  }
  int state = digitalRead(VIBRO_PIN);

  // проверяем переход HIGH -> LOW
  if (vibrolastState == HIGH && state == LOW && !offVolume) {
    Serial.print("Pulse:");
    Serial.println(getVsupply());
    colorDetected = false;
    lastActiveTime = millis();
    vibroLastPulse = millis();
    low_voltage = false;

    if (!vibroActive) {
      vibroActive = true;
      vibroStart = millis();
      Serial.println("START");
    }

    int r = random(0, 2) * 255;
    int g = random(0, 2) * 255;
    int b = random(0, 2) * 255;

    // исключаем случай когда все 0 (чтобы всегда был цвет)
    if (r == 0 && g == 0 && b == 0) r = 255;

    setHeadRGBVal(r, g, b);
  }

  // если вибрация была, но нет импульсов дольше таймаута
  if (vibroActive && (millis() - vibroLastPulse > VIBRO_TIMEOUT)) {
    vibroActive = false;
    unsigned long duration = millis() - vibroStart;
    Serial.print("STOP, duration = ");
    Serial.print(duration);
    Serial.println(" ms");
  }

  vibrolastState = state;  // обновляем прошлое состояние
}

bool isVibroActive() {
  return vibroActive;
}

void playNextTrack() {
  vibroCurrentTrack++;
  if (vibroCurrentTrack > 33) vibroCurrentTrack = 1;  // допустим у нас 10 треков в папке
  vibroRepeatCount = 0;
  Serial.print("Play track: ");
  Serial.println(vibroCurrentTrack);
  mp3.playFolder(4, vibroCurrentTrack);
}

void handleVibroPlayer() {
  static unsigned long trackStartTime = 0;
  static bool trackPlaying = false;

  if (!isVibroActive()) {
    trackPlaying = false;
    vibroRepeatCount = 0;
    return;
  }

  unsigned long currentMillis = millis();

  // если трек не играет — запустить
  if (!trackPlaying) {
    vibroCurrentTrack++;
    if (vibroCurrentTrack > 33) vibroCurrentTrack = 1;  // 33 треков в папке
    mp3.playFolder(4, vibroCurrentTrack);
    trackStartTime = currentMillis;
    trackPlaying = true;
    vibroRepeatCount = 0;
    Serial.print("Start track: ");
    Serial.println(vibroCurrentTrack);
  }

  // проверяем, прошло ли 2.5 секунды
  if (trackPlaying && currentMillis - trackStartTime >= 1500) {
    if (isVibroActive()) {
      vibroRepeatCount++;
      if (vibroRepeatCount < 3) {
        Serial.print("Repeat track: ");
        Serial.println(vibroCurrentTrack);
        mp3.playFolder(4, vibroCurrentTrack);
        trackStartTime = currentMillis;
      } else {
        Serial.print("Switch track after 3 repeats: ");
        vibroCurrentTrack++;
        if (vibroCurrentTrack > 33) vibroCurrentTrack = 1;
        mp3.playFolder(4, vibroCurrentTrack);
        trackStartTime = currentMillis;
        vibroRepeatCount = 0;
      }
    } else {
      trackPlaying = false;  // вибрации нет, останавливаем
    }
  }
}
