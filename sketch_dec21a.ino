#include <EduIntro.h>
#include <Wire.h>
#include <SparkFun_ENS160.h>
#include <SoftwareSerial.h>
//#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "PCF8575.h"

// ==================== CONFIGURAZIONE DISPLAY ====================
#define TFT_CS A1
#define TFT_RST 12
#define TFT_DC A2
// SDA deve essere collegato al pin 11, SCK al pin 13 (Hardware SPI)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ==================== CONFIGURAZIONE PIN ====================
#define pinDHT 2
#define pinCO A0
#define pinBuzzer 6
#define pinRXCO 10
#define pinTXCO 11
#define pinR 7
#define pinG 8
#define pinY 9
#define ensAddress 0x53
#define PINBTN 1
#define PINLED 0
#define PINSLEEPTFT A3
#define PINBTNSLEEPTFT 2

// ==================== COSTANTI E SOGLIE ====================
#define MAX_MEASURE 5
#define MEASURE_INT 100
#define MAX_NOTIFICHE 13
#define LEN_MSG 20
#define MAX_SAMPLES 30
#define GRAPH_Y 30
#define GRAPH_X 30
#define GRAPH_W 98
#define GRAPH_H 90

uint16_t graphIndex = 0;

float graphMin = 999999;
float graphMax = -999999;

// Filtro Persistenza VOC
const float SOGLIA_VOC_ALLARME = 0.5;
const unsigned long TEMPO_PERSISTENZA_VOC = 5000;
unsigned long tempoInizioPiccoVOC = 0;
bool piccoVOCInCorso = false;
bool allarmeVOCConfermato = false;
unsigned long lastUpdate = 0;

// ==================== ISTANZE E VARIABILI ====================
Adafruit_BMP085 bmp;
SoftwareSerial sensorSerial(pinRXCO, pinTXCO);
SparkFun_ENS160 ens;
DHT11 dht(pinDHT);
PCF8575 pcf(0x20);

float ppmCODigital = 0, ppmCOAnalog = 0;
float temp, hum, aqi, tvoc, mgm3, eco2, eth;
float aqi_med, tvoc_med, mgm3_med, eco2_med, eth_med, temp_med, hum_med;

char notifiche[MAX_NOTIFICHE][LEN_MSG];
int idxNotifica = 0;
int currentNotifica = 0;
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 2000;
unsigned long lastAlarmTime = 0;
const unsigned long ALARM_INTERVAL = 120000;
int livelloPrec = 0;

long startAt;
int stato=0, statoCO=0, coCurrLvl=0, coPrecLvl=0, aqiErr=0;
static int livelloAltoCount = 0;
long sampleIdx=0;
bool toClear=true;
bool mute=false;
bool pulsanteMutePremuto=false;
bool tftAbilitato=true;
bool pulsanteTftPremuto=false;

// ==================== STRUTTURE EMA ====================
struct SogliaCO { float mediaDigitale; float mediaAnalogico; float alpha; bool inizializzato; };
struct SogliaECO2 { float media; float alpha; bool inizializzato; };

struct Sample { float aqi; float co; float tvoc; };
Sample samples[MAX_SAMPLES];

SogliaCO sogliaCO15min = {0.0, 0.0, 0.00111, false};
SogliaCO sogliaCO1ora  = {0.0, 0.0, 0.000278, false};
SogliaCO sogliaCO8ore  = {0.0, 0.0, 0.0000347, false};
SogliaCO sogliaCO24ore = {0.0, 0.0, 0.0000116, false};
SogliaECO2 sogliaECO220min = {0.0, 0.00166, false};

// ==================== FUNZIONI UTIL ====================

float convertTVOCtoMG(float ppb) { return ((ppb * 59.3) / 24.45) / 1000.0; }

bool floatDiverso(float a, float b) {
  return abs(a - b) >= 0.005;
}

void disegnaGrafico(int type) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 0);
  tft.setTextSize(1);
  switch (type) {
    case 0:
      tft.print("Ultime 30 rilevazioni AQI");
      break;
    case 1:
      tft.print("Ultime 30 rilevazioni mg/m3");
      tft.setCursor(0, 11);
      tft.print("TVOC");
      break;
    case 2:
      tft.print("Ultime 30 rilevazioni ppm");
      tft.setCursor(0, 11);
      tft.print("CO");
      break;
  }

  float newMin = 999999, newMax = -999999;
  int validCount = 0;
  int nSamples = min((long)MAX_SAMPLES, sampleIdx);

  for (int i = 0; i < nSamples; i++) {
    int idx = (sampleIdx - nSamples + i + MAX_SAMPLES) % MAX_SAMPLES;
    float v = 0;
    switch (type) {
      case 0: v = samples[idx].aqi; break;
      case 1: v = samples[idx].tvoc; break;
      case 2: v = samples[idx].co; break;
    }
    if (v < newMin) newMin = v;
    if (v > newMax) newMax = v;
    validCount++;
  }

  if (validCount == 0) return;

  if (newMax - newMin < 0.001) {
    newMin = max(0.0f, newMin - 0.01f);
    newMax = newMax + 0.01f;
  }

  graphMin = newMin;
  graphMax = newMax;
  float range = graphMax - graphMin;

  tft.setCursor(0, 13);
  char buf[8];

  dtostrf(graphMax, 5, 3, buf);
  tft.setCursor(0, GRAPH_Y);
  tft.print(buf);

  dtostrf((graphMax + graphMin) / 2.0f, 5, 3, buf);
  tft.setCursor(0, GRAPH_Y + GRAPH_H / 2 - 3);
  tft.print(buf);

  dtostrf(graphMin, 5, 3, buf);
  tft.setCursor(0, GRAPH_Y + GRAPH_H - 8);
  tft.print(buf);

  tft.drawFastVLine(GRAPH_X - 2, GRAPH_Y, GRAPH_H, ST77XX_WHITE);
  tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, ST77XX_BLACK);

  int px = -1, py = -1;
  for (int i = 0; i < nSamples; i++) {
    int idx = (sampleIdx - nSamples + i + MAX_SAMPLES) % MAX_SAMPLES;
    float v = samples[idx].tvoc;

    int x = GRAPH_X + (i * GRAPH_W / max(nSamples - 1, 1));
    int y = (GRAPH_Y + GRAPH_H) - (int)((v - graphMin) * GRAPH_H / range);
    y = constrain(y, GRAPH_Y, GRAPH_Y + GRAPH_H - 1);

    if (px >= 0) {
      tft.drawLine(px, py, x, y, ST77XX_GREEN);
    } else {
      tft.drawPixel(x, y, ST77XX_GREEN);
    }
    px = x;
    py = y;
  }
}

void addSample(Sample s) {
  samples[sampleIdx % MAX_SAMPLES] = s;
  sampleIdx += 1;
}

void addNotifica(const char* msg) {
  if (idxNotifica < MAX_NOTIFICHE) {
    strncpy(notifiche[idxNotifica], msg, LEN_MSG - 1);
    notifiche[idxNotifica][LEN_MSG - 1] = '\0';
    idxNotifica++;
  }
}

bool isChecksumValid(byte pf[]) {
  byte sum = 0;
  for (int j = 1; j < 8; j++) sum += pf[j];
  sum = (~sum) + 1;
  return (sum == pf[8]);
}

void aggiornaSogliaCO(SogliaCO &s, float valDig, float valAna) {
  if (!s.inizializzato) {
    s.mediaDigitale = valDig;
    s.mediaAnalogico = valAna;
    s.inizializzato = true;
    return;
  }
  s.mediaDigitale = (valDig * s.alpha) + (s.mediaDigitale * (1.0 - s.alpha));
  s.mediaAnalogico = (valAna * s.alpha) + (s.mediaAnalogico * (1.0 - s.alpha));
}

void aggiornaSogliaECO2(SogliaECO2 &s, float val) {
  if (!s.inizializzato) {
    s.media = val;
    s.inizializzato = true;
    return;
  }
  s.media = (val * s.alpha) + (s.media * (1.0 - s.alpha));
}

void makeAlarm() {
  tone(pinBuzzer, 2000, 200);
  delay(250);
  tone(pinBuzzer, 2000, 200);
}

void aggiornaLED() {
  int level = max(stato, statoCO);
  digitalWrite(pinR, LOW);
  digitalWrite(pinG, LOW);
  digitalWrite(pinY, LOW);
  if (level == 0) digitalWrite(pinG, HIGH);
  else if (level == 1) digitalWrite(pinY, HIGH);
  else digitalWrite(pinR, HIGH);
}

void stampaNotifica(char* msg, bool clear) {
  if (clear) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
  }
  tft.print(msg);
}

void controllaPulsante() {
  if (pcf.read(PINBTNSLEEPTFT) == LOW) {
    if (!pulsanteTftPremuto) {
      tftAbilitato = !tftAbilitato;
      
      if (!tftAbilitato) {
        // spegnimento display
        analogWrite(PINSLEEPTFT, 0);
      } else {
        //accensione display
        analogWrite(PINSLEEPTFT, 1023);
        toClear = true; 
      }
      
      pulsanteTftPremuto = true;
      delay(200); 
    }
  } else {
    pulsanteTftPremuto = false;
  }

  if (pcf.read(PINBTN) == LOW) {
    if (!pulsanteMutePremuto) {
      mute = !mute;
      pulsanteMutePremuto = true;
      pcf.write(PINLED, mute ? LOW : HIGH);
    }
  } else {
    pulsanteMutePremuto = false;
  }
}

/*void controllaPulsante() {
  if (pcf.read(PINSLEEPTFT) == LOW) {
    if (!pulsanteTftPremuto) {
      tftAbilitato = !tftAbilitato;
      if (tftAbilitato) {
        tft.fillScreen(ST77XX_BLACK);
      }
      tft.enableDisplay(tftAbilitato);
      pulsanteTftPremuto = true;
      pcf.write(3, HIGH);
    }
  } else {
    pulsanteTftPremuto = false;
  }

  if (pcf.read(PINBTN) == LOW) {
    if (!pulsanteMutePremuto) {
      mute = !mute;
      pulsanteMutePremuto = true;
      pcf.write(PINLED, mute ? LOW : HIGH);
    }
  } else {
    pulsanteMutePremuto = false;
  }
}*/

void aggiornaDisplayAsincrono() {
  controllaPulsante();

  if (!tftAbilitato) return;
  if (idxNotifica == 0) return;

  unsigned long now = millis();

  if (now - lastDisplayTime >= displayInterval) {
    int totalSlides = idxNotifica + 3;

    if (currentNotifica < idxNotifica) {
      while (currentNotifica < idxNotifica) {
        if (strcmp(notifiche[currentNotifica], "00") == 0) {
          currentNotifica++;
          lastDisplayTime = now;
          toClear = true;
          return;
        }
        stampaNotifica(notifiche[currentNotifica], toClear);
        toClear = false;
        currentNotifica++;
      }
    } else {
      int graphType = currentNotifica - idxNotifica;
      disegnaGrafico(graphType);
      currentNotifica++;
      if (currentNotifica >= totalSlides) {
        currentNotifica = 0;
        toClear = true;
      }
      lastDisplayTime = now;
    }
  }
}

float calcolaAlpha(float tau) {
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;
  return 1.0 - exp(-dt / tau);
}

int livelloCOtotale() {
  int lvl = 0;
  long now = millis();

  if (ppmCODigital >= 100) lvl = max(lvl, 2);
  else if (ppmCODigital >= 90) lvl = max(lvl, 1);

  if (now >= 900) {
    if (sogliaCO15min.mediaDigitale >= 87) lvl = max(lvl, 2);
    else if (sogliaCO15min.mediaDigitale >= (sogliaCO15min.mediaDigitale - (sogliaCO15min.mediaDigitale * 0.1))) lvl = max(lvl, 1);

    if (now >= 3600) {
      if (sogliaCO1ora.mediaDigitale >= 36) lvl = max(lvl, 2);
      else if (sogliaCO1ora.mediaDigitale >= (sogliaCO1ora.mediaDigitale - (sogliaCO1ora.mediaDigitale * 0.1))) lvl = max(lvl, 1);

      if (now >= 28800) {
        if (sogliaCO8ore.mediaDigitale >= 9) lvl = max(lvl, 2);
        else if (sogliaCO8ore.mediaDigitale >= (sogliaCO8ore.mediaDigitale - (sogliaCO8ore.mediaDigitale * 0.1))) lvl = max(lvl, 1);

        if (now >= 86400) {
          if (sogliaCO24ore.mediaDigitale >= 6) lvl = max(lvl, 2);
          else if (sogliaCO24ore.mediaDigitale >= (sogliaCO24ore.mediaDigitale - (sogliaCO24ore.mediaDigitale * 0.1))) lvl = max(lvl, 1);
        }
      }
    }
  }

  return lvl;
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensorSerial.begin(9600);

  pinMode(pinCO, INPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinR, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinY, OUTPUT);
  pinMode(PINSLEEPTFT, OUTPUT);

  ens.begin(Wire, ensAddress);
  ens.setOperatingMode(SFE_ENS160_STANDARD);
  bmp.begin();

  for (int i = 0; i < MAX_SAMPLES; i++) {
    samples[i] = {0, 0, 0};
  }

  if (!pcf.begin()) {
    return;
  }

  pcf.write(PINBTN, HIGH);
  pcf.write(PINSLEEPTFT, HIGH);
  pcf.write(3, LOW);

  startAt = millis();
  analogWrite(PINSLEEPTFT, 1023);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  stampaNotifica((char*)F("Sistema pronto..."), true);
}

// ==================== LOOP ====================

void loop() {
  idxNotifica = 0;
  aqi_med = 0; tvoc_med = 0; mgm3_med = 0; eco2_med = 0; eth_med = 0; temp_med = 0; hum_med = 0;

  for (int i = 0; i < MAX_MEASURE; i++) {
    dht.update();
    temp = bmp.readTemperature();
    hum = dht.readHumidity();
    ens.setTempCompensationCelsius(temp);
    ens.setRHCompensationFloat(hum);

    aqi_med += ens.getAQI();
    tvoc_med += ens.getTVOC();
    eco2_med += ens.getECO2();
    eth_med += ens.getETOH();
    temp_med += temp;
    hum_med += hum;

    unsigned long startWait = millis();
    while (millis() - startWait < MEASURE_INT) {
      aggiornaDisplayAsincrono();
    }
  }

  aqi_med /= MAX_MEASURE;
  tvoc_med /= MAX_MEASURE;
  mgm3_med = convertTVOCtoMG(tvoc_med);
  eco2_med /= MAX_MEASURE;
  eth_med /= MAX_MEASURE;
  temp_med /= MAX_MEASURE;
  hum_med /= MAX_MEASURE;

  if (sensorSerial.available() >= 9) {
    if (sensorSerial.read() == 0xFF) {
      byte packet[9];
      packet[0] = 0xFF;
      for (int i = 1; i < 9; i++) packet[i] = sensorSerial.read();
      if (isChecksumValid(packet)) ppmCODigital = (((packet[4] & 0x1F) * 256) + packet[5]) * 0.1;
    }
  }

  Sample s = {aqi_med, ppmCODigital, mgm3_med};
  addSample(s);

  sogliaCO15min.alpha = calcolaAlpha(900);
  sogliaCO1ora.alpha  = calcolaAlpha(3600);
  sogliaCO8ore.alpha  = calcolaAlpha(28800);
  sogliaCO24ore.alpha = calcolaAlpha(86400);

  aggiornaSogliaCO(sogliaCO15min, ppmCODigital, 0);
  aggiornaSogliaCO(sogliaCO1ora,  ppmCODigital, 0);
  aggiornaSogliaCO(sogliaCO8ore,  ppmCODigital, 0);
  aggiornaSogliaCO(sogliaCO24ore, ppmCODigital, 0);
  aggiornaSogliaECO2(sogliaECO220min, eco2_med);

  // ==================== FILTRO PERSISTENZA VOC ====================
  if (mgm3_med > SOGLIA_VOC_ALLARME) {
    if (!piccoVOCInCorso) {
      piccoVOCInCorso = true;
      tempoInizioPiccoVOC = millis();
    } else if (millis() - tempoInizioPiccoVOC >= TEMPO_PERSISTENZA_VOC) {
      allarmeVOCConfermato = true;
    }
  } else {
    piccoVOCInCorso = false;
    allarmeVOCConfermato = false;
  }

  // ==================== GESTIONE NOTIFICHE ====================
  char buffer[LEN_MSG], f_val[10];

  dtostrf(temp_med, 0, 1, f_val);
  sprintf(buffer, "T: %s C\n", f_val);
  addNotifica(buffer);

  dtostrf(hum_med, 0, 1, f_val);
  sprintf(buffer, "H: %s %", f_val);
  addNotifica(buffer);
  addNotifica("00");

  dtostrf(aqi_med, 0, 1, f_val);
  sprintf(buffer, "AQI: %s\n", f_val);
  addNotifica(buffer);

  dtostrf(mgm3_med, 0, 1, f_val);
  sprintf(buffer, "TVOC: %s mg/m3\n", f_val);
  addNotifica(buffer);

  dtostrf(eco2_med, 0, 1, f_val);
  sprintf(buffer, "eCO2: %s", f_val);
  addNotifica(buffer);
  addNotifica("00");

  dtostrf(ppmCODigital, 0, 1, f_val);
  sprintf(buffer, "CO: %s ppm\n", f_val);
  addNotifica(buffer);

  unsigned long now = millis();
  if (now >= 900000) {
    dtostrf(sogliaCO15min.mediaDigitale, 0, 1, f_val);
    sprintf(buffer, "CO 15min: %s ppm\n", f_val);
    addNotifica(buffer);

    if (now >= 3600000) {
      dtostrf(sogliaCO1ora.mediaDigitale, 0, 1, f_val);
      sprintf(buffer, "CO 1ora: %s ppm\n", f_val);
      addNotifica(buffer);

      if (now >= 28800000) {
        dtostrf(sogliaCO8ore.mediaDigitale, 0, 1, f_val);
        sprintf(buffer, "CO 8ore: %s ppm\n", f_val);
        addNotifica(buffer);

        if (now >= 86400000) {
          dtostrf(sogliaCO24ore.mediaDigitale, 0, 1, f_val);
          sprintf(buffer, "CO 24ore: %s ppm\n", f_val);
          addNotifica(buffer);
        }
      }
    }
  }

  addNotifica("00");

  if (allarmeVOCConfermato) addNotifica("!! ALLARME VOC !!");

  // ==================== LOGICA ALLARME ====================
  bool pericoloCO = (ppmCODigital >= 100);

  if (aqi_med > 3 || ppmCODigital > 100) stato = 2;
  else if (aqi_med > 2 || ppmCODigital > 90) stato = 1;
  else stato = 0;

  aggiornaLED();

  int livello = max(stato, livelloCOtotale());

  bool peggiorato = livello > livelloPrec;
  bool pericolo = livello >= 1 || (allarmeVOCConfermato && (livello != livelloPrec));

  if (peggiorato && !mute) {
    makeAlarm();
    lastAlarmTime = now;
  } else if ((pericolo && now - lastAlarmTime > ALARM_INTERVAL) && !mute) {
    makeAlarm();
    lastAlarmTime = now;
  }

  livelloPrec = livello;

  if (mute) {
    Serial.println(F("MUTO"));
  } else {
    Serial.println(F("NO MUTO"));
  }

  aggiornaDisplayAsincrono();

  delay(500);
}
