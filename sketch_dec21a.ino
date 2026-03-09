#include <EduIntro.h>
#include <Wire.h>
#include <SparkFun_ENS160.h>
#include <SoftwareSerial.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

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

// ==================== COSTANTI E SOGLIE ====================
#define MAX_MEASURE 5
#define MEASURE_INT 100
#define MAX_NOTIFICHE 10
#define LEN_MSG 45

// Filtro Persistenza VOC
const float SOGLIA_VOC_ALLARME = 0.5;      // mg/m3
const unsigned long TEMPO_PERSISTENZA_VOC = 5000; // 5 secondi di "conferma"
unsigned long tempoInizioPiccoVOC = 0;
bool piccoVOCInCorso = false;
bool allarmeVOCConfermato = false;

// ==================== ISTANZE E VARIABILI ====================
Adafruit_BMP085 bmp;
ThreeWire clockInt(3, 4, 5);
RtcDS1302<ThreeWire> clock(clockInt);
SoftwareSerial sensorSerial(pinRXCO, pinTXCO);
SparkFun_ENS160 ens;
DHT11 dht(pinDHT);

float ppmCODigital = 0, ppmCOAnalog = 0;
float temp, hum, aqi, tvoc, mgm3, eco2, eth;
float aqi_med, tvoc_med, mgm3_med, eco2_med, eth_med, temp_med, hum_med;

char notifiche[MAX_NOTIFICHE][LEN_MSG];
int idxNotifica = 0;
int currentNotifica = 0;
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 2000;

RtcDateTime dt;
uint64_t startAt, aqiStart;
int stato=0, statoCO=0, coCurrLvl=0, coPrecLvl=0, aqiErr=0;
static int livelloAltoCount = 0;

// ==================== STRUTTURE EMA ====================
struct SogliaCO { float mediaDigitale; float mediaAnalogico; float alpha; bool inizializzato; };
struct SogliaECO2 { float media; float alpha; bool inizializzato; };

SogliaCO sogliaCO15min = {0.0, 0.0, 0.0012, false};
SogliaCO sogliaCO1ora   = {0.0, 0.0, 0.0003, false};
SogliaCO sogliaCO8ore   = {0.0, 0.0, 0.000038, false};
SogliaCO sogliaCO24ore  = {0.0, 0.0, 0.0000127, false};
SogliaECO2 sogliaECO220min = {0.0, 0.00166, false};

// ==================== FUNZIONI UTIL ====================

void addNotifica(const char* msg) {
  if (idxNotifica < MAX_NOTIFICHE) {
    strncpy(notifiche[idxNotifica], msg, LEN_MSG - 1);
    notifiche[idxNotifica][LEN_MSG - 1] = '\0';
    idxNotifica++;
  }
}

float convertTVOCtoMG(uint16_t ppb) { return ((ppb * 59.3) / 24.45) / 1000.0; }

bool isChecksumValid(byte pf[]) {
  byte sum = 0;
  for (int j = 1; j < 8; j++) sum += pf[j];
  sum = (~sum) + 1;
  return (sum == pf[8]);
}

void aggiornaSogliaCO(SogliaCO &s, float valDig, float valAna) {
  if (!s.inizializzato) { s.mediaDigitale = valDig; s.mediaAnalogico = valAna; s.inizializzato = true; return; }
  s.mediaDigitale = (valDig * s.alpha) + (s.mediaDigitale * (1.0 - s.alpha));
  s.mediaAnalogico = (valAna * s.alpha) + (s.mediaAnalogico * (1.0 - s.alpha));
}

void aggiornaSogliaECO2(SogliaECO2 &s, float val) {
  if (!s.inizializzato) { s.media = val; s.inizializzato = true; return; }
  s.media = (val * s.alpha) + (s.media * (1.0 - s.alpha));
}

void makeAlarm() {
  tone(pinBuzzer, 2000, 200);
  delay(250);
  tone(pinBuzzer, 2000, 200);
}

void aggiornaLED() {
  int level = max(stato, statoCO);
  digitalWrite(pinR, LOW); digitalWrite(pinG, LOW); digitalWrite(pinY, LOW);
  if (level == 0) digitalWrite(pinG, HIGH);
  else if (level == 1) digitalWrite(pinY, HIGH);
  else digitalWrite(pinR, HIGH);
}

const __FlashStringHelper* checkCO(float valore, float limite, long intervallo) {
  if ((dt.Unix64Time() - startAt) < intervallo) return F("Wait");
  if (valore >= limite) { statoCO = 2; return F("DANGER"); }
  if (valore >= (limite * 0.9)) { statoCO = 1; return F("WARN"); }
  return F("OK");
}

void stampaNotifica(char* msg) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(msg);
}

void aggiornaDisplayAsincrono() {
  if (idxNotifica == 0) return;
  unsigned long now = millis();
  if (now - lastDisplayTime >= displayInterval) {
    stampaNotifica(notifiche[currentNotifica]);
    currentNotifica = (currentNotifica + 1) % idxNotifica;
    lastDisplayTime = now;
  }
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensorSerial.begin(9600);

  pinMode(pinCO, INPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinR, OUTPUT); pinMode(pinG, OUTPUT); pinMode(pinY, OUTPUT);

  ens.begin(Wire, ensAddress);
  ens.setOperatingMode(SFE_ENS160_STANDARD);
  bmp.begin();

  dt = clock.GetDateTime();
  startAt = dt.Unix64Time();
  aqiStart = dt.Unix64Time();

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  stampaNotifica("Sistema Avviato...");
}

// ==================== LOOP ====================

void loop() {
  dt = clock.GetDateTime();
  idxNotifica = 0;
  aqi_med = 0; tvoc_med = 0; mgm3_med = 0; eco2_med = 0; eth_med = 0; temp_med = 0; hum_med = 0;

  // Lettura Sensori con Media
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
  mgm3_med = convertTVOCtoMG(tvoc_med / MAX_MEASURE);
  eco2_med /= MAX_MEASURE;
  eth_med /= MAX_MEASURE;
  temp_med /= MAX_MEASURE;
  hum_med /= MAX_MEASURE;

  // CO Digitale
  if (sensorSerial.available() >= 9) {
    if (sensorSerial.read() == 0xFF) {
      byte packet[9]; packet[0] = 0xFF;
      for (int i = 1; i < 9; i++) packet[i] = sensorSerial.read();
      if (isChecksumValid(packet)) ppmCODigital = (((packet[4] & 0x1F) * 256) + packet[5]) * 0.1;
    }
  }

  // EMA e Soglie
  aggiornaSogliaCO(sogliaCO15min, ppmCODigital, 0);
  aggiornaSogliaCO(sogliaCO1ora, ppmCODigital, 0);
  aggiornaSogliaCO(sogliaCO8ore, ppmCODigital, 0);
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

  dtostrf(temp_med, 0, 1, f_val); sprintf(buffer, "T: %s C", f_val); addNotifica(buffer);
  dtostrf(mgm3_med, 0, 2, f_val); sprintf(buffer, "VOC: %s mg/m3", f_val); addNotifica(buffer);
  dtostrf(ppmCODigital, 0, 1, f_val); sprintf(buffer, "CO: %s ppm", f_val); addNotifica(buffer);
  
  if(allarmeVOCConfermato) addNotifica("!! ALLARME VOC !!");

  // ==================== LOGICA ALLARME ====================
  bool pericoloCO = (ppmCODigital >= 100);
  
  // Gestione LED
  if (aqi_med > 3 || ppmCODigital > 100) stato = 2; // Rosso
  else if (aqi_med > 2 || ppmCODigital > 90) stato = 1; // Giallo
  else stato = 0; // Verde

  aggiornaLED();

  // Suona se CO critico O VOC confermati oltre 5 secondi
  if (pericoloCO || allarmeVOCConfermato) {
    makeAlarm();
  }

  aggiornaDisplayAsincrono();
  delay(500);
}