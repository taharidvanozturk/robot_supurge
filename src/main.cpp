/******************************************************************************
 * ROVER SÜPÜRGE — BLE Kontrollü Robot Süpürge
 *
 * Donanım:
 *   - ESP32 (BLE üzerinden Nordic UART servisi ile komut alır)
 *   - 2 adet DC motor + 2 adet L293D motor sürücü (sol/sağ teker)
 *   - HC-SR04 ön mesafe sensörü (otonom modda engel algılama)
 *   - TIP3055 NPN transistör ile sürülen salyangoz fan (aktif-HIGH)
 *   - 2S BMS 18650 Li-ion pillerin sağlıklı kontrolü için (ESP32, motorlar ve fan için)
 *   - 2S 18650 Li-ion pil (ESP32, motorlar ve fan için)
 *   - 12v fan için step-up dönüştürücü
 *   - Şasi, kapak ve montaj parçaları (3D baskı)
 *   - 2S pil şarj devresi (dahili Type-C USB ile şarj)
 *   - Güç Switchi (pil bağlantısını kesmek için)
 * 
 * Çalışma Modları:
 *   1) Manuel mod  — telefondan gelen F/B/L/R tuşları ile küçük adımlı sürüş
 *   2) Otonom mod  — engelden kaçarak kendi başına gezinme + fan otomatik açık
 *
 * BLE Komutları (telefondan tek karakter olarak gönderilir):
 *   F = ileri    B = geri     L = sol     R = sağ
 *   S = tam dur (otonomu da kapatır)
 *   X = otonom modu başlat (fanı da açar)
 *   O = fan AÇ   P = fan KAPAT   V = fan toggle
 *
 * Telemetri (telefona her saniye gönderilir):
 *   "D:<mesafe_cm> F:<0|1> A:<0|1>"  → mesafe, fan durumu, otonom mod
 *
 * Kullanılan Kütüphaneler:
 *   - Arduino.h        → Arduino çekirdek API (pinMode, digitalWrite, millis,
 *                        delay, analogWrite/PWM, Serial vb. temel fonksiyonlar)
 *   - BLEDevice.h      → ESP32 BLE yığınının ana arayüzü; cihazı başlatır
 *                        (BLEDevice::init), server/advertising yönetir
 *   - BLEServer.h      → BLE sunucu nesnesi; bağlantı geri çağrılarını
 *                        (onConnect / onDisconnect) burada tanımlıyoruz
 *   - BLEUtils.h       → BLE yardımcı tipler ve makrolar (UUID dönüşümleri vb.)
 *   - BLE2902.h        → Notify özelliği için gerekli olan standart
 *                        Client Characteristic Configuration Descriptor (CCCD).
 *                        Telefon "notify aboneliği" açabilsin diye TX
 *                        karakteristiğine eklenir.
 *   - soc/soc.h        → ESP32 düşük seviye register tanımları
 *   - soc/rtc_cntl_reg.h → RTC kontrol register adresleri; brownout
 *                        dedektörünü devre dışı bırakmak için kullanılır
 *                        (WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0))
 *
 * NOT: BLE kütüphaneleri (BLEDevice, BLEServer, BLEUtils, BLE2902) ESP32
 * Arduino çekirdeği ile birlikte gelir; ayrıca kurulum gerekmez. platformio.ini
 * içinde "framework = arduino" ve "platform = espressif32" yeterlidir.
 ******************************************************************************/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ===========================================================================
// PİN TANIMLARI
// ===========================================================================

// Sol motor (L293D U2, M1 çıkışı)
const int solEnable = 18;   // PWM hız kontrolü
const int solIn1    = 25;   // Yön kontrolü 1
const int solIn2    = 26;   // Yön kontrolü 2

// Sağ motor (L293D U3, M3 çıkışı)
const int sagEnable = 19;
const int sagIn1    = 16;
const int sagIn2    = 17;

// HC-SR04 ön mesafe sensörü
const int onTrig = 4;       // Tetik çıkışı
const int onEcho = 34;      // Echo girişi (input-only pin)

// Salyangoz fan kontrolü (TIP3055 transistör üzerinden)
// GPIO14 → 1kΩ direnç → TIP3055 base; fan emitter-collector hattından geçer
const int roleFan = 14;
const bool ROLE_AKTIF = HIGH;  // Pin HIGH → transistör iletim → fan ÇALIŞIR
const bool ROLE_PASIF = LOW;   // Pin LOW  → transistör kesim  → fan DURUR

// ===========================================================================
// AYAR SABİTLERİ
// ===========================================================================

// Motor hızları (0-255 PWM aralığı)
#define HIZ_ILERI       210   // Otonom moddaki ileri hız
#define HIZ_GERI        220   // Otonom moddaki geri kaçış hızı
#define HIZ_DONUS       220   // Otonom moddaki dönüş hızı

// Engel algılama parametreleri
#define ENGEL_ESIK_CM   15    // Bu mesafenin altındaki cisimler engel sayılır
#define MESAFE_GECERSIZ 999   // HC-SR04 timeout veya hatalı okuma değeri

// Otonom modda engelden kaçış adımlarının süreleri (ms)
#define SURE_DUR        150   // Engel görünce durup teyit etme süresi
#define SURE_GERI       200   // Engelden uzaklaşmak için geri gitme süresi
#define SURE_DONUS      150   // Yeni yön bulmak için dönüş süresi

// Manuel modda her tuşa basışta yapılacak küçük "dürtme" süreleri
// (Uygulama tek karakter gönderdiği için robot bu süre boyunca hareket eder)
#define DURTME_ILERI    120   // İleri/geri tek tuş süresi
#define DURTME_DONUS    80    // Sağ/sol tek tuş süresi (daha kısa = daha az açı)

// Telefona telemetri gönderim periyodu (ms)
#define SENSOR_PERIYOT  1000

// ===========================================================================
// BLE — Nordic UART Service UUID'leri (standart NUS profili)
// ===========================================================================
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Telefon → ESP32
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 → Telefon

// ===========================================================================
// DURUM DEĞİŞKENLERİ
// ===========================================================================

BLECharacteristic *pSensorCharacteristic;  // Telemetri karakteristiği (notify)
bool deviceConnected = false;              // BLE bağlı mı?
bool otonomMod       = false;              // Otonom mod aktif mi?
bool fanAcik         = false;              // Fan durumu

// Zamanlayıcılar (millis() tabanlı, bloke etmeyen kontrol için)
unsigned long sonSensorZamani    = 0;  // En son telemetri gönderim anı
unsigned long sonKomutZamani     = 0;  // En son manuel komut anı
unsigned long aktifDurtmeSuresi  = 0;  // Mevcut dürtmenin süresi (ileri/dönüş'e göre değişir)

// Otonom mod state machine — engelden kaçış adımları sırayla yürür
enum OtonomState { OTO_ILERI, OTO_DUR, OTO_GERI, OTO_DONUS };
OtonomState otonomState = OTO_ILERI;
unsigned long otonomStateBaslangic = 0;  // Mevcut state'e girilen an
bool donusSaga = true;                   // Her engelde rastgele yön seçilir

// ===========================================================================
// BLE GERİ ÇAĞRILARI
// ===========================================================================

// Bağlantı durumunu takip eder; bağlantı kopunca tekrar advertising başlatır
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    BLEDevice::startAdvertising();  // Tekrar bağlanılabilsin diye reklam at
  }
};

// ===========================================================================
// FAN KONTROL
// ===========================================================================

// Fanı aç/kapat. ac=true → fan çalışır, ac=false → fan durur.
// Fan açılırken küçük bir gecikme verilir: motor PWM gürültüsü ve BLE radyosu
// stabil hale gelsin, ESP32 besleme spike'ı yaşamasın diye.
void fanAyarla(bool ac) {
  if (ac && !fanAcik) {
    delay(50);  // Açılış öncesi besleme stabilize olsun
  }
  fanAcik = ac;
  digitalWrite(roleFan, ac ? ROLE_AKTIF : ROLE_PASIF);
}

// ===========================================================================
// MOTOR KONTROL
// ===========================================================================

// İki tekeri aynı hızda fakat istenen yönlerde sür.
//   hiz       : 0-255 arası PWM değeri
//   sagIleri  : true → sağ teker ileri, false → geri
//   solIleri  : true → sol teker ileri, false → geri
//
// İleri gitmek için ikisi de true, geri için ikisi de false,
// sağa dönmek için sağ=false sol=true, sola dönmek için sağ=true sol=false.
void motorSur(int hiz, bool sagIleri, bool solIleri) {
  digitalWrite(sagIn1, sagIleri ? HIGH : LOW);
  digitalWrite(sagIn2, sagIleri ? LOW  : HIGH);
  analogWrite(sagEnable, hiz);
  digitalWrite(solIn1, solIleri ? HIGH : LOW);
  digitalWrite(solIn2, solIleri ? LOW  : HIGH);
  analogWrite(solEnable, hiz);
}

// İki motoru da durdur (PWM=0 ve yön pinleri LOW).
void motorDur() {
  analogWrite(sagEnable, 0);
  analogWrite(solEnable, 0);
  digitalWrite(sagIn1, LOW); digitalWrite(sagIn2, LOW);
  digitalWrite(solIn1, LOW); digitalWrite(solIn2, LOW);
}

// ===========================================================================
// MESAFE SENSÖRÜ
// ===========================================================================

// HC-SR04'ten tek ölçüm al, mesafeyi cm cinsinden döndür.
// Echo gelmezse (timeout 25ms) MESAFE_GECERSIZ döner.
long mesafeOku() {
  digitalWrite(onTrig, LOW);  delayMicroseconds(2);
  digitalWrite(onTrig, HIGH); delayMicroseconds(10);
  digitalWrite(onTrig, LOW);
  long sure = pulseIn(onEcho, HIGH, 25000);
  return (sure <= 0) ? MESAFE_GECERSIZ : (sure * 0.034 / 2);
}

// 3 ardışık okumanın medyanını döndürür — tek parazitli/sıçramış okumayı eler.
// Otonom modda yanlış engel algılamasını azaltmak için kullanılır.
long mesafeOkuFiltreli() {
  long a = mesafeOku();
  long b = mesafeOku();
  long c = mesafeOku();
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

// ===========================================================================
// BLE KOMUT İŞLEYİCİ
// ===========================================================================

// Telefondan gelen her yazma işleminde tetiklenir. İlk karakteri komut sayar.
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String value = pCharacteristic->getValue().c_str();
    if (value.length() == 0) return;
    char komut = value[0];

    switch (komut) {
      case 'X':  // Otonom modu başlat — fan otomatik açılır, state sıfırlanır
        otonomMod = true;
        otonomState = OTO_ILERI;
        otonomStateBaslangic = millis();
        fanAyarla(true);
        break;

      case 'S':  // Acil durdurma — otonomu da kapatır, motorları durdurur
        otonomMod = false;
        motorDur();
        break;

      case 'V':  // Fan toggle (aç/kapa)
        fanAyarla(!fanAcik);
        break;

      case 'O':  // Fan AÇ
        fanAyarla(true);
        break;

      case 'P':  // Fan KAPAT
        fanAyarla(false);
        break;

      // Manuel sürüş komutları — küçük "dürtme" yapar, watchdog süre sonu durdurur
      case 'F': case 'B': case 'L': case 'R':
        otonomMod = false;
        sonKomutZamani = millis();
        if      (komut == 'F') { motorSur(255, true,  true);  aktifDurtmeSuresi = DURTME_ILERI; }
        else if (komut == 'B') { motorSur(255, false, false); aktifDurtmeSuresi = DURTME_ILERI; }
        else if (komut == 'L') { motorSur(240, true,  false); aktifDurtmeSuresi = DURTME_DONUS; }
        else if (komut == 'R') { motorSur(240, false, true);  aktifDurtmeSuresi = DURTME_DONUS; }
        break;
    }
  }
};

// ===========================================================================
// SETUP — Sistem başlatma
// ===========================================================================
void setup() {
  // Brownout dedektörünü devre dışı bırak: fan/motor kalkış akımında oluşan
  // anlık voltaj düşmelerinde ESP32 kendini resetlemesin diye.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);

  // Motor pinlerini çıkış yap
  pinMode(sagEnable, OUTPUT); pinMode(sagIn1, OUTPUT); pinMode(sagIn2, OUTPUT);
  pinMode(solEnable, OUTPUT); pinMode(solIn1, OUTPUT); pinMode(solIn2, OUTPUT);

  // Mesafe sensörü pinleri
  pinMode(onTrig, OUTPUT);
  pinMode(onEcho, INPUT);

  // Rastgele dönüş yönü için iyi bir seed: boş analog pin + ESP32 donanım RNG
  randomSeed(analogRead(35) ^ esp_random());

  // Fan kontrol pini: TIP3055 aktif-HIGH olduğu için boot anında LOW kalsın
  // (transistör kesim, fan kapalı).
  pinMode(roleFan, OUTPUT);
  digitalWrite(roleFan, ROLE_PASIF);
  fanAcik = false;
  motorDur();

  // BLE servisini başlat (Nordic UART profili)
  BLEDevice::init("Rover_Supurge");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // RX karakteristiği — telefondan komut alır
  BLECharacteristic *pRxChar = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxChar->setCallbacks(new MyCallbacks());

  // TX karakteristiği — telefona telemetri gönderir (notify)
  pSensorCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pSensorCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Rover_Supurge hazir, BLE baglantisi bekleniyor...");
}

// ===========================================================================
// OTONOM MOD STATE MACHINE
// ===========================================================================
//
// Engelden kaçış 4 aşamalı bir state makinesiyle yürütülür. delay() kullanılmaz;
// her aşama millis() ile süre kontrol eder. Bu sayede otonom moddayken bile
// BLE komutları (özellikle 'S' acil dur) anında işlenebilir.
//
//   OTO_ILERI  → düz ilerle, mesafe sensörü ile sürekli engel kontrolü
//   OTO_DUR    → engel görüldü, dur, kısa süre sonra teyit oku (false alarm filtresi)
//   OTO_GERI   → teyit edildi, geri git
//   OTO_DONUS  → rastgele seçilen yöne dön, sonra OTO_ILERI'ye geri dön
// ===========================================================================
void otonomCalistir() {
  unsigned long simdi = millis();
  unsigned long gecen = simdi - otonomStateBaslangic;

  switch (otonomState) {
    case OTO_ILERI: {
      long d = mesafeOkuFiltreli();
      if (d > 0 && d <= ENGEL_ESIK_CM) {
        // Engel algılandı → dur ve teyit aşamasına geç
        motorDur();
        otonomState = OTO_DUR;
        otonomStateBaslangic = simdi;
      } else {
        motorSur(HIZ_ILERI, true, true);
      }
      break;
    }

    case OTO_DUR: {
      if (gecen >= SURE_DUR) {
        // Dur süresi doldu — engel hala duruyor mu kontrol et
        long d = mesafeOkuFiltreli();
        if (d > 0 && d <= ENGEL_ESIK_CM) {
          // Gerçek engel: geri git, dönüş yönünü rastgele seç
          donusSaga = (random(2) == 0);
          motorSur(HIZ_GERI, false, false);
          otonomState = OTO_GERI;
          otonomStateBaslangic = simdi;
        } else {
          // False alarm (parazit, geçici cisim) → ileri yola devam
          otonomState = OTO_ILERI;
        }
      }
      break;
    }

    case OTO_GERI: {
      if (gecen >= SURE_GERI) {
        // Yeterince geri gidildi → seçilen yöne dön
        if (donusSaga) motorSur(HIZ_DONUS, false, true);   // sağa dön
        else           motorSur(HIZ_DONUS, true,  false);  // sola dön
        otonomState = OTO_DONUS;
        otonomStateBaslangic = simdi;
      }
      break;
    }

    case OTO_DONUS: {
      if (gecen >= SURE_DONUS) {
        // Dönüş tamamlandı → tekrar ileri sürüşe geç
        otonomState = OTO_ILERI;
      }
      break;
    }
  }
}

// ===========================================================================
// ANA DÖNGÜ
// ===========================================================================
void loop() {
  // 1) Periyodik olarak telefona telemetri gönder (mesafe, fan, mod)
  if (deviceConnected && (millis() - sonSensorZamani > SENSOR_PERIYOT)) {
    long d = mesafeOku();
    String veri = "D:" + String(d) +
                  " F:" + String(fanAcik ? 1 : 0) +
                  " A:" + String(otonomMod ? 1 : 0);
    pSensorCharacteristic->setValue(veri.c_str());
    pSensorCharacteristic->notify();
    sonSensorZamani = millis();
  }

  // 2) Otonom moddaysa state machine'i ilerlet
  if (otonomMod) {
    otonomCalistir();
  }
  // 3) Manuel moddaysa: dürtme süresi dolduysa motorları durdur (watchdog)
  else if (sonKomutZamani > 0 && (millis() - sonKomutZamani > aktifDurtmeSuresi)) {
    motorDur();
    sonKomutZamani = 0;
  }

  delay(10);  // CPU'yu rahatlatmak için kısa bekleme
}
