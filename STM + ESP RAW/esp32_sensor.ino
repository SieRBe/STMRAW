#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "esp_wpa2.h"
#include <time.h>

// === Pin UART ke STM32 ===
#define STM32_RX 16  // RX2 ESP32 (menerima dari  STM32 TX)
#define STM32_TX 17  // TX2 ESP32 (mengirim ke STM32 RX)

// === Pin I2C untuk sensor cahaya VEML7700 ===
#define SDA_PIN 21
#define SCL_PIN 22

// === Pin untuk dua sensor suhu DS18B20 ===
#define DS18B20_PIN_1 4
#define DS18B20_PIN_2 5

// === WiFi WPA2-Enterprise Configuration ===
#define EAP_IDENTITY "rbsatria123@student.uns.ac.id"
#define EAP_USERNAME "rbsatria123@student.uns.ac.id"
#define EAP_PASSWORD "Tehgelas99"
const char* ssid = "UNS SOLO";  // Ganti dengan SSID yang sesuai

// === NTP Configuration ===
const char* ntpServer = "id.pool.ntp.org";
const long gmtOffset_sec = 25200;      // WIB = GMT+7 (7*3600)
const int daylightOffset_sec = 0;      // Indonesia tidak menggunakan DST

// === Status flags ===
bool wifiConnected = false;
bool ntpSynced = false;
unsigned long lastWifiCheck = 0;
const unsigned long wifiCheckInterval = 5000;  // Check WiFi every 5 seconds

// === Objek sensor ===
Adafruit_VEML7700 veml = Adafruit_VEML7700();
OneWire oneWire1(DS18B20_PIN_1);
OneWire oneWire2(DS18B20_PIN_2);
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);

// === LCD I2C (alamat 0x27, 20x4) ===
LiquidCrystal_I2C lcd(0x27, 20, 4);

// === Timer untuk pengiriman data ===
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10000;  // Kirim setiap 10 detik (sesuai STM32)

// === Timer untuk update LCD ===
unsigned long lastLCDUpdate = 0;
const unsigned long lcdUpdateInterval = 1000;  // Update LCD setiap 1 detik

// === Timer untuk NTP sync ===
unsigned long lastNTPSync = 0;
const unsigned long ntpSyncInterval = 3600000;  // Sync NTP setiap 1 jam

// === Counter untuk debugging ===
int sendCount = 0;

// ===== DEKLARASI FUNGSI =====
void printWiFiStatus();
void connectToWiFi();
void syncNTP();
String getFormattedTime();
void printLocalTime();
unsigned long getUnixTimestamp();
void checkWiFiStatus();
void sendSensorData();
void updateLCDDisplay();

// ===== FUNGSI WIFI STATUS =====
void printWiFiStatus() {
  wl_status_t status = WiFi.status();
  Serial.print("Status WiFi: ");
  switch (status) {
    case WL_CONNECTED:      
      Serial.println("TERHUBUNG ✅"); 
      break;
    case WL_DISCONNECTED:   
      Serial.println("TERPUTUS ❌"); 
      break;
    case WL_NO_SSID_AVAIL:  
      Serial.println("SSID tidak tersedia ⚠️"); 
      break;
    case WL_CONNECT_FAILED: 
      Serial.println("Gagal menyambung ❌"); 
      break;
    default:                
      Serial.println("Status tidak diketahui..."); 
      break;
  }
}

// ===== FUNGSI WIFI WPA2-ENTERPRISE =====
void connectToWiFi() {
  Serial.println("\n=== Menghubungkan ke WiFi WPA2-Enterprise ===");
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);

  esp_wifi_sta_wpa2_ent_enable();
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));

  WiFi.begin(ssid);
  Serial.printf("Menyambung ke SSID: %s\n", ssid);
  Serial.printf("Username: %s\n", EAP_USERNAME);
  
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    printWiFiStatus();
    delay(500);
    retry++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("✅ WiFi Tersambung!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    wifiConnected = false;
    Serial.println("❌ Gagal menyambung WiFi setelah beberapa percobaan.");
    Serial.println("⚠️  Please check credentials and network availability");
  }
}

// ===== FUNGSI NTP TIME SYNC =====
void syncNTP() {
  if (!wifiConnected) {
    Serial.println("❌ [NTP] WiFi not connected, skipping NTP setup");
    return;
  }
  
  Serial.println("🕐 [NTP] Sinkronisasi waktu NTP...");
  
  // Configure NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  // Wait for time sync
  Serial.print("⏳ Syncing time");
  int retries = 0;
  while (time(nullptr) < 100000 && retries < 10) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();
  
  if (time(nullptr) > 100000) {
    ntpSynced = true;
    Serial.println("✅ [NTP] NTP tersinkronisasi!");
    printLocalTime();
  } else {
    ntpSynced = false;
    Serial.println("❌ [NTP] Gagal sinkronisasi NTP");
  }
}

// ===== FUNGSI GET FORMATTED TIMESTAMP =====
String getFormattedTime() {
  if (!ntpSynced) {
    return "TIME_NOT_SYNCED";
  }
  
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

// ===== FUNGSI PRINT LOCAL TIME =====
void printLocalTime() {
  Serial.print("Waktu sekarang: ");
  Serial.println(getFormattedTime());
}

// ===== FUNGSI GET UNIX TIMESTAMP =====
unsigned long getUnixTimestamp() {
  if (!ntpSynced) {
    return 0;
  }
  
  time_t now;
  time(&now);
  return (unsigned long)now;
}

// ===== FUNGSI CHECK WIFI STATUS =====
void checkWiFiStatus() {
  printWiFiStatus();
  
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println("⚠️  [WiFi] Connection lost! Attempting reconnection...");
      wifiConnected = false;
      ntpSynced = false;
    }
    
    // Coba reconnect dulu
    Serial.println("⚠️ WiFi terputus, mencoba reconnect...");
    WiFi.reconnect();
    
    // Tunggu sebentar untuk hasil reconnect
    delay(2000);
    
    // Jika masih gagal, coba koneksi ulang penuh
    if (WiFi.status() != WL_CONNECTED) {
      connectToWiFi();
      if (wifiConnected) {
        syncNTP();
      }
    } else {
      wifiConnected = true;
      Serial.println("✅ WiFi reconnect berhasil!");
      // Sync NTP setelah reconnect
      syncNTP();
    }
  } else {
    if (!wifiConnected) {
      wifiConnected = true;
      Serial.println("✅ WiFi status confirmed connected");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  while(!Serial && millis() < 5000) {
    delay(10);
  }
  
  Serial.println("\n\n\n========== ESP32: STARTUP ==========");
  Serial.println("Initializing sensors...\n");

  // --- Inisialisasi UART ke STM32 ---
  Serial2.begin(9600, SERIAL_8N1, STM32_RX, STM32_TX);
  Serial.println("✓ Serial2 (UART ke STM32) initialized - 9600 baud");

  // --- Inisialisasi I2C & VEML7700 ---
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(500);
  
  // --- Inisialisasi LCD I2C ---
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32 Sensor Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  Serial.println("✓ LCD I2C initialized");
  
  if (!veml.begin()) {
    Serial.println("❌ VEML7700 tidak ditemukan! Periksa koneksi I2C.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR: VEML7700");
    lcd.setCursor(0, 1);
    lcd.print("Check I2C Connection");
    while (1) {
      delay(2000);
    }
  }
  veml.setGain(VEML7700_GAIN_1);
  veml.setIntegrationTime(VEML7700_IT_100MS);
  Serial.println("✓ VEML7700 (Sensor Cahaya) initialized");

  // --- Inisialisasi sensor suhu DS18B20 ---
  sensor1.begin();
  sensor2.begin();
  Serial.println("✓ DS18B20 Sensor 1 initialized");
  Serial.println("✓ DS18B20 Sensor 2 initialized");

  // --- Inisialisasi WiFi WPA2-Enterprise ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi...");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
  
  connectToWiFi();
  
  // --- Inisialisasi NTP ---
  if (wifiConnected) {
    lcd.setCursor(0, 2);
    lcd.print("WiFi: Connected");
    lcd.setCursor(0, 3);
    String ipStr = WiFi.localIP().toString();
    if (ipStr.length() > 20) {
      ipStr = ipStr.substring(0, 20);
    }
    lcd.print(ipStr);
    
    syncNTP();
    lastNTPSync = millis();
    
    if (ntpSynced) {
      Serial.println("✅ All systems initialized successfully!");
    } else {
      Serial.println("⚠️  System initialized (NTP sync failed)");
    }
  } else {
    lcd.setCursor(0, 2);
    lcd.print("WiFi: Failed");
    lcd.setCursor(0, 3);
    lcd.print("Check credentials");
    Serial.println("⚠️  System initialized (WiFi failed)");
  }
  
  delay(3000);

  lastSendTime = millis();
  lastLCDUpdate = millis();
  lastWifiCheck = millis();
  
  Serial.println("\n========== ESP32: READY ==========");
  Serial.println("Mengirim data ke STM32 setiap 10 detik");
  Serial.println("Format: timestamp,lux,temp1,temp2\n");
  
  // --- Display awal di LCD ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32 READY");
  lcd.setCursor(0, 1);
  lcd.print("WiFi:");
  lcd.print(wifiConnected ? "OK" : "FAIL");
  lcd.print(" NTP:");
  lcd.print(ntpSynced ? "OK" : "FAIL");
}

void loop() {
  unsigned long currentTime = millis();

  // Check WiFi status setiap 5 detik
  if (currentTime - lastWifiCheck >= wifiCheckInterval) {
    lastWifiCheck = currentTime;
    checkWiFiStatus();
  }

  // Sync NTP setiap 1 jam (jika WiFi connected)
  if (wifiConnected && (currentTime - lastNTPSync >= ntpSyncInterval)) {
    lastNTPSync = currentTime;
    Serial.println("🕐 [NTP] Periodic sync...");
    syncNTP();
  }

  // Kirim data setiap sendInterval (10 detik)
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    sendSensorData();
  }

  // Update LCD setiap 1 detik dengan data terbaru
  if (currentTime - lastLCDUpdate >= lcdUpdateInterval) {
    lastLCDUpdate = currentTime;
    updateLCDDisplay();
  }

  delay(10);  // Prevent watchdog timeout
}

void sendSensorData() {
  // === Baca sensor cahaya ===
  float lux = veml.readLux();
  if (isnan(lux)) {
    lux = 0.0;
    Serial.println("⚠️  [VEML7700] Gagal membaca data!");
  }

  // === Baca dua sensor suhu ===
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  
  float temp1 = sensor1.getTempCByIndex(0);
  float temp2 = sensor2.getTempCByIndex(0);

  // Validasi data sensor suhu
  if (temp1 == DEVICE_DISCONNECTED_C) {
    Serial.println("⚠️  [Sensor1] Disconnect atau error!");
    temp1 = 0.0;
  }
  
  if (temp2 == DEVICE_DISCONNECTED_C) {
    Serial.println("⚠️  [Sensor2] Disconnect atau error!");
    temp2 = 0.0;
  }

  // === Get timestamp ===
  String timestamp = getFormattedTime();
  unsigned long unixTime = getUnixTimestamp();

  // === Kirim data ke STM32 dalam format CSV ===
  // Format: timestamp,lux,temp1,temp2 atau unix_timestamp,lux,temp1,temp2
  String dataToSend;
  if (ntpSynced) {
    // Kirim unix timestamp untuk STM32 (lebih mudah diproses)
    dataToSend = String(unixTime) + "," + String(lux, 1) + "," + String(temp1, 1) + "," + String(temp2, 1);
  } else {
    // Kirim millis() jika NTP tidak tersedia
    dataToSend = String(millis()/1000) + "," + String(lux, 1) + "," + String(temp1, 1) + "," + String(temp2, 1);
  }
  
  Serial2.println(dataToSend);
  sendCount++;

  // === Debug di Serial Monitor ESP32 ===
  Serial.println("====================================");
  Serial.print("📤 [SEND #");
  Serial.print(sendCount);
  Serial.print("] WiFi: ");
  Serial.print(wifiConnected ? "OK" : "FAIL");
  Serial.print(" NTP: ");
  Serial.println(ntpSynced ? "OK" : "FAIL");
  
  Serial.print("   Timestamp   : ");
  Serial.print(timestamp);
  if (ntpSynced) {
    Serial.print(" (Unix: ");
    Serial.print(unixTime);
    Serial.println(")");
  } else {
    Serial.println(" (NTP not synced)");
  }
  
  Serial.print("   Lux Cahaya  : ");
  Serial.print(lux, 1);
  Serial.println(" lx");
  Serial.print("   Suhu Sensor1: ");
  Serial.print(temp1, 1);
  Serial.println(" °C");
  Serial.print("   Suhu Sensor2: ");
  Serial.print(temp2, 1);
  Serial.println(" °C");
  Serial.print("   Data dikirim: ");
  Serial.println(dataToSend);
  
  if (wifiConnected) {
    Serial.print("   WiFi RSSI   : ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
  
  Serial.println("====================================\n");
}

void updateLCDDisplay() {
  // === Baca sensor untuk display LCD ===
  float lux = veml.readLux();
  if (isnan(lux)) {
    lux = 0.0;
  }

  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  
  float temp1 = sensor1.getTempCByIndex(0);
  float temp2 = sensor2.getTempCByIndex(0);

  if (temp1 == DEVICE_DISCONNECTED_C) {
    temp1 = 0.0;
  }
  
  if (temp2 == DEVICE_DISCONNECTED_C) {
    temp2 = 0.0;
  }

  // === Update LCD Display ===
  lcd.clear();
  
  // Baris 1: Header dengan status koneksi
  lcd.setCursor(0, 0);
  lcd.print("ESP32 #");
  lcd.print(sendCount);
  lcd.setCursor(12, 0);
  lcd.print(wifiConnected ? "W" : "w");  // W=WiFi OK, w=WiFi fail
  lcd.print(ntpSynced ? "T" : "t");      // T=Time OK, t=Time fail
  
  // Show WiFi status indicator
  lcd.setCursor(15, 0);
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print("●");  // Connected indicator
  } else {
    lcd.print("○");  // Disconnected indicator
  }
  
  // Baris 2: Lux dengan RSSI
  lcd.setCursor(0, 1);
  lcd.print("Lux:");
  lcd.print(lux, 1);
  if (wifiConnected) {
    int rssi = WiFi.RSSI();
    lcd.setCursor(12, 1);
    lcd.print(rssi);
    lcd.print("dB");
  }
  
  // Baris 3: Temperature 1
  lcd.setCursor(0, 2);
  lcd.print("T1:");
  lcd.print(temp1, 1);
  lcd.print("C");
  
  // Baris 4: Temperature 2 dengan waktu (jika tersedia)
  lcd.setCursor(0, 3);
  lcd.print("T2:");
  lcd.print(temp2, 1);
  lcd.print("C");
  
  // Tampilkan waktu singkat di kanan bawah jika NTP tersync
  if (ntpSynced) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      lcd.setCursor(11, 3);
      char timeStr[9];
      strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
      lcd.print(timeStr);
    }
  } else {
    // Show last sync attempt time
    lcd.setCursor(11, 3);
    unsigned long uptime = millis() / 1000;
    lcd.print(uptime / 60);
    lcd.print(":");
    if ((uptime % 60) < 10) lcd.print("0");
    lcd.print(uptime % 60);
  }
}
