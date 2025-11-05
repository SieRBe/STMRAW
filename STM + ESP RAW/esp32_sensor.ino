#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <esp_wpa2.h>
#include <time.h>
#include <esp_sntp.h>

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
#define EAP_IDENTITY "your_username"       // Ganti dengan username Anda
#define EAP_PASSWORD "your_password"       // Ganti dengan password Anda
#define WIFI_SSID "UNS_Mahasiswa"         // Ganti dengan SSID WPA2-Enterprise

// === NTP Configuration ===
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 25200               // GMT+7 untuk Indonesia (7*3600)
#define DAYLIGHT_OFFSET_SEC 0              // Tidak ada daylight saving

// === Status flags ===
bool wifiConnected = false;
bool ntpSynced = false;
unsigned long lastWifiCheck = 0;
const unsigned long wifiCheckInterval = 30000;  // Check WiFi every 30 seconds

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

// === Counter untuk debugging ===
int sendCount = 0;

// ===== FUNGSI WIFI WPA2-ENTERPRISE =====
void connectToWiFi() {
  Serial.println("\n🌐 [WiFi] Connecting to WPA2-Enterprise...");
  
  // Disconnect jika sudah terhubung
  WiFi.disconnect(true);
  delay(1000);
  
  // Set WiFi mode
  WiFi.mode(WIFI_STA);
  
  // Configure WPA2-Enterprise
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();
  
  // Start WiFi connection
  WiFi.begin(WIFI_SSID);
  
  Serial.print("📡 Connecting to ");
  Serial.print(WIFI_SSID);
  Serial.print(" as ");
  Serial.print(EAP_IDENTITY);
  Serial.println("...");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\n✅ [WiFi] Connected!");
    Serial.print("📶 IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("📡 Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    wifiConnected = false;
    Serial.println("\n❌ [WiFi] Connection failed!");
    Serial.println("⚠️  Please check credentials and network availability");
  }
}

// ===== FUNGSI NTP TIME SYNC =====
void initializeNTP() {
  if (!wifiConnected) {
    Serial.println("❌ [NTP] WiFi not connected, skipping NTP setup");
    return;
  }
  
  Serial.println("🕐 [NTP] Initializing time synchronization...");
  
  // Configure NTP
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  
  // Wait for time sync
  Serial.print("⏳ Syncing time");
  int syncAttempts = 0;
  while (!time(nullptr) && syncAttempts < 30) {
    delay(1000);
    Serial.print(".");
    syncAttempts++;
  }
  
  if (time(nullptr)) {
    ntpSynced = true;
    Serial.println("\n✅ [NTP] Time synchronized!");
    
    // Display current time
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      Serial.print("🕐 Current time: ");
      Serial.println(getFormattedTime());
    }
  } else {
    ntpSynced = false;
    Serial.println("\n❌ [NTP] Time sync failed!");
  }
}

// ===== FUNGSI GET FORMATTED TIMESTAMP =====
String getFormattedTime() {
  if (!ntpSynced) {
    return "TIME_NOT_SYNCED";
  }
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "TIME_ERROR";
  }
  
  char timeStr[20];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeStr);
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
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println("⚠️  [WiFi] Connection lost! Attempting reconnection...");
      wifiConnected = false;
      ntpSynced = false;
    }
    connectToWiFi();
    if (wifiConnected) {
      initializeNTP();
    }
  }
}

void setup() {
  Serial.begin(115200);  // Debug monitor
  delay(1000);
  
  Serial.println("\n\n========== ESP32: STARTUP ==========");
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
  lcd.print(WIFI_SSID);
  
  connectToWiFi();
  
  if (wifiConnected) {
    lcd.setCursor(0, 2);
    lcd.print("WiFi: Connected");
    lcd.setCursor(0, 3);
    lcd.print(WiFi.localIP().toString());
    
    // --- Inisialisasi NTP ---
    initializeNTP();
    
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
  
  delay(3000);  // Show WiFi status for 3 seconds

  lastSendTime = millis();
  lastLCDUpdate = millis();
  lastWifiCheck = millis();
  
  Serial.println("\n========== ESP32: READY ==========");
  Serial.println("Mengirim data ke STM32 setiap 10 detik");
  Serial.println("Format: timestamp,lux,temp1,temp2\\n\n");
  
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

  // Check WiFi status setiap 30 detik
  if (currentTime - lastWifiCheck >= wifiCheckInterval) {
    lastWifiCheck = currentTime;
    checkWiFiStatus();
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
  Serial.println("]");
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
  Serial.print("   WiFi Status : ");
  Serial.println(wifiConnected ? "Connected" : "Disconnected");
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
  
  // Baris 2: Lux dengan status pengiriman
  lcd.setCursor(0, 1);
  lcd.print("Lux: ");
  lcd.print(lux, 1);
  lcd.print(" lx");
  if (millis() - lastSendTime < 1000) {
    lcd.setCursor(15, 1);
    lcd.print("SENT");
  }
  
  // Baris 3: Temperature 1
  lcd.setCursor(0, 2);
  lcd.print("T1: ");
  lcd.print(temp1, 1);
  lcd.print(" C");
  
  // Baris 4: Temperature 2 dengan waktu (jika tersedia)
  lcd.setCursor(0, 3);
  lcd.print("T2: ");
  lcd.print(temp2, 1);
  lcd.print(" C");
  
  // Tampilkan waktu singkat di kanan bawah jika NTP tersync
  if (ntpSynced) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      lcd.setCursor(12, 3);
      char timeStr[9];
      strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
      lcd.print(timeStr);
    }
  }
}
