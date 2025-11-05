#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

// === Pin UART ke STM32 ===
#define STM32_RX 16  // RX2 ESP32 (menerima dari STM32 TX)
#define STM32_TX 17  // TX2 ESP32 (mengirim ke STM32 RX)

// === Pin I2C untuk sensor cahaya VEML7700 ===
#define SDA_PIN 21
#define SCL_PIN 22

// === Pin untuk dua sensor suhu DS18B20 ===
#define DS18B20_PIN_1 4
#define DS18B20_PIN_2 5

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

  lastSendTime = millis();
  lastLCDUpdate = millis();
  
  Serial.println("\n========== ESP32: READY ==========");
  Serial.println("Mengirim data ke STM32 setiap 10 detik");
  Serial.println("Format: lux,temp1,temp2\\n\n");
  
  // --- Display awal di LCD ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32 READY");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for data...");
}

void loop() {
  unsigned long currentTime = millis();

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

  // === Kirim data ke STM32 dalam format CSV (lux,temp1,temp2) ===
  String dataToSend = String(lux, 1) + "," + String(temp1, 1) + "," + String(temp2, 1);
  Serial2.println(dataToSend);

  sendCount++;

  // === Debug di Serial Monitor ESP32 ===
  Serial.println("====================================");
  Serial.print("📤 [SEND #");
  Serial.print(sendCount);
  Serial.println("]");
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
  
  // Baris 1: Header
  lcd.setCursor(0, 0);
  lcd.print("ESP32 Sensor #");
  lcd.print(sendCount);
  
  // Baris 2: Lux
  lcd.setCursor(0, 1);
  lcd.print("Lux: ");
  lcd.print(lux, 1);
  lcd.print(" lx");
  
  // Baris 3: Temperature 1
  lcd.setCursor(0, 2);
  lcd.print("T1: ");
  lcd.print(temp1, 1);
  lcd.print(" C");
  
  // Baris 4: Temperature 2
  lcd.setCursor(0, 3);
  lcd.print("T2: ");
  lcd.print(temp2, 1);
  lcd.print(" C");
  
  // Status pengiriman di kolom kanan
  if (millis() - lastSendTime < 1000) {
    lcd.setCursor(15, 1);
    lcd.print("SENT");
  }
}
