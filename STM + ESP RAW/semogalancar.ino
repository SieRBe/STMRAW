#include <ModbusMaster.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// Pin MAX485
#define MAX485_DE      PA0
#define MAX485_RE      PA1

// Pin Relay
#define RELAY_Inv      PB15
#define RELAY_Batt     PB14
#define RELAY_ATS_N    PB13
#define RELAY_ATS_F    PB12

// SPI
#define SPI1_NSS_PIN PA4  // SPI_1 digunakan untuk komunikasi dengan SD Card

// Alamat slave untuk PZEM
static uint8_t pzemSlaveAddrPanel = 0x01;
static uint8_t pzemSlaveAddrBattery = 0x02;
static uint8_t pzemSlaveAddrAC = 0x03;

// Objek ModbusMaster
ModbusMaster nodePanel;
ModbusMaster nodeBattery;
ModbusMaster nodeAC;
Adafruit_INA219 ina219;
 
// Variabel untuk PZEM Panel
float PZEMVoltagePanel = 0.00, PZEMCurrentPanel = 0.00, PZEMPowerPanel = 0.00, PZEMEnergyPanel = 0.00;

// Variabel untuk PZEM Baterai
float PZEMVoltageBattery = 0.00, PZEMCurrentBattery = 0.00, PZEMPowerBattery = 0.00, PZEMEnergyBattery = 0.00;

// Variable untuk PSEM AC
float voltageAC, currentAC, powerAC, energyAC, frequencyAC, powerFactorAC;

// Variabel untuk PZEM AC PLTS
float PLTSVoltage = 0.00, PLTSCurrent = 0.00, PLTSPower = 0.00, PLTSEnergy = 5, PLTSHz = 0.00, PLTSPf = 0.00;

// Variabel untuk PZEM AC Grid
float GridVoltage = 0.00, GridCurrent = 0.00, GridPower = 0.00, GridEnergy = 4, GridHz = 0.00, GridPf = 0.00;

// Variabel untuk INA219
float ShuntVoltage = 0.00, INA219Voltage = 0.00, INA219Current = 0.00;
bool ina219Available = false;  // Flag untuk track status INA219

// Tambahan variabel BMS
float v1 = 0.0, v2 = 0.0, v3 = 0.0, v4 = 0.0;
float total_voltage = 0.0, soc = 0.0;

// Variabel untuk menerima data ESP32
float espLux = 0.0, espTemp1 = 0.0, espTemp2 = 0.0;
bool espDataReceived = false;
unsigned long lastESP32Time = 0;

// ===== STRUKTUR DATA UNTUK FIFO BUFFER =====
struct DataRecord {
    int no;
    char waktu[10];
    float soci2_v_pv, soci2_i_pv, soci2_p_pv, soci2_e_pv;
    float soci2_v_batt, soci2_i_batt, soci2_p_batt, soci2_e_batt, soci2_soc;
    float bms_v1, bms_v2, bms_v3, bms_v4, bms_total_v, bms_soc;
    float energi_p_batt, energi_e_batt, energi_soc;
    float energi_plts_p, energi_plts_e, energi_grid_p, energi_grid_e;
    float energi_lux, energi_temp1, energi_temp2;
    int pendingWrites;  // Bitmask: 1=SOCi2, 2=BMS, 4=Energi
};

// FIFO Buffer (ukuran 20 slot)
#define FIFO_SIZE 20
DataRecord fifoBuffer[FIFO_SIZE];
int fifoHead = 0;
int fifoTail = 0;
int fifoCount = 0;

// Timer untuk ESP32
unsigned long startMillisESP;
unsigned long currentMillisESP;
const unsigned long periodESP = 10000;  // 10 detik timeout (disesuaikan dengan testing)

// State Machine untuk penulisan file sekuensial
enum WriteState {
    WRITE_IDLE = 0,
    WRITE_SOCI2 = 1,
    WRITE_BMS = 2,
    WRITE_ENERGI = 3,
    WRITE_CLEANUP = 4
};
WriteState currentWriteState = WRITE_IDLE;

// Timer untuk state machine penulisan
unsigned long writeStateTimer = 0;
const unsigned long writeStateDelay = 100;  // 100ms delay antar penulisan file (dipercepat untuk testing)

// Pengolahan Data
int previousStatus = -1;  // Inisialisasi dengan nilai tidak valid
int count = 0, SOCt = 0, Ah = 100, interval = 0;
float deltaT = 0.0, SOCo = 0.0, kapasitas = 0.0;
char waktu[10];

// Status Alat Terkini
int statusbatt = 0, statuscsv = 0;

// Variable callback
const char* hari = "";
const char* bulan = "";
int tanggal, tahun, jam, menit, detik;

// Variabel global yang dibutuhkan
bool isPLTS = false;                  // true = relay ON (pakai PLTS)
bool waitToTurnOn = false;           // status delay sebelum ON
unsigned long timeToTurnOn = 0;      // waktu mulai delay

//Timer untuk Pembaharuan Data
unsigned long startMillisPZEM;
unsigned long currentMillisPZEM;
const unsigned long periodPZEM = 10000; // 10 detik (1 cycle untuk testing)

//Timer untuk INA219
unsigned long startMillisINA;
unsigned long currentMillisINA;
const unsigned long periodINA = 10000; // 10 detik (sinkron dengan PZEM)

//Timer untuk ATS
unsigned long startMillisATS;
unsigned long currentMillisATS;
const unsigned long periodATS = 10000; // 10 detik (sinkron dengan PZEM)

//Timer untuk SD CARD (TESTING: 10 detik per cycle - 1 cycle saja)
unsigned long startMillisSOC;
unsigned long currentMillisSOC;
const unsigned long periodSOC = 10000;  // 10 detik untuk 1 cycle testing

//Timer untuk SD Card Check (setiap 30 detik)
unsigned long startMillisSDCheck;
unsigned long currentMillisSDCheck;
const unsigned long periodSDCheck = 30000;  // 30 detik

// Status SD Card
bool sdCardAvailable = true;
bool sdCardWasRemoved = false;  // Flag untuk track jika SD card pernah dicabut

void setup() {
  // Serial Monitor
  Serial.begin(9600);
  while (!Serial);
  
  // Debug: Pastikan serial terhubung
  delay(2000);
  Serial.println("\n\n========== STARTUP ==========");
  Serial.println("STM32 System Starting...");

  // Komunikasi Antar Perangkat
  Serial2.begin(9600, SERIAL_8N2); // Untuk komunikasi PZEM
  Serial3.begin(9600);  // Untuk komunikasi dengan ESP32 (9600 baud)
  delay(500);
  Serial.println("✓ Serial2 & Serial3 initialized");
  Serial.println("ℹ️ Serial3: 9600 baud for ESP32 communication");

  pinMode(SPI1_NSS_PIN, OUTPUT);
  digitalWrite(SPI1_NSS_PIN, HIGH);

  // Inisialisasi SD Card
  Serial.print("Initializing SD card...");
  digitalWrite(SPI1_NSS_PIN, LOW);
  if (!SD.begin(SPI1_NSS_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  Serial.println("✓ SD Card initialized");
  digitalWrite(SPI1_NSS_PIN, HIGH);
  delay(500);

  // Konfigurasi MAX485
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
  Serial.println("✓ MAX485 configured");

  // Konfigurasi Relay
  pinMode(RELAY_ATS_F, OUTPUT);
  pinMode(RELAY_Inv, OUTPUT);
  pinMode(RELAY_Batt, OUTPUT);
  pinMode(RELAY_ATS_N, OUTPUT);

  digitalWrite(RELAY_ATS_F, HIGH);
  digitalWrite(RELAY_Inv, HIGH);
  digitalWrite(RELAY_Batt, HIGH);
  digitalWrite(RELAY_ATS_N, HIGH);
  Serial.println("✓ Relays configured");

  // Inisialisasi Modbus
  nodePanel.begin(pzemSlaveAddrPanel, Serial2);
  nodePanel.preTransmission(preTransmission);
  nodePanel.postTransmission(postTransmission);

  nodeBattery.begin(pzemSlaveAddrBattery, Serial2);
  nodeBattery.preTransmission(preTransmission);
  nodeBattery.postTransmission(postTransmission);

  nodeAC.begin(pzemSlaveAddrAC, Serial2);
  nodeAC.preTransmission(preTransmission);
  nodeAC.postTransmission(postTransmission);
  Serial.println("✓ Modbus initialized");

  // Inisialisasi I2C (Wire)
  Serial.println("Initializing Wire (I2C)...");
  Wire.begin();
  delay(500);
  Serial.println("✓ Wire (I2C) initialized");
  
  // Inisialisasi INA219
  if (! ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
    while (1) 
    {
      delay(10);
    }
  }
  ina219.setCalibration_32V_1A();
  ina219Available = true;
  Serial.println("✓ INA219 initialized");

  startMillisPZEM = millis();
  startMillisSOC = millis();
  startMillisESP = millis();
  startMillisINA = millis();
  startMillisATS = millis();
  startMillisSDCheck = millis();
  
  // Inisialisasi file CSV di SD Card
  Serial.println("\nInitializing CSV files on SD Card...");
  initializeCSVFiles();
  
  Serial.println("========== STARTUP COMPLETE ==========\n");
}

void preTransmission() {
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
  delay(1);
}

// ===== FUNGSI INISIALISASI FILE CSV =====
void initializeCSVFiles() {
    digitalWrite(SPI1_NSS_PIN, LOW);
    
    // Hapus file lama jika ada (opsional - uncomment jika ingin reset)
    // SD.remove("/SOCi2.csv");
    // SD.remove("/BMS.csv");
    // SD.remove("/Energi.csv");
    
    // === Inisialisasi SOCi2.csv ===
    if (!SD.exists("/SOCi2.csv")) {
        File file = SD.open("/SOCi2.csv", FILE_WRITE);
        if (file) {
            file.println("No,Waktu,Interval (s),V PV (V),I PV (A),P PV (W),E PV (Wh),V Batt (V),I Batt (A),P Batt (W),E Batt (Wh),SOC (%)");
            file.close();
            Serial.println("✓ SOCi2.csv created with header");
        } else {
            Serial.println("❌ Failed to create SOCi2.csv");
        }
    } else {
        Serial.println("✓ SOCi2.csv already exists");
    }
    
    // === Inisialisasi BMS.csv ===
    if (!SD.exists("/BMS.csv")) {
        File file = SD.open("/BMS.csv", FILE_WRITE);
        if (file) {
            file.println("No,Waktu,V1 (V),V2 (V),V3 (V),V4 (V),Total Voltage (V),SOC (%)");
            file.close();
            Serial.println("✓ BMS.csv created with header");
        } else {
            Serial.println("❌ Failed to create BMS.csv");
        }
    } else {
        Serial.println("✓ BMS.csv already exists");
    }
    
    // === Inisialisasi Energi.csv ===
    if (!SD.exists("/Energi.csv")) {
        File file = SD.open("/Energi.csv", FILE_WRITE);
        if (file) {
            file.println("No,Waktu,P Batt (W),E Batt (Wh),SOC (%),PLTS Power (W),PLTS Energy (Wh),Grid Power (W),Grid Energy (Wh),Lux (lx),Temp1 (C),Temp2 (C)");
            file.close();
            Serial.println("✓ Energi.csv created with header");
        } else {
            Serial.println("❌ Failed to create Energi.csv");
        }
    } else {
        Serial.println("✓ Energi.csv already exists");
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
    Serial.println("📦 All CSV files ready!\n");
}

// ===== FUNGSI PENGECEKAN DAN REINISIALISASI SD CARD =====
bool checkAndReinitializeSD() {
    // Coba akses SD card dengan membaca direktori root
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);  // Stabilisasi SPI
    
    File root = SD.open("/");
    if (!root) {
        // SD card tidak dapat diakses
        digitalWrite(SPI1_NSS_PIN, HIGH);
        Serial.println("⚠️ [SD] Card not accessible, attempting reinitialization...");
        
        // Set flag bahwa SD card pernah dicabut
        if (!sdCardWasRemoved) {
            sdCardWasRemoved = true;
            Serial.println("🚨 [SD] Card removal detected!");
        }
        
        delay(500);  // Beri waktu lebih lama untuk stabilisasi
        
        // Reset SPI dan coba reinisialisasi beberapa kali
        for (int attempt = 1; attempt <= 3; attempt++) {
            Serial.print("🔄 [SD] Reinit attempt ");
            Serial.print(attempt);
            Serial.println("/3...");
            
            digitalWrite(SPI1_NSS_PIN, LOW);
            delay(100);
            
            if (SD.begin(SPI1_NSS_PIN)) {
                Serial.println("✅ [SD] Successfully reinitialized!");
                
                // VALIDATE FILE ACCESS CAPABILITY
                Serial.println("🧪 [SD] Testing file access after reinit...");
                File testAccess = SD.open("/SOCi2.csv", FILE_WRITE);
                if (testAccess) {
                    testAccess.close();
                    Serial.println("✅ [SD] File access test SUCCESS!");
                } else {
                    Serial.println("❌ [SD] File access test FAILED - retrying...");
                    digitalWrite(SPI1_NSS_PIN, HIGH);
                    delay(500);
                    continue;  // Try next attempt
                }
                
                // Pastikan file CSV masih ada, jika tidak buat ulang
                if (!SD.exists("/SOCi2.csv") || !SD.exists("/BMS.csv") || !SD.exists("/Energi.csv")) {
                    Serial.println("📋 [SD] CSV files missing, recreating...");
                    digitalWrite(SPI1_NSS_PIN, HIGH);
                    initializeCSVFiles();
                }
                
                // Reset write state machine untuk memulai dari awal
                currentWriteState = WRITE_IDLE;
                writeStateTimer = millis();
                Serial.println("🔄 [SD] Write state machine reset to IDLE");
                
                // Update status flags
                sdCardAvailable = true;
                sdCardWasRemoved = false;
                
                digitalWrite(SPI1_NSS_PIN, HIGH);
                return true;
            } else {
                Serial.print("❌ [SD] Attempt ");
                Serial.print(attempt);
                Serial.println(" failed");
                digitalWrite(SPI1_NSS_PIN, HIGH);
                delay(1000);  // Wait before next attempt
            }
        }
        
        Serial.println("❌ [SD] All reinitialization attempts FAILED!");
        sdCardAvailable = false;
        return false;
        
    } else {
        // SD card OK, tutup file dan VALIDATE FILE ACCESS
        root.close();
        
        // Test actual file access capability
        File testFile = SD.open("/SOCi2.csv", FILE_WRITE);
        if (!testFile) {
            Serial.println("⚠️ [SD] Directory OK but file access FAILED!");
            digitalWrite(SPI1_NSS_PIN, HIGH);
            // Force reinit since directory is OK but file access fails
            return checkAndReinitializeSD();
        }
        testFile.close();
        
        // Jika sebelumnya pernah dicabut, beri notifikasi recovery
        if (sdCardWasRemoved) {
            Serial.println("🔄 [SD] Card reinserted and accessible!");
            sdCardWasRemoved = false;
            
            // Reset write state machine untuk memulai fresh
            currentWriteState = WRITE_IDLE;
            writeStateTimer = millis();
            Serial.println("🔄 [SD] Ready to resume writing operations");
        }
        
        sdCardAvailable = true;
        digitalWrite(SPI1_NSS_PIN, HIGH);
        return true;
    }
}

// ===== FUNGSI DEBUG SD CARD MANUAL =====
void debugSDCard() {
    Serial.println("\n=================== SD CARD DEBUG ===================");
    
    // Test 1: Cek status SPI
    Serial.print("🔧 SPI NSS Pin Status: ");
    Serial.println(digitalRead(SPI1_NSS_PIN) ? "HIGH" : "LOW");
    
    // Test 2: Coba akses direktori root
    Serial.println("📁 Testing root directory access...");
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(50);
    
    File root = SD.open("/");
    if (root) {
        Serial.println("✅ Root directory accessible");
        root.close();
    } else {
        Serial.println("❌ Root directory NOT accessible");
    }
    
    // Test 3: Cek keberadaan file CSV
    Serial.println("📋 Checking CSV files...");
    Serial.print("   SOCi2.csv: ");
    Serial.println(SD.exists("/SOCi2.csv") ? "EXISTS" : "MISSING");
    Serial.print("   BMS.csv: ");
    Serial.println(SD.exists("/BMS.csv") ? "EXISTS" : "MISSING");
    Serial.print("   Energi.csv: ");
    Serial.println(SD.exists("/Energi.csv") ? "EXISTS" : "MISSING");
    
    // Test 4: Coba tulis file test
    Serial.println("✍️  Testing write capability...");
    File testFile = SD.open("/test.txt", FILE_WRITE);
    if (testFile) {
        testFile.println("SD Test - " + String(millis()));
        testFile.flush();
        testFile.close();
        Serial.println("✅ Write test SUCCESS");
        SD.remove("/test.txt");  // Hapus file test
    } else {
        Serial.println("❌ Write test FAILED");
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
    
    // Test 5: Panggil fungsi check
    Serial.println("🔍 Testing checkAndReinitializeSD() function...");
    bool result = checkAndReinitializeSD();
    Serial.print("   Result: ");
    Serial.println(result ? "SUCCESS" : "FAILED");
    
    Serial.println("=================== DEBUG COMPLETE ===================\n");
}

void postTransmission() {
  delay(3);
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
}

void readPZEMDC(ModbusMaster &node, float &voltage, float &current, float &power, float &energy) {
    while (Serial2.available()) Serial2.read(); // Bersihkan buffer serial
    delay(100); // Beri waktu switching RS485
    uint8_t result = node.readInputRegisters(0x0000, 6);
    if (result == node.ku8MBSuccess) {
        uint32_t tempdouble = 0x00000000;
        voltage = node.getResponseBuffer(0x0000) / 100.0;
        current = node.getResponseBuffer(0x0001) / 100.0;
        tempdouble = (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002);
        power = tempdouble / 10.0;
        tempdouble = (node.getResponseBuffer(0x0005) << 16) + node.getResponseBuffer(0x0004);
        energy = tempdouble;
    }
}

void readPZEMAC(ModbusMaster &node, float &voltageAC, float &currentAC, float &powerAC, float &energyAC, float &frequencyAC, float &powerFactorAC) {
    while (Serial2.available()) Serial2.read(); // Bersihkan buffer serial
    delay(100); // Beri waktu switching RS485
    uint8_t result = node.readInputRegisters(0x0000, 9);
    if (result == node.ku8MBSuccess) {
        uint32_t tempdouble = 0x00000000;
        voltageAC = node.getResponseBuffer(0x0000) / 10.0;
        tempdouble = (node.getResponseBuffer(0x0002) << 16) + node.getResponseBuffer(0x0001);
        currentAC = tempdouble / 1000.00;
        tempdouble = (node.getResponseBuffer(0x0004) << 16) + node.getResponseBuffer(0x0003);
        powerAC = tempdouble / 10.0;
        tempdouble = (node.getResponseBuffer(0x0006) << 16) + node.getResponseBuffer(0x0005);
        energyAC = tempdouble;
        frequencyAC = node.getResponseBuffer(0x0007) / 10.0;
        powerFactorAC = node.getResponseBuffer(0x0008) / 100.0;
    }
}

void readPZEMData() {
    readPZEMDC(nodePanel, PZEMVoltagePanel, PZEMCurrentPanel, PZEMPowerPanel, PZEMEnergyPanel);
    Serial.print("⚡ PZEM Panel: ");
    Serial.print(PZEMVoltagePanel, 1); Serial.print("V ");
    Serial.print(PZEMCurrentPanel, 1); Serial.print("A ");
    Serial.print(PZEMPowerPanel, 0); Serial.println("W");
    
    delay(200);  // Kurangi dari 500ms menjadi 200ms

    readPZEMDC(nodeBattery, PZEMVoltageBattery, PZEMCurrentBattery, PZEMPowerBattery, PZEMEnergyBattery);
    Serial.print("🔋 PZEM Batt: ");
    Serial.print(PZEMVoltageBattery, 1); Serial.print("V ");
    Serial.print(PZEMCurrentBattery, 1); Serial.print("A ");
    Serial.print(PZEMPowerBattery, 0); Serial.println("W");
    
    delay(200);  // Kurangi dari 500ms menjadi 200ms

    readPZEMAC(nodeAC, voltageAC, currentAC, powerAC, energyAC, frequencyAC, powerFactorAC);
    if (isPLTS) {
        PLTSVoltage = voltageAC;
        PLTSCurrent = currentAC;
        PLTSPower = powerAC;
        PLTSEnergy = energyAC;
        PLTSHz = frequencyAC;
        PLTSPf = powerFactorAC;
        Serial.print("☀️ PLTS: ");
        Serial.print(PLTSPower, 0); Serial.println("W");
    } else {
        GridVoltage = voltageAC;
        GridCurrent = currentAC;
        GridPower = powerAC;
        GridEnergy = energyAC;
        GridHz = frequencyAC;
        GridPf = powerFactorAC;
        Serial.print("🌐 Grid: ");
        Serial.print(GridPower, 0); Serial.println("W");
    }
    delay(200);  // Kurangi dari 500ms menjadi 200ms
}

void readINA219Data() {
    if (!ina219Available) {
        INA219Voltage = 0;
        INA219Current = 0;
        return;
    }
    
    ShuntVoltage = ina219.getShuntVoltage_mV();
    INA219Voltage = ina219.getBusVoltage_V();
    INA219Current = ina219.getCurrent_mA() / 1000.0;
    
    Serial.print("🔌 INA219: ");
    Serial.print(INA219Voltage, 2); Serial.print("V ");
    Serial.print(INA219Current, 2); Serial.println("A");
}

// ===== FUNGSI FIFO BUFFER =====
void addToFIFO(DataRecord data) {
    if (fifoCount < FIFO_SIZE) {
        fifoBuffer[fifoTail] = data;
        fifoTail = (fifoTail + 1) % FIFO_SIZE;
        fifoCount++;
        Serial.print("📦 [FIFO] Data ditambahkan. Total: ");
        Serial.print(fifoCount);
        Serial.println("/");
        Serial.println(FIFO_SIZE);
    } else {
        Serial.println("⚠️ [FIFO] Buffer penuh! Data ditolak.");
    }
}

DataRecord getFromFIFO() {
    DataRecord data;
    if (fifoCount > 0) {
        data = fifoBuffer[fifoHead];
        fifoHead = (fifoHead + 1) % FIFO_SIZE;
        fifoCount--;
        return data;
    }
    return data;
}

bool isFIFOEmpty() {
    return fifoCount == 0;
}

int getFIFOCount() {
    return fifoCount;
}

// ===== FUNGSI MENERIMA DATA DARI ESP32 =====
void receiveESP32Data() {
    if (Serial3.available()) {
        String receivedData = Serial3.readStringUntil('\n');
        receivedData.trim();
        
        // Format: lux,temp1,temp2
        int commaIndex1 = receivedData.indexOf(',');
        int commaIndex2 = receivedData.lastIndexOf(',');
        
        if (commaIndex1 > 0 && commaIndex2 > commaIndex1) {
            espLux = receivedData.substring(0, commaIndex1).toFloat();
            espTemp1 = receivedData.substring(commaIndex1 + 1, commaIndex2).toFloat();
            espTemp2 = receivedData.substring(commaIndex2 + 1).toFloat();
            
            espDataReceived = true;
            lastESP32Time = millis();
            
            Serial.print("📡 [ESP32] Lux: ");
            Serial.print(espLux, 1);
            Serial.print(" | Temp1: ");
            Serial.print(espTemp1, 1);
            Serial.print(" | Temp2: ");
            Serial.println(espTemp2, 1);
        }
    }
    
    // Check timeout
    currentMillisESP = millis();
    if (espDataReceived && (currentMillisESP - lastESP32Time) >= periodESP) {
        Serial.println("⏱️ [ESP32] Timeout: Data tidak diperbarui");
        espDataReceived = false;
    }
}

// ===== FUNGSI MEMBUAT DATA LENGKAP UNTUK BUFFER =====
void prepareCompleteData() {
    if (!espDataReceived) {
        Serial.println("⚠️ [BUFFER] Menunggu data ESP32...");
        return;
    }
    
    DataRecord newData;
    newData.no = count;
    strcpy(newData.waktu, waktu);
    
    // Data untuk SOCi2.csv (DUMMY DATA untuk testing)
    newData.soci2_v_pv = 48.5 + (random(-10, 10) / 10.0);  // 48.5V ± random
    newData.soci2_i_pv = 5.2 + (random(-5, 5) / 10.0);     // 5.2A ± random
    newData.soci2_p_pv = 250.0 + random(-20, 20);          // 250W ± random
    newData.soci2_e_pv = 1500.0 + random(-100, 100);       // 1500Wh ± random
    newData.soci2_v_batt = 48.0 + (random(-15, 15) / 10.0); // 48V ± random
    newData.soci2_i_batt = 3.5 + (random(-8, 8) / 10.0);   // 3.5A ± random
    newData.soci2_p_batt = 170.0 + random(-20, 20);        // 170W ± random
    newData.soci2_e_batt = 1200.0 + random(-100, 100);     // 1200Wh ± random
    newData.soci2_soc = SOCo;
    
    // Data untuk BMS.csv (DUMMY DATA untuk testing)
    newData.bms_v1 = 12.0 + (random(-5, 5) / 10.0);       // 12V cell ± random
    newData.bms_v2 = 12.1 + (random(-5, 5) / 10.0);       // 12.1V cell ± random
    newData.bms_v3 = 11.95 + (random(-5, 5) / 10.0);      // 11.95V cell ± random
    newData.bms_v4 = 12.05 + (random(-5, 5) / 10.0);      // 12.05V cell ± random
    newData.bms_total_v = 48.1 + (random(-10, 10) / 10.0); // Total 48.1V ± random
    newData.bms_soc = soc;
    
    // Data untuk Energi.csv (REAL DATA dari sensor)
    newData.energi_p_batt = PZEMPowerBattery;
    newData.energi_e_batt = PZEMEnergyBattery;
    newData.energi_soc = SOCo;
    newData.energi_plts_p = PLTSPower;
    newData.energi_plts_e = PLTSEnergy;
    newData.energi_grid_p = GridPower;
    newData.energi_grid_e = GridEnergy;
    newData.energi_lux = espLux;
    newData.energi_temp1 = espTemp1;
    newData.energi_temp2 = espTemp2;
    
    // Bitmask penulisan: 1=SOCi2, 2=BMS, 4=Energi
    newData.pendingWrites = 7;  // Semua file perlu ditulis
    
    addToFIFO(newData);
    espDataReceived = false;
    
    Serial.print("📝 [DUMMY] SOCi2 V_pv: ");
    Serial.print(newData.soci2_v_pv, 1);
    Serial.print("V | BMS V1: ");
    Serial.print(newData.bms_v1, 2);
    Serial.print("V | Energi Lux: ");
    Serial.println(newData.energi_lux, 1);
}

// ===== FUNGSI MENULIS DATA KE SOCi2.csv DARI BUFFER =====
void writeSOCi2FromBuffer() {
    if (isFIFOEmpty()) {
        return;
    }
    
    // Selalu cek SD card sebelum menulis
    Serial.println("🔍 [SOCi2] Checking SD card before write...");
    if (!checkAndReinitializeSD()) {
        Serial.println("❌ [SOCi2] SD Card tidak tersedia, skip penulisan");
        return;
    }
    
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);  // Stabilisasi SPI
    
    // Tulis data yang memiliki flag SOCi2
    for (int i = 0; i < fifoCount; i++) {
        int index = (fifoHead + i) % FIFO_SIZE;
        DataRecord& data = fifoBuffer[index];
        
        if (data.pendingWrites & 1) {  // Check bit 0
            // Try multiple times to open file
            File file;
            bool fileOpened = false;
            
            for (int openAttempt = 1; openAttempt <= 3; openAttempt++) {
                file = SD.open("/SOCi2.csv", FILE_WRITE);
                if (file) {
                    fileOpened = true;
                    break;
                } else {
                    Serial.print("❌ [SOCi2] File open attempt ");
                    Serial.print(openAttempt);
                    Serial.println("/3 failed");
                    delay(100);  // Short delay between attempts
                }
            }
            
            if (fileOpened) {
                file.print(data.no); file.print(", ");
                file.print(data.waktu); file.print(", ");
                file.print(interval); file.print(", ");
                file.print(data.soci2_v_pv, 2); file.print(", ");
                file.print(data.soci2_i_pv, 3); file.print(", ");
                file.print(data.soci2_p_pv, 1); file.print(", ");
                file.print(data.soci2_e_pv, 0); file.print(", ");
                file.print(data.soci2_v_batt, 2); file.print(", ");
                file.print(data.soci2_i_batt, 3); file.print(", ");
                file.print(data.soci2_p_batt, 1); file.print(", ");
                file.print(data.soci2_e_batt, 0); file.print(", ");
                file.print(data.soci2_soc, 2); file.println();
                file.flush();  // Force write ke SD card
                file.close();
                
                Serial.print("✍️  [SOCi2] Data written - No: ");
                Serial.print(data.no);
                Serial.print(" | V_pv: ");
                Serial.print(data.soci2_v_pv, 2);
                Serial.print("V | SOC: ");
                Serial.print(data.soci2_soc, 1);
                Serial.println("%");
                
                data.pendingWrites &= ~1;  // Clear bit 0
            } else {
                Serial.println("❌ [SOCi2] Failed to open file for writing");
                // Coba cek dan reinit SD card jika file gagal dibuka
                Serial.println("🔄 [SOCi2] Attempting SD card recovery...");
                digitalWrite(SPI1_NSS_PIN, HIGH);  // Release SPI first
                delay(200);  // Give time for SPI release
                
                if (checkAndReinitializeSD()) {
                    Serial.println("✅ [SOCi2] SD card recovered, retrying write immediately...");
                    
                    // Reset SPI state for retry
                    digitalWrite(SPI1_NSS_PIN, LOW);
                    delay(50);  // SPI stabilization
                    
                    // RETRY WRITE SETELAH RECOVERY
                    File retryFile;
                    bool retryOpened = false;
                    
                    for (int retryAttempt = 1; retryAttempt <= 3; retryAttempt++) {
                        retryFile = SD.open("/SOCi2.csv", FILE_WRITE);
                        if (retryFile) {
                            retryOpened = true;
                            break;
                        } else {
                            Serial.print("❌ [SOCi2] Retry open attempt ");
                            Serial.print(retryAttempt);
                            Serial.println("/3 failed");
                            delay(200);
                        }
                    }
                    
                    if (retryOpened) {
                        retryFile.print(data.no); retryFile.print(", ");
                        retryFile.print(data.waktu); retryFile.print(", ");
                        retryFile.print(interval); retryFile.print(", ");
                        retryFile.print(data.soci2_v_pv, 2); retryFile.print(", ");
                        retryFile.print(data.soci2_i_pv, 3); retryFile.print(", ");
                        retryFile.print(data.soci2_p_pv, 1); retryFile.print(", ");
                        retryFile.print(data.soci2_e_pv, 0); retryFile.print(", ");
                        retryFile.print(data.soci2_v_batt, 2); retryFile.print(", ");
                        retryFile.print(data.soci2_i_batt, 3); retryFile.print(", ");
                        retryFile.print(data.soci2_p_batt, 1); retryFile.print(", ");
                        retryFile.print(data.soci2_e_batt, 0); retryFile.print(", ");
                        retryFile.print(data.soci2_soc, 2); retryFile.println();
                        retryFile.flush();
                        retryFile.close();
                        
                        Serial.println("🎯 [SOCi2] RETRY SUCCESS - Data written after recovery!");
                        data.pendingWrites &= ~1;  // Clear bit 0
                    } else {
                        Serial.println("❌ [SOCi2] RETRY FAILED - File still not accessible");
                        // Try one more time with extended delay
                        delay(500);
                        File finalRetry;
                        bool finalOpened = false;
                        
                        for (int finalAttempt = 1; finalAttempt <= 3; finalAttempt++) {
                            finalRetry = SD.open("/SOCi2.csv", FILE_WRITE);
                            if (finalRetry) {
                                finalOpened = true;
                                break;
                            } else {
                                Serial.print("❌ [SOCi2] Final attempt ");
                                Serial.print(finalAttempt);
                                Serial.println("/3 failed");
                                delay(300);
                            }
                        }
                        
                        if (finalOpened) {
                            finalRetry.print(data.no); finalRetry.print(", ");
                            finalRetry.print(data.waktu); finalRetry.print(", ");
                            finalRetry.print(interval); finalRetry.print(", ");
                            finalRetry.print(data.soci2_v_pv, 2); finalRetry.print(", ");
                            finalRetry.print(data.soci2_i_pv, 3); finalRetry.print(", ");
                            finalRetry.print(data.soci2_p_pv, 1); finalRetry.print(", ");
                            finalRetry.print(data.soci2_e_pv, 0); finalRetry.print(", ");
                            finalRetry.print(data.soci2_v_batt, 2); finalRetry.print(", ");
                            finalRetry.print(data.soci2_i_batt, 3); finalRetry.print(", ");
                            finalRetry.print(data.soci2_p_batt, 1); finalRetry.print(", ");
                            finalRetry.print(data.soci2_e_batt, 0); finalRetry.print(", ");
                            finalRetry.print(data.soci2_soc, 2); finalRetry.println();
                            finalRetry.flush();
                            finalRetry.close();
                            Serial.println("🎯 [SOCi2] FINAL RETRY SUCCESS!");
                            data.pendingWrites &= ~1;
                        } else {
                            Serial.println("❌ [SOCi2] ALL RETRIES FAILED - Will try next cycle");
                        }
                    }
                } else {
                    Serial.println("❌ [SOCi2] SD card recovery failed");
                }
            }
            break;  // Tulis satu data sekaligus
        }
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
}

// ===== FUNGSI MENULIS DATA KE BMS.csv DARI BUFFER =====
void writeBMSFromBuffer() {
    if (isFIFOEmpty()) {
        return;
    }
    
    // Selalu cek SD card sebelum menulis
    if (!checkAndReinitializeSD()) {
        Serial.println("❌ [BMS] SD Card tidak tersedia, skip penulisan");
        return;
    }
    
    digitalWrite(SPI1_NSS_PIN, LOW);
    
    // Tulis data yang memiliki flag BMS
    for (int i = 0; i < fifoCount; i++) {
        int index = (fifoHead + i) % FIFO_SIZE;
        DataRecord& data = fifoBuffer[index];
        
        if (data.pendingWrites & 2) {  // Check bit 1
            File file = SD.open("/BMS.csv", FILE_WRITE);
            if (file) {
                file.print(data.no); file.print(", ");
                file.print(data.waktu); file.print(", ");
                file.print(data.bms_v1, 2); file.print(", ");
                file.print(data.bms_v2, 2); file.print(", ");
                file.print(data.bms_v3, 2); file.print(", ");
                file.print(data.bms_v4, 2); file.print(", ");
                file.print(data.bms_total_v, 2); file.print(", ");
                file.print(data.bms_soc, 1); file.println();
                file.close();
                
                Serial.print("✍️  [BMS] Data written - No: ");
                Serial.print(data.no);
                Serial.print(" | V1: ");
                Serial.print(data.bms_v1, 2);
                Serial.print("V | Total: ");
                Serial.print(data.bms_total_v, 2);
                Serial.println("V");
                
                data.pendingWrites &= ~2;  // Clear bit 1
            } else {
                Serial.println("❌ [BMS] Failed to open file for writing");
                // Coba cek dan reinit SD card jika file gagal dibuka
                Serial.println("🔄 [BMS] Attempting SD card recovery...");
                digitalWrite(SPI1_NSS_PIN, HIGH);  // Release SPI first
                delay(200);  // Give time for SPI release
                
                if (checkAndReinitializeSD()) {
                    Serial.println("✅ [BMS] SD card recovered, retrying write immediately...");
                    
                    // Reset SPI state for retry
                    digitalWrite(SPI1_NSS_PIN, LOW);
                    delay(50);  // SPI stabilization
                    
                    // RETRY WRITE SETELAH RECOVERY
                    File retryFile = SD.open("/BMS.csv", FILE_WRITE);
                    if (retryFile) {
                        retryFile.print(data.no); retryFile.print(", ");
                        retryFile.print(data.waktu); retryFile.print(", ");
                        retryFile.print(data.bms_v1, 2); retryFile.print(", ");
                        retryFile.print(data.bms_v2, 2); retryFile.print(", ");
                        retryFile.print(data.bms_v3, 2); retryFile.print(", ");
                        retryFile.print(data.bms_v4, 2); retryFile.print(", ");
                        retryFile.print(data.bms_total_v, 2); retryFile.print(", ");
                        retryFile.print(data.bms_soc, 1); retryFile.println();
                        retryFile.flush();
                        retryFile.close();
                        
                        Serial.println("🎯 [BMS] RETRY SUCCESS - Data written after recovery!");
                        data.pendingWrites &= ~2;  // Clear bit 1
                    } else {
                        Serial.println("❌ [BMS] RETRY FAILED - File still not accessible");
                        // Try one more time with extended delay
                        delay(500);
                        File finalRetry = SD.open("/BMS.csv", FILE_WRITE);
                        if (finalRetry) {
                            finalRetry.print(data.no); finalRetry.print(", ");
                            finalRetry.print(data.waktu); finalRetry.print(", ");
                            finalRetry.print(data.bms_v1, 2); finalRetry.print(", ");
                            finalRetry.print(data.bms_v2, 2); finalRetry.print(", ");
                            finalRetry.print(data.bms_v3, 2); finalRetry.print(", ");
                            finalRetry.print(data.bms_v4, 2); finalRetry.print(", ");
                            finalRetry.print(data.bms_total_v, 2); finalRetry.print(", ");
                            finalRetry.print(data.bms_soc, 1); finalRetry.println();
                            finalRetry.flush();
                            finalRetry.close();
                            Serial.println("🎯 [BMS] FINAL RETRY SUCCESS!");
                            data.pendingWrites &= ~2;
                        } else {
                            Serial.println("❌ [BMS] ALL RETRIES FAILED - Will try next cycle");
                        }
                    }
                } else {
                    Serial.println("❌ [BMS] SD card recovery failed");
                }
            }
            break;  // Tulis satu data sekaligus
        }
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
}

// ===== FUNGSI MENULIS DATA KE ENERGI.csv DARI BUFFER =====
void writeEnergiFromBuffer() {
    if (isFIFOEmpty()) {
        return;
    }
    
    // Selalu cek SD card sebelum menulis
    if (!checkAndReinitializeSD()) {
        Serial.println("❌ [Energi] SD Card tidak tersedia, skip penulisan");
        return;
    }
    
    digitalWrite(SPI1_NSS_PIN, LOW);
    
    // Tulis data yang memiliki flag Energi
    for (int i = 0; i < fifoCount; i++) {
        int index = (fifoHead + i) % FIFO_SIZE;
        DataRecord& data = fifoBuffer[index];
        
        if (data.pendingWrites & 4) {  // Check bit 2
            File file = SD.open("/Energi.csv", FILE_WRITE);
            if (file) {
                file.print(data.no); file.print(", ");
                file.print(data.waktu); file.print(", ");
                file.print(data.energi_p_batt, 1); file.print(", ");
                file.print(data.energi_e_batt, 0); file.print(", ");
                file.print(data.energi_soc, 2); file.print(", ");
                file.print(data.energi_plts_p, 1); file.print(", ");
                file.print(data.energi_plts_e, 0); file.print(", ");
                file.print(data.energi_grid_p, 1); file.print(", ");
                file.print(data.energi_grid_e, 0); file.print(", ");
                file.print(data.energi_lux, 1); file.print(", ");
                file.print(data.energi_temp1, 1); file.print(", ");
                file.print(data.energi_temp2, 1); file.println();
                file.close();
                
                Serial.print("✍️  [Energi] Data written - No: ");
                Serial.print(data.no);
                Serial.print(" | P_Batt: ");
                Serial.print(data.energi_p_batt, 1);
                Serial.print("W | Lux: ");
                Serial.print(data.energi_lux, 1);
                Serial.println("lx");
                
                data.pendingWrites &= ~4;  // Clear bit 2
            } else {
                Serial.println("❌ [Energi] Failed to open file for writing");
                // Coba cek dan reinit SD card jika file gagal dibuka
                Serial.println("🔄 [Energi] Attempting SD card recovery...");
                digitalWrite(SPI1_NSS_PIN, HIGH);  // Release SPI first
                delay(200);  // Give time for SPI release
                
                if (checkAndReinitializeSD()) {
                    Serial.println("✅ [Energi] SD card recovered, retrying write immediately...");
                    
                    // Reset SPI state for retry
                    digitalWrite(SPI1_NSS_PIN, LOW);
                    delay(50);  // SPI stabilization
                    
                    // RETRY WRITE SETELAH RECOVERY
                    File retryFile = SD.open("/Energi.csv", FILE_WRITE);
                    if (retryFile) {
                        retryFile.print(data.no); retryFile.print(", ");
                        retryFile.print(data.waktu); retryFile.print(", ");
                        retryFile.print(data.energi_p_batt, 1); retryFile.print(", ");
                        retryFile.print(data.energi_e_batt, 0); retryFile.print(", ");
                        retryFile.print(data.energi_soc, 2); retryFile.print(", ");
                        retryFile.print(data.energi_plts_p, 1); retryFile.print(", ");
                        retryFile.print(data.energi_plts_e, 0); retryFile.print(", ");
                        retryFile.print(data.energi_grid_p, 1); retryFile.print(", ");
                        retryFile.print(data.energi_grid_e, 0); retryFile.print(", ");
                        retryFile.print(data.energi_lux, 1); retryFile.print(", ");
                        retryFile.print(data.energi_temp1, 1); retryFile.print(", ");
                        retryFile.print(data.energi_temp2, 1); retryFile.println();
                        retryFile.flush();
                        retryFile.close();
                        
                        Serial.println("🎯 [Energi] RETRY SUCCESS - Data written after recovery!");
                        data.pendingWrites &= ~4;  // Clear bit 2
                    } else {
                        Serial.println("❌ [Energi] RETRY FAILED - File still not accessible");
                        // Try one more time with extended delay
                        delay(500);
                        File finalRetry = SD.open("/Energi.csv", FILE_WRITE);
                        if (finalRetry) {
                            finalRetry.print(data.no); finalRetry.print(", ");
                            finalRetry.print(data.waktu); finalRetry.print(", ");
                            finalRetry.print(data.energi_p_batt, 1); finalRetry.print(", ");
                            finalRetry.print(data.energi_e_batt, 0); finalRetry.print(", ");
                            finalRetry.print(data.energi_soc, 2); finalRetry.print(", ");
                            finalRetry.print(data.energi_plts_p, 1); finalRetry.print(", ");
                            finalRetry.print(data.energi_plts_e, 0); finalRetry.print(", ");
                            finalRetry.print(data.energi_grid_p, 1); finalRetry.print(", ");
                            finalRetry.print(data.energi_grid_e, 0); finalRetry.print(", ");
                            finalRetry.print(data.energi_lux, 1); finalRetry.print(", ");
                            finalRetry.print(data.energi_temp1, 1); finalRetry.print(", ");
                            finalRetry.print(data.energi_temp2, 1); finalRetry.println();
                            finalRetry.flush();
                            finalRetry.close();
                            Serial.println("🎯 [Energi] FINAL RETRY SUCCESS!");
                            data.pendingWrites &= ~4;
                        } else {
                            Serial.println("❌ [Energi] ALL RETRIES FAILED - Will try next cycle");
                        }
                    }
                } else {
                    Serial.println("❌ [Energi] SD card recovery failed");
                }
            }
            break;  // Tulis satu data sekaligus
        }
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
}

// ===== FUNGSI CLEANUP BUFFER =====
void cleanupCompletedData() {
    while (!isFIFOEmpty()) {
        DataRecord& data = fifoBuffer[fifoHead];
        if (data.pendingWrites == 0) {
            getFromFIFO();  // Hapus data yang sudah selesai ditulis ke 3 file
            Serial.println("🗑️  [FIFO] Data dihapus setelah ditulis ke semua file");
        } else {
            break;  // Hentikan jika masih ada file yang belum ditulis
        }
    }
}

// ===== STATE MACHINE UNTUK PENULISAN FILE SEKUENSIAL =====
void handleWriteStateMachine() {
    unsigned long currentTime = millis();
    
    // Jika buffer kosong, reset ke IDLE
    if (isFIFOEmpty()) {
        currentWriteState = WRITE_IDLE;
        return;
    }
    
    // Jika sudah lewat delay, lanjut ke state berikutnya
    // DELAY DIPERCEPAT UNTUK TESTING (100ms dari 500ms)
    if (currentTime - writeStateTimer >= writeStateDelay) {
        switch (currentWriteState) {
            case WRITE_IDLE:
                // Mulai penulisan dari state SOCI2
                if (!isFIFOEmpty()) {
                    currentWriteState = WRITE_SOCI2;
                    writeStateTimer = currentTime;
                    Serial.println("🔄 [STATE] Mulai WRITE_SOCI2");
                }
                break;
                
            case WRITE_SOCI2:
                // Tulis ke SOCi2.csv
                writeSOCi2FromBuffer();
                currentWriteState = WRITE_BMS;
                writeStateTimer = currentTime;
                Serial.println("🔄 [STATE] Lanjut ke WRITE_BMS");
                break;
                
            case WRITE_BMS:
                // Tulis ke BMS.csv
                writeBMSFromBuffer();
                currentWriteState = WRITE_ENERGI;
                writeStateTimer = currentTime;
                Serial.println("🔄 [STATE] Lanjut ke WRITE_ENERGI");
                break;
                
            case WRITE_ENERGI:
                // Tulis ke Energi.csv
                writeEnergiFromBuffer();
                currentWriteState = WRITE_CLEANUP;
                writeStateTimer = currentTime;
                Serial.println("🔄 [STATE] Lanjut ke WRITE_CLEANUP");
                break;
                
            case WRITE_CLEANUP:
                // Cleanup data yang sudah ditulis
                cleanupCompletedData();
                currentWriteState = WRITE_IDLE;
                writeStateTimer = currentTime;
                Serial.println("🔄 [STATE] Kembali ke WRITE_IDLE");
                break;
        }
    }
}

// ===== FUNGSI DEBUG STATE MACHINE =====
void printWriteState() {
    const char* stateNames[] = {"IDLE", "SOCI2", "BMS", "ENERGI", "CLEANUP"};
    Serial.print("📍 [STATE] Current: ");
    Serial.print(stateNames[currentWriteState]);
    Serial.print(" | FIFO Count: ");
    Serial.print(getFIFOCount());
    Serial.print("/");
    Serial.println(FIFO_SIZE);
}

void urgent() {
    if (PZEMCurrentBattery > 10.0) {
        digitalWrite(RELAY_Batt, LOW);
        Serial.print("⚠️ OVERCURRENT: "); Serial.print(PZEMCurrentBattery); Serial.println("A > 10.0A");
    } else {
        digitalWrite(RELAY_Batt, HIGH);
    }
}

void calculateSOC() {
    deltaT = periodSOC / 3600000.0;
    kapasitas = ((PZEMCurrentBattery * deltaT) / Ah) * 100.0;

    if (INA219Current < -0.2) { // charging
        if (previousStatus != statusbatt) count = 0;
        count++;
        SOCo += kapasitas;
        statusbatt = 1;
        Serial.println("⬆️  Baterai Charging");
    } else if (INA219Current > 0.2) { // discharging
        if (previousStatus != statusbatt) count = 0;
        count++;
        SOCo -= kapasitas;
        statusbatt = 0;
        Serial.println("⬇️  Baterai Discharging");
    }
    previousStatus = statusbatt;
    interval = (count * periodSOC)/60000;
    SOCo = constrain(SOCo, 0, 100);
    SOCt = static_cast<int>(SOCo);
    Serial.print("📊 SOC: "); Serial.print(SOCo, 1); Serial.println("%");
}

void ATS() {
  unsigned long now = millis();
  bool waktuAktif = (jam >= 18 || jam < 11);  // jam 18:00 s/d 05:59

  // === 1. Jika di luar jam aktif → matikan semua relay ===
  if (!waktuAktif) {
    digitalWrite(RELAY_Inv, LOW);
    digitalWrite(RELAY_ATS_F, LOW);
    digitalWrite(RELAY_ATS_N, LOW);
    isPLTS = false;
    waitToTurnOn = false;
    Serial.println("⛔ Di luar jam aktif - Relay OFF (PLN)");
    return;
  }

  // === 2. Jika SOC < 30% → matikan relay dan reset status ===
  if (SOCt < 30) {
    digitalWrite(RELAY_Inv, LOW);
    digitalWrite(RELAY_ATS_F, LOW);
    digitalWrite(RELAY_ATS_N, LOW);
    isPLTS = false;
    waitToTurnOn = false;
    Serial.println("🔋 SOC < 30% - Relay OFF (PLN)");
    return;
  }

  // === 3. Jika SOC ≥ 90% dan belum ON → tunggu 5 detik ===
  if (SOCt >= 90 && !isPLTS) {
    if (!waitToTurnOn) {
      waitToTurnOn = true;
      timeToTurnOn = now;
      Serial.println("⏳ SOC ≥ 90% - Tunggu 5 detik sebelum Relay ON");
    }

    if (now - timeToTurnOn >= 5000) {
      digitalWrite(RELAY_Inv, HIGH);
      digitalWrite(RELAY_ATS_F, HIGH);
      digitalWrite(RELAY_ATS_N, HIGH);
      isPLTS = true;
      waitToTurnOn = false;
      Serial.println("✅ SOC ≥ 90% stabil - Relay ON (PLTS)");
    } else {
      digitalWrite(RELAY_Inv, LOW);
      digitalWrite(RELAY_ATS_F, LOW);
      digitalWrite(RELAY_ATS_N, LOW);
    }
    return;
  }

  // === 4. SOC 30–89% → pertahankan status sebelumnya ===
  digitalWrite(RELAY_Inv, isPLTS ? HIGH : LOW);
  digitalWrite(RELAY_ATS_F, isPLTS ? HIGH : LOW);
  digitalWrite(RELAY_ATS_N, isPLTS ? HIGH : LOW);

  Serial.print("📶 SOC ");
  Serial.print(SOCt);
  Serial.print("% - Relay ");
  Serial.print(isPLTS ? "NYALA ✅" : "MATI ❌");
  Serial.println(" - Sumber: " + String(isPLTS ? "PLTS" : "PLN"));
}

void logtoSDcard() {
    // Siapkan data lengkap dari semua sensor dan masukkan ke buffer
    prepareCompleteData();
    
    // Trigger state machine untuk penulisan sekuensial
    if (currentWriteState == WRITE_IDLE && !isFIFOEmpty()) {
        currentWriteState = WRITE_SOCI2;
        writeStateTimer = millis();
        Serial.println("⏱️ [TRIGGER] Penulisan file dimulai");
    }
    
    // Display buffer status
    Serial.print("📊 [FIFO Status] Count: ");
    Serial.print(getFIFOCount());
    Serial.print("/");
    Serial.println(FIFO_SIZE);
}

void loop() {
    currentMillisPZEM = millis();
    currentMillisSOC = millis();
    currentMillisINA = millis();
    currentMillisATS = millis();
    currentMillisSDCheck = millis();

    // Command dari Serial Monitor untuk debug SD card
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();
        
        if (command == "sd" || command == "sdtest") {
            debugSDCard();
        } else if (command == "sdcheck") {
            Serial.println("🔍 Manual SD Check...");
            bool result = checkAndReinitializeSD();
            Serial.println(result ? "✅ SD OK" : "❌ SD FAILED");
        } else if (command == "sdinit") {
            Serial.println("🔄 Manual SD Reinitialization...");
            digitalWrite(SPI1_NSS_PIN, LOW);
            if (SD.begin(SPI1_NSS_PIN)) {
                Serial.println("✅ SD Reinit SUCCESS");
                initializeCSVFiles();
                sdCardAvailable = true;
                sdCardWasRemoved = false;
            } else {
                Serial.println("❌ SD Reinit FAILED");
                sdCardAvailable = false;
            }
            digitalWrite(SPI1_NSS_PIN, HIGH);
        } else if (command == "reset" || command == "statereset") {
            Serial.println("🔄 Force Reset State Machine...");
            currentWriteState = WRITE_IDLE;
            writeStateTimer = millis();
            Serial.println("✅ State machine reset to IDLE");
        } else if (command == "fifo" || command == "buffer") {
            Serial.print("📊 [FIFO] Current count: ");
            Serial.print(getFIFOCount());
            Serial.print("/");
            Serial.println(FIFO_SIZE);
            Serial.print("📍 [STATE] Current: ");
            const char* stateNames[] = {"IDLE", "SOCI2", "BMS", "ENERGI", "CLEANUP"};
            Serial.println(stateNames[currentWriteState]);
        }
    }

    // Terima data dari ESP32 secara kontinyu (NO TIMER)
    receiveESP32Data();

    // Pengecekan SD Card berkala setiap 30 detik
    if (currentMillisSDCheck - startMillisSDCheck >= periodSDCheck) {
        startMillisSDCheck += periodSDCheck;
        Serial.println("🔍 [SD] Periodic check...");
        bool sdResult = checkAndReinitializeSD();
        if (sdResult) {
            Serial.println("✅ [SD] Periodic check: OK");
        } else {
            Serial.println("❌ [SD] Periodic check: FAILED");
        }
    }

    // Baca PZEM setiap 10 detik
    if (currentMillisPZEM - startMillisPZEM >= periodPZEM) {
        startMillisPZEM += periodPZEM;
        readPZEMData();
    }

    // Baca INA219 setiap 10 detik
    if (currentMillisINA - startMillisINA >= periodINA) {
        startMillisINA += periodINA;
        readINA219Data();
    }

    // Cek ATS setiap 10 detik
    if (currentMillisATS - startMillisATS >= periodATS) {
        startMillisATS += periodATS;
        urgent();
        ATS();
    }

    // Handle penulisan file secara state machine (jangan blocking)
    handleWriteStateMachine();

    // Logging SOC dan trigger penulisan file setiap 10 detik
    if ((currentMillisSOC - startMillisSOC) >= periodSOC) {
        startMillisSOC += periodSOC;
        calculateSOC();
        logtoSDcard();
        printWriteState();  // Debug: tampilkan state
    }

    delay(10);  // Small delay to prevent watchdog trigger
}
