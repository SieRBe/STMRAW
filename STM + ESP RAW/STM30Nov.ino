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
float bms_temp1 = 0.0, bms_temp2 = 0.0;  // <-- BARU: temperature BMS dari ESP32
float bms_current = 0.0;  // <-- BARU: current BMS dari ESP32

// Variabel untuk menerima data ESP32
unsigned long espTimestamp = 0;
float espLux = 0.0, espTemp1 = 0.0, espTemp2 = 0.0;
float espTotalVoltage = 0.0, espTotalCurrent = 0.0, espSOC = 0.0;  // <-- UPDATE: tambah current
float espBMSTemp1 = 0.0, espBMSTemp2 = 0.0;  // <-- BARU: BMS temperature
float espCellV1 = 0.0, espCellV2 = 0.0, espCellV3 = 0.0, espCellV4 = 0.0;  // <-- BARU: Cell voltages
bool espDataReceived = false;
unsigned long lastESP32Time = 0;

// ===== VARIABEL GLOBAL TIMEREF =====
unsigned long globalTimeRef = 0;        // Timestamp referensi global
unsigned long globalTimeRefBaseMillis = 0; // Basis millis() saat timeref di-set
bool timeRefAvailable = false;           // Flag ketersediaan timeref
unsigned long lastTimeRefUpdate = 0;     // Terakhir kali timeref diupdate
const unsigned long timeRefUpdateInterval = 300000; // Update timeref setiap 5 menit

// ===== VARIABEL ENERGY RESET SYSTEM =====
unsigned long lastEnergyReset = 0;
const unsigned long energyResetInterval = 86400000; // 24 jam dalam milidetik
bool energyResetToday = false;
int lastResetDay = -1;  // Track hari terakhir reset

// Struktur untuk menyimpan baseline energy
struct EnergyBaseline {
    float panelEnergy;
    float batteryEnergy;  
    float acEnergy;
    float gridEnergy;     // Tambahan untuk Grid energy
    float pltsEnergy;     // Tambahan untuk PLTS energy
    unsigned long resetTime;
};
EnergyBaseline energyBaseline;

// Daily energy counters
float dailyEnergyPanel = 0.0, dailyEnergyBattery = 0.0, dailyEnergyAC = 0.0;
float dailyEnergyGrid = 0.0, dailyEnergyPLTS = 0.0;

// ===== STRUKTUR DATA UNTUK FIFO BUFFER (UPDATE) =====
struct DataRecord {
    int no;
    char waktu[10];
    unsigned long timestamp;
    float pzempv_v_pv, pzempv_i_pv, pzempv_p_pv, pzempv_e_pv;
    float pzembatt_v_batt, pzembatt_i_batt, pzembatt_p_batt, pzembatt_e_batt, pzembatt_soc;
    float bms_v1, bms_v2, bms_v3, bms_v4, bms_total_v, bms_total_i, bms_soc;  // <-- UPDATE: tambah total_i
    float bms_temp1, bms_temp2;  // <-- BARU: BMS temperatures
    float energi_p_batt, energi_e_batt, energi_soc;
    float load_plts_p, load_plts_e, load_grid_p, load_grid_e;
    float energi_lux, energi_temp1, energi_temp2;
    
    // TAMBAHAN: Daily energy counters
    float daily_energy_panel, daily_energy_battery, daily_energy_ac;
    float daily_energy_grid, daily_energy_plts;
    
    int pendingWrites;
};

// ===== DUAL-PATH: REALTIME DATA STRUCTURE =====
// Fast path: Latest sensor readings (always available for immediate transmission)
struct RealtimeData {
    unsigned long timestamp;
    
    // PZEM Panel
    float pzem_panel_v, pzem_panel_i, pzem_panel_p, pzem_panel_e;
    
    // PZEM Battery
    float pzem_batt_v, pzem_batt_i, pzem_batt_p, pzem_batt_e;
    
    // INA219
    float ina_voltage, ina_current, shunt_voltage;
    
    // BMS (from ESP32)
    float bms_v1, bms_v2, bms_v3, bms_v4;
    float bms_total_v, bms_total_i, bms_soc;
    float bms_temp1, bms_temp2;
    
    // Load data
    float grid_power, grid_energy;
    float plts_power, plts_energy;
    
    // Environmental (from ESP32)
    float lux, temp1, temp2;
    
    // Data validity flag
    bool dataValid;
};

// Global realtime data (updated immediately after sensor reading)
RealtimeData realtimeData;
unsigned long lastRealtimeUpdate = 0;

// FIFO Buffer (ukuran 20 slot)
#define FIFO_SIZE 20
DataRecord fifoBuffer[FIFO_SIZE];
int fifoHead = 0;
int fifoTail = 0;
int fifoCount = 0;

// Timer untuk ESP32
unsigned long startMillisESP;
unsigned long currentMillisESP;
const unsigned long periodESP = 60000;  // 10 detik timeout (disesuaikan dengan testing)

// State Machine untuk penulisan file sekuensial
enum WriteState {
    WRITE_IDLE = 0,
    WRITE_PZEMPV = 1,
    WRITE_PZEMBATT=2,
    WRITE_BMS = 3,
    WRITE_LOAD = 4,
    WRITE_CLEANUP = 5
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
const unsigned long periodPZEM = 60000; // 10 detik (1 cycle untuk testing)

//Timer untuk INA219
unsigned long startMillisINA;
unsigned long currentMillisINA;
const unsigned long periodINA = 60000; // 10 detik (sinkron dengan PZEM)

//Timer untuk ATS
unsigned long startMillisATS;
unsigned long currentMillisATS;
const unsigned long periodATS = 60000; // 10 detik (sinkron dengan PZEM)

//Timer untuk SD CARD (TESTING: 10 detik per cycle - 1 cycle saja)
unsigned long startMillisSOC;
unsigned long currentMillisSOC;
const unsigned long periodSOC = 60000;  // 1 menit untuk 1 cycle testing

//Timer untuk SD Card Check (setiap 30 detik)
unsigned long startMillisSDCheck;
unsigned long currentMillisSDCheck;
const unsigned long periodSDCheck = 60000;  // 30 detik

// Status SD Card
bool sdCardAvailable = true;
bool sdCardWasRemoved = false;  // Flag untuk track jika SD card pernah dicabut
bool isRecursiveCheck = false;  // Proteksi terhadap rekursi berlebihan

// Initial SOC
bool soc_initialized = false;  // Global flag

// ===== FUNGSI-FUNGSI ENERGY RESET SYSTEM =====

bool loadEnergyBaseline() {
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);
    
    File energyFile = SD.open("/energy_baseline.txt", FILE_READ);
    if (!energyFile) {
        Serial.println("❌ [ENERGY] Failed to open energy_baseline.txt for reading");
        digitalWrite(SPI1_NSS_PIN, HIGH);
        return false;
    }
    
    if (energyFile.available() >= sizeof(EnergyBaseline)) {
        energyFile.read((uint8_t*)&energyBaseline, sizeof(EnergyBaseline));
        energyFile.close();
        digitalWrite(SPI1_NSS_PIN, HIGH);
        
        Serial.println("✅ [ENERGY] Loaded energy baseline from SD:");
        Serial.print("   Panel: "); Serial.print(energyBaseline.panelEnergy); Serial.println(" Wh");
        Serial.print("   Battery: "); Serial.print(energyBaseline.batteryEnergy); Serial.println(" Wh");
        Serial.print("   AC: "); Serial.print(energyBaseline.acEnergy); Serial.println(" Wh");
        Serial.print("   Grid: "); Serial.print(energyBaseline.gridEnergy); Serial.println(" Wh");
        Serial.print("   PLTS: "); Serial.print(energyBaseline.pltsEnergy); Serial.println(" Wh");
        Serial.print("   Reset Time: "); Serial.println(energyBaseline.resetTime);
        
        return true;
    } else {
        Serial.println("❌ [ENERGY] Invalid energy_baseline.txt content");
        energyFile.close();
        digitalWrite(SPI1_NSS_PIN, HIGH);
        return false;
    }
}

bool saveEnergyBaseline() {
    if (!sdCardAvailable) {
        Serial.println("❌ [ENERGY] SD card not available, cannot save energy baseline");
        return false;
    }
    
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);
    
    File energyFile = SD.open("/energy_baseline.txt", FILE_WRITE);
    if (!energyFile) {
        Serial.println("❌ [ENERGY] Failed to open energy_baseline.txt for writing");
        digitalWrite(SPI1_NSS_PIN, HIGH);
        return false;
    }
    
    energyFile.seek(0);
    energyFile.write((uint8_t*)&energyBaseline, sizeof(EnergyBaseline));
    energyFile.flush();
    energyFile.close();
    digitalWrite(SPI1_NSS_PIN, HIGH);
    
    Serial.println("💾 [ENERGY] Saved energy baseline to SD");
    return true;
}

void initializeEnergyBaseline() {
    // Set baseline dengan nilai current energy readings
    energyBaseline.panelEnergy = PZEMEnergyPanel;
    energyBaseline.batteryEnergy = PZEMEnergyBattery;
    energyBaseline.acEnergy = (isPLTS ? PLTSEnergy : GridEnergy);
    energyBaseline.gridEnergy = GridEnergy;
    energyBaseline.pltsEnergy = PLTSEnergy;
    energyBaseline.resetTime = getCurrentTimestamp();
    
    // Reset daily counters
    dailyEnergyPanel = 0.0;
    dailyEnergyBattery = 0.0;
    dailyEnergyAC = 0.0;
    dailyEnergyGrid = 0.0;
    dailyEnergyPLTS = 0.0;
    
    lastEnergyReset = millis();
    energyResetToday = true;
    
    Serial.println("🔄 [ENERGY] Initialized energy baseline:");
    Serial.print("   Panel: "); Serial.print(energyBaseline.panelEnergy); Serial.println(" Wh");
    Serial.print("   Battery: "); Serial.print(energyBaseline.batteryEnergy); Serial.println(" Wh");
    Serial.print("   AC: "); Serial.print(energyBaseline.acEnergy); Serial.println(" Wh");
    Serial.print("   Reset Time: "); Serial.println(energyBaseline.resetTime);
    
    saveEnergyBaseline();
}

void checkDailyEnergyReset() {
    unsigned long currentTime = getCurrentTimestamp();
    
    // Convert timestamp to day number (days since epoch)
    int currentDay = currentTime / 86400;  // 86400 seconds = 1 day
    
    // Check if it's a new day
    if (lastResetDay != currentDay || (millis() - lastEnergyReset) >= energyResetInterval) {
        Serial.println("🌅 [ENERGY] Daily reset trigger activated!");
        Serial.print("   Last reset day: "); Serial.println(lastResetDay);
        Serial.print("   Current day: "); Serial.println(currentDay);
        Serial.print("   Time since last reset: "); 
        Serial.print((millis() - lastEnergyReset) / 3600000.0); Serial.println(" hours");
        
        // Perform daily reset
        initializeEnergyBaseline();
        lastResetDay = currentDay;
        
        // Log reset event to SD card
        digitalWrite(SPI1_NSS_PIN, LOW);
        File resetLog = SD.open("/energy_reset_log.txt", FILE_WRITE);
        if (resetLog) {
            resetLog.print("RESET: ");
            resetLog.print(currentTime);
            resetLog.print(" Day: ");
            resetLog.println(currentDay);
            resetLog.close();
        }
        digitalWrite(SPI1_NSS_PIN, HIGH);
        
        Serial.println("✅ [ENERGY] Daily energy reset completed!");
    }
}

void updateDailyEnergyCounters() {
    // Calculate daily energy as difference from baseline
    float currentPanelEnergy = PZEMEnergyPanel;
    float currentBatteryEnergy = PZEMEnergyBattery;
    float currentGridEnergy = GridEnergy;
    float currentPLTSEnergy = PLTSEnergy;
    
    // Handle potential energy counter overflow/reset by PZEM
    if (currentPanelEnergy >= energyBaseline.panelEnergy) {
        dailyEnergyPanel = currentPanelEnergy - energyBaseline.panelEnergy;
    } else {
        // PZEM counter reset detected
        Serial.println("⚠ [ENERGY] Panel energy counter reset detected");
        energyBaseline.panelEnergy = 0;
        dailyEnergyPanel = currentPanelEnergy;
    }
    
    if (currentBatteryEnergy >= energyBaseline.batteryEnergy) {
        dailyEnergyBattery = currentBatteryEnergy - energyBaseline.batteryEnergy;
    } else {
        Serial.println("⚠ [ENERGY] Battery energy counter reset detected");
        energyBaseline.batteryEnergy = 0;
        dailyEnergyBattery = currentBatteryEnergy;
    }
    
    if (currentGridEnergy >= energyBaseline.gridEnergy) {
        dailyEnergyGrid = currentGridEnergy - energyBaseline.gridEnergy;
    } else {
        Serial.println("⚠ [ENERGY] Grid energy counter reset detected");
        energyBaseline.gridEnergy = 0;
        dailyEnergyGrid = currentGridEnergy;
    }
    
    if (currentPLTSEnergy >= energyBaseline.pltsEnergy) {
        dailyEnergyPLTS = currentPLTSEnergy - energyBaseline.pltsEnergy;
    } else {
        Serial.println("⚠ [ENERGY] PLTS energy counter reset detected");
        energyBaseline.pltsEnergy = 0;
        dailyEnergyPLTS = currentPLTSEnergy;
    }
    
    // Update AC energy based on current mode
    dailyEnergyAC = isPLTS ? dailyEnergyPLTS : dailyEnergyGrid;
}

void debugEnergyCounters() {
    Serial.println("📊 === ENERGY COUNTERS DEBUG ===");
    Serial.print("Current readings - Panel: "); Serial.print(PZEMEnergyPanel);
    Serial.print(" Battery: "); Serial.print(PZEMEnergyBattery);
    Serial.print(" Grid: "); Serial.print(GridEnergy);
    Serial.print(" PLTS: "); Serial.println(PLTSEnergy);
    
    Serial.print("Baseline - Panel: "); Serial.print(energyBaseline.panelEnergy);
    Serial.print(" Battery: "); Serial.print(energyBaseline.batteryEnergy);
    Serial.print(" Grid: "); Serial.print(energyBaseline.gridEnergy);
    Serial.print(" PLTS: "); Serial.println(energyBaseline.pltsEnergy);
    
    Serial.print("Daily counters - Panel: "); Serial.print(dailyEnergyPanel);
    Serial.print(" Battery: "); Serial.print(dailyEnergyBattery);
    Serial.print(" Grid: "); Serial.print(dailyEnergyGrid);
    Serial.print(" PLTS: "); Serial.println(dailyEnergyPLTS);
    Serial.print(" AC (current): "); Serial.println(dailyEnergyAC);
    
    Serial.print("Reset info - Last reset: "); 
    Serial.print((millis() - lastEnergyReset) / 3600000.0); Serial.print(" hours ago");
    Serial.print(" | Last day: "); Serial.print(lastResetDay);
    Serial.print(" | Current day: "); Serial.println(getCurrentTimestamp() / 86400);
    Serial.print("Time until next reset: "); 
    Serial.print(24.0 - ((millis() - lastEnergyReset) / 3600000.0)); Serial.println(" hours");
    Serial.println("=================================");
}

void setup() {
  // Serial Monitor
  Serial.begin(9600);
  while (!Serial);
  
  // Debug: Pastikan serial terhubung
  delay(2000);
  Serial.println("\n\n🚀 ==== STM32 MONITORING SYSTEM START ====");
  Serial.println("📅 Version: STM 30 NOV 2024 with Dual-Path Architecture");
  Serial.println("⭐ Features: DUAL PZEM + BMS + INA219 + SD Logging + ESP32 Interface + AUTO TIMEREF + ENERGY RESET");
  Serial.println("🔋 Energy Reset: Daily baseline tracking for accurate energy monitoring");
  Serial.println("⚡ Dual-Path: Fast path for realtime (<1s) + Slow path for SD logging");
  Serial.println("📊 Architecture: Realtime monitoring independent from SD card operations");
  Serial.println("==================================================");
  Serial.println("STM32 System Starting...");

  // Komunikasi Antar Perangkat
  Serial2.begin(9600, SERIAL_8N2); // Untuk komunikasi PZEM
  Serial3.begin(9600);  // Untuk komunikasi dengan ESP32 (9600 baud)
  delay(500);
  Serial.println("✓ Serial2 & Serial3 initialized");
  Serial.println("ℹ Serial3: 9600 baud for ESP32 communication");

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
  ina219.setCalibration_32V_50A();
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
  
  // Inisialisasi TimeRef
  Serial.println("\nInitializing TimeRef system...");
  initializeTimeRef();
  
  // Inisialisasi Energy Baseline System
  Serial.println("\nInitializing Energy Reset System...");
  if (!loadEnergyBaseline()) {
      Serial.println("🔄 [ENERGY] No existing baseline found, will initialize after first PZEM reading");
  } else {
      // Determine current day for reset tracking
      lastResetDay = getCurrentTimestamp() / 86400;
      Serial.print("✅ [ENERGY] System ready, current day: "); Serial.println(lastResetDay);
  }
  
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
    
    // === Inisialisasi PZEMPV.csv ===
    if (!SD.exists("/PZEMPV.csv")) {
        File file = SD.open("/PZEMPV.csv", FILE_WRITE);
        if (file) {
            file.println("No;Timestamp;V PV (V);I PV (A);P PV (W);E PV (Wh)");
            file.close();
            Serial.println("✓ PZEMPV.csv created with header");
        } else {
            Serial.println("❌ Failed to create PZEMPV.csv");
        }
    } else {
        Serial.println("✓ PZEMPV.csv already exists");
    }

    // === Inisialisasi PZEMBATT.csv ===
    if (!SD.exists("/PZEMBATT.csv")) {
        File file = SD.open("/PZEMBATT.csv", FILE_WRITE);
        if (file) {
            file.println("No;Timestamp;V Batt (V);I Batt (A);P Batt (W);E Batt (Wh)");
            file.close();
            Serial.println("✓ PZEMBATT.csv created with header");
        } else {
            Serial.println("❌ Failed to create PZEMBATT.csv");
        }
    } else {
        Serial.println("✓ PZEMBATT.csv already exists");
    }
    
    // === Inisialisasi BMS.csv (UPDATE HEADER - tambah Current dan Temperature) ===
    if (!SD.exists("/BMS.csv")) {
        File file = SD.open("/BMS.csv", FILE_WRITE);
        if (file) {
            file.println("No;Timestamp;V1 (V);V2 (V);V3 (V);V4 (V);Total Voltage (V);Total Current (A);Temp1 (C);Temp2 (C);SOC (%)");
            file.close();
            Serial.println("✓ BMS.csv created with header (with Current and Temperature)");
        } else {
            Serial.println("❌ Failed to create BMS.csv");
        }
    } else {
        Serial.println("✓ BMS.csv already exists");
    }
    
    // === Inisialisasi LOAD&ENV.csv ===
    if (!SD.exists("/LOAD.csv")) {
        File file = SD.open("/LOAD.csv", FILE_WRITE);
        if (file) {
            file.println("No;Timestamp;Load Power (W);Load Energy (Wh);Lux (lx);Temp1 (C);Temp2 (C)");
            file.close();
            Serial.println("✓ load.csv created with header");
        } else {
            Serial.println("❌ Failed to create LOAD.csv");
        }
    } else {
        Serial.println("✓ load.csv already exists");
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
    Serial.println("📦 All CSV files ready!\n");
}

// ===== FUNGSI PENGECEKAN DAN REINISIALISASI SD CARD =====
bool checkAndReinitializeSD() {
    // Proteksi terhadap rekursi berlebihan
    if (isRecursiveCheck) {
        Serial.println("🛡 [SD] Recursive check prevented!");
        return false;
    }
    
    // Coba akses SD card dengan membaca direktori root
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);  // Stabilisasi SPI
    
    File root = SD.open("/");
    if (!root) {
        // SD card tidak dapat diakses
        digitalWrite(SPI1_NSS_PIN, HIGH);
        Serial.println("⚠ [SD] Card not accessible, attempting reinitialization...");
        
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
                File testAccess = SD.open("/PZEMPV.csv", FILE_WRITE);
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
                if (!SD.exists("/PZEMPV.csv") || !SD.exists("/BMS.csv") || !SD.exists("/PZEMPV.csv")) {
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
        File testFile = SD.open("/PZEMPV.csv", FILE_WRITE);
        if (!testFile) {
            Serial.println("⚠ [SD] Directory OK but file access FAILED!");
            digitalWrite(SPI1_NSS_PIN, HIGH);
            
            // Set rekursi flag dan coba reinit sekali saja
            if (!isRecursiveCheck) {
                isRecursiveCheck = true;
                Serial.println("🔄 [SD] Attempting single recovery for file access...");
                
                delay(500);  // Give SD card time to stabilize
                
                // Single attempt to reinit
                digitalWrite(SPI1_NSS_PIN, LOW);
                delay(100);
                
                if (SD.begin(SPI1_NSS_PIN)) {
                    Serial.println("✅ [SD] Recovery reinit SUCCESS!");
                    
                    // Test file access again
                    File retestFile = SD.open("/PZEMPV.csv", FILE_WRITE);
                    if (retestFile) {
                        retestFile.close();
                        Serial.println("✅ [SD] File access RECOVERED!");
                        sdCardAvailable = true;
                        digitalWrite(SPI1_NSS_PIN, HIGH);
                        isRecursiveCheck = false;
                        return true;
                    } else {
                        Serial.println("❌ [SD] File access still FAILED after recovery");
                        sdCardAvailable = false;
                        digitalWrite(SPI1_NSS_PIN, HIGH);
                        isRecursiveCheck = false;
                        return false;
                    }
                } else {
                    Serial.println("❌ [SD] Recovery reinit FAILED");
                    sdCardAvailable = false;
                    digitalWrite(SPI1_NSS_PIN, HIGH);
                    isRecursiveCheck = false;
                    return false;
                }
            } else {
                Serial.println("🛡 [SD] Recursive check blocked - marking SD as unavailable");
                sdCardAvailable = false;
                return false;
            }
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
        
        // Reset rekursi flag jika berhasil
        isRecursiveCheck = false;
        sdCardAvailable = true;
        digitalWrite(SPI1_NSS_PIN, HIGH);
        return true;
    }
}

// ===== FUNGSI DEBUG SD CARD MANUAL =====
void debugSDCard() {
    Serial.println("\n=================== SD CARD DEBUG ===================");
    
    // Test 1: Cek status SPI1
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
    Serial.print("   PZEMPV.csv: ");
    Serial.println(SD.exists("/PZEMPV.csv") ? "EXISTS" : "MISSING");
    Serial.print("   PZEMBATT.csv: ");
    Serial.println(SD.exists("/PZEMBATT.csv") ? "EXISTS" : "MISSING");
    Serial.print("   BMS.csv: ");
    Serial.println(SD.exists("/BMS.csv") ? "EXISTS" : "MISSING");
    Serial.print("   LOAD.csv: ");
    Serial.println(SD.exists("/LOAD.csv") ? "EXISTS" : "MISSING");

    // Test 4: Coba tulis file test
    Serial.println("✍  Testing write capability...");
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

// ===== FUNGSI TIMEREF FILE MANAGEMENT =====
void initializeTimeRef() {
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);
    
    // Cek apakah file timeref.txt sudah ada
    if (SD.exists("/timeref.txt")) {
        Serial.println("📅 [TIMEREF] Found existing timeref.txt, loading...");
        loadTimeRefFromSD();
    } else {
        Serial.println("📅 [TIMEREF] No timeref.txt found, will create on first timestamp");
        timeRefAvailable = false;
        globalTimeRef = 0;
        globalTimeRefBaseMillis = 0;
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
}

bool loadTimeRefFromSD() {
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);
    
    File timeRefFile = SD.open("/timeref.txt", FILE_READ);
    if (!timeRefFile) {
        Serial.println("❌ [TIMEREF] Failed to open timeref.txt for reading");
        digitalWrite(SPI1_NSS_PIN, HIGH);
        return false;
    }
    
    String line = timeRefFile.readStringUntil('\n');
    line.trim();
    timeRefFile.close();
    digitalWrite(SPI1_NSS_PIN, HIGH);
    
    if (line.length() > 0) {
        globalTimeRef = line.toInt();
        globalTimeRefBaseMillis = millis();
        timeRefAvailable = true;
        
        Serial.print("✅ [TIMEREF] Loaded timestamp: ");
        Serial.print(globalTimeRef);
        Serial.print(" at millis: ");
        Serial.println(globalTimeRefBaseMillis);
        return true;
    } else {
        Serial.println("❌ [TIMEREF] Invalid timeref.txt content");
        return false;
    }
}

bool saveTimeRefToSD(unsigned long timestamp) {
    if (!sdCardAvailable) {
        Serial.println("❌ [TIMEREF] SD card not available, cannot save timeref");
        return false;
    }
    
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);
    
    File timeRefFile = SD.open("/timeref.txt", FILE_WRITE);
    if (!timeRefFile) {
        Serial.println("❌ [TIMEREF] Failed to open timeref.txt for writing");
        digitalWrite(SPI1_NSS_PIN, HIGH);
        return false;
    }
    
    // Truncate file dan tulis timestamp baru
    timeRefFile.seek(0);
    timeRefFile.println(timestamp);
    timeRefFile.flush();
    timeRefFile.close();
    digitalWrite(SPI1_NSS_PIN, HIGH);
    
    Serial.print("💾 [TIMEREF] Saved timestamp: ");
    Serial.println(timestamp);
    return true;
}

void updateTimeRef(unsigned long newTimestamp) {
    // Update timestamp referensi global
    globalTimeRef = newTimestamp;
    globalTimeRefBaseMillis = millis();
    timeRefAvailable = true;
    lastTimeRefUpdate = millis();
    
    // Simpan ke SD card
    saveTimeRefToSD(newTimestamp);
    
    Serial.print("🕐 [TIMEREF] Updated global timeref to: ");
    Serial.print(globalTimeRef);
    Serial.print(" (base millis: ");
    Serial.print(globalTimeRefBaseMillis);
    Serial.println(")");
}

unsigned long getCurrentTimestamp() {
    if (!timeRefAvailable) {
        // FALLBACK MECHANISM - prioritas:
        // 1. ESP32 timestamp jika tersedia dan valid
        // 2. Millis/1000 + base offset
        
        if (espTimestamp > 1000000 && espDataReceived) {
            // Gunakan timestamp ESP32 jika valid dan data baru diterima
            return espTimestamp;
        } else {
            // Fallback ke millis dengan base offset 1700000000 (sekitar 2023)
            return 1700000000 + (millis() / 1000);
        }
    }
    
    // Hitung timestamp saat ini berdasarkan timeref + elapsed millis
    unsigned long currentMillis = millis();
    
    // Handle millis overflow (reset setelah ~49 hari)
    if (currentMillis < globalTimeRefBaseMillis) {
        // Millis overflow terdeteksi, reset base
        Serial.println("⚠ [TIMEREF] Millis overflow detected, recalibrating...");
        globalTimeRefBaseMillis = currentMillis;
        // Simpan ulang timeref
        saveTimeRefToSD(globalTimeRef);
    }
    
    unsigned long elapsedSeconds = (currentMillis - globalTimeRefBaseMillis) / 1000;
    unsigned long currentTimestamp = globalTimeRef + elapsedSeconds;
    
    return currentTimestamp;
}

String getFormattedTimestampFromRef() {
    if (!timeRefAvailable) {
        return "NO_TIMEREF";
    }
    
    unsigned long timestamp = getCurrentTimestamp();
    
    // Simple formatting (akan lebih baik jika ada library time)
    // Format: UNIX_TIMESTAMP untuk sekarang
    return String(timestamp);
}

void debugTimeRef() {
    Serial.println("\n=================== TIMEREF DEBUG ===================");
    Serial.print("TimeRef Available: "); Serial.println(timeRefAvailable ? "YES" : "NO");
    Serial.print("Global TimeRef: "); Serial.println(globalTimeRef);
    Serial.print("Base Millis: "); Serial.println(globalTimeRefBaseMillis);
    Serial.print("Current Millis: "); Serial.println(millis());
    Serial.print("Current Timestamp: "); Serial.println(getCurrentTimestamp());
    Serial.print("Last Update: "); Serial.print(millis() - lastTimeRefUpdate); Serial.println(" ms ago");
    
    // Test read timeref.txt
    digitalWrite(SPI1_NSS_PIN, LOW);
    if (SD.exists("/timeref.txt")) {
        File timeRefFile = SD.open("/timeref.txt", FILE_READ);
        if (timeRefFile) {
            Serial.print("TimeRef File Content: ");
            String content = timeRefFile.readStringUntil('\n');
            Serial.println(content);
            timeRefFile.close();
        }
    } else {
        Serial.println("TimeRef File: NOT EXISTS");
    }
    digitalWrite(SPI1_NSS_PIN, HIGH);
    
    Serial.println("=================== TIMEREF DEBUG END ===================\n");
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

// ===== FAST PATH: UPDATE REALTIME DATA =====
void updateRealtimeData() {
    realtimeData.timestamp = getCurrentTimestamp();
    
    // PZEM Panel
    realtimeData.pzem_panel_v = PZEMVoltagePanel;
    realtimeData.pzem_panel_i = PZEMCurrentPanel;
    realtimeData.pzem_panel_p = PZEMPowerPanel;
    realtimeData.pzem_panel_e = PZEMEnergyPanel;
    
    // PZEM Battery
    realtimeData.pzem_batt_v = PZEMVoltageBattery;
    realtimeData.pzem_batt_i = PZEMCurrentBattery;
    realtimeData.pzem_batt_p = PZEMPowerBattery;
    realtimeData.pzem_batt_e = PZEMEnergyBattery;
    
    // INA219
    realtimeData.ina_voltage = INA219Voltage;
    realtimeData.ina_current = INA219Current;
    realtimeData.shunt_voltage = ShuntVoltage;
    
    // BMS from ESP32
    realtimeData.bms_v1 = espCellV1;
    realtimeData.bms_v2 = espCellV2;
    realtimeData.bms_v3 = espCellV3;
    realtimeData.bms_v4 = espCellV4;
    realtimeData.bms_total_v = espTotalVoltage;
    realtimeData.bms_total_i = espTotalCurrent;
    realtimeData.bms_soc = espSOC;
    realtimeData.bms_temp1 = espBMSTemp1;
    realtimeData.bms_temp2 = espBMSTemp2;
    
    // Load data
    realtimeData.grid_power = GridPower;
    realtimeData.grid_energy = GridEnergy;
    realtimeData.plts_power = PLTSPower;
    realtimeData.plts_energy = PLTSEnergy;
    
    // Environmental data from ESP32
    realtimeData.lux = espLux;
    realtimeData.temp1 = espTemp1;
    realtimeData.temp2 = espTemp2;
    
    realtimeData.dataValid = true;
    lastRealtimeUpdate = millis();
    
    Serial.println("✅ [FAST PATH] Realtime data updated");
}

// ===== FAST PATH: SEND REALTIME DATA TO ESP32 (NON-BLOCKING) =====
void sendRealtimeToESP32() {
    if (!realtimeData.dataValid) {
        Serial.println("⚠ [FAST PATH] No valid realtime data to send");
        return;
    }
    
    // Format: All sensor data in one line (CSV format) - 28 fields
    String data = String(realtimeData.timestamp) + "," +
                  // PZEM Panel (4 fields)
                  String(realtimeData.pzem_panel_v, 2) + "," +
                  String(realtimeData.pzem_panel_i, 3) + "," +
                  String(realtimeData.pzem_panel_p, 1) + "," +
                  String(realtimeData.pzem_panel_e, 0) + "," +
                  // PZEM Battery (4 fields)
                  String(realtimeData.pzem_batt_v, 2) + "," +
                  String(realtimeData.pzem_batt_i, 3) + "," +
                  String(realtimeData.pzem_batt_p, 1) + "," +
                  String(realtimeData.pzem_batt_e, 0) + "," +
                  // INA219 (3 fields)
                  String(realtimeData.ina_voltage, 2) + "," +
                  String(realtimeData.ina_current, 3) + "," +
                  String(realtimeData.shunt_voltage, 2) + "," +
                  // BMS (9 fields)
                  String(realtimeData.bms_v1, 3) + "," +
                  String(realtimeData.bms_v2, 3) + "," +
                  String(realtimeData.bms_v3, 3) + "," +
                  String(realtimeData.bms_v4, 3) + "," +
                  String(realtimeData.bms_total_v, 2) + "," +
                  String(realtimeData.bms_total_i, 2) + "," +
                  String(realtimeData.bms_soc, 1) + "," +
                  String(realtimeData.bms_temp1, 1) + "," +
                  String(realtimeData.bms_temp2, 1) + "," +
                  // Load (4 fields)
                  String(realtimeData.grid_power, 1) + "," +
                  String(realtimeData.grid_energy, 0) + "," +
                  String(realtimeData.plts_power, 1) + "," +
                  String(realtimeData.plts_energy, 0) + "," +
                  // Environment (3 fields)
                  String(realtimeData.lux, 1) + "," +
                  String(realtimeData.temp1, 1) + "," +
                  String(realtimeData.temp2, 1) + "\n";
    
    Serial3.print(data);
    
    Serial.println("📤 [FAST PATH] Realtime data sent to ESP32");
    Serial.print("   Size: "); Serial.print(data.length()); Serial.println(" bytes");
    Serial.println("   Fields: 28 (timestamp,pv[4],batt[4],ina[3],bms[9],load[4],env[3])");
}

void readPZEMData() {
    readPZEMDC(nodePanel, PZEMVoltagePanel, PZEMCurrentPanel, PZEMPowerPanel, PZEMEnergyPanel);
    Serial.print("⚡PZEM-017 Panel:");
    Serial.print(PZEMVoltagePanel, 1); Serial.print("V ");
    Serial.print(PZEMCurrentPanel, 1); Serial.print("A ");
    Serial.print(PZEMPowerPanel, 0); Serial.println("W");
    Serial.print(PZEMEnergyPanel, 0); Serial.println("Wh");
    
    delay(200);  // Kurangi dari 500ms menjadi 200ms

    readPZEMDC(nodeBattery, PZEMVoltageBattery, PZEMCurrentBattery, PZEMPowerBattery, PZEMEnergyBattery);
    Serial.print("🔋 PZEM Batt: ");
    Serial.print(PZEMVoltageBattery, 1); Serial.print("V ");
    Serial.print(PZEMCurrentBattery, 1); Serial.print("A ");
    Serial.print(PZEMPowerBattery, 0); Serial.println("W");
    Serial.print(PZEMEnergyBattery, 0); Serial.println("W");
    
    delay(200);  // Kurangi dari 500ms menjadi 200ms

    readPZEMAC(nodeAC, voltageAC, currentAC, powerAC, energyAC, frequencyAC, powerFactorAC);
    if (isPLTS) {
        PLTSVoltage = voltageAC;
        PLTSCurrent = currentAC;
        PLTSPower = powerAC;
        PLTSEnergy = energyAC;
        PLTSHz = frequencyAC;
        PLTSPf = powerFactorAC;
        Serial.print("☀ PLTS: ");
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
    
    // ✅ FAST PATH: Update realtime data immediately (NON-BLOCKING)
    updateRealtimeData();
    
    // ✅ FAST PATH: Send to ESP32 immediately if ESP32 data available
    if (espDataReceived) {
        sendRealtimeToESP32();
    } else {
        Serial.println("⏳ [FAST PATH] Waiting for ESP32 data before sending");
    }
    
    // Initialize energy baseline if this is the first successful reading
    static bool energyBaselineInitialized = false;
    if (!energyBaselineInitialized && PZEMEnergyPanel > 0 && PZEMEnergyBattery > 0) {
        if (energyBaseline.resetTime == 0) {  // No baseline loaded from SD
            Serial.println("🔄 [ENERGY] First PZEM reading complete, initializing energy baseline...");
            initializeEnergyBaseline();
            energyBaselineInitialized = true;
        } else {
            energyBaselineInitialized = true;
            Serial.println("✅ [ENERGY] Using loaded baseline from SD card");
        }
    }
}

// Fungsi untuk membaca SOC Awal berdasarkan tegangan baterai
void initialSOC () {
    if (!soc_initialized && soc > 0) {  // Pastikan SOC dari ESP valid
    SOCo = soc;
    soc_initialized = true;
    Serial.println("Inisialisasi SOC Awal dari ESP:");
    Serial.println("SOCo = " + String(SOCo) + " %");
  }
}

void readINA219Data() {
    ShuntVoltage = ina219.getShuntVoltage_mV();
    INA219Voltage = ina219.getBusVoltage_V();
    INA219Current = ina219.getCurrent_mA() / 1000.0;
    Serial.println("INA219 Batt:");
    Serial.print("Shunt Voltage: "); Serial.print(ShuntVoltage); Serial.print(" mV   ");
    Serial.print("INA219 Voltage: "); Serial.print(INA219Voltage, 2); Serial.print(" V   ");
    Serial.print("INA219 Current: "); Serial.print(INA219Current, 3); Serial.println(" A");
}

// Tambahkan fungsi ini di bawah fungsi readINA219Data()
void sendDataToESP() {
    String data = String(PZEMVoltagePanel) + "," + String(PZEMCurrentPanel) + "," + String(PZEMPowerPanel) + "," + String(PZEMEnergyPanel) + "," +
                  String(PZEMVoltageBattery) + "," + String(PZEMCurrentBattery) + "," + String(PZEMPowerBattery) + "," + String(PZEMEnergyBattery) + "," +
                  String(INA219Voltage) + "," + String(INA219Current) + "," + String(ShuntVoltage) + "\n";
    Serial3.println(data); // Kirim ke ESP32
    Serial.print("Data to ESP: ");
    Serial.println(data);  // Debug di serial monitor STM
}

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
        Serial.println("⚠ [FIFO] Buffer penuh! Data ditolak.");
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
        
        // Count commas untuk deteksi format
        int commaCount = 0;
        for (int i = 0; i < receivedData.length(); i++) {
            if (receivedData.charAt(i) == ',') commaCount++;
        }
        
        // Format baru: 13 field (12 comma)
        // timestamp,lux,temp1,temp2,voltage,current,soc,bms_temp1,bms_temp2,v1,v2,v3,v4
        if (commaCount == 12) {
            int pos[13];
            pos[0] = 0;
            int commaIndex = 1;
            
            // Find all comma positions
            for (int i = 0; i < receivedData.length() && commaIndex < 13; i++) {
                if (receivedData.charAt(i) == ',') {
                    pos[commaIndex++] = i;
                }
            }
            
            if (commaIndex == 13) {
                // Parse 13 fields
                espTimestamp = receivedData.substring(pos[0], pos[1]).toInt();
                espLux = receivedData.substring(pos[1] + 1, pos[2]).toFloat();
                espTemp1 = receivedData.substring(pos[2] + 1, pos[3]).toFloat();
                espTemp2 = receivedData.substring(pos[3] + 1, pos[4]).toFloat();
                espTotalVoltage = receivedData.substring(pos[4] + 1, pos[5]).toFloat();
                espTotalCurrent = receivedData.substring(pos[5] + 1, pos[6]).toFloat();
                espSOC = receivedData.substring(pos[6] + 1, pos[7]).toFloat();
                espBMSTemp1 = receivedData.substring(pos[7] + 1, pos[8]).toFloat();
                espBMSTemp2 = receivedData.substring(pos[8] + 1, pos[9]).toFloat();
                espCellV1 = receivedData.substring(pos[9] + 1, pos[10]).toFloat();
                espCellV2 = receivedData.substring(pos[10] + 1, pos[11]).toFloat();
                espCellV3 = receivedData.substring(pos[11] + 1, pos[12]).toFloat();
                espCellV4 = receivedData.substring(pos[12] + 1).toFloat();
                
                espDataReceived = true;
                lastESP32Time = millis();
                
                // UPDATE TIMEREF JIKA TIMESTAMP VALID DAN LEBIH BARU
                if (espTimestamp > 1000000 && espTimestamp > globalTimeRef) {
                    if (!timeRefAvailable || (millis() - lastTimeRefUpdate) >= timeRefUpdateInterval) {
                        updateTimeRef(espTimestamp);
                    }
                }
                
                Serial.println("📡 [ESP32] 13-field format detected");
                Serial.print("   Timestamp: "); Serial.println(espTimestamp);
                Serial.print("   🌡 DS18B20: Lux:"); Serial.print(espLux, 1);
                Serial.print(" T1:"); Serial.print(espTemp1, 1);
                Serial.print("°C T2:"); Serial.print(espTemp2, 1); Serial.println("°C");
                Serial.print("   🔋 BMS: V:"); Serial.print(espTotalVoltage, 2);
                Serial.print("V I:"); Serial.print(espTotalCurrent, 2);
                Serial.print("A SOC:"); Serial.print(espSOC, 1); Serial.println("%");
                Serial.print("   🌡 BMS Temp: T1:"); Serial.print(espBMSTemp1, 1);
                Serial.print("°C T2:"); Serial.print(espBMSTemp2, 1); Serial.println("°C");
                Serial.print("   🔬 Cells: V1:"); Serial.print(espCellV1, 3);
                Serial.print("V V2:"); Serial.print(espCellV2, 3);
                Serial.print("V V3:"); Serial.print(espCellV3, 3);
                Serial.print("V V4:"); Serial.print(espCellV4, 3); Serial.println("V");
                
                return;  // Exit after successful parsing
            }
        }
                
        // Format lama lainnya untuk backward compatibility
        Serial.println("⚠ [ESP32] Unknown format, trying fallback parsing...");
    }
    
    // Check timeout
    currentMillisESP = millis();
    if (espDataReceived && (currentMillisESP - lastESP32Time) >= periodESP) {
        Serial.println("⏱ [ESP32] Timeout: Data tidak diperbarui");
        espDataReceived = false;
    }
}

// ===== FUNGSI MEMBUAT DATA LENGKAP UNTUK BUFFER (UPDATE) =====
void prepareCompleteData() {
    if (!espDataReceived) {
        Serial.println("⚠ [BUFFER] Menunggu data ESP32...");
        return;
    }
    
    DataRecord newData;
    newData.no = count;
    strcpy(newData.waktu, waktu);
    
    // GUNAKAN TIMESTAMP DARI TIMEREF SEBAGAI SUMBER UTAMA
    newData.timestamp = getCurrentTimestamp();
    
    // Data untuk PZEMPV.csv
    newData.pzempv_v_pv = PZEMVoltagePanel;
    newData.pzempv_i_pv = PZEMCurrentPanel;
    newData.pzempv_p_pv = PZEMPowerPanel;
    newData.pzempv_e_pv = PZEMEnergyPanel;
    
    // Data untuk PZEMBATT.csv
    newData.pzembatt_v_batt = PZEMVoltageBattery;
    newData.pzembatt_i_batt = PZEMCurrentBattery;
    newData.pzembatt_p_batt = PZEMPowerBattery;
    newData.pzembatt_e_batt = PZEMEnergyBattery;
    
    // Data untuk BMS.csv
    newData.bms_v1 = espCellV1;
    newData.bms_v2 = espCellV2;
    newData.bms_v3 = espCellV3;
    newData.bms_v4 = espCellV4;
    newData.bms_total_v = espTotalVoltage;
    newData.bms_total_i = espTotalCurrent;
    newData.bms_temp1 = espBMSTemp1;
    newData.bms_temp2 = espBMSTemp2;
    newData.bms_soc = espSOC;
    
    // Data untuk LOAD.csv
    newData.energi_p_batt = PZEMPowerBattery;
    newData.energi_e_batt = PZEMEnergyBattery;
    newData.load_plts_p = PLTSPower;
    newData.load_plts_e = PLTSEnergy;
    newData.load_grid_p = GridPower;
    newData.load_grid_e = GridEnergy;
    newData.energi_lux = espLux;
    newData.energi_temp1 = espTemp1;
    newData.energi_temp2 = espTemp2;
    
    // Update daily energy counters dan check reset
    updateDailyEnergyCounters();
    checkDailyEnergyReset();
    
    // Daily energy counters
    newData.daily_energy_panel = dailyEnergyPanel;
    newData.daily_energy_battery = dailyEnergyBattery;
    newData.daily_energy_ac = dailyEnergyAC;
    newData.daily_energy_grid = dailyEnergyGrid;
    newData.daily_energy_plts = dailyEnergyPLTS;
    
    // Bitmask penulisan: bit 0=PZEMPV, bit 1=BMS, bit 2=LOAD, bit 3=PZEMBATT
    // Total: 1 + 2 + 4 + 8 = 15 (semua file)
    newData.pendingWrites = 15;  // PERBAIKAN: ubah dari 7 ke 15
    
    addToFIFO(newData);
    espDataReceived = false;
    
    Serial.print("📝 [BUFFER] Data ditambahkan! Timestamp: ");
    Serial.print(newData.timestamp);
    Serial.print(" | PendingWrites: ");
    Serial.println(newData.pendingWrites);
    Serial.print("   PZEMPV: V:"); Serial.print(newData.pzempv_v_pv, 2);
    Serial.print("V | PZEMBATT: V:"); Serial.print(newData.pzembatt_v_batt, 2);
    Serial.println("V");
}

// ===== FUNGSI MENULIS DATA KE PZEMPV.csv DARI BUFFER =====
void writePZEMPVFromBuffer() {
    if (isFIFOEmpty()) {
        return;
    }
    
    // Cek SD card sebelum menulis
    Serial.println("🔍 [PZEMPV] Checking SD card before write...");
    bool sdReady = checkAndReinitializeSD();
    
    if (!sdReady) {
        Serial.println("❌ [PZEMPV] SD Card tidak tersedia, skip penulisan (data tetap di buffer)");
        return;
    }
    
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);
    
    // Tulis data yang memiliki flag PZEMPV (bit 0)
    for (int i = 0; i < fifoCount; i++) {
        int index = (fifoHead + i) % FIFO_SIZE;
        DataRecord& data = fifoBuffer[index];
        
        if (data.pendingWrites & 1) {  // Check bit 0
            File file;
            bool fileOpened = false;
            
            for (int openAttempt = 1; openAttempt <= 3; openAttempt++) {
                file = SD.open("/PZEMPV.csv", FILE_WRITE);
                if (file) {
                    fileOpened = true;
                    break;
                } else {
                    Serial.print("❌ [PZEMPV] File open attempt ");
                    Serial.print(openAttempt);
                    Serial.println("/3 failed");
                    delay(100);
                }
            }
            
            if (fileOpened) {
                file.print(data.no); file.print("; ");
                file.print(data.timestamp); file.print("; ");
                file.print(data.pzempv_v_pv, 2); file.print("; ");
                file.print(data.pzempv_i_pv, 3); file.print("; ");
                file.print(data.pzempv_p_pv, 1); file.print("; ");
                file.print(data.pzempv_e_pv, 0); file.println();  // <-- PERBAIKAN: tambah println()
                file.flush();
                file.close();
                
                Serial.print("✍  [PZEMPV] Data written - No: ");
                Serial.print(data.no);
                Serial.print(" | V:"); Serial.print(data.pzempv_v_pv, 2);
                Serial.print("V I:"); Serial.print(data.pzempv_i_pv, 3);
                Serial.print("A P:"); Serial.print(data.pzempv_p_pv, 1);
                Serial.println("W");
                
                data.pendingWrites &= ~1;  // Clear bit 0
            } else {
                Serial.println("❌ [PZEMPV] Failed to open file - marking SD sebagai unavailable");
                sdCardAvailable = false;
                digitalWrite(SPI1_NSS_PIN, HIGH);
                return;
            }
            break;
        }
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
}

// ===== FUNGSI MENULIS DATA KE PZEMBATT.csv DARI BUFFER =====
void writePZEMBATTFromBuffer() {
    if (isFIFOEmpty()) {
        return;
    }
    
    // Cek SD card sebelum menulis
    Serial.println("🔍 [PZEMBATT] Checking SD card before write...");
    bool sdReady = checkAndReinitializeSD();
    
    if (!sdReady) {
        Serial.println("❌ [PZEMBATT] SD Card tidak tersedia, skip penulisan (data tetap di buffer)");
        return;
    }
    
    digitalWrite(SPI1_NSS_PIN, LOW);
    delay(10);
    
    // Tulis data yang memiliki flag PZEMBATT (bit 3) - PERBAIKAN: ubah dari bit 0 ke bit 3
    for (int i = 0; i < fifoCount; i++) {
        int index = (fifoHead + i) % FIFO_SIZE;
        DataRecord& data = fifoBuffer[index];
        
        if (data.pendingWrites & 8) {  // Check bit 3 (2^3 = 8) - PERBAIKAN
            File file;
            bool fileOpened = false;
            
            for (int openAttempt = 1; openAttempt <= 3; openAttempt++) {
                file = SD.open("/PZEMBATT.csv", FILE_WRITE);
                if (file) {
                    fileOpened = true;
                    break;
                } else {
                    Serial.print("❌ [PZEMBATT] File open attempt ");
                    Serial.print(openAttempt);
                    Serial.println("/3 failed");
                    delay(100);
                }
            }
            
            if (fileOpened) {
                file.print(data.no); file.print("; ");
                file.print(data.timestamp); file.print("; ");
                file.print(data.pzembatt_v_batt, 2); file.print("; ");
                file.print(data.pzembatt_i_batt, 3); file.print("; ");
                file.print(data.pzembatt_p_batt, 1); file.print("; ");
                file.print(data.pzembatt_e_batt, 0); file.println();  // <-- PERBAIKAN: tambah println()
                file.flush();
                file.close();
                
                Serial.print("✍  [PZEMBATT] Data written - No: ");
                Serial.print(data.no);
                Serial.print(" | V:"); Serial.print(data.pzembatt_v_batt, 2);
                Serial.print("V I:"); Serial.print(data.pzembatt_i_batt, 3);
                Serial.print("A P:"); Serial.print(data.pzembatt_p_batt, 1);
                Serial.println("W");
                
                data.pendingWrites &= ~8;  // Clear bit 3 - PERBAIKAN
            } else {
                Serial.println("❌ [PZEMBATT] Failed to open file - marking SD sebagai unavailable");
                sdCardAvailable = false;
                digitalWrite(SPI1_NSS_PIN, HIGH);
                return;
            }
            break;
        }
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
}

// ===== FUNGSI MENULIS DATA KE BMS.csv DARI BUFFER (UPDATE) =====
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
                file.print(data.no); file.print("; ");
                file.print(data.timestamp); file.print("; ");
                file.print(data.bms_v1, 3); file.print("; ");      // 3 decimal untuk cell voltage
                file.print(data.bms_v2, 3); file.print("; ");
                file.print(data.bms_v3, 3); file.print("; ");
                file.print(data.bms_v4, 3); file.print("; ");
                file.print(data.bms_total_v, 2); file.print("; ");
                file.print(data.bms_total_i, 2); file.print("; ");  // <-- BARU: current
                file.print(data.bms_temp1, 1); file.print("; ");    // <-- BARU: temperature 1
                file.print(data.bms_temp2, 1); file.print("; ");    // <-- BARU: temperature 2
                file.print(data.bms_soc, 1); file.println();
                file.close();
                
                Serial.print("✍  [BMS] Data written - No: ");
                Serial.print(data.no);
              
                data.pendingWrites &= ~2;  // Clear bit 1
            } else {
                Serial.println("❌ [BMS] Failed to open file for writing");
                // Recovery logic
                Serial.println("🔄 [BMS] Attempting SD card recovery...");
                digitalWrite(SPI1_NSS_PIN, HIGH);
                delay(200);
                
                if (checkAndReinitializeSD()) {
                    Serial.println("✅ [BMS] SD card recovered, retrying write...");
                    digitalWrite(SPI1_NSS_PIN, LOW);
                    delay(50);
                    
                    File retryFile = SD.open("/BMS.csv", FILE_WRITE);
                    if (retryFile) {
                        retryFile.print(data.no); retryFile.print("; ");
                        retryFile.print(data.timestamp); retryFile.print("; ");
                        retryFile.print(data.bms_v1, 3); retryFile.print("; ");
                        retryFile.print(data.bms_v2, 3); retryFile.print("; ");
                        retryFile.print(data.bms_v3, 3); retryFile.print("; ");
                        retryFile.print(data.bms_v4, 3); retryFile.print("; ");
                        retryFile.print(data.bms_total_v, 2); retryFile.print("; ");
                        retryFile.print(data.bms_total_i, 2); retryFile.print("; ");
                        retryFile.print(data.bms_temp1, 1); retryFile.print("; ");
                        retryFile.print(data.bms_temp2, 1); retryFile.print("; ");
                        retryFile.print(data.bms_soc, 1); retryFile.println();
                        retryFile.flush();
                        retryFile.close();
                        
                        Serial.println("🎯 [BMS] RETRY SUCCESS!");
                        data.pendingWrites &= ~2;
                    } else {
                        Serial.println("❌ [BMS] RETRY FAILED");
                    }
                }
            }
            break;  // Tulis satu data sekaligus
        }
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
}

// ===== FUNGSI MENULIS DATA KE ENERGI.csv DARI BUFFER =====
void writeLOADFromBuffer() {
    if (isFIFOEmpty()) {
        return;
    }
    
    // Selalu cek SD card sebelum menulis
    if (!checkAndReinitializeSD()) {
        Serial.println("❌ [Load] SD Card tidak tersedia, skip penulisan");
        return;
    }
    
    digitalWrite(SPI1_NSS_PIN, LOW);
    
    // Tulis data yang memiliki flag Energi
    for (int i = 0; i < fifoCount; i++) {
        int index = (fifoHead + i) % FIFO_SIZE;
        DataRecord& data = fifoBuffer[index];
        
        if (data.pendingWrites & 4) {  // Check bit 2
            File file = SD.open("/LOAD.csv", FILE_WRITE);
            if (file) {
                file.print(data.no); file.print("; ");
                file.print(data.timestamp); file.print("; ");
                file.print(data.load_grid_p, 1); file.print("; ");
                file.print(data.load_grid_e, 0); file.print("; ");
                file.print(data.energi_lux, 1); file.print("; ");
                file.print(data.energi_temp1, 1); file.print("; ");
                file.print(data.energi_temp2, 1); file.println();
                file.close();
                
                Serial.print("✍  [LOAD] Data written - No: ");
                Serial.print(data.no);
                                
                data.pendingWrites &= ~4;  // Clear bit 2
            } else {
                Serial.println("❌ [LOAD] Failed to open file for writing");
                // Coba cek dan reinit SD card jika file gagal dibuka
                Serial.println("🔄 [LOAD] Attempting SD card recovery...");
                digitalWrite(SPI1_NSS_PIN, HIGH);  // Release SPI first
                delay(200);  // Give time for SPI release
                
                if (checkAndReinitializeSD()) {
                    Serial.println("✅ [LOAD] SD card recovered, retrying write immediately...");
                    
                    // Reset SPI state for retry
                    digitalWrite(SPI1_NSS_PIN, LOW);
                    delay(50);  // SPI stabilization
                    
                    // RETRY WRITE SETELAH RECOVERY
                    File retryFile = SD.open("/LOAD.csv", FILE_WRITE);
                    if (retryFile) {
                        retryFile.print(data.no); retryFile.print(", ");
                        retryFile.print(data.waktu); retryFile.print(", ");
                        retryFile.print(data.load_grid_p, 1); retryFile.print(", ");
                        retryFile.print(data.load_grid_e, 0); retryFile.print(", ");
                        retryFile.print(data.energi_lux, 1); retryFile.print(", ");
                        retryFile.print(data.energi_temp1, 1); retryFile.print(", ");
                        retryFile.print(data.energi_temp2, 1); retryFile.println();
                        retryFile.flush();
                        retryFile.close();
                        
                        Serial.println("🎯 [LOAD] RETRY SUCCESS - Data written after recovery!");
                        data.pendingWrites &= ~4;  // Clear bit 2
                    } else {
                        Serial.println("❌ [LOAD] RETRY FAILED - File still not accessible");
                        // Try one more time with extended delay
                        delay(500);
                        File finalRetry = SD.open("/LOAD.csv", FILE_WRITE);
                        if (finalRetry) {
                            finalRetry.print(data.no); finalRetry.print(", ");
                            finalRetry.print(data.waktu); finalRetry.print(", ");                            
                            finalRetry.print(data.load_grid_p, 1); finalRetry.print(", ");
                            finalRetry.print(data.load_grid_e, 0); finalRetry.print(", ");
                            finalRetry.print(data.energi_lux, 1); finalRetry.print(", ");
                            finalRetry.print(data.energi_temp1, 1); finalRetry.print(", ");
                            finalRetry.print(data.energi_temp2, 1); finalRetry.println();
                            finalRetry.flush();
                            finalRetry.close();
                            Serial.println("🎯 [LOAD] FINAL RETRY SUCCESS!");
                            data.pendingWrites &= ~4;
                        } else {
                            Serial.println("❌ [LOAD] ALL RETRIES FAILED - Will try next cycle");
                        }
                    }
                } else {
                    Serial.println("❌ [LOAD] SD card recovery failed");
                }
            }
            break;  // Tulis satu data sekaligus
        }
    }
    
    digitalWrite(SPI1_NSS_PIN, HIGH);
}

// ===== CLEANUP BUFFER =====
void cleanupCompletedData() {
    while (!isFIFOEmpty()) {
        DataRecord& data = fifoBuffer[fifoHead];
        if (data.pendingWrites == 0) {
            getFromFIFO();  // Hapus data yang sudah selesai ditulis ke 3 file
            Serial.println("🗑  [FIFO] Data dihapus setelah ditulis ke semua file");
        } else {
            break;  // Hentikan jika masih ada file yang belum ditulis
        }
    }
}

// ===== STATE MACHINE UNTUK PENULISAN FILE SEKUENSIAL =====
void handleWriteStateMachine() {
    unsigned long currentTime = millis();
    
    if (isFIFOEmpty()) {
        currentWriteState = WRITE_IDLE;
        return;
    }
    
    if (currentTime - writeStateTimer >= writeStateDelay) {
        switch (currentWriteState) {
            case WRITE_IDLE:
                if (!isFIFOEmpty()) {
                    currentWriteState = WRITE_PZEMPV;
                    writeStateTimer = currentTime;
                    Serial.println("🔄 [STATE] Mulai WRITE_PZEMPV");
                }
                break;
                
            case WRITE_PZEMPV:
                writePZEMPVFromBuffer();
                currentWriteState = WRITE_PZEMBATT;
                writeStateTimer = currentTime;
                Serial.println("🔄 [STATE] Lanjut ke WRITE_PZEMBATT");
                break;

            case WRITE_PZEMBATT:
                writePZEMBATTFromBuffer();  // PERBAIKAN: panggil fungsi yang benar
                currentWriteState = WRITE_BMS;
                writeStateTimer = currentTime;
                Serial.println("🔄 [STATE] Lanjut ke WRITE_BMS");
                break;
                
            case WRITE_BMS:
                writeBMSFromBuffer();
                currentWriteState = WRITE_LOAD;
                writeStateTimer = currentTime;
                Serial.println("🔄 [STATE] Lanjut ke WRITE_LOAD");
                break;
                
            case WRITE_LOAD:
                writeLOADFromBuffer();
                currentWriteState = WRITE_CLEANUP;
                writeStateTimer = currentTime;
                Serial.println("🔄 [STATE] Lanjut ke WRITE_CLEANUP");
                break;
                
            case WRITE_CLEANUP:
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
    const char* stateNames[] = {"IDLE", "PZEMPV","PZEMBATT", "BMS", "LOAD", "CLEANUP"};
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
        Serial.print("⚠ OVERCURRENT: "); Serial.print(PZEMCurrentBattery); Serial.println("A > 10.0A");
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
        Serial.println("⬆  Baterai Charging");
    } else if (INA219Current > 0.2) { // discharging
        if (previousStatus != statusbatt) count = 0;
        count++;
        SOCo -= kapasitas;
        statusbatt = 0;
        Serial.println("⬇  Baterai Discharging");
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
        currentWriteState = WRITE_PZEMPV;
        writeStateTimer = millis();
        Serial.println("⏱ [TRIGGER] Penulisan file dimulai");
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
                isRecursiveCheck = false;  // Reset rekursi flag
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
            const char* stateNames[] = {"IDLE", "PZEMPV","PZEMBATT", "BMS", "LOAD", "CLEANUP"};
            Serial.println(stateNames[currentWriteState]);
        } else if (command == "clearflag" || command == "clearflags") {
            Serial.println("🧹 Clearing all flags...");
            isRecursiveCheck = false;
            sdCardWasRemoved = false;
            sdCardAvailable = true;
            Serial.println("✅ All flags cleared - system reset");
        } else if (command == "status") {
            Serial.println("📊 === SYSTEM STATUS ===");
            Serial.print("SD Available: "); Serial.println(sdCardAvailable ? "YES" : "NO");
            Serial.print("SD Was Removed: "); Serial.println(sdCardWasRemoved ? "YES" : "NO");
            Serial.print("Recursive Check: "); Serial.println(isRecursiveCheck ? "ACTIVE" : "INACTIVE");
            Serial.print("FIFO Count: "); Serial.print(getFIFOCount()); Serial.print("/"); Serial.println(FIFO_SIZE);
            Serial.print("ESP32 Data: "); Serial.println(espDataReceived ? "READY" : "WAITING");
            const char* stateNames[] = {"IDLE", "PZEMPV","PZEMBATT", "BMS", "LOAD", "CLEANUP"};
            Serial.print("Write State: "); Serial.println(stateNames[currentWriteState]);
            Serial.print("TimeRef Available: "); Serial.println(timeRefAvailable ? "YES" : "NO");
            Serial.print("Current Timestamp: "); Serial.println(getCurrentTimestamp());
        } else if (command == "timeref" || command == "timeinfo") {
            debugTimeRef();
        } else if (command.startsWith("settime ")) {
            // Command: settime 1730794225
            String timestampStr = command.substring(8);
            unsigned long newTimestamp = timestampStr.toInt();
            if (newTimestamp > 1000000) {
                updateTimeRef(newTimestamp);
                Serial.print("✅ [TIMEREF] Manual timestamp set to: ");
                Serial.println(newTimestamp);
            } else {
                Serial.println("❌ [TIMEREF] Invalid timestamp (must be > 1000000)");
            }
        } else if (command == "resettimeref") {
            timeRefAvailable = false;
            globalTimeRef = 0;
            globalTimeRefBaseMillis = 0;
            // Hapus file timeref.txt
            digitalWrite(SPI1_NSS_PIN, LOW);
            if (SD.exists("/timeref.txt")) {
                SD.remove("/timeref.txt");
                Serial.println("🗑 [TIMEREF] Removed timeref.txt file");
            }
            digitalWrite(SPI1_NSS_PIN, HIGH);
            Serial.println("🔄 [TIMEREF] Reset to fallback mode");
        } else if (command == "energy" || command == "energyinfo") {
            debugEnergyCounters();
        } else if (command == "resetenergy") {
            Serial.println("🔄 [ENERGY] Manual energy reset requested...");
            initializeEnergyBaseline();
            Serial.println("✅ [ENERGY] Manual reset completed");
        } else if (command == "energybaseline" || command == "baseline") {
            Serial.println("📊 === ENERGY BASELINE INFO ===");
            Serial.print("Panel: "); Serial.print(energyBaseline.panelEnergy); Serial.println(" Wh");
            Serial.print("Battery: "); Serial.print(energyBaseline.batteryEnergy); Serial.println(" Wh");
            Serial.print("Grid: "); Serial.print(energyBaseline.gridEnergy); Serial.println(" Wh");
            Serial.print("PLTS: "); Serial.print(energyBaseline.pltsEnergy); Serial.println(" Wh");
            Serial.print("Reset Time: "); Serial.println(energyBaseline.resetTime);
            Serial.print("Reset Day: "); Serial.println(lastResetDay);
            Serial.print("Hours since reset: "); 
            Serial.println((millis() - lastEnergyReset) / 3600000.0);
        } else if (command == "realtime" || command == "rt") {
            Serial.println("📊 === REALTIME DATA STATUS ===");
            Serial.print("Data Valid: "); Serial.println(realtimeData.dataValid ? "YES" : "NO");
            Serial.print("Last Update: ");
            if (lastRealtimeUpdate > 0) {
                Serial.print((millis() - lastRealtimeUpdate) / 1000.0);
                Serial.println(" seconds ago");
            } else {
                Serial.println("NEVER");
            }
            Serial.print("Timestamp: "); Serial.println(realtimeData.timestamp);
            Serial.println("\n--- PZEM Panel ---");
            Serial.print("V: "); Serial.print(realtimeData.pzem_panel_v, 2); Serial.println(" V");
            Serial.print("I: "); Serial.print(realtimeData.pzem_panel_i, 3); Serial.println(" A");
            Serial.print("P: "); Serial.print(realtimeData.pzem_panel_p, 1); Serial.println(" W");
            Serial.print("E: "); Serial.print(realtimeData.pzem_panel_e, 0); Serial.println(" Wh");
            Serial.println("\n--- PZEM Battery ---");
            Serial.print("V: "); Serial.print(realtimeData.pzem_batt_v, 2); Serial.println(" V");
            Serial.print("I: "); Serial.print(realtimeData.pzem_batt_i, 3); Serial.println(" A");
            Serial.print("P: "); Serial.print(realtimeData.pzem_batt_p, 1); Serial.println(" W");
            Serial.print("E: "); Serial.print(realtimeData.pzem_batt_e, 0); Serial.println(" Wh");
            Serial.println("\n--- BMS ---");
            Serial.print("Total V: "); Serial.print(realtimeData.bms_total_v, 2); Serial.println(" V");
            Serial.print("Total I: "); Serial.print(realtimeData.bms_total_i, 2); Serial.println(" A");
            Serial.print("SOC: "); Serial.print(realtimeData.bms_soc, 1); Serial.println(" %");
            Serial.print("Cells: V1="); Serial.print(realtimeData.bms_v1, 3);
            Serial.print(" V2="); Serial.print(realtimeData.bms_v2, 3);
            Serial.print(" V3="); Serial.print(realtimeData.bms_v3, 3);
            Serial.print(" V4="); Serial.print(realtimeData.bms_v4, 3); Serial.println();
            Serial.println("=================================");
        } else if (command == "sendrt" || command == "sendnow") {
            Serial.println("📤 [MANUAL] Sending realtime data to ESP32...");
            if (realtimeData.dataValid) {
                sendRealtimeToESP32();
            } else {
                Serial.println("❌ [MANUAL] No valid realtime data available");
            }
        }
    }

    // ✅ PRIORITY 1: Receive ESP32 data (HIGH PRIORITY - NO BLOCKING)
    receiveESP32Data();

    // ✅ PRIORITY 2: Periodic SD check (LOW PRIORITY - BACKGROUND)
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

    // ✅ PRIORITY 3: Read PZEM sensors (FAST PATH - includes realtime send)
    if (currentMillisPZEM - startMillisPZEM >= periodPZEM) {
        startMillisPZEM += periodPZEM;
        readPZEMData();  // This automatically sends via fast path
    }

    // ✅ PRIORITY 4: Read INA219 sensor
    if (currentMillisINA - startMillisINA >= periodINA) {
        startMillisINA += periodINA;
        readINA219Data();
        // Note: Realtime data already sent via fast path in readPZEMData()
    }

    // ✅ PRIORITY 5: ATS control
    if (currentMillisATS - startMillisATS >= periodATS) {
        startMillisATS += periodATS;
        urgent();
        ATS();
    }

    // ✅ PRIORITY 6: SD Card logging (SLOW PATH - BACKGROUND, NON-BLOCKING)
    // This runs independently and doesn't block realtime data
    handleWriteStateMachine();

    // ✅ PRIORITY 7: SOC calculation and trigger SD logging
    if ((currentMillisSOC - startMillisSOC) >= periodSOC) {
        startMillisSOC += periodSOC;
        calculateSOC();
        logtoSDcard();
        printWriteState();  // Debug: tampilkan state
    }

    delay(10);  // Small delay to prevent watchdog trigger
}
