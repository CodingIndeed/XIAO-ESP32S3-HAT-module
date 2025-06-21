#include <Wire.h>                          // I2C bus
#include <SPI.h>                           // SPI bus
#include <SD.h>                            // SD-card library
#include "SparkFun_BNO08x_Arduino_Library.h"   // IMU driver

/* ---------- SD-card pin mapping (SPI bus #1) --------------------- */
#define SD_CS   D0     // Chip-select
#define SD_SCK  D8     // SPI clock
#define SD_MOSI D10    // MOSI
#define SD_MISO D9     // MISO
SPIClass spiSD(1);     // use ESP32-S3's second SPI peripheral

File logfile;          // handle for /data.csv

/* ---------- BNO08x IMU pins & address ---------------------------- */
#define BNO08X_INT  D2  // INT pin (required for high-rate events)
#define BNO08X_RST  D1  // RST pin (optional but recommended)
#define BNO08X_ADDR 0x4B   // I²C address when ADR pin = HIGH

BNO08x myIMU;          // IMU object

/* ---------- globals for latest sensor values --------------------- */
float ax, ay, az;      // accelerometer (m/s^2)
float gx, gy, gz;      // gyro (rad/s)
float mx, my, mz;      // magnetometer (uT)
bool  haveAccel = false, haveGyro = false, haveMag = false;

/* ---------- logging helpers -------------------------------------- */
char     logBuffer[256];        // holds one CSV line
uint16_t lineCounter   = 0;     // lines since last flush
const uint16_t FLUSH_LINES = 100; // flush SD every 100 lines


/* ====================  SETUP  ==================================== */
void setup() {
  Serial.begin(115200);
  while (!Serial);              // wait for USB-CDC

  /* --- SD-card init (SPI bus #1) -------------------------------- */
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("SD initialization failed!");
    while (true) delay(1000);
  }

  logfile = SD.open("/data.csv", FILE_WRITE);
  if (!logfile) {
    Serial.println("Failed to open /data.csv");
    while (true) delay(1000);
  }

  logfile.println("time_ms,ax,ay,az,gx,gy,gz,mx,my,mz"); // CSV header
  logfile.flush();               // ensure header is on disk

  /* --- I²C + IMU init ------------------------------------------- */
  Wire.begin();                  // start I2C
  Wire.setClock(400000);         // raise bus to 400 kHz

  if (!myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST)) {
    Serial.println("IMU not detected. Freezing …");
    while (true);
  }

  setReports();                  // configure data rates
  Serial.println("Logging started!");
}


/* Configure BNO08x report intervals (ms) -------------------------- */
void setReports() {
  myIMU.enableAccelerometer(1);   // 1 ms → ≈1000 Hz
  myIMU.enableGyro(1);            // 1 ms → ≈1000 Hz
  myIMU.enableMagnetometer(5);    // 5 ms → 200 Hz
  delay(100);                     // allow sensor to settle
}


/* ====================  LOOP  ===================================== */
void loop() {
  /* Re-enable reports after any IMU reset ------------------------ */
  if (myIMU.wasReset()) setReports();

  /* Drain IMU FIFO completely ------------------------------------ */
  while (myIMU.getSensorEvent()) {
    uint8_t id = myIMU.getSensorEventID();

    if (id == SENSOR_REPORTID_ACCELEROMETER) {
      ax = myIMU.getAccelX(); ay = myIMU.getAccelY(); az = myIMU.getAccelZ();
      haveAccel = true;
    } else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      gx = myIMU.getGyroX(); gy = myIMU.getGyroY(); gz = myIMU.getGyroZ();
      haveGyro = true;
    } else if (id == SENSOR_REPORTID_MAGNETIC_FIELD) {
      mx = myIMU.getMagX();  my = myIMU.getMagY();  mz = myIMU.getMagZ();
      haveMag = true;
    }
  }

  /* When we have fresh data from all 3 sensors, log a line ------- */
  if (haveAccel && haveGyro && haveMag) {
    snprintf(logBuffer, sizeof(logBuffer),
             "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
             millis(), ax, ay, az, gx, gy, gz, mx, my, mz);

    logfile.println(logBuffer);   // write to SD
    Serial.println(logBuffer);    // echo to Serial

    /* flush SD periodically to limit wear ----------------------- */
    if (++lineCounter >= FLUSH_LINES) {
      logfile.flush();
      lineCounter = 0;
    }

    /* reset flags so we wait for new packets -------------------- */
    haveAccel = haveGyro = haveMag = false;
  }
}
