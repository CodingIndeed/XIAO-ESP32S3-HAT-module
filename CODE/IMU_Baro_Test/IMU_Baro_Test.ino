#include <Wire.h>                          // For I2C communication
#include <SPI.h>                           // For SPI communication (used for SD card)
#include <SD.h>                            // SD card library
#include <math.h>                          // For altitude calculation using pressure
#include "SparkFun_BNO08x_Arduino_Library.h"   // BNO08x IMU driver library
#include "SparkFun_BMP581_Arduino_Library.h"   // BMP581 barometer driver library

// -------- SD Card Pin Definitions (SPI Bus 1) --------
#define SD_CS     D0     // Chip Select for SD card
#define SD_SCK    D8     // SPI Clock
#define SD_MOSI   D10    // Master Out, Slave In (MOSI)
#define SD_MISO   D9     // Master In, Slave Out (MISO)
SPIClass spiSD(1);       // Define SPI bus #1 for SD card

File logfile;            // Handle for the log file on the SD card

// -------- BMP581 Barometer Setup --------
BMP581 baro;                                       // Create barometer object
const uint8_t BMP_ADDR = BMP581_I2C_ADDRESS_DEFAULT;  // Default I2C address = 0x47
const float SEA_LEVEL_Pa = 101325.0f;              // Sea-level pressure in Pascals

// -------- BNO08x IMU Setup --------
#define BNO08X_INT  D2      // IMU interrupt pin
#define BNO08X_RST  D1      // IMU reset pin
#define BNO08X_ADDR 0x4B    // IMU I2C address when ADR pin is HIGH
BNO08x myIMU;               // Create IMU object

// -------- Sensor Reading Variables --------
float ax, ay, az;           // Accelerometer data (m/s²)
float gx, gy, gz;           // Gyroscope data (rad/s)
float mx, my, mz;           // Magnetometer data (µT)
bool haveAccel = false, haveGyro = false, haveMag = false; // Flags to check if fresh data is available

// -------- Logging Variables --------
char logBuffer[256];                 // Buffer to hold one line of CSV data
uint16_t lineCounter = 0;            // Counts lines written to SD since last flush
const uint16_t FLUSH_LINES = 100;    // Flush to SD card every 100 lines
const uint32_t SAMPLE_PERIOD_MS = 100;  // Minimum delay between samples (100ms = 10Hz)
unsigned long lastSample = 0;        // Timestamp of last sample taken

// -------- Function to halt execution and print error --------
void fatal(const char *msg) {
  Serial.println(msg);               // Print the error message
  while (true) delay(1000);          // Infinite loop to halt program
}

// -------- Function to enable IMU reports --------
void setReports() {
  myIMU.enableAccelerometer(1);      // Enable accelerometer (1ms = ~1000Hz)
  myIMU.enableGyro(1);               // Enable gyroscope
  myIMU.enableMagnetometer(5);       // Enable magnetometer (5ms = ~200Hz)
  delay(100);                        // Wait for sensor to settle
}

// -------- SETUP: Runs once on power-up --------
void setup() {
  Serial.begin(115200);              // Start serial communication
  while (!Serial);                   // Wait for serial port to open (for USB-CDC)

  // --- Initialize SD card on SPI bus 1 ---
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) fatal("SD init failed!");  // Halt if SD fails
  logfile = SD.open("/data.csv", FILE_WRITE, true);       // Always overwrite file on boot
  if (!logfile) fatal("Can't open /data.csv");

  // --- Write CSV header ---
  logfile.println("time_ms,ax,ay,az,gx,gy,gz,mx,my,mz,tempC,pressurePa,altitudeM");
  logfile.flush();                   // Ensure header is written to SD
  Serial.println("time_ms,ax,ay,az,gx,gy,gz,mx,my,mz,tempC,pressurePa,altitudeM");

  // --- Initialize I2C bus ---
  Wire.begin();                      // Start I2C
  Wire.setClock(400000);             // Set I2C speed to 400kHz

  // --- Initialize IMU (BNO08x) ---
  if (!myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST)) {
    fatal("IMU not detected");       // Halt if IMU not detected
  }
  setReports();                      // Enable IMU sensors

  // --- Initialize Barometer (BMP581) ---
  while (baro.beginI2C(BMP_ADDR) != BMP5_OK) {
    Serial.println("BMP581 not found - check wiring/I2C address");
    delay(1000);                     // Retry every second until connected
  }
}

// -------- LOOP: Runs continuously after setup --------
void loop() {
  // --- Re-enable IMU reports if sensor has reset ---
  if (myIMU.wasReset()) setReports();

  // --- Read IMU events from FIFO ---
  while (myIMU.getSensorEvent()) {
    uint8_t id = myIMU.getSensorEventID();

    // Read accelerometer
    if (id == SENSOR_REPORTID_ACCELEROMETER) {
      ax = myIMU.getAccelX(); ay = myIMU.getAccelY(); az = myIMU.getAccelZ();
      haveAccel = true;
    }
    // Read gyroscope
    else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      gx = myIMU.getGyroX(); gy = myIMU.getGyroY(); gz = myIMU.getGyroZ();
      haveGyro = true;
    }
    // Read magnetometer
    else if (id == SENSOR_REPORTID_MAGNETIC_FIELD) {
      mx = myIMU.getMagX(); my = myIMU.getMagY(); mz = myIMU.getMagZ();
      haveMag = true;
    }
  }

  // --- When all IMU sensors have been read and sample time elapsed ---
  if (haveAccel && haveGyro && haveMag && (millis() - lastSample >= SAMPLE_PERIOD_MS)) {
    lastSample = millis();  // Update timestamp

    // --- Read barometer data ---
    bmp5_sensor_data d = {0, 0};  // Struct to hold pressure/temp
    if (baro.getSensorData(&d) != BMP5_OK) {
      Serial.println("baro_error");  // Print error if read fails
      return;
    }

    // --- Compute altitude using barometric formula ---
    float altM = 44330.0f * (1.0f - powf(d.pressure / SEA_LEVEL_Pa, 0.1903f));

    // --- Format all sensor values into CSV line ---
    snprintf(logBuffer, sizeof(logBuffer),
             "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
             millis(), ax, ay, az, gx, gy, gz, mx, my, mz,
             d.temperature, d.pressure, altM);

    // --- Write to SD and Serial monitor ---
    logfile.println(logBuffer);
    Serial.println(logBuffer);

    // --- Flush to SD card every FLUSH_LINES lines ---
    if (++lineCounter >= FLUSH_LINES) {
      logfile.flush();
      lineCounter = 0;
    }

    // --- Reset flags to wait for new IMU data ---
    haveAccel = haveGyro = haveMag = false;
  }
}
