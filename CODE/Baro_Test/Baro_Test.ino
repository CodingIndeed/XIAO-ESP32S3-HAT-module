#include <Wire.h>                        // For I2C communication
#include <SPI.h>                         // For SPI communication (used for SD card)
#include <SD.h>                          // SD card library
#include <math.h>                        // Math functions for altitude calculation
#include "SparkFun_BMP581_Arduino_Library.h"  // SparkFun BMP581 sensor library

// -------- SD Card Pin Definitions (SPI Bus 1) --------
#define SD_CS     D0     // Chip Select for SD card
#define SD_SCK    D8     // SPI Clock
#define SD_MOSI   D10    // Master Out, Slave In (MOSI)
#define SD_MISO   D9     // Master In, Slave Out (MISO)
SPIClass spiSD(1);       // Define SPI bus #1 for SD card

File logfile;            // Handle for log file on SD card

// -------- BMP581 Barometer Setup --------
BMP581 baro;                                // Create BMP581 sensor object
const uint8_t BMP_ADDR = BMP581_I2C_ADDRESS_DEFAULT;  // Default I2C address (0x47)
const float SEA_LEVEL_Pa = 101325.0f;       // Sea-level pressure in Pascals (used for altitude calc)

// -------- Sampling and Logging Settings --------
const uint32_t SAMPLE_PERIOD_MS = 100;      // Sample every 100 ms = 10 Hz
const uint16_t FLUSH_LINES       = 100;     // Flush SD buffer every 100 lines to reduce wear

// -------- Internal Variables --------
char logBuffer[96];             // Buffer for a single CSV line
uint16_t lineCounter = 0;       // Counts how many lines have been written since last flush
unsigned long lastSample = 0;   // Timestamp of last sample taken

// -------- Error Handler --------
void fatal(const char *msg)
{
  Serial.println(msg);          // Print error message
  while (true) delay(1000);     // Halt the program
}


// -------- SETUP: runs once --------
void setup()
{
  Serial.begin(115200);         // Start Serial communication at 115200 baud
  while (!Serial);              // Wait for USB Serial connection

  // --- Initialize SD Card on SPI bus 1 ---
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD))
    fatal("SD init failed!");   // Halt if SD card cannot be initialized

  // --- Open or Create log file on SD card ---
  logfile = SD.open("/barolog.csv", FILE_WRITE);
  if (!logfile) fatal("Can't open /barolog.csv");

  // --- Write CSV header if the file is new (empty) ---
  if (logfile.size() == 0) {
    logfile.println("time_ms,tempC,pressurePa,altitudeM");
    logfile.flush();            // Save header to disk
  }

  // Also print CSV header to Serial
  Serial.println("time_ms,tempC,pressurePa,altitudeM");

  // --- Initialize I2C and BMP581 Sensor ---
  Wire.begin();                 // Start I2C communication
  Wire.setClock(400000);        // Increase I2C speed to 400kHz (safe for most sensors)

  // Try to connect to the BMP581 sensor
  while (baro.beginI2C(BMP_ADDR) != BMP5_OK) {
    Serial.println("BMP581 not found - check wiring/I2C address");
    delay(1000);                // Retry every second until sensor is found
  }
}


// -------- LOOP: runs continuously --------
void loop()
{
  // --- Wait for next sample period ---
  if (millis() - lastSample < SAMPLE_PERIOD_MS) return;
  lastSample = millis();  // Update timestamp for next sample

  // --- Get temperature and pressure data from BMP581 ---
  bmp5_sensor_data d = {0, 0};  // Struct to hold sensor data
  if (baro.getSensorData(&d) != BMP5_OK) {
    Serial.println("sensor_error");  // Handle sensor read error
    return;
  }

  // --- Calculate altitude using barometric formula ---
  float altM = 44330.0f * (1.0f - powf(d.pressure / SEA_LEVEL_Pa, 0.1903f));

  // --- Format data as CSV row and store in buffer ---
  snprintf(logBuffer, sizeof(logBuffer), "%lu,%.2f,%.2f,%.2f",
           millis(), d.temperature, d.pressure, altM);

  // --- Write to Serial Monitor and SD file ---
  logfile.println(logBuffer);
  Serial.println(logBuffer);

  // --- Periodically flush data to SD card ---
  if (++lineCounter >= FLUSH_LINES) {
    logfile.flush();            // Write buffered data to SD card
    lineCounter = 0;            // Reset flush counter
  }
}
