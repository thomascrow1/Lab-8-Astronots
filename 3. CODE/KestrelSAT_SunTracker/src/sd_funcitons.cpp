
#include "definitions.h"     // Project definitions (this directory)
#include "sd_functions.h"

// File-scope objects (NOT in the header)
static SdFat sd;

/*---------------------------------------------------------------------------------------------*/
// Function Definitions:
/*---------------------------------------------------------------------------------------------*/

/**
 * @brief Initializes the SD card with the specified chip select pin.
 * 
 * @param csPin The chip select pin for the SD card.
 * @return true if initialization was successful.
 */
bool sd_init(uint8_t csPin)
{
    pinMode(SD_CS_PIN, OUTPUT);
    if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(25))) {
        while (1) { Serial.println("[ERROR] SD card initialization failed. Card present?"); delay(2000); }
    } else {Serial.println("[INFO] SD Card Initialized.");}
    return true;
}


/**
 * @brief Creates a new data file on the SD card with a custom filename preamble.
 *
 * Example output: "lab99_data001.csv"
 *
 * @param dataFile     Pointer to a File object that will be opened.
 * @param preamble     String to replace the "lab" portion of the filename.
 * @return true        If the file was successfully created and opened.
 * @return false       If file creation/opening failed.
 */
bool sd_createDataFile(FsFile *dataFile, const char *preamble) {

  char filename[40]; 
  int fileNumber = 1;

  // Safety: handle null pointer
  if (preamble == nullptr || strlen(preamble) == 0) {
    preamble = "lab";
  }

  // Optional: enforce a max preamble length for FAT compatibility
  const size_t MAX_PREAMBLE_LEN = 12;

  char safePreamble[MAX_PREAMBLE_LEN + 1];
  strncpy(safePreamble, preamble, MAX_PREAMBLE_LEN);
  safePreamble[MAX_PREAMBLE_LEN] = '\0';

  // Find next available file number
  do {
    snprintf(filename, sizeof(filename),
             "%s_data%03d.csv",
             safePreamble,
             fileNumber);
    fileNumber++;

  } while (sd.exists(filename) && fileNumber <= 999);

  if (fileNumber > 999) {
    Serial.println("[ERROR] Maximum file number exceeded (999).");
    return false;
  }

  Serial.print("[INFO] Creating file: ");
  Serial.println(filename);

  *dataFile = sd.open(filename, FILE_WRITE);

  if (!*dataFile) {
    Serial.println("[ERROR] could not create file.");
    return false;
  }

  return true;
}
