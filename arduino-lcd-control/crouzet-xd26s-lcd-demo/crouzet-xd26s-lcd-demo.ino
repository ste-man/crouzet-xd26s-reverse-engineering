/*
 * Sketch for Arduino UNO (ATmega328P) connected to LCD from Crouzet XD26S.
 *
 * Display complete LCD character map (16*16 characters) and segment map
 * (7 visible positions).
 *
 * Arduino UNO is SPI communication master.
 * LCD controller is SPI slave/periphial.
 *
 * Connection list/pinout:
 * See related electrical schematic for reference
 *
 * Arduino UNO          - Crouzet XD26S LCD connector J2
 * MOSI (COPI),  PIN 11 - PIN 12
 * MISO (CIPO)), PIN 12 - PIN 3 (not required! no output from LCD)
 * SCK:          PIN 13 - PIN 14
 * SS (CS):      PIN 10 - PIN 5
 * +5VDC:               - PIN 1
 * GND:                 - PIN 2 or PIN 13
 */

// Include SPI library
#include<SPI.h>

// Setup SPI slave select ("chip select") pin.
// This pin can be changed to any availble I/O pin.
const int PIN_SLAVESELECT = 10;

// Enable/disable debugging messages over Serial communication
const bool debug = false;

void setup() {
  pinMode(PIN_SLAVESELECT, OUTPUT);

  // Crouzet XD26S ATmega168 MCU speed is 7.37 MHz (external oscillator)
  // Max SPI speed set to 6MHZ
  // SPI MODE info: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Clock_polarity_and_phase
  SPI.beginTransaction(SPISettings(6000000, LSBFIRST, SPI_MODE3));

  // Initialize SPI:
  SPI.begin();

  // Initialize Serial communication
  if (debug) {
    Serial.begin(500000);
    Serial.println("Serial communication started!");
  }
}

void loop() {
  // Take it easy after startup!
  delay(1000); // Crouzet XD26s MCU sends first SPI packet ~500ms after bootup.

  // Initialize LCD
  initLcd();

  // Print full character map on LCD
  printFullCharacterMap();

  // Clear LCD
  clearLcdChar();

  // Print full segment map on LCD
  printFullSegmentMap();

  delay(5000);
}

// Send one SPI data packet (1 packet = 3 bytes)
void sendPacket(int byte1, int byte2, int byte3) {
  digitalWrite(PIN_SLAVESELECT, LOW); // enable SPI slave device LCD controller
  delayMicroseconds(2); // Crouzet XD26S measured delay: ~2uS
  SPI.transfer(byte1);
  delayMicroseconds(8); // Crouzet XD26S measured delay: ~8uS
  SPI.transfer(byte2);
  delayMicroseconds(8); // Crouzet XD26S measured delay: ~8uS
  SPI.transfer(byte3);
  delayMicroseconds(62);  // Crouzet XD26S measured delay: 60.4uS
  digitalWrite(PIN_SLAVESELECT, HIGH);  // disable SPI slave device (LCD controller)
  delayMicroseconds(2); // Crouzet XD26S measured delay: 2uS between SPI packets

  if (debug) {
    Serial.print("Data sent: 0x");
    Serial.print(byte1, HEX);
    Serial.print(", 0x");
    Serial.print(byte2, HEX);
    Serial.print(", 0x");
    Serial.println(byte3, HEX);
  }
}

// Print a total of 16 * 16 characters.
// Print 4 sets of "full displays".
// Print 4 rows per "full display".
// Print 16 characters per row.
void printFullCharacterMap() {
  for (int i = 0; i <= 3; i++) {
    clearLcdChar();
    for (int j = 0; j <= 3; j++) {
      moveCursor(j + 1,0);
      for (int k = 0; k <= 15; k++) {
        sendPacket(0x5F, k, (i * 4) + j);
      }
    }
    delay(3000);  // allow time for user to view full display
  }
}

// Move to each visible segment position and print all possible segments.
// Several virtual segment positions are not visible and some positions only
// have limited segment symbol options.
// 16 virtual segment positions.
// 16 possible segments symbols at each position.
// Only 7 visible segment positions have been detected. valid segment options
// have not been counted.
void printFullSegmentMap() {
  // Visible segment positions
  const int segPos[] = { 0, 1, 2, 8, 9, 10, 11 };

  setSegmentMode();
  writeText("Seg. position:", 1, 0);
  writeText("Seg. symbol:", 2, 0);
  for (int i : segPos) {
    writeText(String(i) + " ", 1, 15);
    setSegmentMode();
    for (int j = 0; j <= 15; j++) {
      writeText(String(j) + " ", 2, 15);
      setSegmentMode();
      // Move cursor to bottom segment row, location i (0-16)
      sendPacket(0x1F, i, 0x04);
      sendPacket(0x5F, j, 0x00);  // write segment j (0-16)
      delay(250); // allow user time to view printed segment
    }
  }
  clearLcdSeg();
  setCharacterMode();
}

// Setup LCD by sending specific SPI messages after startup
void initLcd() {
  setSegmentMode();
  // Send mystery message (something important during LCD initialization)
  sendPacket(0x1F, 0x09, 0x00);
  setCharacterMode();
  // Enable LCD, disable cursor, disable blinking cursor
  sendPacket(0x1F, 0x0C, 0x00);
  if (debug) {
    Serial.println("LCD initialized!");
  }
}

void clearLcdChar() {
  sendPacket(0x1F, 0x01, 0x00);
  delay(2); // allow time for LCD controller to clear full display
  if (debug) {
    Serial.println("LCD cleared!");
  }
}

void clearLcdSeg() {
  for (int i = 0; i < 16; i++) {  // for each segment positions
    lcd_seg_mode();
    // Move cursor to first segment position
    lcd_send_spi_packet(0x1F, i, 0x04);
    // Write blank segment
    lcd_send_spi_packet(0x5F, 0x00, 0x00);
  }
}
}

void setCharacterMode() {
  sendPacket(0x1F, 0x00, 0x03);
  if (debug) {
    Serial.println("Character mode set!");
  }
}

void setSegmentMode() {
  sendPacket(0x1F, 0x04, 0x03);
  if (debug) {
    Serial.println("Segment mode set!");
  }
}

void moveCursor(int row, int col) {
  // Sanitize input data
  if (row < 1) { row = 1; }
  if (row > 4) { row = 4; }
  if (col < 0)  { col = 0; }
  if (col > 17) { col = 17; }

  // Convert requested row + col to LCD controller DDRAM position
  // Byte 2, bit 3 in SPI packet enables "select DDRAM position" function and
  // must be set.
  // Requested row: byte 3, bit 1..2
  // Requested column: bit 0..3 in byte 2 + bit 0 in byte 3
  int tmp = 0b10000000 | ((row - 1) << 5) | col;
  sendPacket(0x1F, tmp & 0b00001111, tmp >> 4);

  if (debug) {
    Serial.print("Cursor moved to row ");
    Serial.print(row, DEC);
    Serial.print(", column: ");
    Serial.print(col, DEC);
    Serial.println("!");
  }
}

// Write text at selected row.
// Characters A-Z,a-z,0-9 and a few symbols corresponds to ASCII values. Other
// characters does not.
void writeText(String text, int row, int col) {
  setCharacterMode();
  moveCursor(row, col);

  for (int i = 0; i < text.length(); i++) {
    sendPacket(0x5F, text[i] & B00001111, (text[i] & B11110000) >> 4);
  }
}
