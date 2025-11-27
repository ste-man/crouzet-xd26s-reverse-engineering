
# Introduction and history

A Crouzet Millenium 3 XD26S (not the newer "SMART" version) PLC was left over from discontinued industrial equipment and given to me as a gift. The hardware seemed to be well designed and manufactured, it has a potent 4x18 character LCD display, 6 user interface buttons and a capable 8-bit ATMEL ATmega128-AU microcontroller so I decided to reverse engineer the device in order to program the microcontroller directly instead of using the manufacturers automation software.


## TLDR, can this device be useful for my general hobby project?

Yes. The ATmega128 fuse bits on my device was by default set to e:0xff h:0x8a l:0x1f which allows for JTAG communication. XD26S PCB (hardware) also supports JTAG (there are additional resistors installed on the 4 JTAG signal lines) and both JTAG + ICSP SPI pins can be accessed via test/solder points on backside of main PCB. Take a look at the electrical schematics and photos of PCB.


# Crouzet Millenium 3 XD26S device information

## Electrical schematics

Take a look at the reverse engineered [electrical schematics](https://github.com/ste-man/crouzet-xd26s-reverse-engineering/blob/main/reverse-engineering/ecad/crouzet-xd26s-schematic/crouzet-xd26s_schematic.pdf) for a good overview of device capabilities. Min circuit board does not have silkscreen and is missing reference designators. New reference designators has been created by me. See related annotated photos of circuit board for reference. Electrical schematics have been created with Kicad 9.0.** (<https://www.kicad.org/>).


## Main circuit board microcontroller ("main MCU")

The main circuit board has an 8-bit ATMEL ATmega128-AU microcontroller (<https://www.microchip.com/en-us/product/atmega128>).


### Program demo

A simple demo program has been created in order to provide a very quick way of testing device hardware. A more serious solution would be to create a software library suitable for this device and more Crouzet devices in same model series with interface to display, buttons, I/O and real time clock etc. See video of device operating with new demo software: <https://www.youtube.com/watch?v=avJbvae7_Jc>.


### Access to programming main microcontroller

See electrical schematic for reference.
Main circuit board hardware supports programming via JTAG. The MCU JTAG-TAP (Test Access Port) pins are also used as general INPUT pins, but there are resistors installed separating MCU pins from rest of circuit. JTAG-TAP pins can be easily accessed via test/solder points on backside of main PCB. Programming via SPI can be enabled (via JTAG) and ICSP-SPI pins can also be accessed via test/solder points on backside of main PCB and SPI-SCK pin on side "extension connector". See annotated photos of main circuit board.


### Crouzet XD26S ATmega128 startup sequence

| Time after MCU bootup [ms] | Task |
| :------------------------- | :--- |
| ~500 | Starting SPI communication. Sending first SPI packets to LCD. LCD initialization. |
| ~510 | Starting I2C communication with Real Time Clock module. |
| ~525 | Sending SPI packets to extension module 1 and 2. Extension modules are not required. |
| ~690 | Starts sending BUTTON CLOCK signals (polling pushbutton status from LCD circuit board). |
| ~740 | Starts updating LCD display buffer continuously (SPI communication). |


## LCD display

- LCD display can display 4x18 characters and fixed segments below row 4 (bottom of display). See demo program video for reference.
- Each LCD character consists of 5x8 pixels.
- Row order: 1 (top), 2, 3, 4 (bottom).
- Column order: 1 (left), .. 18 (right)
- Main MCU communicates via SPI (<https://en.wikipedia.org/wiki/Serial_Peripheral_Interface>) with LCD controller.
- LCD controller resembles Hitachi HD44780 (<https://en.wikipedia.org/wiki/Hitachi_HD44780_LCD_controller>), but there are some major differences and additional functionality. LCD controller is hidden beneath a black epoxy blob and no effort has been made to try and determine exact model. Controller(s) might - for example - consist of a standard type of microcontroller (with SPI capabilities) interfacing one or more standard LCD driver IC(s).


### ATmega 128 SPI communication with LCD

- Data is LSB (Least Significant Bit first)
- SPI mode 3. From Wikipedia <https://en.wikipedia.org/wiki/Serial_Peripheral_Interface>:
- There is no traffic on SPI-MISO PIN (LCD controller does never send SPI information/feedback to SPI master main MCU).
- Each SPI packet from main MCU to LCD controller consists of 3 bytes. Each byte is 8 bits. See SPI packet details.


#### LCD “modes”

The LCD controller seems to have 2 distinct "modes". Some messages are interpreted differently depending on which mode the controller currently is set to. The mode names are made up to improve understanding of mode functionality.

| Mode | Functionality |
| :--- | :------------ |
| SEGMENT MODE   | Move cursor to selected locations below 4x18 character row 4 and write fixed segments (including a line). Control other stuff that are not yet understood. |
| CHARACTER MODE | Move cursor to selected location on 4x18 LCD and write characters at selected location. Control general aspects of LCD (see details below). |


#### SPI packet details:

| Byte | Function |
| :--- | :------- |
| Byte 1: | 0x1F = Control stuff! For example: move cursor, enable/disable LCD, clear LCD<br>0x5F = Write segment or character to current cursor location |

| Byte | Function in SEGMENT mode | Function in CHARACTER mode |
| :--- | :--------------------------- | :--------------------------- |
| Byte 2: | If byte 1 = 0x5F: Bit 0..3 corresponds to selected segment to write at current position.<br>Otherwise: Part of control message. | Functionality of bit 0..3 resembles Hitatchi HD44780U LCD controller datasheet. See image below. |
| Byte 3: | Seems to be part of control message. | Functionality of bit 0..3 resembles Hitatchi HD44780U LCD controller. See image below.  |

Correlation between Hitatchi HD44780U LCD controller and SPI packet byte 2 and 3 in CHARACTER mode:
![hd44780u-vs-lcd-spi-commnuication](reverse-engineering/communication-spi/lcd-info/hd44780u-table-6-modified/hd44780u_table-6-modified.svg)


#### SPI control messages

| Packet data (HEX) | Function in SEGMENT MODE and CHARACTER MODE (mode independent) |
| :---------------- | :------------------------------------------------------------- |
| 1F 04 03 | Change mode to SEGMENT MODE |
| 1F 00 03 | Change mode to CHARACTER MODE |
| 1F 01 00 | Corresponds to functionality in HD44780.<br>Clear display (make 4x20 characters blank) etc.<br>**Important:**<br>Does not clear fixed segments (below character row 4, line + rotating arrows etc.). Wait ≥1.5ms after sending this message before sending next message (as per HD44780U datasheet) in order to allow LCD to clear display. LCD will fail to read (or ignore) messages sent <1.5ms after this message. Crouzet XD26S ATmega 128 has a 2ms delay after sending this message. |


#### Crouzet XD26S LCD initialization message

After main MCU bootup, the first SPI message sent from MCU to LCD controller is following:

| Packet number | Packet data (HEX) | Description |
| :------------ | :---------------- | :---------- |
| 1  | 1F 04 03 | Change mode to SEGMENT MODE |
| 2  | 1F 09 00 | Mystery message. Something important during LCD initialization.<br>LCD does not function without this message here. |
| 3  | 1F 00 03 | Change mode to CHARACTER MODE |
| 4  | 1F 0C 00 | Enable LCD. Cursor OFF. Blinking cursor DISABLED. |
| 5  | 1F 01 00 | Clear display (make 4x20 characters blank) |
| 6  | 1F 06 00 | Set write direction to [left > right] |
| 7  | 1F 00 04 | Move "cursor" to Character Generator RAM |
| 8 .. 71 | 5F \*\* \*\* | Send 64 packets of data. Each packet (3 bytes) contains 5 pixels for custom characters. 8 custom characters are created. See image below. |

Crouzet XD26S CGRAM data (8 custom characters):
**TODO: image here**


#### Crouzet XD26S Periodic communication with LCD

The messages below are sent from Crouzet XD26S main MCU to LCD controller every ~160 ms.

| Packet number | Packet data (HEX) | Brief description |
| :------------ | :---------------- | :---------------- |
| 1        | 1F 00 08       | Move cursor to row 1, column 1 |
| 2 .. 19  | 5F \*\* \*\*   | Write characters to row 1, column 1 .. 18 |
| 20       | 1F 00 0A       | Move cursor to row 2, column 1 |
| 21 .. 38 | 5F \*\* \*\*   | Write characters to row 2, column 1 .. 18 |
| 39       | 1F 00 0A       | Move cursor to row 3, column 1 |
| 40 .. 57 | 5F \*\* \*\*   | Write characters to row 3, column 1 .. 18 |
| 58       | 1F 00 0A       | Move cursor to row 4, column 1 |
| 59 .. 76 | 5F \*\* \*\*   | Write characters to row 4, column 1 .. 18 |
| 77       | 1F 04 03       | Change mode to SEGMENT MODE |
| 78       | 1F 00 04       | Move cursor to bottom segment row (below character row 4), location 1 from left |
| 79       | 5F \*\* \*\*   | Write segment at current cursor location |
| 80 .. 91 | \*\* \*\* \*\* | Move cursor and write more segments.. |
| 92       | 1F 00 03       | Change mode to CHARACTER MODE |


### LCD button status communication:

- There are 6 buttons (A, B, ESC, -, +, OK) on the display control panel. The LCD controller/MCU is monitoring button status (pressed or not pressed) and indicating button status to main MCU via 3 shared signal lines.
- Each signal line is idle HIGH and active LOW. See electrical schematic for reference and connector pin numbers.
- Main MCU (ATmega128) is sending a periodic clock pulse every 19.7ms (\~20 ms).
Clock pulse is idle HIGH (+5VDC) and active LOW (GND). Pulse length is \~3us.
- In order for main MCU to determine which button is activated and if a button is activated or not not, the button signal lines must be queried at the conditions active "button clock cycle" or active LCD SPI CHIP SELECT.

For example, see truth table below for **signal line status** of electrical schematic **net "BUTTON-A/-"**.

| Physical button status | Button clock pulse IDLE (HIGH) | Button clock pulse ACTIVE (LOW) | SPI-CS-LCD IDLE (HIGH) | SPI-CS-LCD ACTIVE (LOW) |
| :--------------------- | :----------------------------- | :------------------------------ | :--------------------- | :---------------------- |
| no button pressed  | IDLE | IDLE   | IDLE | IDLE   |
| button "A" pressed | IDLE | ACTIVE | IDLE | IDLE   |
| button "-" pressed | IDLE | IDLE   | IDLE | ACTIVE |

See trace in image below. Note that image is composed of several traces with different time base. Pulse width is not accurate.

**TODO: SCREENSHOT FROM PICOSCOPE HERE WITH BUTTON PRESSES.**


### "LCD RESET signal" LCD connector J2 PIN 9

Signal state of "LCD RESET" is controlled by main MCU and seems to perform a partial reset of LCD controller. LCD controller turns off/resets if signal state is LOW during LCD operation and LCD must be re-initialized in order to function. Exact purpose and functionality of this signal is not yet understood. Signal state is HIGH (IDLE?) during normal operation. Main MCU is changing state of this signal during startup and when power supply Voltage is detected to be under a specific threshold.


# Support and contributions?

Have you detected a mistake somewhere in the project or want to improve it. Send me a pull request or message. My intention is that most of the uploaded and written information should be self explanatory, but feel free to request minor support related to this or similar project(s).
