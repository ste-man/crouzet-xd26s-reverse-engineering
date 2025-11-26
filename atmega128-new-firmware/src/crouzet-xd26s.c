/* Crouzet XD26S
 * Last updated:  2025-11-24
 *
 * HARDWARE INFO
 * MCU: Atmel ATmega128-16AU
 * External resonator: 7.37 MHz
 * VCC + AVCC supply: 5 VDC
 *
 * COMMENTS
 * Some registers are explicitly declared to default value for clarity.
 * Code is excessively commmented to guide novice users/readers and save time
 * by minimizing required referencing external documentation.
 *
 * FUSE SETTINGS
 * Original fuse bits (programming via JTAG):
 * efuse: 0xff
 * hfuse: 0xaa (SPI programming disabled, JTAG enabled)
 * lfuse: 0x1f
 * https://www.engbedded.com/fusecalc/
 */

#include <avr/io.h> // includes I/O definitions. MCU type is defined in compiler.
// https://www.nongnu.org/avr-libc/user-manual/group__avr__io.html
#define F_CPU 7370000UL // MCU clock speed. Defined for util/delay.h
#include <util/delay.h>
// https://www.nongnu.org/avr-libc/user-manual/group__util__delay.html
#include <stdio.h>  // sprintf function + more
// https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html

/* HELPFUL GENERAL MACROS */
#define TRUE  1
#define FALSE 0
#define bitset(value, bitno)             ((value) |=  (1<<(bitno)))
#define bitclear(value, bitno)           ((value) &= ~(1<<(bitno)))
#define bitflip(value, bitno)            ((value) ^=  (1<<(bitno)))
#define bitread(value, bitno)            ((value) &   (1<<(bitno)))
#define bitwrite(value, bitno, bitvalue) (bitvalue ? bitset(value, bitno) : bitclear(value, bitno))

/* MCU I/O PINS AND RELATED FUNCTIONS*/
#define IN1  PE6
#define IN2  PE7
#define IN3  PA0
#define IN4  PA1
#define IN5  PA2
#define IN6  PA3
#define IN7  PA4
#define IN8  PA5
#define IN9  PA6
#define IN10 PA7  // IA
#define IN11 PF2  // IB
#define IN12 PF3  // IC
#define IN13 PF4  // ID
#define IN14 PF5  // IE
#define IN15 PF6  // IF
#define IN16 PF7  // IG

#define AIN1 PF0  // Supply Voltage before buffer
#define AIN2 PF1  // Supply Voltage buffer

#define OUT1  PE0
#define OUT2  PE1
#define OUT3  PE2
#define OUT4  PE3
#define OUT5  PE4
#define OUT6  PE5
#define OUT7  PB4
#define OUT8  PB5
#define OUT9  PB6
#define OUT10 PB7 // OA

#define BTN_CLK    PD5 // button clock signal
#define BTN_A_M    PG2 // button A and [-]
#define BTN_B_P    PG1 // button B and [+]
#define BTN_ESC_OK PG0 // button ESC and OK

#define LCD_RESET  PG4

#define SPI_MISO    PB3 //
#define SPI_MOSI    PB2 //
#define SPI_SCK     PB1 //
#define SPI_CS_LCD  PD4 // SPI Chip Select for LCD
#define SPI_CS_EXT1 PB0 // SPI Chip Select for Extension module 1
#define SPI_CS_EXT2 PD6 // SPI Chip Select for Extension module 2

#define SPI_ENABLE        bitset(SPCR, SPE)
#define SPI_DISABLE       bitclear(SPCR, SPE)
#define SPI_DATA_REG      SPDR // SPI Data Register
#define SPI_SEL_EXT1      bitclear(PORTB, SPI_CS_EXT1);
#define SPI_DESELECT_EXT1 bitset(PORTB, SPI_CS_EXT1);
#define SPI_SEL_EXT2      bitclear(PORTD, SPI_CS_EXT2);
#define SPI_DESELECT_EXT2 bitset(PORTD, SPI_CS_EXT2);
#define SPI_SELECT_LCD    bitclear(PORTD, SPI_CS_LCD);
#define SPI_DESELECT_LCD  bitset(PORTD, SPI_CS_LCD);

#define I2C_SCL PD0 //
#define I2C_SDA PD1 //

#define LCD_BACKLIGHT PD7 // LCD backlight control

#define LCD_ENABLE_BACKLIGHT  bitset(PORTD, LCD_BACKLIGHT)
#define LCD_DISABLE_BACKLIGHT bitclear(PORTD, LCD_BACKLIGHT)

#define ADC_ENABLE        (ADCSRA |= (1 << ADEN))
#define ADC_DISABLE       (ADCSRA &= ~(1 << ADEN))
#define ADC_START_CONV    (ADCSRA |= (1 << ADSC))
#define ADC_CONV_COMPL    (ADCSRA & (1 << ADIF))  // check if conversion is completed
#define ADC_RESULT_8BIT   (ADCH)  // 8 most significant bits from 10-bit ADC result
#define ADC_RESET_RESULT  (ADCSRA |= (1 << ADIF))
#define ADC_SELECT_ADC0   (bitclear(ADMUX, MUX0))
#define ADC_SELECT_ADC1   (bitwrite(ADMUX, MUX0))

/* USER SETTING VARIABLES */

/* GLOBAL VARIABLES */
// Variable btn_status stores an array (bit field) of binary input values.
// Bn = 0 means button is currently not pressed/activated
// Bn = 1 means button is currently pressed/activated
// Variable declared volatile due to being used by ISR.
volatile uint8_t btn_status = 0;

// Bit field positions for variable btn_status.
 #define STAT_BTN_A   0
 #define STAT_BTN_B   1
 #define STAT_BTN_ESC 2
 #define STAT_BTN_M   3
 #define STAT_BTN_P   4
 #define STAT_BTN_OK  5

// Description: Setup general purpose I/O
void setup_gpio() {
  // Port B Data Direction Register
  DDRB |= (1 << OUT7)
       |  (1 << OUT8)
       |  (1 << OUT9)
       |  (1 << OUT10);
  // WARNING!! pins PC3..7 are hardwired to GND. pins SHOULD BE INPUT!
  // Port C Data Direction Register
  DDRC = (0 << PC3)
       | (0 << PC4)
       | (0 << PC5)
       | (0 << PC6)
       | (0 << PC7);
  // Port D Data Direction Register
  DDRD |= (1 << BTN_CLK)
       |  (1 << LCD_BACKLIGHT);
  // Port E Data Direction Register
  DDRE |= (1 << OUT1)
       |  (1 << OUT2)
       |  (1 << OUT3)
       |  (1 << OUT4)
       |  (1 << OUT5)
       |  (1 << OUT6);
  // Port G Data Register
  DDRG = (1 << LCD_RESET);

  // no internal pullup resistors are required
}

// Description: Setup ADC
void setup_adc() {
  // ADC Multiplexer Selection Register
  ADMUX = (0 << REFS1)  // analog Vref: AREF, Internal Vref turned off
        | (0 << REFS0)  // analog Vref: AREF, Internal Vref turned off
        | (1 << ADLAR)  // left adjusted result data
        | (0 << MUX4)   // select input channel ADC0
        | (0 << MUX3)   // select input channel ADC0
        | (0 << MUX2)   // select input channel ADC0
        | (0 << MUX1)   // select input channel ADC0
        | (0 << MUX0);  // select input channel ADC0
  // ADC Control and Status Register A
  ADCSRA = (0 << ADEN)    // leave ADC disabled
         | (0 << ADSC)    // ADC Start Conversion
         | (0 << ADFR)    // manual ADC trigger
         | (0 << ADIF)    // ADC Interrupt Flag
         | (1 << ADIE)    // ADC Interrupt Enable
         | (1 << ADPS2)   // prescaler division factor 64 -> ~115kHz ADC
         | (1 << ADPS1)   // prescaler division factor...
         | (0 << ADPS0);  // prescaler division factor...
}

// Description: Setup SPI communication bus
void setup_spi() {
  // Unit operating at SPI mode 3.
  // See https://en.wikipedia.org/wiki/Serial_Peripheral_Interface

  // De-select all SPI slaves before setting pins to OUTPUT
  SPI_DESELECT_EXT1;
  SPI_DESELECT_EXT2;
  SPI_DESELECT_LCD;
  bitset(PORTB, SPI_SCK); // prevent signal SCK to drop LOW during SPI startup
  // Port B Data Direction register
  DDRB |= (1 << SPI_MISO)
       |  (1 << SPI_MOSI)
       |  (1 << SPI_SCK)
       |  (1 << SPI_CS_EXT1);
  // Port D Data Direction register
  DDRD |= (1 << SPI_CS_LCD)
       |  (1 << SPI_CS_EXT2);
  // SPI Control Register
  SPCR = (1 << SPIE)  // SPI Interrupt Enable
       | (1 << SPE)   // SPI Enable
       | (1 << DORD)  // Data Order = Least Significant Bit
       | (1 << MSTR)  // Device is SPI Master (not slave)
       | (1 << CPOL)  // Clock Polarity, see SPI mode 3
       | (1 << CPHA)  // Clock Phase, see SPI mode 3
       | (0 << SPR1)  // SPI Clock Rate Select, 1.83MHz -> fOSC/4
       | (0 << SPR0); // SPI Clock Rate Select...
  // SPI Status Register
  SPSR |= (0 << SPI2X);  // SPI Clock Rate Select...
}

// Description: Read current state of pushbuttons and update global variable.
// 6 buttons are sharing 3 physical signal lines and must be queried under
// different conditions (active button clock or SPI Chip Select signal).
// Button clock, buttons and LCD SPI Chip Select signals are active LOW.
void update_btn_status() {
  if (!bitread(PIND, BTN_CLK)) {
    bitwrite(btn_status, STAT_BTN_A,   !bitread(PING, BTN_A_M));
    bitwrite(btn_status, STAT_BTN_B,   !bitread(PING, BTN_B_P));
    bitwrite(btn_status, STAT_BTN_ESC, !bitread(PING, BTN_ESC_OK));
  }
  if (!bitread(PIND, SPI_CS_LCD)) {
    bitwrite(btn_status, STAT_BTN_M,  !bitread(PING, BTN_A_M));
    bitwrite(btn_status, STAT_BTN_P,  !bitread(PING, BTN_B_P));
    bitwrite(btn_status, STAT_BTN_OK, !bitread(PING, BTN_ESC_OK));
  }
}

// Description: Read status of INPUT pins
// Return value: Bit field. Bit 0..15 = INPUT1..INPUT16
uint16_t read_input_pins() {
  uint16_t status = 0;
  bitwrite(status, 0, bitread(PINE, IN1));
  bitwrite(status, 1, bitread(PINE, IN2));
  bitwrite(status, 2, bitread(PINA, IN3));
  bitwrite(status, 3, bitread(PINA, IN4));
  bitwrite(status, 4, bitread(PINA, IN5));
  bitwrite(status, 5, bitread(PINA, IN6));
  bitwrite(status, 6, bitread(PINA, IN7));
  bitwrite(status, 7, bitread(PINA, IN8));
  bitwrite(status, 8, bitread(PINA, IN9));
  bitwrite(status, 9, bitread(PINA, IN10));
  bitwrite(status, 10, bitread(PINF, IN11));
  bitwrite(status, 11, bitread(PINF, IN12));
  bitwrite(status, 12, bitread(PINF, IN13));
  bitwrite(status, 13, bitread(PINF, IN14));
  bitwrite(status, 14, bitread(PINF, IN15));
  bitwrite(status, 15, bitread(PINF, IN16));
  return status;
}

// Description: Set status of OUTPUT pins.
// Function parameter variable bit 0..9 = status of OUTPUT1..OUTPUT10
void set_output_pins(uint16_t status) {
  bitwrite(PORTE, OUT1, bitread(status, 0));
  bitwrite(PORTE, OUT2, bitread(status, 1));
  bitwrite(PORTE, OUT3, bitread(status, 2));
  bitwrite(PORTE, OUT4, bitread(status, 3));
  bitwrite(PORTE, OUT5, bitread(status, 4));
  bitwrite(PORTE, OUT6, bitread(status, 5));
  bitwrite(PORTB, OUT7, bitread(status, 6));
  bitwrite(PORTB, OUT8, bitread(status, 7));
  bitwrite(PORTB, OUT9, bitread(status, 8));
  bitwrite(PORTB, OUT10, bitread(status, 9));
}

// Description: Send single byte via SPI communication
void send_spi_byte(uint8_t byte) {
  SPI_DATA_REG = byte;
  // Wait until serial transfer is complete.
  while (!(bitread(SPSR, SPIF))) {}
}

// Description: Reset LCD controller by enabling LCD RESET pin for a defined
// amount of time. LCD controller need to be re-initialized after a reset.
// LCD segments are not cleared after reset. LCD ddram (characters) seems to not
// be cleared, but since settings are lost, LCD does not display characters
// correct.
void lcd_reset() {
  // Enable LCD (release RESET signal)
  bitclear(PORTG, LCD_RESET);
  _delay_ms(50);  // This value has not been evaluated. TODO: Test!!
  bitset(PORTG, LCD_RESET);
}

// Description: Send single SPI packet to LCD pheriphial. 1 packet = 3 bytes
void lcd_send_spi_packet(uint8_t byte1, uint8_t byte2, uint8_t byte3) {
  SPI_SELECT_LCD;
  _delay_us(1); // Crouzet XD26S measured delay = 2us
  send_spi_byte(byte1);
  _delay_us(6);  // Crouzet XD26S measured delay = 8us
  send_spi_byte(byte2);
  _delay_us(6);  // Crouzet XD26S measured delay = 8us
  send_spi_byte(byte3);

  // LCD SPI Chip Select pin is active. Check if any panel button is activated.
  update_btn_status();
  _delay_us(92);  // Crouzet XD26S measured delay = 95us
  SPI_DESELECT_LCD;
  _delay_us(3);
}

// Description: Set LCD to SEGMENT mode
void lcd_seg_mode() {
  lcd_send_spi_packet(0x1F, 0x04, 0x03);
}

// Description: Set LCD to CHARACTER mode
void lcd_char_mode() {
  lcd_send_spi_packet(0x1F, 0x00, 0x03);
}

// Description: Clear 4x18 LCD characters. Does not clear segments!
void lcd_clear_char() {
  lcd_send_spi_packet(0x1F, 0x01, 0x00);
  // Allow extra time for LCD to clear complete display
  _delay_ms(2); // measured delay = 2 ms
}

// Description: Clear LCD segments. Does not clear 4x18 LCD characters!
void lcd_clear_seg() {
  for (uint8_t i = 0; i < 16; i++) {  // for each segment positions
    lcd_seg_mode();
    // Move cursor to first segment position
    lcd_send_spi_packet(0x1F, i, 0x04);
    // Write blank segment
    lcd_send_spi_packet(0x5F, 0x00, 0x00);
  }
}

// Description: Move LCD cursor to requested position on 4x18 LCD
// Valid rows: 1..4
// Visible columns: 0..17
// Valid columns (in memory): 0..19
void lcd_move_cursor(uint8_t col, uint8_t row) {
  if (row < 1)  { row = 1; }
  if (row > 4)  { row = 4; }
  if (col < 0)  { col = 0; }
  if (col > 17) { col = 17; }
  uint8_t ddram = 0b10000000 | ((row - 1) << 5) | col;
  // SPI packet byte 2: ddram 4 Least Significant Bits
  // SPI packet byte 3: ddram 4 Most Significant Bits
  lcd_char_mode();
  lcd_send_spi_packet(0x1F, ddram & 0b00001111, ddram >> 4);
}

// Description: Initialize LCD by sending data to LCD via SPI
void lcd_init() {
  // Button clock signal is idle high
  bitset(PORTD, BTN_CLK);
  // Enable LCD (release RESET signal)
  bitset(PORTG, LCD_RESET);
  // Allow time for LCD controller to startup.
  // Amount of time required has not been measured.
  _delay_ms(100);
  lcd_seg_mode();
  // Send initialization mystery message
  lcd_send_spi_packet(0x1F, 0x09, 0x00);
  lcd_char_mode();
  // Enable LCD, disable cursor, disable blinking cursor
  lcd_send_spi_packet(0x1F, 0x0C, 0x00);
  // Enable LCD, enable cursor, enable blinking cursor
  //lcd_send_spi_packet(0x1F, 0x0F, 0x00);
  lcd_clear_char();
  lcd_clear_seg();
}

// Description: Display character(s) on LCD. Characters compatible with ASCII:
// A-Z, a-z, 0-9 and a few common symbols.
void lcd_print(char str[]) {
  lcd_char_mode();
  // Loop until character NULL (NULL = end of string).
  for (uint8_t i = 0; str[i]; i++) {
    // SPI byte 2: character bit 0..3
    // SPI byte 3: character bit 4..7
    lcd_send_spi_packet(0x5F, str[i] & 0b00001111, str[i] >> 4);
  }
}

// Description: Print full character map.
// Print a total of 16 * 16 characters.
// Print 4 sets of "full displays".
// Print 4 rows per "full display".
// Print 16 characters per row.
void lcd_demo() {
  lcd_move_cursor(4, 1);
  lcd_print("LCD DEMO");
  lcd_move_cursor(1, 3);
  lcd_print("Print complete");
  lcd_move_cursor(1, 4);
  lcd_print("char + seg map.");
  _delay_ms(4000);
  lcd_clear_char();

  for (uint8_t i = 0; i <= 3; i++) {  // full display
    lcd_clear_char();
    for (uint8_t j = 0; j <= 3; j++) {  // row
      lcd_move_cursor(0, j + 1);
      for (uint8_t k = 0; k <= 15; k++) { // column
        lcd_send_spi_packet(0x5F, k, (i * 4) + j);
      }
    }
    _delay_ms(3000);  // allow time for user to view full display
  }

  lcd_clear_char();
  lcd_move_cursor(0, 2);
  lcd_print("Seg. position:");
  lcd_move_cursor(0, 3);
  lcd_print("Seg. symbol:");

  // Visible segment positions. Bit 0..15 = segment position 0..15.
  // 0 -> position is not visible on LCD. No point of displaying this segment
  // position. 1 -> position is visible.
  const uint16_t visible_seg_pos = 0b0000111100000111;

  for (uint8_t i = 0; i < 16; i++) {
    if (bitread(visible_seg_pos, i)) {
      char str[] = "00";  // default string
      lcd_move_cursor(15,2);
      sprintf(str, "%2u", i); // set string to segment position value
      lcd_print(str);

      for (uint8_t j = 0; j < 16; j++) {
        lcd_move_cursor(15,3);
        sprintf(str, "%2u", j); // set string to segment value
        lcd_print(str);

        lcd_seg_mode();
        // Move cursor to bottom symbol row, location i (0-16)
        lcd_send_spi_packet(0x1F, i, 0x04);
        // Write segment j (0-16)
        lcd_send_spi_packet(0x5F, j, 0x00);
        _delay_ms(250); // allow user time to view each displayed segment
      }
    }
  }
  _delay_ms(3000);
  lcd_clear_seg();
}

// Description: User can press front panel pushbuttons and see visual feedback
// on LCD. Function ends after a defined amount of time. No effort has been
// spent to keep time unit specified.
// Buttons A, B and ESC are queried during active button clock pulse.
// A Crouzet XD26S button clock pulse is 3us long and sent every ~20ms.
// Button clock pulse is generated manually in this demo, but should normally be
// called from timed interrupt (depending on button functionality).
// Buttons -, + and OK are updated when LCD SPI Chip Select signal is active.
void pushbutton_demo() {
  lcd_move_cursor(1, 1);
  lcd_print("PUSHBUTTON DEMO");
  lcd_move_cursor(0, 3);
  lcd_print("Demo is active for");
  lcd_move_cursor(0, 4);
  lcd_print("approx 10 seconds.");
  _delay_ms(4000);

  lcd_clear_char();
  lcd_move_cursor(1, 1);
  lcd_print("Press buttons on");
  lcd_move_cursor(1, 2);
  lcd_print("front panel.");
  lcd_move_cursor(1, 3);

  uint16_t i = 750;  // ~10 seconds
  while (i) {
    // Activate "button clock" and query if any buttons are pressed
    bitclear(PORTD, BTN_CLK); // activate button clock signal
    _delay_us(3); // Crouzet XD26S measured pulse length is 3 us
    update_btn_status();
    bitset(PORTD, BTN_CLK); // deactivate button clock signal

    char str[] = "A B ESC M P OK"; // default LCD string (no button pressed)

    // Inverted characters 0-9 and A-H are placed 80 steps offset from "normal"
    // characters 0-9 and A-H.
    for (uint8_t j = 0; j < 6; j++) { // for each button
      if (bitread(btn_status,j)) {
        switch (j) {
          case STAT_BTN_A:
            str[0] += 80; // Inverted capital letter A
            break;
          case STAT_BTN_B:
            str[2] += 80; // Inverted capital letter B
            break;
          case STAT_BTN_ESC:
            str[4] += 80; // Inverted capital letter E
            str[5] = 176; // Inverted capital letter S
            str[6] += 80; // Inverted capital letter C
            break;
          case STAT_BTN_M:
            str[8] = 156; // Inverted capital letter M
            break;
          case STAT_BTN_P:
            str[10] = 157; // Inverted capital letter P
            break;
          case STAT_BTN_OK:
            str[12] = 128; // Inverted capital letter/digit O
            str[13] = 154; // Inverted capital letter K
            break;
        }
      }
    }
    lcd_move_cursor(2,4);
    lcd_print(str);
    _delay_ms(10);
    i--;
  }
}

// Description: Disable JTAG in order to enable GPIO pins ID..IG (PF4..PF7).
void disable_jtag() {
  // From MCU datasheet:
  // In order to avoid unintentional disabling or enabling of the JTAG interface,
  // a timed sequence must be followed when changing this bit: The application
  // software must write this bit to the desired value twice within four cycles to
  // change its value.
  bitset(MCUCSR, JTD);
  bitset(MCUCSR, JTD);
}

// Description: General Purpose Input Output demo. Presents visible feedback for
// input pins and sequentially enable each output pin for a short time.
void io_demo() {
  disable_jtag(); // JTAG prevents monitoring of input pins ID..IG
  lcd_move_cursor(4, 1);
  lcd_print("IO DEMO");
  lcd_move_cursor(0, 3);
  lcd_print("Demo is active for");
  lcd_move_cursor(0, 4);
  lcd_print("approx 15 seconds.");
  _delay_ms(4000);

  lcd_clear_char();
  lcd_move_cursor(1, 1);
  lcd_print("INPUT STATUS:");
  lcd_move_cursor(1, 3);
  lcd_print("OUTPUT STATUS");

  // Bit field variable for all output pins. Bit 0..10 = Output 0..10
  uint16_t output_status = 1;
  for (uint8_t i = 0; i < 10; i++) {  // for each output pin
    uint8_t j = 100;  // ~1.5 seconds
    set_output_pins(output_status);

    while (j) {
      char str_in[] = "123456789ABCDEFG"; // Default string (no input activated)
      char str_out[] = "123456789A";  // Default string (no output activated)

      uint16_t input_status = read_input_pins();

      for (uint16_t k = 0; k < 16; k++) { // for each input pin
        if (bitread(input_status, k)) { // if input is active
          // Inverted characters 0-9 and A-H are placed 80 steps offset from
          // "normal" characters 0-9 and A-H.
          str_in[k] = str_in[k] + 80; // change to inverted character on LCD
        }
      }
      for (uint16_t k = 0; k < 10; k++) { // for each output pin
        if (bitread(output_status, k)) { // if output is active
          str_out[k] = str_out[k] + 80; // change to inverted character on LCD
        }
      }

      lcd_move_cursor(1, 2);
      lcd_print(str_in);
      lcd_move_cursor(1, 4);
      lcd_print(str_out);
      _delay_ms(10);
      j--;
    }
    // Shift to next output pin
    output_status = (output_status << 1);
  }
}

// Description: Analog to Digital Conversion demo. Supply input Voltage is
// measured and presented on LCD.
void adc_demo() {
  lcd_move_cursor(4, 1);
  lcd_print("ADC DEMO");
  lcd_move_cursor(0, 3);
  lcd_print("Demo is active for");
  lcd_move_cursor(0, 4);
  lcd_print("approx 10 seconds.");
  _delay_ms(4000);

  lcd_clear_char();
  lcd_move_cursor(2, 2);
  lcd_print("INPUT VOLTAGE:");

  ADC_ENABLE;
  ADC_SELECT_ADC0;

  for (uint16_t i = 0; i < 500; i++) {  // ~10 seconds
    char str[] = "00.0";  // Default string
    ADC_START_CONV;
    while (!ADC_CONV_COMPL) {}
    // Supply Voltage = 24VDC -> ADC value 195
    // Supply Voltage = 11VDC -> ADC value 87
    // -0.52 VDC offset on measured Voltage due to diode on input
    // Calculated Voltage value below has a tolerance of +-0.2 VDC
    uint16_t tmp = (ADC_RESULT_8BIT * 12) + 62; // = [VDC * 100]
    // Function sprintf adds >1k flash memory. There are most likely more
    // efficient ways of converting a multi-digit value to character values.
    sprintf (str, "%3u", (tmp/10));
    // Shift least significant digit one step to right and add a decimal "."
    // character.
    str[3] = str[2];
    str[2] = 46; // character . ("dot")

    lcd_move_cursor(5, 3);
    lcd_print(str);
    lcd_move_cursor(10,3);
    lcd_print("VDC");

    _delay_ms(20);
  }
  ADC_DISABLE;
}

int main(void) {
  setup_gpio();
  setup_adc();
  setup_spi();

  // Don't be hasty master Meriadoc!
  _delay_ms(1000);  // Crouzet XD26S measured delay is ~500 ms

  while (1) {
    lcd_init();
    LCD_ENABLE_BACKLIGHT;

    // Print welcome message on LCD
    lcd_move_cursor(3, 2);
    lcd_print("Welcome :)");
    _delay_ms(2000);

    lcd_clear_char();
    lcd_demo(); // prints full character and segment map

    lcd_clear_char();
    pushbutton_demo();  // test front panel pushbuttons

    lcd_clear_char();
    io_demo();  // test input and output pins

    lcd_clear_char();
    adc_demo(); // present supply Voltage on LCD

    lcd_clear_char();
    lcd_move_cursor(3, 2); // row 1, column 8
    lcd_print("End of demo.");

    _delay_ms(2000);
    lcd_clear_char();
    LCD_DISABLE_BACKLIGHT;
  }
}
