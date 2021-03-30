/*
  AVR-EPROM-READ

 just a proof-of-concept for hw/sw design

 has to be implemented in pure C + assembly later

*/

//#include <avr/pgmspace.h>

// PIN defs - arduino/hardware/arduino/avr/variants/standard/pins_arduino.h
//
//                         ARDUINO Nano
//
//                      id +-[ usb ]-+ id
//     mem_RD  SCK  PB5 13 | [     ] | 12  PB4 MISO   ic4040_clk
//          x      +3v3    |  -----  | 11 ~PB3 MOSI 
//      C_ref      Aref    |         | 10 ~PB2 SS     [pwm+5]
//     mem_D0   A0  PC0 14 |         | 09 ~PB1        [pwm+12]
//     mem_D1   A1  PC1 15 |         | 08  PB0        ic4040_rst
//       on-5   A2  PC2 16 |         | 07  PD7        mem_D7
//      on+12   A3  PC3 17 |         | 06 ~PD6        mem_D6
//       on+5   A4  PC4 18 | P P     | 05 ~PD5        mem_D5
//        +5v   A5  PC5 19 | B w R T | 04  PD4        mem_D4
//       +12v   A6 ADC6 20 | 5 r x x | 03 ~PD3        mem_D3
//        -5v   A7 ADC7 21 | @ @ @ @ | 02  PD2        mem_D2
//     ic_vcc       +5v    |         |     Gnd        ic_gnd   
//          x Reset PC6    |         |     PC6 Reset  x   
//     ic_gnd       Gnd    |  o o o  | 00  PD0 Rx     <
//          x       Vin    #  # o o  # 01  PD1 Tx     > 
//                         +---------+
//
//               ICSP:    5v  MOSI GND
//                          o   o   o
//                          #   o   o
//                        MISO SCK RST
//
// connection to PB5 will trigger LED = mem_RD

// TODO: voltage measurements
// TODO: store measured/real Vref in eeprom
// TODO: adjust formula for -5V
// TODO: power-up/down sequence
// TODO: xmodem protocol

const char version[] PROGMEM = "2021.03.30";

// NOTE: only \n at the end of the string will be correctly processed (replaced)

const char usage_00[] PROGMEM = "available commands: \n";
const char usage_01[] PROGMEM = "?         ... this usage help \n";
const char usage_02[] PROGMEM = "ver       ... show firmware version \n";
const char usage_03[] PROGMEM = "eol  char ... set end-of-line char (cr, lf, crlf, lfcr) \n";
const char usage_04[] PROGMEM = "     cr   ... dos-windows style end-of-line \n";
const char usage_05[] PROGMEM = "     lf   ... unix-linux-mac style end-of-line \n";
const char usage_06[] PROGMEM = "     crlf ... max. compatibility mode cr+lf (default) \n";
const char usage_07[] PROGMEM = "echo mode ... set echo mode (off, on, dec, hex) \n";
const char usage_08[] PROGMEM = "     off  ... do not echo received chars \n";
const char usage_09[] PROGMEM = "     on   ... echo back each received char (default) \n";
const char usage_10[] PROGMEM = "     dec  ... echo decimal code of each received char (for debug) \n";
const char usage_11[] PROGMEM = "     hex  ... echo hexadec code of each received char (for debug) \n";
const char usage_12[] PROGMEM = "bd speed  ... set serial communication speed (default 9600) \n";
const char usage_13[] PROGMEM = "\n";
const char usage_14[] PROGMEM = "rst       ... reset address counter (set address to 0) \n";
const char usage_15[] PROGMEM = "inc       ... increment address counter (next address) \n";
const char usage_16[] PROGMEM = "v?        ... measure all voltages Vcc=+5V, Vbb=-5V, Vdd=+12V \n";
const char usage_17[] PROGMEM = "rd        ... read actual address \n";
const char usage_18[] PROGMEM = "rd++      ... read and increment address \n";
const char usage_19[] PROGMEM = "addr adr  ... set address counter to adr (next read will start here) \n";
const char usage_20[] PROGMEM = "addr?     ... show actual address counter \n";
const char usage_21[] PROGMEM = "dump      ... dump block 16 bytes \n";
const char usage_22[] PROGMEM = "dump size ... dump block size (16 bytes per line) \n";
const char usage_23[] PROGMEM = "xmdm size ... read block size and send via xmodem protocol to PC \n";
const char usage_24[] PROGMEM = "isff size ... check if entire block size is 0xFF (EPROM empty) \n";
const char usage_25[] PROGMEM = "@ff  size ... loop checking block size and display percentage of 0xFF cells \n";
const char usage_26[] PROGMEM = "is00 size ... check if entire block size is 0x00 (PROM empty) \n";
const char usage_27[] PROGMEM = "@00  size ... loop checking block size and display percentage of 0x00 cells \n";
const char usage_28[] PROGMEM = "\n";
const char usage_29[] PROGMEM = "numbers (size, address, speed) can be entered as: \n";
const char usage_30[] PROGMEM = "    12345 ... decimal number     (max 65535) \n";
const char usage_31[] PROGMEM = "    $1234 ... hexadecimal number (max $ffff) \n";
const char usage_32[] PROGMEM = "    123k  ... decimal number in kilo-bytes (x1024, max  64k) \n";
const char usage_33[] PROGMEM = "    $12k  ... hexadecimal nr in kilo-bytes (x1024, max $40k) \n";
const char usage_34[] PROGMEM = "    m64   ... memory chip capacity [kb], for 2764 use m64 (max m512) \n";

const char *const usage[] PROGMEM = {
    usage_00, usage_01, usage_02, usage_03, usage_04, usage_05, usage_06, usage_07, usage_08, usage_09,
    usage_10, usage_11, usage_12, usage_13, usage_14, usage_15, usage_16, usage_17, usage_18, usage_19,
    usage_20, usage_21, usage_22, usage_23, usage_24, usage_25, usage_26, usage_27, usage_28, usage_29,
    usage_30, usage_31, usage_32, usage_33, usage_34 //, usage_35, usage_36, usage_37, usage_38, usage_39.
};

#define RX_BUFF_SIZE  16
#define TX_BUFF_SIZE 132    // SOH + # + #'1 + 128xDATA + SUM

#define NOP __asm__ __volatile__ ("nop\n\t")

// 16MHz
#define delay125ns __asm__ __volatile__ ("nop \n\t nop \n\t")
#define delay250ns __asm__ __volatile__ ("nop \n\t nop \n\t nop \n\t nop \n\t")

// HW
const uint8_t IC4040_RST =  8;
// falling edge H->L
const uint8_t IC4040_CLK = 12;
// RD/OE pin
const uint8_t MEM_RD_OE  = 13;
// data bus
const int MEM_DATA_BUS[8] = {
     7,    // D7
     6,    // D6
     5,    // D5
     4,    // D4
     3,    // D3
     2,    // D2
    15,    // D1
    14     // D0
};
// voltage measurements
// adc pins - A0 .. A7
// pin
const uint8_t V5P  = 19;	//  5V Positive
const uint8_t V5N  = 20;    //  5V Negative
const uint8_t V12  = 21;    // 12V positive
// resitor voltage divider
// Vpos o-[R1]--(Va)--[R2]-o Vneg
//   Vpos = Va * (R1+R2)/R2 + Vneg * R1/R2
//   Vneg = Va * (R1+R2)/R1 - Vpos * R2/R1
// simplified voltage divider (Vneg = 0)
// Vpos o-[R1]--(Va)--[R2]-GND
//   Vpos = Va * (R1+R2)/R2
#define Rdivider(r1, r2) (r1 + r2) / float(r2)
#define Rratio(r1, r2) (r1) / float(r2)
// we want nominal voltage aroung 1V on analog input pin (Vref = 1.1 .. 1.2 V)
const float V5Pdivider = Rdivider(46.2, 9.84);
const float V5Ndivider = Rdivider(68, 47);
const float V5Nratio   = Rratio(68, 47);
const float V12divider = Rdivider(120.1, 9.85);
// reference voltage (measured on Aref pin)
const float Vref = 1.081;

// local echo mode
enum { echoOFF, echoON, echoDEC, echoHEX } ECHO_MODE = echoON;

// end-of-line string - CR/LF/CRLF
char EOL[3] = "\r\n";

// sequencing: -5 -> +5 -> +12

// xmodem control codes
const struct {
    uint8_t SOH = 0x01;
    uint8_t EOT = 0x04;
    uint8_t ACK = 0x06;
    uint8_t NAK = 0x15;
    uint8_t PAD = 0x1a;
} control;

// === PROGMEEM / FLASHMEM STRINGS ===

const char eol_detect[] PROGMEM = "Press <ENTER> to auto-detect end-of-line [CR/LF/CRLF]: ";
const char fw[] PROGMEM         = "= firmware revision: ";
const char prompt_str[] PROGMEM = "reader > ";
const char pardon[] PROGMEM     = "Pardon ? \n";

const char xmodem_start[] PROGMEM = "activate xmodem to save a file ";
const char xmodem_0[] PROGMEM = " OK";
const char xmodem_1[] PROGMEM = " ERR - SYNC NACK not received"; 
const char xmodem_2[] PROGMEM = " ERR - DATA BLOCK ACK not received"; 
const char xmodem_3[] PROGMEM = " ERR - EOT ACK not received"; 

const char *const xmodem[] PROGMEM = {
    xmodem_0, xmodem_1, xmodem_2, xmodem_3
};

const char bd_0[] PROGMEM = "= setting communication speed to ";
const char bd_1[] PROGMEM = " Bd = sending char 'U' till <NL> is received =";

const char bsum_0[]   PROGMEM = "= block checksum";
const char bsum_add[] PROGMEM = " = add: ";
const char bsum_xor[] PROGMEM = " = xor: ";
const char bsum_and[] PROGMEM = " = and: ";
const char bsum_or[]  PROGMEM = " = or: ";
const char eq_eoln[]  PROGMEM = " =\n";

const char volt_5N[]  PROGMEM = "voltage -5V Vbb=-";
const char volt_12[]  PROGMEM = "voltage 12V Vdd=";
const char volt_5P[]  PROGMEM = "voltage +5V Vcc=+";
const char volt_adc[] PROGMEM = "V adc:$";
const char volt_pin[] PROGMEM = " Vpin=";
const char volt_div[] PROGMEM = " divider=";

const char isempty_anykey[] PROGMEM = "= loop checking = each . represents 1kB = press ENTER to abort the loop =\n";
const char isempty_addr[]  PROGMEM = "= block addr: ";
const char isempty_empty[] PROGMEM = " = empty: ";
const char isempty_val[]   PROGMEM = "% = bytes[$";
const char isempty_nval[]  PROGMEM = "/!$";
const char isempty_cnt_e[] PROGMEM = "]: $";
const char isempty_cntne[] PROGMEM = " / $";
const char isempty_bit1[]  PROGMEM = " = bits[1/0]: $";
const char isempty_bit0[]  PROGMEM = " / $";

// === GLOBAl VARIABLES ===

uint16_t cntAddress  = 0;

// global rx/tx buffers
char RX_BUFFER[RX_BUFF_SIZE] = "";
char TX_BUFFER[TX_BUFF_SIZE] = "";

// === setup ===

// the setup function runs once when you press reset or power the board
void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    //pinMode(LED_BUILTIN, OUTPUT);
    // pins
    pinMode(IC4040_CLK, OUTPUT); address_inc();
    pinMode(IC4040_RST, OUTPUT); address_0();
    pinMode(MEM_RD_OE,  OUTPUT); digitalWrite(MEM_RD_OE, HIGH);
    // analog
    analogReference(INTERNAL);  // 1.1-1.2 V
    //analogReference(DEFAULT);   // Vcc=5V
    // 9600 Bd
    Serial.begin(9600);
    Serial.setTimeout(5000);
    // autodetect looks useless - disabled
    // initiate autodetec EOL if EOL is auto (empty) 
    //if (! strlen(EOL))
    //    autodetect_eol();
    //
    //tx_eol();
    prompt();
    // dummy read analog pins (due to doc the first few readings are off)
    for (uint8_t i=0; i<3; i++) {
        analogRead(V5P);
        analogRead(V5N);
        analogRead(V12);
    }
}

// tx string stored in program memory at address
void tx_pgm_arr(const char * const *src) {
    //
    uint8_t len = strlcpy_P(TX_BUFFER, (char *)pgm_read_word(src), TX_BUFF_SIZE);
    adjust_tx_buffer_eol(len);
    Serial.print(TX_BUFFER);
}

void cmd_help() {
    for(uint8_t idx=0; idx<35; idx++)
        tx_pgm_arr(&(usage[idx]));
}

void tx_pgm_txt(const char *src) {
    //
    uint8_t len = strlcpy_P(TX_BUFFER, src, TX_BUFF_SIZE-2);
    adjust_tx_buffer_eol(len);
    Serial.print(TX_BUFFER);
}

void adjust_tx_buffer_eol(uint8_t idx) {
    // nothing to adjust wthout \n
    if (TX_BUFFER[idx-1] != '\n')
        return;
    // copy EOL
    strcpy(TX_BUFFER + idx-1, EOL);
}

void prompt() {
    tx_pgm_txt(prompt_str);
}

void unknown_command() {
    tx_pgm_txt(pardon);
}

void cmd_ver() {
    tx_pgm_txt(fw);
    tx_pgm_txt(version);
    tx_pgm_txt(eq_eoln);
}

void address_0() {
    digitalWrite(IC4040_RST, HIGH);
    cntAddress = 0;
    digitalWrite(IC4040_RST, LOW);
}

void address_inc() {
    digitalWrite(IC4040_CLK, HIGH);
    cntAddress++;
    digitalWrite(IC4040_CLK, LOW);
}

void address_set(uint16_t addr) {
    address_0();
    for(uint16_t a = 0; a < addr; a++)
        address_inc();
}

float tx_voltage(int pin, float divider, float offset) {
    uint16_t adc = analogRead(pin);
    float pin_voltage = adc * Vref / 1024 + offset;
    float voltage = pin_voltage * divider;
    Serial.print(voltage, 3);
    tx_pgm_txt(volt_adc);
    tx_hexa_word(adc);
    tx_pgm_txt(volt_pin);
    Serial.print(pin_voltage);
    tx_pgm_txt(volt_div);
    Serial.print(divider, 3);
    //
    return voltage;
}

void voltages() {
    tx_pgm_txt(volt_5P);
    float Pos5V = tx_voltage(V5P, V5Pdivider, 0);
    tx_eol();
    tx_pgm_txt(volt_12);
    tx_voltage(V12, V12divider, 0);
    tx_eol();
    tx_pgm_txt(volt_5N);
    tx_voltage(V5N, V5Ndivider, -1 * Pos5V * V5Nratio);
    tx_eol();
}

char nibble_to_hexa(uint8_t nibble) {
    nibble &= 0xF;
    return nibble < 10 ? nibble + '0' : nibble - 10 + 'a';
}

void tx_hexa_byte(uint8_t data) {
    // high nibble
    Serial.print(nibble_to_hexa(data >> 4));
    // low nibble
    Serial.print(nibble_to_hexa(data));
}

void tx_hexa_word(uint16_t data) {
    // high byte
    tx_hexa_byte(data >> 8);
    // low  byte
    tx_hexa_byte(data & 0xFF);
}

void tx_hexa_3bytes(uint32_t data) {
    // high byte
    tx_hexa_byte(data >> 16);
    // low  byte
    tx_hexa_word(data & 0xFFFF);
}

void tx_hexa_dword(uint32_t data) {
    // high word
    tx_hexa_word(data >> 16);
    // low  byte
    tx_hexa_word(data & 0xFFFF);
}

// combine all bits to data byte (hw dependent)
// can combine any hw connection
uint8_t read_data_anyhw() {
    uint8_t data = 0;

    // /RD active
    digitalWrite(MEM_RD_OE, LOW);
    delay250ns;
    for(uint8_t bit = 0; bit < 8; bit++) {
        data <<= 1;
        if (digitalRead(MEM_DATA_BUS[bit]) == HIGH)
            data |= 1;
    }
    // RD inactive
    digitalWrite(MEM_RD_OE, HIGH);
    return data;
}

// combine all bits to data byte (hw dependent)
// specific hw only
uint8_t read_data() {
    uint8_t data = 0;

    // /RD active
    digitalWrite(MEM_RD_OE, LOW);
    delay250ns;
    // combine data
    data = (PIND & 0xFC) | (PINC & 0x03);
    // RD inactive
    digitalWrite(MEM_RD_OE, HIGH);
    return data;
}

void tx_address() {
    // address
    Serial.write("$");
    tx_hexa_word(cntAddress);
}

uint8_t do_rd() {
    uint8_t data;

    // address
    tx_address();
    // data
    data = read_data();
    tx_hexa_byte(data);
    tx_eol();
    //
    return data;
}

void do_dump(uint16_t size) {
    uint8_t  sumAdd = 0, sumXor = 0, sumOr = 0, sumAnd = 0xff;
    // loop
    for(uint16_t idx=0; idx<size; idx++) {
        // arrdess each 16 bytes
        if (idx % 16 == 0) {
            tx_eol();
            tx_address();
            // separator
            Serial.print(":\t");
        // add separator each 8 bytes
        } else if (idx % 8 == 0)
            Serial.print("| ");
        // data
        uint8_t data = read_data();
        tx_hexa_byte(data);
        // sums
        sumAdd += data;
        sumXor ^= data;
        sumOr  |= data;
        sumAnd &= data;
        // next address
        address_inc();
        Serial.print(" ");
    }
    // checksums
    tx_eol();
    tx_pgm_txt(bsum_0);
    tx_pgm_txt(bsum_add); tx_hexa_byte(sumAdd);
    tx_pgm_txt(bsum_xor); tx_hexa_byte(sumXor);
    tx_pgm_txt(bsum_and); tx_hexa_byte(sumAnd);
    tx_pgm_txt(bsum_or);  tx_hexa_byte(sumOr);
    tx_pgm_txt(eq_eoln);
}

// <SOH><blk #><255-blk #><--128 data bytes--><cksum>
void rd_xmdm_block(uint8_t block_num, uint16_t block_size, char *buffer, uint16_t padidx) {
    uint8_t data_sum = 0;

    // SOH
    *buffer++ = control.SOH;
    // num
    *buffer++ = block_num;
    // 1's complement of num
    *buffer++ = ~block_num;
    // payload
    for(uint16_t idx=0; idx<block_size; idx++) {
        // optional pad EOT
        uint8_t data = (idx >= padidx) ? control.PAD : read_data();
        *buffer++ = data;
        // sum
        data_sum += data;
        // next address
        address_inc();
    }
    // checksum
    *buffer = data_sum;
}

// receive single character with timeout [ms]
char rx_char_timeout(uint16_t ms) {
    unsigned long int timeout = millis() + ms;
    //
    do {
        if (Serial.available())
            return (char) Serial.read();
    } while (millis() < timeout);
    return '\0';
}

uint8_t send_xmodem(uint16_t size) {
    uint16_t block_size = 128;
    uint8_t retry = 16;
    uint8_t block_num = 1;
    char received;

    // wait for NAK
    for(uint8_t attempt=0; attempt<retry; attempt++) {
        received = rx_char_timeout(1000);
        if (received == control.NAK) break;
        Serial.write('.');
        delay(250);
    }
    // sync NACK not received = exit code 1
    if (received != control.NAK) return 1;

    // data block
    for(uint16_t sent=0; sent<size; sent+=block_size) {
        // read to buffer (xmodem format)
        rd_xmdm_block(block_num, block_size, TX_BUFFER, size-sent);
        // send and wait ACK
        for(uint8_t attempt=0; attempt<retry; attempt++) {
            // send block from buffer
            Serial.write(TX_BUFFER, block_size+4);
            // wait for ACK
            received = rx_char_timeout(1000);
            if (received == control.ACK) break;
        }
        // ACK not received after retry attempts = exit code 2
        if (received != control.ACK) return 2;
        // next block
        block_num++;
    }

    // EOT
    Serial.write(control.EOT);
    // ACK
    for(uint8_t attempt=0; attempt<retry; attempt++) {
        // wait for ACK
        received = rx_char_timeout(1000);
        if (received == control.ACK) break;
    }
    // EOT ACK not received = exit code 3
    if (received != control.ACK) return 3;
    // ok = exit code 0
    return 0;
}

void do_xmodem(uint16_t size) {
    //
    tx_pgm_txt(xmodem_start);
    // wait until tx buffer is empty
    Serial.flush();
    delay(1000);
    uint8_t code = send_xmodem(size);
    // ttx back status message from error code
    tx_pgm_arr(&(xmodem[code & 0x3]));
    tx_eol();
}

bool chip_is_empty(uint16_t size, uint8_t mask) {
    uint16_t cnt_empty = 0, cnt_nonempty = 0;
    uint32_t cnt_1 = 0, cnt_0 = 0; 
    //
    // range
    tx_pgm_txt(isempty_addr);
    tx_address();
    // loop
    for(uint16_t idx=0; idx<size; idx++) {        
        // progress each 1k
        if (! (idx & 0x3ff))
            Serial.write('.');
        // data
        uint8_t data = read_data();
        // coun match/mismatch
        if (data == mask)
            cnt_empty++;
        else
            cnt_nonempty++;
        // count 1's
        for(uint8_t bit=0; bit<8; bit++) {
            if ((data & 0x01) == 0x01)
                cnt_1++;
            else
                cnt_0++;
            data >>= 1;
        }
        // next address
        address_inc();
    }
    // calc aux values
    float percent = 100 * float(cnt_empty) / float(size);
    //cnt_nonempty = size - cnt_empty;
    //cnt_0 = (size << 8) - cnt_1;
    // result
    //tx_address(); // shows addr+1
    Serial.write('$'); tx_hexa_word(cntAddress-1);
    tx_pgm_txt(isempty_empty);
    Serial.print(percent);
    tx_pgm_txt(isempty_val);  tx_hexa_byte(mask);
    tx_pgm_txt(isempty_nval); tx_hexa_byte(mask);
    tx_pgm_txt(isempty_cnt_e); tx_hexa_word(cnt_empty);
    tx_pgm_txt(isempty_cntne); tx_hexa_word(cnt_nonempty);
    tx_pgm_txt(isempty_bit1);  tx_hexa_3bytes(cnt_1);
    tx_pgm_txt(isempty_bit0);  tx_hexa_3bytes(cnt_0);
    tx_pgm_txt(eq_eoln);    
    // 
    return cnt_empty == size;
}

void loop_chip_is_empty(uint16_t size, uint8_t mask) {
    // save current address
    uint16_t start_addr = cntAddress, cnt = 0;
    //
    tx_pgm_txt(isempty_anykey);
    // any char will stop loop
    while (! enter_received()) {
        Serial.print(cnt++);
        Serial.write(' ');
        chip_is_empty(size, mask);
        address_set(start_addr);
        // small delay
        delay(1000);
    };            
}

uint16_t parse_hexa(char *ascii) {
    uint16_t num  =  0;
    uint8_t  base = 10;
    uint8_t  shl  =  0; // shift left

    while (*ascii) {
        // trim leading spaces
        if (*ascii == ' ') {
            ascii++;
            continue;
        }
        // hexa prefix ?
        if (*ascii == '$') {
            base = 16;
            ascii++;
            continue;
        }
        // memory prefix ?
        if (*ascii == 'm') {
            shl = 7;
            ascii++;
            continue;
        }
        // size suffix
        if (*ascii == 'k') {
            shl = 10;
            ascii++;
            break;
        }
        // break on non-hexa char
        if (*ascii < '0' || *ascii > 'f' || (*ascii > '9' && *ascii < 'a'))
            break;
        // current digit value 0..15
        uint8_t digit = *ascii >= 'a' ? *ascii - 'a' + 10 : *ascii - '0';
        // rotate left and add
        num = base*num + digit;
        // next char in string
        ascii++;
    }
    if (shl) num <<= shl;
    return num;
}

void rx_discard_whitespaces() {
    while (Serial.available()) {
        int chr = Serial.peek();
        if ((char) chr != '\n' && (char) chr != '\r' 
         && (char) chr != '\t' && (char) chr != ' ')
            break;
        chr = Serial.read();
    }
}

void tx_echo_char(int chr) {
    //
    switch (ECHO_MODE) {
        case echoON:
            Serial.print((char) chr);
            break;
        case echoDEC:
            Serial.print("[");
            Serial.print(chr, DEC);
            Serial.print("]");
            break;
        case echoHEX:
            Serial.print("[$");
            Serial.print(chr, HEX);
            Serial.print("]");
            break;
        default:
            break;
    }
}

// tx end-of-line
void tx_eol() {
    //
    Serial.write(EOL);
}

void tx_block(uint16_t size, uint8_t *buffer) {
    for(uint16_t idx=0; idx<size; idx++)
        tx_echo_char(*buffer++);
    /*
    // if debug mode
    if (ECHO_MODE == echoHEX || ECHO_MODE == echoDEC) {
        for(uint16_t idx=0; idx<size; idx++)
            tx_echo_char(*buffer++);
    } else {
        Serial.write(buffer, size);
    }
    */
}

void set_echo_mode(char *mode) {
    //
    if (strcmp("on", mode) == 0)
        ECHO_MODE = echoON;
    else if (strcmp("off", mode) == 0)
        ECHO_MODE = echoOFF;
    else if (strcmp("dec", mode) == 0)
        ECHO_MODE = echoDEC;
    else if (strcmp("hex", mode) == 0)
        ECHO_MODE = echoHEX;
}

// autodetect looks useless - not used for now
void autodetect_eol() {
    unsigned long int timeout    = 0;
    unsigned long int timeout_ms = 500;
    uint8_t idx = 0;
    //    
    tx_pgm_txt(eol_detect);
    // wait for ENTER
    while (idx < 2) {
        if (Serial.available()) {
            char c = (char) Serial.read();
            if (c == '\r' || c == '\n') {
                EOL[idx] = c;
                idx++;
                EOL[idx] = '\0';
                // show
                if (c == '\r') Serial.write("<CR>");
                if (c == '\n') Serial.write("<LF>");
                // 2nd char must arrive within timeout_ms (500ms)
                timeout = millis() + timeout_ms;
            } else {
                idx = 0;
                Serial.write('.');
            }
        }
        // 2nd chr timeout - NOTE: this condition cannot be in while / compiler glitch ?!
        if (timeout > 0 && millis() > timeout) 
            break;
    }
}

void set_eol_mode(char *mode) {
    //
    if (strcmp("cr", mode) == 0)
        strcpy(EOL, "\r");
    else if (strcmp("lr", mode) == 0)
        strcpy(EOL, "\n");
    else if (strcmp("crlf", mode) == 0)
        strcpy(EOL, "\r\n");
    else if (strcmp("lfcr", mode) == 0)
        strcpy(EOL, "\n\r");
    //else if (strcmp("auto", mode) == 0)
    //    autodetect_eol();
}

// true if \r or \n received
bool enter_received() {
    // read incoming chars
    while (Serial.available()) {
        int chr = Serial.read();
        if ((char) chr == '\n' || (char) chr == '\r')
            return true;
    }
    return false;
}

void set_bd_speed(uint32_t speed) {
    //
    tx_eol();
    tx_pgm_txt(bd_0);
    Serial.print(speed);
    tx_pgm_txt(bd_1);
    Serial.flush();
    Serial.end();
    delay(1000);
    Serial.begin(speed);
    // echo U until \n or \r receiver
    while (! enter_received()) {
        Serial.print('U');
        delay(1000);
    }
}

// block until line from pc is received
char *rx_line_until_eoln_for_autodetect() {
    uint8_t idx = 0;
    //
    // discard leading white chars
    //rx_discard_whitespaces();
    //
    while (1) {
        if (Serial.available() > 0) {
            int chr = Serial.read();
            // local echo
            tx_echo_char(chr);
            // received char to buffer
            RX_BUFFER[idx] = (char) chr;
            idx++;
            RX_BUFFER[idx] = '\0';
            // line completed when '\n' or 'r' received
            if (strends(EOL, RX_BUFFER)) {
                // cut EOL part in buffer
                RX_BUFFER[idx - strlen(EOL)] = '\0';
                break;
            }
        }
    }
    // received string
    return RX_BUFFER;
}

char *rx_line_until_eoln() {
    uint8_t idx = 0;
    //
    // discard leading white chars
    //rx_discard_whitespaces();
    //
    while (1) {
        if (Serial.available() > 0) {
            int chr = Serial.read();
            // line completed when '\n' or 'r' received
            if ((char) chr == '\r' || (char) chr == '\n') {
                // ignore \r ]n at the beginning of line
                if (idx == 0)
                    continue;
                // local echo of eol
                tx_eol();
                // cur \r \n in buffer, leave clean string
                RX_BUFFER[idx] = '\0';
                break;
            }
            // local echo
            tx_echo_char(chr);
            // received char to buffer
            RX_BUFFER[idx] = (char) chr;
            idx++;
        }
    }
    // received string
    return RX_BUFFER;
}

// str starts with prefix
bool strstarts(const char *prefix, const char *str)
{
    uint8_t lenpre = strlen(prefix),
            lenstr = strlen(str);
    return lenstr < lenpre ? false : memcmp(prefix, str, lenpre) == 0;
}

// str ends with suffix
bool strends(const char *suffix, const char *str)
{
    uint8_t lensuf = strlen(suffix),
            lenstr = strlen(str);
    return lenstr < lensuf ? false : memcmp(suffix, str+lenstr-lensuf, lensuf) == 0;
}

void tx_str(char * str) {
    while (*str) {
        tx_hexa_byte(*str);
        Serial.write('-');
        str++;
    }
    Serial.write("00 ");
}

// the loop function runs over and over again forever
void loop() {
    // receive 
    if (Serial.available() > 0) {
        char *cmd = rx_line_until_eoln();
        // dispatch commands
        // ?
        if (strcmp("?", cmd) == 0)
            cmd_help();
        // version
        else if (strcmp("ver", cmd) == 0)
            cmd_ver();      
        // reset pulse
        else if (strcmp("rst", cmd) == 0)
            address_0();      
        // clk pulse
        else if (strcmp("inc", cmd) == 0)
            address_inc();      
        // voltages
        else if (strcmp("v?", cmd) == 0)
            voltages();      
        // echo mode
        else if (strstarts("echo", cmd)) {
            char *mode = &cmd[5];
            set_echo_mode(mode);
        // eol mode
        } else if (strstarts("eol", cmd)) {
            char *mode = &cmd[4];
            set_eol_mode(mode);
        // show address counter
        } else if (strcmp("addr?", cmd) == 0) {
            tx_address();
            tx_eol();
        // set address counter
        } else if (strstarts("addr", cmd)) {
            uint16_t addr = parse_hexa(&cmd[5]);
            address_set(addr);
        // set baud rate
        } else if (strstarts("bd", cmd)) {
            uint16_t speed = parse_hexa(&cmd[3]);
            set_bd_speed(speed);
        // read
        } else if (strstarts("rd", cmd)) {
            do_rd();
            // optional increment
            if (strcmp("++", &cmd[2]) == 0)
                address_inc();
        // dump
        } else if (strstarts("dump", cmd)) {
            uint16_t size = parse_hexa(&cmd[5]);
            // default size is 16
            if (size == 0) size = 16;
            do_dump(size);
        // xmodem
        } else if (strstarts("xmdm", cmd)) {
            uint16_t size = parse_hexa(&cmd[5]);
            do_xmodem(size);
        // is 0xFF
        } else if (strstarts("isff", cmd)) {
            uint16_t size = parse_hexa(&cmd[5]);
            chip_is_empty(size, 0xff);
        // is 0x00
        } else if (strstarts("isff", cmd)) {
            uint16_t size = parse_hexa(&cmd[5]);
            chip_is_empty(size, 0x00);
        // loop checking 0xff percentage
        } else if (strstarts("@ff", cmd)) {
            uint16_t size = parse_hexa(&cmd[4]);
            loop_chip_is_empty(size, 0xff);
        // loop checking 0x00 percentage
        } else if (strstarts("@00", cmd)) {
            uint16_t size = parse_hexa(&cmd[4]);
            loop_chip_is_empty(size, 0x00);
        // unknown command
        } else
            unknown_command();

        prompt();
    }
    
}
