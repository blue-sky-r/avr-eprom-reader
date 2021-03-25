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
//      C_ref      Aref    |         | 10 ~PB2 SS     
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

const char version[] PROGMEM = "2021.03.25";

const char usage_00[] PROGMEM = "available commands: \n";
const char usage_01[] PROGMEM = "?         ... this usage help \n";
const char usage_02[] PROGMEM = "ver       ... show firmware version \n";
const char usage_03[] PROGMEM = "echo mode ... set echo mode (off, on, dec, hex) \n";
const char usage_04[] PROGMEM = "     off  ... do not echo received chars \n";
const char usage_05[] PROGMEM = "     on   ... echo back each received char (default) \n";
const char usage_06[] PROGMEM = "     dec  ... echo decimal code of each received char (for debug) \n";
const char usage_07[] PROGMEM = "     hex  ... echo hexadec code of each received char (for debug) \n";
const char usage_08[] PROGMEM =  "bd speed  ... set serial communication speed (default 9600) \n";
const char usage_09[] PROGMEM =  "rst       ... reset address counter (set address to 0) \n";
const char usage_10[] PROGMEM =  "inc       ... increment address counter (next address) \n";
const char usage_11[] PROGMEM =  "v?        ... measure all voltages Vcc=+5V, Vbb=-5V, Vdd=+12V \n";
const char usage_12[] PROGMEM =  "rd        ... read actual address \n";
const char usage_13[] PROGMEM =  "rd++      ... read and increment address \n";
const char usage_14[] PROGMEM =  "addr adr  ... set address counter to adr (next read will start here) \n";
const char usage_15[] PROGMEM =  "addr?     ... show actual address counter \n";
const char usage_16[] PROGMEM =  "dump      ... dump block 16 bytes \n";
const char usage_17[] PROGMEM =  "dump size ... dump block size (16 bytes per line) \n";
const char usage_18[] PROGMEM =  "xmdm size ... read block size and send via xmodem protocol to PC \n";
const char usage_19[] PROGMEM =  "isff size ... check if entire block size is 0xFF (EPROM empty and ready for prog) \n";
const char usage_20[] PROGMEM =  "ff%  size ... loop checking block size and display percentage of 0xFF cells \n";
const char usage_21[] PROGMEM =  "\n";
const char usage_22[] PROGMEM =  "numbers (size, address, speed) can be entered as: \n";
const char usage_23[] PROGMEM =  "    12345 ... decimal number     (max 65535) \n";
const char usage_24[] PROGMEM =  "    $1234 ... hexadecimal number (max $ffff) \n";
const char usage_25[] PROGMEM =  "    123k  ... decimal number in kilo-bytes (x1024, max  64k) \n";
const char usage_26[] PROGMEM =  "    $12k  ... hexadecimal nr in kilo-bytes (x1024, max $40k) \n";
const char usage_27[] PROGMEM =  "    m64   ... memory chip capacity [kb], for 2764 use m64 (max m512) \n";

const char *const usage[] PROGMEM = {
    usage_00, usage_01, usage_02, usage_03, usage_04, usage_05, usage_06, usage_07, usage_08, usage_09,
    usage_10, usage_11, usage_12, usage_13, usage_14, usage_15, usage_16, usage_17, usage_18, usage_19,
    usage_20, usage_21, usage_22, usage_23, usage_24, usage_25, usage_26, usage_27 //, usage_28, usage_29,
};

#define RX_BUFF_SIZE  16
#define TX_BUFF_SIZE 132    // SOH + # + #'1 + 128xDATA + SUM

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
#define Rdivider(r1, r2) (r1 + r2) / float(r2)
const float V5Pdivider = Rdivider(47, 10);
const float V5Ndivider = Rdivider(2*47, 2*10);
const float V12divider = Rdivider(120, 10);
// reference voltage (measured on Aref pin)
const float Vref = 1.1;

// local echo mode
enum { echoOFF, echoON, echoDEC, echoHEX } ECHO_MODE = echoON;

// sequencing: -5 -> +5 -> +12

// xmodem control codes
const struct {
    uint8_t SOH = 0x01;
    uint8_t EOT = 0x04;
    uint8_t ACK = 0x06;
    uint8_t NAK = 0x15;
    uint8_t PAD = 0x1a;
} control;

const char fw[] PROGMEM         = "= firmware revision: ";
const char prompt_str[] PROGMEM = "reader > ";
const char pardon[] PROGMEM     = "Pardon ?\n";

const char xmodem_start[] PROGMEM = "activate xmodem to save a file ";
const char xmodem_0[] PROGMEM = "OK";
const char xmodem_1[] PROGMEM = "SYNC NACK not received"; 
const char xmodem_2[] PROGMEM = "DATA BLOCK ACK not received"; 
const char xmodem_3[] PROGMEM = "EOT ACK not received"; 

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
const char volt_div[] PROGMEM = " divider=";

const char isff_0[]  PROGMEM = "= block check";
const char isff_ff[] PROGMEM = " = $ff (empty): $";
const char isff_xx[] PROGMEM = " = !$ff (programmed): $";

uint16_t cntAddress  = 0;

// global rx/tx buffers
char RX_BUFFER[RX_BUFF_SIZE] = "";
char TX_BUFFER[TX_BUFF_SIZE] = "";

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
    //
    prompt();
}

// tx string stored in program memory at address
void tx_pgm_arr(const char * const *src) {
    //
    strlcpy_P(TX_BUFFER, (char *)pgm_read_word(src), TX_BUFF_SIZE);
    Serial.print(TX_BUFFER);
}

void cmd_help() {
    for(uint8_t idx=0; idx<26; idx++)
        tx_pgm_arr(&(usage[idx]));
}

void tx_pgm_txt(const char *src) {
    //
    strlcpy_P(TX_BUFFER, src, TX_BUFF_SIZE);
    Serial.print(TX_BUFFER);
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

void tx_voltage(int pin, float divider) {
    uint16_t adc = analogRead(pin);
    float voltage = adc * Vref * divider / 1024;
    Serial.print(voltage, 3);
    tx_pgm_txt(volt_adc);
    tx_hexa_word(adc);
    tx_pgm_txt(volt_div);
    Serial.print(divider, 3);
}

void voltages() {
    tx_pgm_txt(volt_5N);
    tx_voltage(V5N, V5Ndivider);
    Serial.println();
    tx_pgm_txt(volt_12);
    tx_voltage(V12, V12divider);
    Serial.println();
    tx_pgm_txt(volt_5P);
    tx_voltage(V5P, V5Pdivider);
    Serial.println();
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
    tx_hexa_byte(data);
}

// combine all bits to data byte (hw dependent)
uint8_t read_data() {
    uint8_t data = 0;

    // /RD active
    digitalWrite(MEM_RD_OE, LOW);
    for(uint8_t bit = 0; bit < 8; bit++) {
        if (digitalRead(MEM_DATA_BUS[bit]) == HIGH)
            data |= 1;
        data <<= 1;
    }
    // RD inactive
    digitalWrite(MEM_RD_OE, HIGH);
    return data;
}

void tx_address() {
    // address
    Serial.println();
    Serial.print("$");
    tx_hexa_word(cntAddress);
    // separator
    Serial.print(":\t");
}

uint8_t do_rd() {
    uint8_t data;

    // address
    tx_address();
    // data
    data = read_data();
    tx_hexa_byte(data);
    Serial.println();
    //
    return data;
}

void do_dump(uint16_t size) {
    uint8_t  sumAdd = 0, sumXor = 0, sumOr = 0, sumAnd = 0;
    // loop
    for(uint16_t idx=0; idx<size; idx++) {
        // arrdess each 16 bytes
        if (idx % 16 == 0) {
            //if (idx > 0) Serial.println();
            tx_address();
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
    Serial.println();
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

uint8_t do_xmodem(uint16_t size) {
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

uint8_t do_isff(uint16_t size) {
    uint16_t  cntFF = 0, cntXX = 0;
    // loop
    for(uint16_t idx=0; idx<size; idx++) {
        // data
        uint8_t data = read_data();
        // sums
        if (data == 0xff)
            cntFF++;
        else
            cntXX++;
        // next address
        address_inc();
    }
    // result
    tx_pgm_txt(isff_0);
    tx_pgm_txt(isff_ff); tx_hexa_word(cntFF);
    tx_pgm_txt(isff_xx); tx_hexa_word(cntXX);
    tx_pgm_txt(eq_eoln);    
}

uint16_t parse_hexa(char *ascii) {
    uint16_t num  =  0;
    uint8_t  base = 10;
    uint8_t  mult = 1;

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
            mult = 128;
            ascii++;
            continue;
        }
        // size suffix
        if (*ascii == 'k') {
            num <<= 10;
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
    num *= mult;
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

// block until line from pc is received
char *rx_line_until_eoln() {
    uint8_t idx = 0;
    //
    // discard leading white chars
    rx_discard_whitespaces();
    //
    while (1) {
        if (Serial.available() > 0) {
            int chr = Serial.read();
            // local echo
            tx_echo_char(chr);
            // line completed when '\n' or 'r' received
            if (chr == '\n' || chr == '\r') {
                RX_BUFFER[idx] = '\0';
                rx_discard_whitespaces();
                break;
            }
            // received char to buffer
            RX_BUFFER[idx] = (char) chr;
            idx++;
        }
    }
    // received string
    return RX_BUFFER;
}

bool strstarts(const char *prefix, const char *str)
{
    uint8_t lenpre = strlen(prefix),
            lenstr = strlen(str);
    return lenstr < lenpre ? false : memcmp(prefix, str, lenpre) == 0;
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
            if (strcmp("on", mode) == 0)
                ECHO_MODE = echoON;
            else if (strcmp("off", mode) == 0)
                ECHO_MODE = echoOFF;
            else if (strcmp("dec", mode) == 0)
                ECHO_MODE = echoDEC;
            else if (strcmp("hex", mode) == 0)
                ECHO_MODE = echoHEX;
        // show address counter
        } else if (strcmp("addr?", cmd) == 0) {
            tx_address();
            Serial.println();
        // set address counter
        } else if (strstarts("addr", cmd)) {
            uint16_t addr = parse_hexa(&cmd[5]);
            address_0();
            for(uint16_t a = 0; a < addr; a++)
                address_inc();
        // set baud rate
        } else if (strstarts("bd", cmd)) {
            uint16_t speed = parse_hexa(&cmd[3]);
            Serial.println();
            tx_pgm_txt(bd_0);
            Serial.print(speed);
            tx_pgm_txt(bd_1);
            Serial.flush();
            Serial.end();
            Serial.begin(speed);
            while (1) {
                delay(1000);
                Serial.print('U');
                if (Serial.available()) {
                    int chr = Serial.read();
                    if ((char) chr == '\n' || (char) chr == '\r')
                        break;
                }
            }
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
            tx_pgm_txt(xmodem_start);
            // wait until tx buffer is empty
            Serial.flush();
            delay(1000);
            uint8_t code = do_xmodem(size);
            Serial.write(' ');
            // ttx back status message from error code
            tx_pgm_arr(&(xmodem[code & 0x3]));
        // is 0xFF
        } else if (strstarts("isff", cmd)) {
            uint16_t size = parse_hexa(&cmd[5]);
            do_isff(size);
        } else
            unknown_command();

        prompt();
    }
    
}
