/*
  AVR-EPROM-READ

 just a proof-of-concept for hw/sw design

 has to be implemented in pure C + assembly later

*/

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

const char version[] = "2021.03.23";

const uint8_t RX_BUFF_SIZE = 10;

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

uint16_t cntAddress  = 0;

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

void prompt() {
    //Serial.println();
    Serial.print("reader > ");
}

void unknown_command() {
    //Serial.println();
    Serial.println("Pardon ?");
}

void cmd_ver() {
    //Serial.println();
    Serial.print("= firmware revision: ");
    Serial.print(version);
    Serial.println(" =");
}

void cmd_help() {
    //Serial.println();
    Serial.println("available commands:");
    Serial.println("?         ... this usage help");
    Serial.println("ver       ... show internal firmware version");
    Serial.println("echo mode ... set echo mode (off, on, dec, hex)");
    Serial.println("     off  ... do not echo received chars");
    Serial.println("     on   ... echo back each received char (default)");
    Serial.println("     dec  ... echo decimal code of each received char (for debug)");
    Serial.println("     hex  ... echo hexadec code of each received char (for debug)");
    Serial.println("bd speed  ... set serial communication speed (default 9600)");
    Serial.println("rst       ... reset address counter (set address to 0)");
    Serial.println("inc       ... increment address counter (next address)");
    Serial.println("v?        ... measure all voltages Vcc=+5V, Vbb=-5V, Vdd=+12V");
    Serial.println("rd        ... read actual address");
    Serial.println("rd++      ... read and increment address");
    Serial.println("addr adr  ... set address counter to adr (next rd or dump will start from adr)");
    Serial.println("dump      ... dump block 16 bytes");
    Serial.println("dump size ... dump block size (16 bytes per line)");
    Serial.println("xmdm size ... read block size and transfer it via xmodem protocol to PC (xmodem-read)");
    Serial.println();
    Serial.println("numbers (size, address, speed) can be entered as:");    
    Serial.println("    12345 ... decimal number     (max 65535)");
    Serial.println("    $1234 ... hexadecimal number (max $ffff)");
    Serial.println("    123k  ... decimal number in kilo-bytes (x1024, max  64k)");
    Serial.println("    $12k  ... hexadecimal nr in kilo-bytes (x1024, max $40k)");
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
    Serial.print("V adc:$");
    tx_hexa_word(adc);
    Serial.print(" divider=");
    Serial.print(divider, 3);
}

void voltages() {
    Serial.print("voltage -5V Vbb=-");
    tx_voltage(V5N, V5Ndivider);
    Serial.println();
    Serial.print("voltage 12V Vdd=");
    tx_voltage(V12, V12divider);
    Serial.println();
    Serial.print("voltage +5V Vcc=+");
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
            data += 1;
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
    Serial.print("= block checksum");
    Serial.print(" = add: "); tx_hexa_byte(sumAdd);
    Serial.print(" = xor: "); tx_hexa_byte(sumXor);
    Serial.print(" = and: "); tx_hexa_byte(sumAnd);
    Serial.print(" = or: ");  tx_hexa_byte(sumOr);
    Serial.println(" =");
}

const struct {
    uint8_t SOH = 0x01;
    uint8_t EOT = 0x04;
    uint8_t ACK = 0x06;
    uint8_t NAK = 0x15;
    uint8_t PAD = 0x1a;
} control;

// <SOH><blk #><255-blk #><--128 data bytes--><cksum>
void rd_xmdm_block(uint8_t block_num, uint16_t block_size, uint8_t *buffer, uint16_t padidx) {
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
        uint8_t data = (padidx>=idx) ? control.PAD : read_data();
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
    } while (millis() >= timeout);
    return '\0';
}

int8_t do_xmodem(uint16_t size) {
    uint16_t block_size = 128;
    // buffer size >= block_size + 4 
    uint8_t buffer[132];
    uint8_t retry = 16;
    uint8_t block_num = 1;
    char received;

    // wait for NAK
    for(uint8_t attempt=0; attempt<retry; attempt++) {
        received = rx_char_timeout(1000);
        if (received == control.NAK) break;
    }
    // sync NACK not received = exit code -1
    if (received != control.NAK) return -1;
    
    // data block
    for(uint16_t sent=0; sent<size; sent+=block_size) {
        // read to buffer (xmodem format)
        rd_xmdm_block(block_num, block_size, buffer, size-sent);
        // send and wait ACK
        for(uint8_t attempt=0; attempt<retry; attempt++) {
            // for debug

            Serial.write(buffer, block_size+4);
            // wait for ACK
            received = rx_char_timeout(1000);
            if (received == control.ACK) break;
        }
        // ACK not received after retry attempts = exit code -2
        if (received != control.ACK) return -2;
    }

    // EOT
    Serial.write(control.EOT);
    // ACK
    for(uint8_t attempt=0; attempt<retry; attempt++) {
        // wait for ACK
        received = rx_char_timeout(1000);
        if (received == control.ACK) break;
    }
    // EOT ACK not received = exit code -3
    if (received != control.ACK) return -3;
    // ok = exit code 0
    return 0;
}

uint16_t parse_hexa(char *ascii) {
    uint16_t num  =  0;
    uint8_t  base = 10;

    while(*ascii) {
        // trim leading spaces
        if (*ascii == ' ') 
            continue;
        // hexa prefix ?
        if (*ascii == '$') {
            base = 16;
            continue;
        }
        // size suffix
        if (*ascii == 'k') {
            num <<= 10;
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
    static char rx_string[RX_BUFF_SIZE];
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
                rx_string[idx] = '\0';
                rx_discard_whitespaces();
                break;
            }
            // received char to buffer
            rx_string[idx] = (char) chr;
            idx++;
        }
    }
    // received string
    return rx_string;
}

// the loop function runs over and over again forever
void loop() {
    // command from PC
    //String command = "";
    //command.reserve(RX_BUFF_SIZE);
    // receive 
    if (Serial.available() > 0) {
        char *cmd = rx_line_until_eoln();
        String command;
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
        else if (strcmp("echo", cmd) == 0) {
            char *mode = &cmd[5];
            if (strcmp("on", mode) == 0)
                ECHO_MODE = echoON;
            else if (strcmp("off", mode) == 0)
                ECHO_MODE = echoOFF;
            else if (strcmp("dec", mode) == 0)
                ECHO_MODE = echoDEC;
            else if (strcmp("hex", mode) == 0)
                ECHO_MODE = echoHEX;
        // set address counter
        } else if (strcmp("addr", cmd) == 0) {
            uint16_t addr = parse_hexa(&cmd[5]);
            address_0();
            for(uint16_t a = 0; a < addr; a++)
                address_inc();
        // set baud rate
        } else if (strcmp("bd", cmd) == 0) {
            uint16_t speed = parse_hexa(&cmd[3]);
            Serial.println();
            Serial.print("= setting communication speed to ");
            Serial.print(speed);
            Serial.println(" Bd = sending char 'U' till <NL> is received =");
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
        } else if (strcmp("rd", cmd) == 0) {
            do_rd();
            // optional increment
            if (strcmp("++", &cmd[2]) == 0)
                address_inc();
        // dump
        } else if (strcmp("dump", cmd) == 0) {
            uint16_t size = parse_hexa(&cmd[5]);
            // default size is 16
            if (size == 0) size = 16;
            do_dump(size);
        // xmodem
        } else if (strcmp("xmdm", cmd) == 0) {
            uint16_t size = parse_hexa(&cmd[5]);
            Serial.println("activate xmodem to save a file ...");
            // wait 1 sec
            delay(1000);
            do_xmodem(size);
        } else {
            unknown_command();
        }

        prompt();
    }
    
}
