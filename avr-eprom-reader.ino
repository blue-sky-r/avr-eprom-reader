/*
  AVR-EPROM-READ

*/

// PIN defs - arduino/hardware/arduino/avr/variants/standard/pins_arduino.h
//
//                       ARDUINO Nano
//
//                      id +-[ usb ]-+ id
//     mem_RD  SCK  PB5 13 | [     ] | 12  PB4 MISO   ic4040_clk
//          x      +3v3    |  -----  | 11 ~PB3 MOSI 
//      C_ref      Aref    |         | 10 ~PB2 SS     
//     mem_D0   A0  PC0 14 |         | 09 ~PB1        
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
// TODO: adjust formula for -5V
// TODO: power-up/down sequence
// TODO: xmodem protocol

#define RX_BUFF_SIZE    80

// HW
#define IC4040_RST PB0
// falling edge H->L
#define IC4040_CLK PB4
// RD/OE pin
#define MEM_RD_OE  PB2
// data bus
const int MEM_DATA_BUS[8] = {
    PD7,    // D7
    PD6,    // D6
    PD5,    // D5
    PD4,    // D4
    PD3,    // D3
    PD2,    // D2
    PC1,    // D1
    PC0     // D0
};
// voltage measurements
// adc pins - A0 .. A7
// pin
#define V5P     A5	//  5V Positive
#define V5N     A6	//  5V Negative
#define V12     A7	// 12V positive
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
const char version[] = "2021.03.15";

// the setup function runs once when you press reset or power the board
void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    // pins
    pinMode(IC4040_CLK, OUTPUT); //address_inc();
    pinMode(IC4040_RST, OUTPUT); //address_0();
    pinMode(MEM_RD_OE,  OUTPUT); 
    //
    digitalWrite(MEM_RD_OE, HIGH);
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
    Serial.println("dump size ... dump block size (16 bytes per line) where:");
    Serial.println();
    Serial.println("numbers (size, address, speed) can be entered as:");    
    Serial.println("    12345 ... decimal number     (max 65535)");
    Serial.println("    $1234 ... hexadecimal number (max $ffff)");
    Serial.println("    123k  ... decimal number in kilo-bytes (x1024, max  64k)");
    Serial.println("    $12k  ... hexadecimal nr in kilo-bytes (x1024, max $40k)");
}

void address_0() {
    digitalWrite(IC4040_RST, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    cntAddress = 0;
    digitalWrite(IC4040_RST, LOW);
    digitalWrite(LED_BUILTIN, LOW);
}

void address_inc() {
    digitalWrite(IC4040_CLK, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    cntAddress++;
    digitalWrite(IC4040_CLK, LOW);
    digitalWrite(LED_BUILTIN, LOW);
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

uint16_t parse_hexa(String ascii) {
    uint16_t num  =  0;
    uint8_t  base = 10;

    for(char x : ascii) {
        // trim leading spaces
        if (x == ' ') 
            continue;
        // hexa prefix ?
        if (x == '$') {
            base = 16;
            continue;
        }
        // size suffix
        if (x == 'k') {
            num <<= 10;
            break;
        }
        // break on non-hexa char
        if (x < '0' || x > 'f' || (x > '9' && x < 'a'))
            break;
        // current digit value 0..15
        uint8_t digit = x >= 'a' ? x - 'a' + 10 : x - '0';
        // rotate left and add
        num = base*num + digit;
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

// block until line from pc is received
String rx_line_until_eoln() {
    static String rx_string;
    //
    rx_string = "";
    rx_string.reserve(RX_BUFF_SIZE);
    // discard leading white chars
    rx_discard_whitespaces();
    //
    while (1) {
        if (Serial.available() > 0) {
            int chr = Serial.read();
            // local echo
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
            };
            // line completed when '\n' or 'r' received
            if (chr == '\n' || chr == '\r') {
                rx_discard_whitespaces();
                break;
            }
            // received char to buffer
            rx_string += (char) chr;
        }
    };
    rx_string.trim();
    rx_string.toLowerCase();
    return rx_string;
}

// the loop function runs over and over again forever
void loop() {
    // command from PC
    String command = "";
    // receive 
    if (Serial.available() > 0) {
        command = rx_line_until_eoln();
        //Serial.print("<" + command + ">");
        digitalWrite(LED_BUILTIN, HIGH); // turn the LED on

        // dispatch commands
        // ?
        if (command.startsWith("?"))
            cmd_help();
        // version
        else if (command.startsWith("ver"))
            cmd_ver();      
        // reset pulse
        else if (command.startsWith("rst"))
            address_0();      
        // clk pulse
        else if (command.startsWith("inc"))
            address_inc();      
        // voltages
        else if (command.startsWith("v?"))
            voltages();      
        // echo mode
        else if (command.startsWith("echo")) {
            String mode = command.substring(5);
            if (mode.startsWith("on"))
                ECHO_MODE = echoON;
            else if (mode.startsWith("off"))
                ECHO_MODE = echoOFF;
            else if (mode.startsWith("dec"))
                ECHO_MODE = echoDEC;
            else if (mode.startsWith("hex"))
                ECHO_MODE = echoHEX;
        // set address counter
        } else if (command.startsWith("addr")) {
            uint16_t addr = parse_hexa(command.substring(5));
            address_0();
            for(uint16_t a = 0; a < addr; a++)
                address_inc();
        // set baud rate
        } else if (command.startsWith("bd")) {
            uint16_t speed = parse_hexa(command.substring(3));
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
            };
        // read
        } else if (command.startsWith("rd")) {
            do_rd();
            // optional increment
            if (command.endsWith("++"))
                address_inc();
        // dump
        } else if (command.startsWith("dump")) {
            uint16_t size = parse_hexa(command.substring(5));
            // default size is 16
            if (size == 0) size = 16;
            do_dump(size);
        } else
            unknown_command();

        prompt();
    }
    
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off
}