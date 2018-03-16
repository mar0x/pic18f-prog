// I wrote it according to the following datasheet:
// http://ww1.microchip.com/downloads/en/DeviceDoc/39622L.pdf
// For PIC18F2XK20: http://ww1.microchip.com/downloads/en/DeviceDoc/41297F.pdf

//pin out config
#define PGC  6
#define PGD  7  // 5
#define PGM  8
#define MCLR 10 // 7
#define VDD  16 // 9

#define PSERIAL       Serial1
#define HI_VOLTAGE    1
#define PIC18F2XK20   1
#define ENABLE_WRITE  0  // erase and write functions not tested after rework

#define CMD_CORE_INSTRUCTION        B0000
#define CMD_SHIFT_OUT_TABLAT        B0010
#define CMD_TABLE_READ              B1000
#define CMD_TABLE_READ_POST_INC     B1001
#define CMD_TABLE_WRITE             B1100
#define CMD_TABLE_WRITE_POST_INC    B1101
#define CMD_TABLE_WRITE_START_PROG  B1111

#define ADDR_BLOCK0     0x00000800
#define ADDR_CODE_END   0x00004000
#define ADDR_ID         0x00200000
#define ADDR_CONFIG     0x00300000
#define ADDR_ERASE1     0x003C0005
#define ADDR_ERASE2     0x003C0004
#define ADDR_DEVICE_ID  0x003FFFFE
#define ADDR_EEPROM     0x00F00000

#define WRITE_BUFFER_SIZE  32
#define ERASE_BUFFER_SIZE  64

struct SerialInput {
    bool push(int c) {
        if (c == 'X') {
            complete = true;
            return true;
        }

        if (cmd == 0) {
            cmd = c;
            return false;
        }

        if (len < sizeof(buf)) {
            buf[len++] = c;
        }

        return false;
    }

    operator bool() const { return complete; }

    void reset() {
        complete = false;
        cmd = 0;
        len = 0;
    }

    bool complete = false;
    uint8_t cmd = 0;
    uint8_t buf[200];
    int len = 0;
};

SerialInput input;

void setup() {
    PSERIAL.begin(115200);

    pinMode(PGC,  OUTPUT);
    pinMode(PGD,  OUTPUT);
    pinMode(PGM,  OUTPUT);
    pinMode(MCLR, OUTPUT);
    pinMode(VDD,  OUTPUT);

#if HI_VOLTAGE
    digitalWrite(VDD,  LOW);
    digitalWrite(MCLR, LOW);
    digitalWrite(PGM,  HIGH);
#else
    digitalWrite(VDD,  HIGH);
    digitalWrite(PGM,  LOW);
    digitalWrite(MCLR, LOW);
#endif
}

void loop() {
    if (PSERIAL.available())
        mainFunction();
}

void pgc_tick() {
    digitalWrite(PGC, HIGH);
    delay_P2A();

    digitalWrite(PGC, LOW);
    delay_P2B();
}

struct ProgMode {
    ProgMode() {
#if HI_VOLTAGE
        digitalWrite(VDD, HIGH);
        delay_P13();

        digitalWrite(MCLR, HIGH);
        delay_P12();
#else
        digitalWrite(PGM, HIGH);
        delay_P15();

        digitalWrite(MCLR, HIGH);
        delay_P12();
#endif

        delay(1);
    }

    ~ProgMode() {
#if HI_VOLTAGE
        delay_P16();
        digitalWrite(MCLR, LOW);

        delay_P17();
        digitalWrite(VDD, LOW);
#else
        delay_P16();
        digitalWrite(MCLR, LOW);

        delay_P18();
        digitalWrite(PGM, LOW);
#endif
    }
};

uint32_t slomo = 0;

void delayMcs(uint32_t mcs) {
    if (mcs < slomo)
        mcs = slomo;

    if (mcs > 10000) {
        delay(mcs / 1000);
    } else {
        delayMicroseconds(mcs);
    }
}

// P2A  TPGCL  Serial Clock (PGC) Low Time (min 40 nanoseconds)
void delay_P2A() { delayMcs(1); }

// P2B  TPGCH  Serial Clock (PGC) High Time (min 40 nanoseconds)
void delay_P2B() { delayMcs(1); }

// P5   TDLY1   Delay between 4-bit Command and Command Operand (min 40 nanoseconds)
void delay_P5()  { delayMcs(1); }

// P5A  TDLY1A  Delay between 4-bit Command Operand and next 4-bit Command (min 40 nanoseconds)
void delay_P5A() { delayMcs(1); }

// P6   TDLY2   Delay between Last PGC ↓ of Command Byte to First PGC ↑ of Read of Data Word (min 20 nanoseconds)
void delay_P6()  { delayMcs(1); }

// P9   TDLY5   PGC High Time (minimum programming time) (min 1 millisecond)
void delay_P9()  { delayMcs(1000); }

// P9A  TDLY5A  PGC High Time (min 5 milliseconds)

// P10  TDLY6   PGC Low Time after Programming (high-voltage discharge time) (min 200 microseconds)
void delay_P10() { delayMcs(200); }

// P11  TDLY7   Delay to allow Self-Timed Data Write or Bulk Erase to occur (min 5 milliseconds)
void delay_P11() { delayMcs(5000); }

// P12  THLD2   Input Data Hold Time from MCLR/VPP/RE3 ↑ (min 2 microseconds)
void delay_P12() { delayMcs(2); }

// P13  TSET2   VDD ↑ Setup Time to MCLR/VPP/RE3 ↑ (min 100 nanoseconds)
void delay_P13() { delayMcs(1); }

// P14  TVALID  Data Out Valid from PGC ↑ (min 10 nanoseconds)
void delay_P14() { delayMcs(1); }

// P15  TSET3   PGM ↑ Setup Time to MCLR/VPP/RE3 ↑ (min 2 microseconds)
void delay_P15() { delayMcs(2); }

// P16  TDLY8   Delay between Last PGC ↓ and MCLR/VPP/RE3 ↓ (min 0 seconds)
void delay_P16() { delayMcs(1); }

// P17  THLD3   MCLR/VPP/RE3 ↓ to VDD ↓ (max 100 nanoseconds)
void delay_P17() { delayMcs(1); }

// P18  THLD4   MCLR/VPP/RE3 ↓ to PGM ↓ (min 0 seconds)
void delay_P18() { delayMcs(1); }


void mainFunction() {

    while (PSERIAL.available()) {
        if (input.push(PSERIAL.read())) {
            break;
        }
    }

    if (!input) {
        return;
    }

    // Say hello
    if (input.cmd == 'H') {
        delay(1);
        PSERIAL.print("H");
        input.reset();

        return;
    }

    // Get device ID
    if (input.cmd == 'D') { // read device
        delay(100);
        PSERIAL.print("K");

        {
            ProgMode pm;

            uint8_t device_id[2];

            read_code(ADDR_DEVICE_ID, device_id, 2);

            // This can be done on receiver side
            //device_id[0] = device_id[0] & 0b11100000;

            serialPrintHex(device_id[0]);
            serialPrintHex(device_id[1]);
        }

        PSERIAL.print("X");
        input.reset();

        return;
    }

    if (input.cmd == 'S') {
        if (input.len != 2 * 2) {
            PSERIAL.print("L");
            input.reset();

            return;
        }

        slomo = hex2uint16(input.buf);

        PSERIAL.print("K");
        PSERIAL.print(slomo);
        PSERIAL.print("X");
        input.reset();

        return;
    }

    if (input.cmd == 'R') { //READ

        if (input.len != 3 * 2) {
            PSERIAL.print("L");
            input.reset();

            return;
        }

        uint32_t addr = hex2uint24(input.buf);

        PSERIAL.print("K");
        //read
        PSERIAL.print("R");

        serialPrintHex((addr & 0x00FF0000) >> 16);
        serialPrintHex((addr & 0x0000FF00) >> 8);
        serialPrintHex(addr & 0x000000FF);

        {
            ProgMode pm;

            for (int i = 0; i < WRITE_BUFFER_SIZE; i++) {

                uint8_t r = 0;
                if (addr >= ADDR_EEPROM) {
                    r = read_data(addr);
                } else {
                    r = read_code(addr);
                }
                serialPrintHex(r);

                addr++;
            }
        }

        PSERIAL.println("X");
        input.reset();

        return;
    }


    if (input.cmd == 'W') {
        uint32_t addr;
        int len;
        uint8_t buf[32];
        uint8_t *start;

        switch (input.len) {
        case ((2 + 32) * 2):
            // Program: WAAAADD{32}X
            addr = hex2uint16(input.buf);
            len = 32;
            start = input.buf + 2 * 2;
            break;

        case ((3 + 32) * 2):
            // EEPROM: WAAAAAADD{32}X
            addr = hex2uint24(input.buf);
            len = 32;
            start = input.buf + 3 * 2;
            break;

        case ((3 + 8) * 2):
            // ID memory: WAAAAAADD{8}X
            addr = hex2uint24(input.buf);
            len = 8;
            start = input.buf + 3 * 2;
            break;

        default:
            PSERIAL.print("L");
            input.reset();

            return;
        }

        for (int i = 0; i < len; i++) {
            buf[i] = hex2uint8(start + 2 * i);
        }

        {
            ProgMode pm;

            if (addr < ADDR_CODE_END) {
                // Code
                write_code(addr, buf);
            }

            if (addr >= ADDR_EEPROM) {
                // EEPROM Data
                for(int i = 0; i < len; i++) {
                    if (buf[i] == 0xFF) { // Speedup, eeprom is erased, only write if bits changea
                        continue;
                    }
                    write_eeprom(addr + i, buf[i]);
                }
            }

            if (addr == ADDR_ID) {
                // ID
                write_id(buf);
            }
        }

        PSERIAL.print("K");
        input.reset();

        return;
    }


    if (input.cmd == 'E') { //Erase all

        {
            ProgMode pm;

            //erase
            erase_bulk();
            delay(1);
            erase_eeprom();
        }

        PSERIAL.print("K");
        input.reset();

        return;
    }

    if (input.cmd == 'C') { //config

        if (input.len != 3) {
            PSERIAL.print("L");
            input.reset();

            return;
        }

        {
            ProgMode pm;

            write_config(hex2uint8(input.buf[0]), hex2uint8(input.buf + 1));
        }

        PSERIAL.print("K");
        input.reset();

        return;
    }

    PSERIAL.print("L");
    input.reset();
}

byte tbl_read(byte cmd = CMD_TABLE_READ) {

    byte value = 0;

    send4bitcommand(cmd);

    for (byte i = 0; i < 8; i++) { // read
        pgc_tick();
    }

    delay_P6();

    pinMode(PGD, INPUT);
    digitalWrite(PGD, LOW);

    for (byte i = 0; i < 8; i++) { // shift out
        pgc_tick();
        if (digitalRead(PGD) == HIGH)
            value += 1 << i; // sample PGD
    }

    return value;
}


void send_bits(uint32_t data, int bits) {
    pinMode(PGD, OUTPUT);
    for (byte i = 0; i < bits; i++) {
        digitalWrite(PGD, ((1 << i) & data) ? HIGH : LOW);
        pgc_tick();
    }
}

void send4bitcommand(byte data) { send_bits(data, 4); delay_P5(); }
void send16bit(unsigned int data) { send_bits(data, 16); delay_P5A(); }
void send16bit(byte lsb, byte msb) { send_bits(lsb, 8); send_bits(msb, 8); delay_P5A(); }

void core_inst(unsigned int data) {
    send4bitcommand(CMD_CORE_INSTRUCTION);
    send16bit(data);
}

void movlw(byte addr) { core_inst(0x0e00 | addr); }
void movwf_tblptru() { core_inst(0x6ef8); }
void movwf_tblptrh() { core_inst(0x6ef7); }
void movwf_tblptrl() { core_inst(0x6ef6); }

void tblptr(byte u, byte h, byte l) {
    movlw(u);
    movwf_tblptru();

    movlw(h);
    movwf_tblptrh();

    movlw(l);
    movwf_tblptrl();
}

void tblptr(uint32_t addr) {
    movlw((addr & 0x00FF0000) >> 16);
    movwf_tblptru();

    movlw((addr & 0x0000FF00) >> 8);
    movwf_tblptrh();

    movlw(addr & 0x000000FF);
    movwf_tblptrl();
}

void movwf_eeadr()  { core_inst(0x6ea9); }
void movwf_eeadrh() { core_inst(0x6eaa); }
void movwf_eedata() { core_inst(0x6ea8); }
void movwf_tablat() { core_inst(0x6ef5); }

void eeadr(byte addr, byte addrh) {
    movlw(addr);
    movwf_eeadr();

    movlw(addrh);
    movwf_eeadrh();
}

void eeadr(uint32_t addr) {
    movlw((addr & 0x0000FF00) >> 8);
    movwf_eeadr();

    movlw(addr & 0x000000FF);
    movwf_eeadrh();
}

void nop()              { core_inst(0x0000); }

// NOP - hold PGC high for time P9 and low for time P10.
void nop_P9_P10() {
    pinMode(PGD, OUTPUT);
    digitalWrite(PGD, LOW);

    pgc_tick();

    digitalWrite(PGC, HIGH);
    delay_P9();

    digitalWrite(PGC, LOW);
    delay_P10();

    send16bit(0x0000);
}

void bsf_eecon1_eepgd() { core_inst(0x8ea6); }
void bsf_eecon1_cfgs()  { core_inst(0x8ca6); }
void bsf_eecon1_wren()  { core_inst(0x84a6); }
void bsf_eecon1_wr()    { core_inst(0x82a6); }
void bsf_eecon1_rd()    { core_inst(0x80a6); }
void bcf_eecon1_eepgd() { core_inst(0x9ea6); }
void bcf_eecon1_cfgs()  { core_inst(0x9ca6); }
void bcf_eecon1_wren()  { core_inst(0x94a6); }

void movf_eedata_w_0()  { core_inst(0x50a8); }
void movf_eecon1_w_0()  { core_inst(0x50a6); }


byte read_code(uint32_t addr) {

    tblptr(addr);

    return tbl_read(CMD_TABLE_READ);
}

void read_code(uint32_t addr, uint8_t *buf, int len) {

    tblptr(addr);

    for (int i = 0; i < len; i++) {
        buf[i] = tbl_read(CMD_TABLE_READ_POST_INC);
    }
}

byte read_data(uint32_t addr) {

    // Step 1: Direct access to data EEPROM
    bcf_eecon1_eepgd();
    bcf_eecon1_cfgs();

    // Step 2: Set the data EEPROM Address Pointer
    eeadr(addr);

    // Step 3: Initiate a memory read
    bsf_eecon1_rd();

    //Step 4: Load data into the PSERIAL Data Holding register
    movf_eedata_w_0();
    movwf_tablat();

    nop();

    return tbl_read(CMD_SHIFT_OUT_TABLAT);
}

void erase_bulk() { //for some reason the chip stops response, just reconnect voltage
#if ENABLE_WRITE
    tblptr(ADDR_ERASE1);

    send4bitcommand(CMD_TABLE_WRITE);
#if PIC18F2XK20
    send16bit(0x0F0F);
#else 
    send16bit(0x3F3F);
#endif

    tblptr(ADDR_ERASE2);

    send4bitcommand(CMD_TABLE_WRITE);
    send16bit(0x8F8F);

    nop();

    delay(2);

    nop();

    delay(2);

    digitalWrite(PGD, LOW);

    delay_P11();
    delay_P10();

    digitalWrite(PGD, HIGH);
#endif
}

void erase_eeprom() { //for some reason the chip stops response, just reconnect voltage
#if ENABLE_WRITE
    tblptr(ADDR_ERASE1);

    send4bitcommand(CMD_TABLE_WRITE);
    send16bit(0x0000); //

    tblptr(ADDR_ERASE2);

    send4bitcommand(CMD_TABLE_WRITE);
    send16bit(0x8484);

    nop();

    delay(2);

    nop();

    delay(2);

    digitalWrite(PGD, LOW);

    delay_P11();

    digitalWrite(PGD, HIGH);
#endif
}

void write_config(byte address, byte data) {
#if ENABLE_WRITE
    bsf_eecon1_eepgd();
    bsf_eecon1_cfgs();
#if PIC18F2XK20
    bsf_eecon1_wren();
#endif

    tblptr(ADDR_CONFIG + address);

    //PSERIAL.println("programming");

    send4bitcommand (CMD_TABLE_WRITE_START_PROG);
    if (address & 0x01) {
        send16bit(0, data); // if odd MSB read, LSB ignored
    } else {
        send16bit(data, 0); // if even MSB ignored, LSB read
    }

    nop_P9_P10();
#endif
}

void write_eeprom(uint32_t addr, uint8_t data) {
#if ENABLE_WRITE
    // Step 1: direct access to data EEPROM
    bcf_eecon1_eepgd();
    bcf_eecon1_cfgs();

    // Step 2: Set the data EEPROM Address Pointer
    eeadr(addr);

    // Step 3: Load the data to be written
    movlw(data);
    movwf_eedata();

    // Step 4: Enable memory writes
    bsf_eecon1_wren();

    // Step 5: Initiate write
    bsf_eecon1_wr();

    nop();
    nop();

    // Step 6: Poll WR bit, repeat until the bit is clear
    byte wr = 0x01;
    while(wr & 0x01) {
        movf_eecon1_w_0();
        movwf_tablat();

        nop();

        wr = tbl_read(CMD_SHIFT_OUT_TABLAT);

        pinMode(PGD, OUTPUT);
        digitalWrite(PGD, LOW);

        delay_P5();
    }

    // Step 7: Hold PGC low for time P10
    delay_P10();

    // Step 8: Disable writes
    bcf_eecon1_wren();

    nop_P9_P10();
#endif
}

void write_id(uint8_t *buf) {
#if ENABLE_WRITE
    //step 1
    bsf_eecon1_eepgd();
    bcf_eecon1_cfgs();
#if PIC18F2XK20
    bsf_eecon1_wren();
#endif

    //step 2
    tblptr(ADDR_ID);

    //step 3
    send4bitcommand (CMD_TABLE_WRITE_POST_INC);
    send16bit(buf[0], buf[1]);

    send4bitcommand (CMD_TABLE_WRITE_POST_INC);
    send16bit(buf[2], buf[3]);

    send4bitcommand (CMD_TABLE_WRITE_POST_INC);
    send16bit(buf[4], buf[5]);

    send4bitcommand (CMD_TABLE_WRITE_START_PROG);
    send16bit(buf[6], buf[7]);

    //nop
    nop_P9_P10();

    //done
#endif
}

void write_code(uint32_t addr, uint8_t *buf) {
#if ENABLE_WRITE
    if ((addr % WRITE_BUFFER_SIZE) != 0) {
        PSERIAL.println("address should be aligned");
        return;
    }

    //step 1
    bsf_eecon1_eepgd();
    bcf_eecon1_cfgs();
#if PIC18F2XK20
    bsf_eecon1_wren()
#endif

    //step 2
    tblptr(addr);

    //step 3
    uint8_t *buf_end = buf + WRITE_BUFFER_SIZE - 2;

    for (; buf < buf_end; buf += 2) {
        send4bitcommand(CMD_TABLE_WRITE_POST_INC);
        send16bit(buf[0], buf[1]);
    }

    //step 4
    send4bitcommand (CMD_TABLE_WRITE_START_PROG);
    send16bit(buf[0], buf[1]);

    //nop
    nop_P9_P10();

    //done
#endif
}

uint8_t hex2uint8(uint8_t h) {
    if (h >= '0' && h <= '9') {
        return h - '0';
    }

    if (h >= 'A' && h <= 'F') {
        return 10 + (h - 'A');
    }

    return 0;
}

uint8_t hex2uint8(uint8_t *start) {
    return (hex2uint8(start[0]) << 4) + hex2uint8(start[1]);
}

uint16_t hex2uint16(uint8_t *start) {
    uint16_t h = hex2uint8(start);
    uint16_t l = hex2uint8(start + 2);

    return (h << 8) + l;
}

uint32_t hex2uint24(uint8_t *start) {
    uint32_t u = hex2uint8(start);
    uint32_t h = hex2uint8(start + 2);
    uint32_t l = hex2uint8(start + 4);

    return (u << 16) + (h << 8) + l;
}

void serialPrintHex(byte b) { 
    if (b < 0x10) {
        PSERIAL.print("0");
    }
    PSERIAL.print(b, HEX); 
}

