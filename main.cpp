#include <SDL2/SDL.h>
#include "init.h"
#include <string>
#include <iostream>
#include <bitset>
#include <cstdint>
#include <iomanip>


std::uint8_t RED =      0b11000011;
std::uint8_t ORANGE =   0b11010011;
std::uint8_t YELLOW =   0b11110011;
std::uint8_t GREEN =    0b00110011;
std::uint8_t BLUE =     0b00001111;
std::uint8_t VIOLET =   0b11001111;
std::uint8_t WHITE =    0b11111111;

// number of program instructions completed per frame
int cycleSpeed = 7;
bool paused = false;

// external stuff //
// screen
uint8_t ScreenOnColor = WHITE;
uint8_t ScreenOffColor = 0x00;

// keyboard stuff                ----UDLR
uint8_t keyboard_input_state = 0b00000000;


// breakpoint stuff
uint8_t breakpoint_addr = 0x32;
bool breakpoints_enabled = false;

// tracelogging
bool tracelog = 0;






struct instruction {
    uint8_t opc;
    uint8_t reg;
    uint8_t dat;
};

struct instruction_return {
    bool c;
    bool z;
    bool n;
};





//ram
std::array<uint8_t, 0x800> RAM;

//cpu registers
uint8_t addr_output_register_high = 0x00;
uint8_t addr_output_register_low = 0x00;

uint8_t PC = 0x00;
uint8_t SP = 0x00;

bool flag_carry = 0;
bool flag_zero = 0;
bool flag_neg = 0;


// PROGRAM ROM
#include "PRGROM.hpp"


// DATA ROM
#include "DATROM.hpp"


//registers
std::array<uint8_t, 0x10> registers = {
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
};

std::array<uint8_t, 0x10> callStack = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

// reset emulator
bool reset_queue = false;
void reset() {
    printf("reseting...\n");

    PC = 0;
    SP = 0;
    flag_carry = 0;
    flag_zero = 0;
    flag_neg = 0;
    addr_output_register_high = 0x00;
    addr_output_register_low = 0x00;

    clearScreen();
    keyboard_input_state = 0;
    for(int i = 0; i < 0x800; i++) {
        RAM[i] = 0x00;
    }
    for(int i = 0; i < 0x10; i++) {
        registers[i] = 0x00;
    }
}

uint8_t push(uint8_t value) {
    callStack[SP] = value;
    SP++;
    SP &= 0x0F;
    return SP;
}

uint8_t pull() {
    SP--;
    SP &= 0x0F;
    return callStack[SP];
}

instruction fetch_inst(std::bitset<24> ins) {
    instruction fetched_ins;
    uint32_t a = ins.to_ulong();
    uint8_t out_partial = (uint8_t)(a & 0x000000FF);
    fetched_ins.dat = out_partial;
    a = a >> 8;
    out_partial = (uint8_t)(a & 0x000000FF);
    fetched_ins.reg = out_partial;
    a = a >> 8;
    out_partial = (uint8_t)(a & 0x000000FF);
    fetched_ins.opc = out_partial;
    return fetched_ins;
}

void display_execution_state() {
        uint8_t index;
        printf("\nProgram counter: 0x%02X   Stack pointer: 0x%X\n"
        "Return address: 0x%02X    Flags: C = %X, Z = %X, N = %X\n"
        , PC, SP, callStack[SP - 1], flag_carry, flag_zero, flag_neg);


        printf("Register states:\n"); // print register contents
        index = 0;
        for(uint8_t i : registers) {
            printf("r%X = %02X", index, i); // print register contents
            if(index < 0xF) printf(",  ");
            if((index % 4) == 3) printf("\n");
            index++;
        }

        printf( "\n%02X: %06X"
                "\n%02X: %06X < NEXT INSTRUCTION"
                "\n%02X: %06X\n"

        , PC-1, (int)PGR[PC-1].to_ulong(), PC, (int)PGR[PC].to_ulong(), PC+1, (int)PGR[PC+1].to_ulong());
}

bool check_breakpoint(uint8_t breakpoint) {
    if(breakpoint == PC) {
        printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
        printf("BREAKPOINT HIT! Execution information:"); // print general info
        display_execution_state();

        printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
        printf("Press enter to continue...");
        std::cin.get(); // wait for enter
        printf("\n\n\n");

        return true;
    }
    return false;
}

// memory access
uint8_t write_external(uint8_t data) {
    uint16_t write_addr = ((uint16_t)addr_output_register_high << 8);
    write_addr |= addr_output_register_low;
    write_addr &= 0x0FFF;

    if(tracelog) printf("Writing value 0x%02X to address 0x%03X:  ", data, write_addr);


    if(write_addr < 0x800) { // write to RAM
        RAM[write_addr] = data;
        if(tracelog) printf("Writing to RAM...");
    } 

    else if(write_addr == 0xF00) { // clear input
        keyboard_input_state = 0;
        if(tracelog) printf("Clearing input accumulator");
    }

    else if((write_addr >= 0xF80) && write_addr <= 0xF82) { // display control
        uint16_t pixel_x, pixel_y;
        switch(write_addr) {
            case 0xF80: // clear screen
                clearScreen(0x00);
                if(tracelog) printf("Clearing screen...");
                break;

            case 0xF81: // draw pixel
                pixel_x = (uint16_t)(data & 0x0F);
                pixel_y = (uint16_t)((data & 0xF0) >> 4);
                setPixel(pixel_x, pixel_y, ScreenOnColor);
                if(tracelog) printf("Drawing pixel at ( %X, %X )", pixel_x, pixel_y);
                break;

            case 0xF82: // erase pixel
                pixel_x = (uint16_t)(data & 0x0F);
                pixel_y = (uint16_t)((data & 0xF0) >> 4);
                setPixel(pixel_x, pixel_y, ScreenOffColor);
                if(tracelog) printf("Clearing pixel at ( %X, %X )", pixel_x, pixel_y);
                break;
        }
    }

    if(tracelog) printf("\n");
    return 0;
}

uint8_t read_external() {
    uint16_t read_addr = ((uint16_t)addr_output_register_high << 8);
    read_addr |= addr_output_register_low;
    read_addr &= 0x0FFF;
    uint8_t data = 0x00;

    if(tracelog) printf("Reading value from address 0x%03X:  ", read_addr);


    if(read_addr < 0x800) { // write to RAM
        data = RAM[read_addr];
        if(tracelog) printf("Read %02X from RAM...", data);
    }

    else if(read_addr == 0xF00) { // read input
        data = keyboard_input_state;
        if(tracelog) printf("Read %02X (binary %X%X%X%X%X%X%X%X) from input accumulator...", data, 
        (data >> 7),((data >> 6) & 0x1),((data >> 5) & 0x1),((data >> 4) & 0x1),
        ((data >> 3) & 0x1),((data >> 2) & 0x1),((data >> 1) & 0x1),(data & 0x1)); // the lengths you have to go to to printf in binary...
        keyboard_input_state = 0;
    }

    if(tracelog) printf("\n");
    return data;
}

//#####################################################################################################################
//#####################################################################################################################
// INSTRUCTIONS
//#####################################################################################################################
//#####################################################################################################################



// 0x08 XOR RR
instruction_return ins_xor(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t regB = (dat & 0x0F);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = ( registers[regA] ^ registers[regB] );// rQ is rA XOR rB
    printf("XOR r%X ^ r%X = %X -> r%X\n",regA, regB, Q, regQ);


    return_values.c = 0;
    return_values.z = (Q == 0x00);
    return_values.n = ((Q & 0x80) != 0x00);

    registers[regQ] = Q;

    return return_values;
}

// 0x10 OR RR
instruction_return ins_or(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t regB = (dat & 0x0F);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = ( registers[regA] | registers[regB] );// rQ is rA OR rB
    if(tracelog) printf("OR r%X | r%X = %X -> r%X\n",regA, regB, Q, regQ);

    return_values.c = 0;
    return_values.z = (Q == 0x00);
    return_values.n = ((Q & 0x80) != 0x00);

    registers[regQ] = Q;

    return return_values;
}

// 0x18 ADD RR
instruction_return ins_add(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t regB = (dat & 0x0F);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint16_t Q = ( (uint16_t)registers[regA] + (uint16_t)registers[regB] );// rQ is rA + rB

    return_values.c = (Q >= 256);

    Q &= 0x00FF;
    registers[regQ] = (uint8_t)Q;
    if(tracelog) printf("ADD r%X + r%X = %X -> r%X\n",regA, regB, Q, regQ);

    return_values.z = (Q == 0x0000);
    return_values.n = ((Q & 0x80) != 0x00);


    return return_values;
}

// 0x20 CMP RR
instruction_return ins_cmp(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t regB = (dat & 0x0F);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = ( registers[regA] - registers[regB] );// rQ is rA - rB
    if(tracelog) printf("CMP r%X - r%X = %X -> r%X\n",regA, regB, Q, regQ);


    return_values.c = (registers[regA] >= registers[regB]);
    return_values.z = (Q == 0x00);
    return_values.n = (registers[regA] < registers[regB]);

    registers[regQ] = Q;

    return return_values;
}

// 0x28 AND RR
instruction_return ins_and(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t regB = (dat & 0x0F);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = ( registers[regA] & registers[regB] );// rQ is rA AND rB
    if(tracelog) printf("AND r%X & r%X = %X -> r%X\n",regA, regB, Q, regQ);

    return_values.c = 0;
    return_values.z = (Q == 0x00);
    return_values.n = ((Q & 0x80) != 0x00);

    registers[regQ] = Q;

    return return_values;
}


// 0x30 MOV
instruction_return ins_mov(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = registers[regA];// rQ is rA
    if(tracelog) printf("MOV r%X -> r%X\n",regA, regQ);

    return_values.c = 0;
    return_values.z = (Q == 0x00);
    return_values.n = ((Q & 0x80) != 0x00);

    registers[regQ] = Q;

    return return_values;
}

// 0x38 LSR
instruction_return ins_lsr(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = (registers[regA] >> 1);// rQ is rA >> 1
    if(tracelog) printf("LSR r%X -> r%X\n",regA, regQ);

    return_values.c = (registers[regA] & 0x01);
    return_values.z = (Q == 0x00);
    return_values.n = ((Q & 0x80) != 0x00);

    registers[regQ] = Q;

    return return_values;
}

// 0x40 ADC
instruction_return ins_adc(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t regB = (dat & 0x0F);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint16_t Q = ( (uint16_t)registers[regA] + (uint16_t)registers[regB] + (uint16_t)flag_zero );// rQ is rA + rB + c

    return_values.c = (Q >= 256);

    Q &= 0x00FF;
    registers[regQ] = (uint8_t)Q;
    if(tracelog) printf("ADC r%X + r%X + c = %X -> r%X\n",regA, regB, Q, regQ);

    return_values.z = (Q == 0x0000);
    return_values.n = ((Q & 0x80) != 0x00);


    return return_values;
}

// 0x60 LDD IMM
instruction_return ins_ldd_imm(uint8_t reg, uint8_t dat) {
    uint8_t datarom_addr = dat;
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    if(tracelog) printf("LDD  r%X <- &0x%02X\n", regQ, datarom_addr);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    registers[regQ] = DATROM[datarom_addr];
    return return_values;
}

// 0x68 LDD PTR
instruction_return ins_ldd_ptr(uint8_t reg, uint8_t dat) {
    uint8_t regPtr = (reg >> 4);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    if(tracelog) printf("LDD  r%X <- *r%X (&0x%02X)\n", regQ, regPtr, registers[regPtr]);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    registers[regQ] = DATROM[registers[regPtr]];
    return return_values;
}


// 0x88 XOR IMM
instruction_return ins_xor_immediate(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t immData = (dat);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = ( registers[regA] ^ immData );// rQ is rA XOR imm
    printf("XOR r%X ^ %X = %X -> r%X\n",regA, immData, Q, regQ);


    return_values.c = 0;
    return_values.z = (Q == 0x00);
    return_values.n = ((Q & 0x80) != 0x00);

    registers[regQ] = Q;

    return return_values;
}

// 0x90 OR IMM
instruction_return ins_or_immediate(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t immData = (dat);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = ( registers[regA] | immData );// rQ is rA OR imm
    if(tracelog) printf("OR r%X | %X = %X -> r%X\n",regA, immData, Q, regQ);

    return_values.c = 0;
    return_values.z = (Q == 0x00);
    return_values.n = ((Q & 0x80) != 0x00);

    registers[regQ] = Q;

    return return_values;
}

// 0x98 ADD IMM
instruction_return ins_add_immediate(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t immData = (dat);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint16_t Q = ( (uint16_t)registers[regA] + (uint16_t)immData );// rQ is rA + imm

    return_values.c = (Q >= 256);

    Q &= 0x00FF;
    registers[regQ] = (uint8_t)Q;
    if(tracelog) printf("ADD r%X + %X = %X -> r%X\n",regA, immData, Q, regQ);

    return_values.z = (Q == 0x0000);
    return_values.n = ((Q & 0x80) != 0x00);


    return return_values;
}

// 0xA0 CMP IMM
instruction_return ins_cmp_immediate(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t immData = (dat);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = ( registers[regA] - immData );// rQ is rA - imm
    if(tracelog) printf("CMP r%X - %X = %X -> r%X\n",regA, immData, Q, regQ);


    return_values.c = (registers[regA] >= immData);
    return_values.z = (Q == 0x00);
    return_values.n = (registers[regA] < immData);

    registers[regQ] = Q;

    return return_values;
}

// 0xA8 AND IMM
instruction_return ins_and_immediate(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t immData = (dat);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = ( registers[regA] & immData );// rQ is rA AND imm
    if(tracelog) printf("AND r%X & %X = %X -> r%X\n",regA, immData, Q, regQ);

    return_values.c = 0;
    return_values.z = (Q == 0x00);
    return_values.n = ((Q & 0x80) != 0x00);

    registers[regQ] = Q;

    return return_values;
}

// 0xB0 LDR
instruction_return ins_ldr(uint8_t reg, uint8_t dat) {
    uint8_t immData = (dat);
    uint8_t regQ = (reg & 0x0F);
    instruction_return return_values;

    uint8_t Q = immData;// rQ is rA
    if(tracelog) printf("LDR r%X = %X\n",regQ, immData);

    return_values.c = 0;
    return_values.z = (Q == 0x00);
    return_values.n = ((Q & 0x80) != 0x00);

    registers[regQ] = Q;

    return return_values;
}

// 0xB8 LDR
instruction_return ins_sar(uint8_t reg, uint8_t dat) {
    uint8_t ar_low = (dat);
    uint8_t ar_high = (reg & 0x0F);
    instruction_return return_values;

    if(tracelog) printf("SAR  &0x%X%02X\n", ar_high, ar_low);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    addr_output_register_high = ar_high;
    addr_output_register_low = ar_low;


    return return_values;
}

// 0xC0 WRI immediate
instruction_return ins_wri_imm(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t ar_low = (dat);
    uint8_t ar_high = (reg & 0x0F);

    instruction_return return_values;

    if(tracelog) printf("WRI  r%X -> &0x%X%02X\n", regA, ar_high, ar_low);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    addr_output_register_high = ar_high;
    addr_output_register_low = ar_low;

    write_external(registers[regA]);
    return return_values;
}

// 0xC8 WRI pointer
instruction_return ins_wri_ptr(uint8_t reg, uint8_t dat) {
    uint8_t regA = (reg >> 4);
    uint8_t regPtr = (dat & 0x0F);
    uint8_t ar_low = registers[regPtr];


    instruction_return return_values;

    if(tracelog) printf("WRI  r%X -> *r%X (&0x%X%02X)\n", regA, regPtr, addr_output_register_high, ar_low);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    addr_output_register_low = ar_low;

    write_external(registers[regA]);
    return return_values;
}

// 0xD0 REA immediate
instruction_return ins_rea_imm(uint8_t reg, uint8_t dat) {
    uint8_t regQ = (reg & 0x0F);
    uint8_t ar_low = (dat);

    instruction_return return_values;

    if(tracelog) printf("REA  r%X <- &0x%X%02X\n", regQ, addr_output_register_high, ar_low);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    addr_output_register_low = ar_low;

    registers[regQ] = read_external();
    return return_values;
}

// 0xD8 REA ptr
instruction_return ins_rea_ptr(uint8_t reg, uint8_t dat) {
    uint8_t regQ = (reg & 0x0F);
    uint8_t regPtr = (dat & 0x0F);
    uint8_t ar_low = registers[regPtr];

    instruction_return return_values;

    if(tracelog) printf("REA  r%X <- *r%X (&0x%X%02X)\n", regQ, regPtr, addr_output_register_high, ar_low);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    addr_output_register_low = ar_low;

    registers[regQ] = read_external();
    return return_values;
}

// 0xE0 JSR absolute ptr
instruction_return ins_jsr_abs_ptr(uint8_t reg, uint8_t dat) {
    uint8_t regPtr = (reg >> 4);
    uint8_t jsr_addr = registers[regPtr];

    instruction_return return_values;

    if(tracelog) printf("JSR  r%X (&%02X)\n", regPtr, jsr_addr);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    push(PC+1);

    PC = jsr_addr;

    return return_values;
}

// 0xE4 JSR relative ptr
instruction_return ins_jsr_rel_ptr(uint8_t reg, uint8_t dat) {
    uint8_t regPtr = (reg >> 4);
    uint8_t jsr_addr = registers[regPtr] + PC;

    instruction_return return_values;

    if(tracelog) printf("JSR  PC + r%X (&%02X)\n", regPtr, jsr_addr);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    push(PC+1);

    PC = jsr_addr;

    return return_values;
}

// 0xE8 JSR immediate
instruction_return ins_jsr_imm(uint8_t reg, uint8_t dat) {
    uint8_t jsr_addr = dat;

    instruction_return return_values;

    if(tracelog) printf("JSR  &%02X\n", jsr_addr);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    push(PC+1);

    PC = jsr_addr;

    return return_values;
}

// 0xF0 RTS
instruction_return ins_rts(uint8_t reg, uint8_t dat) {

    instruction_return return_values;

    uint8_t return_addr = pull();

    if(tracelog) printf("RTS  (&%02X)\n", return_addr);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    PC = return_addr;

    return return_values;
}

// 0xF8 JMP
instruction_return ins_jmp(uint8_t reg, uint8_t dat) {
    uint8_t bra_addr = dat;

    instruction_return return_values;

    if(tracelog) printf("JMP  &%02X\n", bra_addr);

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    PC = bra_addr;

    return return_values;
}

// 0xF9 BEQ
instruction_return ins_beq(uint8_t reg, uint8_t dat) {
    uint8_t bra_addr = dat;
    bool branching = flag_zero;

    instruction_return return_values;

    if(tracelog) {
        printf("BEQ  &%02X  ", bra_addr);
        if(branching) {
            printf(" (branching)\n");
        } else {
            printf(" (skipping branch)\n");
        }
    }

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    if(branching) {
        PC = bra_addr;
    } else {
        PC++;
    }

    return return_values;
}

// 0xFA BRN
instruction_return ins_brn(uint8_t reg, uint8_t dat) {
    uint8_t bra_addr = dat;
    bool branching = flag_neg;

    instruction_return return_values;

    if(tracelog) {
        printf("BRN  &%02X  ", bra_addr);
        if(branching) {
            printf(" (branching)\n");
        } else {
            printf(" (skipping branch)\n");
        }
    }

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    if(branching) {
        PC = bra_addr;
    } else {
        PC++;
    }

    return return_values;
}

// 0xFB BCS
instruction_return ins_bcs(uint8_t reg, uint8_t dat) {
    uint8_t bra_addr = dat;
    bool branching = flag_carry;

    instruction_return return_values;

    if(tracelog) {
        printf("BCS  &%02X  ", bra_addr);
        if(branching) {
            printf(" (branching)\n");
        } else {
            printf(" (skipping branch)\n");
        }
    }

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    if(branching) {
        PC = bra_addr;
    } else {
        PC++;
    }

    return return_values;
}

// 0xFC BNE
instruction_return ins_bne(uint8_t reg, uint8_t dat) {
    uint8_t bra_addr = dat;
    bool branching = !flag_zero;

    instruction_return return_values;

    if(tracelog) {
        printf("BNE  &%02X  ", bra_addr);
        if(branching) {
            printf(" (branching)\n");
        } else {
            printf(" (skipping branch)\n");
        }
    }

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    if(branching) {
        PC = bra_addr;
    } else {
        PC++;
    }

    return return_values;
}

// 0xFF BCC
instruction_return ins_bcc(uint8_t reg, uint8_t dat) {
    uint8_t bra_addr = dat;
    bool branching = !flag_carry;

    instruction_return return_values;

    if(tracelog) {
        printf("BCC  &%02X  ", bra_addr);
        if(branching) {
            printf(" (branching)\n");
        } else {
            printf(" (skipping branch)\n");
        }
    }

    return_values.c = 0;
    return_values.z = 0;
    return_values.n = 0;

    if(branching) {
        PC = bra_addr;
    } else {
        PC++;
    }

    return return_values;
}







int exec_inst(instruction ins) {
    instruction_return return_values;
    bool update_flags = false;
    bool increment_pc = true;
    registers[15] = 0x00;

    if(breakpoints_enabled) {
        check_breakpoint(breakpoint_addr);
    }
    

    if(tracelog) printf(" %02X:  ",PC);

    switch(ins.opc) {
        case 0x08:
            return_values = ins_xor(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x10:
            return_values = ins_or(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x18:
            return_values = ins_add(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x20:
            return_values = ins_cmp(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x28:
            return_values = ins_and(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x30:
            return_values = ins_mov(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x38:
            return_values = ins_lsr(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x40:
            return_values = ins_adc(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x60:
            return_values = ins_ldd_imm(ins.reg, ins.dat);
            break;

        case 0x68:
            return_values = ins_ldd_ptr(ins.reg, ins.dat);
            break;

        case 0x88:
            return_values = ins_xor_immediate(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x90:
            return_values = ins_or_immediate(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0x98:
            return_values = ins_add_immediate(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0xA0:
            return_values = ins_cmp_immediate(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0xA8:
            return_values = ins_and_immediate(ins.reg, ins.dat);
            update_flags = true;
            break;
            
        case 0xB0:
            return_values = ins_ldr(ins.reg, ins.dat);
            update_flags = true;
            break;

        case 0xB8:
            return_values = ins_sar(ins.reg, ins.dat);
            break;

        case 0xC0:
            return_values = ins_wri_imm(ins.reg, ins.dat);
            break;

        case 0xC8:
            return_values = ins_wri_ptr(ins.reg, ins.dat);
            break;

        case 0xD0:
            return_values = ins_rea_imm(ins.reg, ins.dat);
            break;

        case 0xD8:
            return_values = ins_rea_ptr(ins.reg, ins.dat);
            break;

        case 0xE0:
            return_values = ins_jsr_abs_ptr(ins.reg, ins.dat);
            increment_pc = false;
            break;

        case 0xE4:
            return_values = ins_jsr_rel_ptr(ins.reg, ins.dat);
            increment_pc = false;
            break;

        case 0xE8:
            return_values = ins_jsr_imm(ins.reg, ins.dat);
            increment_pc = false;
            break;

        case 0xF0:
            return_values = ins_rts(ins.reg, ins.dat);
            increment_pc = false;
            break;

        case 0xF8:
            return_values = ins_jmp(ins.reg, ins.dat);
            increment_pc = false;
            break;

        case 0xF9:
            return_values = ins_beq(ins.reg, ins.dat);
            increment_pc = false;
            break;

        case 0xFA:
            return_values = ins_brn(ins.reg, ins.dat);
            increment_pc = false;
            break;

        case 0xFB:
            return_values = ins_bcs(ins.reg, ins.dat);
            increment_pc = false;
            break;

        case 0xFC:
            return_values = ins_bne(ins.reg, ins.dat);
            increment_pc = false;
            break;

        case 0xFF:
            return_values = ins_bcc(ins.reg, ins.dat);
            increment_pc = false;
            break;

        default:
            if(tracelog) printf("NOP (0x%X)\n",ins.opc);
            break;

    }

    if(update_flags) {
        flag_carry = return_values.c;
        flag_zero = return_values.z;
        flag_neg = return_values.n;
    }
    if(increment_pc) PC++;
    return 0;
}

//#####################################################################################################################
//#####################################################################################################################
// UPDATE
//#####################################################################################################################
//#####################################################################################################################




uint8_t bruhx = 0;
bool apressed = 0;

void onEvent(SDL_Event event) {
    if(event.type == SDL_KEYDOWN) {
        if(event.key.keysym.sym == SDLK_r) {
            reset_queue = true; // reset emulator
        } 
        else if(event.key.keysym.sym == SDLK_b) {
            breakpoints_enabled = !breakpoints_enabled; // toggle breakpoints
            if(breakpoints_enabled) {
                printf("Breakpoints enabled\n");
            } else {
                printf("Breakpoints disabled\n");
            }
        } 
        else if(event.key.keysym.sym == SDLK_v) {
            tracelog = !tracelog; // toggle tracelogging
            if(tracelog) {
                printf("Tracelogging enabled\n");
            } else {
                printf("Tracelogging disabled\n");
            }
        } 
        else if(event.key.keysym.sym == SDLK_t) {
            paused = !paused; // toggle pause
            if(paused) {
                printf("paused\n");
            } else {
                printf("unpaused\n");
            }
        } 
        if(!paused) { // program input does not occur when paused
            if(event.key.keysym.sym == SDLK_w) {
                keyboard_input_state |= 0b1000; // up
            } 
            else if(event.key.keysym.sym == SDLK_s) {
                keyboard_input_state |= 0b0100; // down
            }
            else if(event.key.keysym.sym == SDLK_a) {
                keyboard_input_state |= 0b0010; // left
            }
            else if(event.key.keysym.sym == SDLK_d) {
                keyboard_input_state |= 0b0001; // right
            }
        }
    }
}


int update() {
    if(!paused) {
        for(int i = 0; i < cycleSpeed; i++) {
            exec_inst(fetch_inst(PGR[PC]));
        }
    }
    
    if(reset_queue) {
        reset();
        reset_queue = false;
    }

    return 0;
}

void program_end() {
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf("Ending execution. Final program state:"); // print general info
    display_execution_state();
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

}
int main() {
    printf( "\n||  Lucas's computer emulator             ||"
            "\n||  WASD keys are used for program input  ||"
            "\n||  press 't' to pause emulation          ||"
            "\n||  press 'r' to reset                    ||"
            "\n||  press 'b' to toggle breakpoints       ||"
            "\n||  press 'v' to toggle tracelogging      ||"
            "\n\n"
    );

    init(16,16,32);
    return 0;
}
