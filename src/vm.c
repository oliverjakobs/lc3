#include "platform.c"

/* default starting position is 0x3000 */
#define PC_START 0x3000

typedef enum {
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC, /* program counter */
    R_COND,
    R_COUNT
} RegisterID;

/* Memory Mapped Registers */
typedef enum {
    MR_KBSR = 0xFE00, /* keyboard status */
    MR_KBDR = 0xFE02  /* keyboard data */
} MemoryMappedRegisters;

typedef enum {
    FL_POS = 1 << 0, /* P */
    FL_ZRO = 1 << 1, /* Z */
    FL_NEG = 1 << 2, /* N */
} ConditionFlags;

typedef enum {
    OP_BR   = 0b0000, /* branch */
    OP_ADD  = 0b0001, /* add  */
    OP_LD   = 0b0010, /* load */
    OP_ST   = 0b0011, /* store */
    OP_JSR  = 0b0100, /* jump register */
    OP_AND  = 0b0101, /* bitwise and */
    OP_LDR  = 0b0110, /* load register */
    OP_STR  = 0b0111, /* store register */
    OP_RTI  = 0b1000, /* unused */
    OP_NOT  = 0b1001, /* bitwise not */
    OP_LDI  = 0b1010, /* load indirect */
    OP_STI  = 0b1011, /* store indirect */
    OP_JMP  = 0b1100, /* jump */
    OP_RES  = 0b1101, /* reserved (unused) */
    OP_LEA  = 0b1110, /* load effective address */
    OP_TRAP = 0b1111  /* execute trap */
} OpCodes;

typedef enum {
    TRAP_GETC  = 0x20, /* get character from keyboard, not echoed onto the terminal */
    TRAP_OUT   = 0x21, /* output a character */
    TRAP_PUTS  = 0x22, /* output a word string */
    TRAP_IN    = 0x23, /* get character from keyboard, echoed onto the terminal */
    TRAP_PUTSP = 0x24, /* output a byte string */
    TRAP_HALT  = 0x25  /* halt the program */
} TrapCodes;

/* 65536 memory locations */
uint16_t memory[UINT16_MAX];

/* register storage */
uint16_t registers[R_COUNT];

static int running = 0;

/* Memory Access */
void mem_write(uint16_t address, uint16_t val) {
    memory[address] = val;
}

uint16_t mem_read(uint16_t address) {
    if (address == MR_KBSR) {
        if (check_key()) {
            memory[MR_KBSR] = (1 << 15);
            memory[MR_KBDR] = getchar();
        } else {
            memory[MR_KBSR] = 0;
        }
    }
    return memory[address];
}

/* Utilities */
#define DR(instr)  (((instr) >> 9) & 0x7)
#define SR(instr)  (((instr) >> 9) & 0x7)
#define SR1(instr) (((instr) >> 6) & 0x7)
#define SR2(instr) ((instr) & 0x7)

#define BASE_R(instr) (((instr) >> 6) & 0x7)

#define IMM5(instr)         (sign_extend((instr) & 0x1F, 5))
#define OFFSET6(instr)      (sign_extend((instr) & 0x3F, 6))
#define PC_OFFSET9(instr)   (sign_extend((instr) & 0x1FF, 9))
#define PC_OFFSET11(instr)  (sign_extend((instr) & 0x7FF, 11))
#define TRAP_VECT8(instr)   ((instr) & 0xFF)

#define BIT5(instr)   (((instr) >> 5) & 0x1)
#define BIT11(instr)  (((instr) >> 11) & 0x1)

void setcc(uint16_t reg) {
    if (registers[reg] == 0) {
        registers[R_COND] = FL_ZRO;
    } else if (registers[reg] >> 15) { /* a 1 in the left-most bit indicates negative */
        registers[R_COND] = FL_NEG;
    } else {
        registers[R_COND] = FL_POS;
    }
}

uint16_t sign_extend(uint16_t x, int bit_count) {
    if ((x >> (bit_count - 1)) & 1) x |= (0xFFFF << bit_count);
    return x;
}

uint16_t swap16(uint16_t x) { return (x << 8) | (x >> 8); }

/* trap functions */
void trap_getchar() {
    /* read a single ASCII char */
    registers[R_R0] = (uint16_t)getchar();
    setcc(R_R0);
}

void trap_out() {
    putc((char)registers[R_R0], stdout);
    fflush(stdout);
}

void trap_puts() {
    /* one char per word */
    for (uint16_t* c = memory + registers[R_R0]; *c; ++c)
        putc((char)*c, stdout);
    fflush(stdout);
}

void trap_in() {
    printf("Enter a character: ");
    char c = getchar();
    putc(c, stdout);
    fflush(stdout);
    registers[R_R0] = (uint16_t)c;
    setcc(R_R0);
}

void trap_putsp() {
    /*
     * one char per byte (two bytes per word) here we need to swap
     * back to big endian format
     */
    for (uint16_t* c = memory + registers[R_R0]; *c; ++c) {
        char char1 = (*c) & 0xFF;
        putc(char1, stdout);
        char char2 = (*c) >> 8;
        if (char2) putc(char2, stdout);
    }
    fflush(stdout);
}

void trap_halt() {
    puts("HALT");
    fflush(stdout);
    running = 0;
}

void vm_setup() {
    /* setup */
    signal(SIGINT, handle_interrupt);
    disable_input_buffering();

    /* since exactly one condition flag should be set at any given time, set the Z flag */
    registers[R_COND] = FL_ZRO;

    /* set the PC to starting position */
    registers[R_PC] = PC_START;

    running = 1;
}

void vm_run() {
    while (running) {
        /* FETCH */
        uint16_t instr = mem_read(registers[R_PC]++);
        uint16_t op = instr >> 12;

        switch (op) {
            case OP_ADD: {
                /**
                 * ADD (register mode)
                 * |15       12|11     9| 8     6| 5| 4  3| 2     0|
                 * |-- -- -- --|-- -- --|-- -- --|--|-- --|-- -- --|
                 * |   00 01   |   DR   |  SR1   | 0| 0  0|  SR2   |
                 * |-- -- -- --|-- -- --|-- -- --|--|-- --|-- -- --|
                 *
                 * ADD (immedieate mode)
                 * |15       12|11     9| 8     6| 5| 4           0|
                 * |-- -- -- --|-- -- --|-- -- --|--|-- -- -- -- --|
                 * |   00 01   |   DR   |  SR1   | 1|     imm5     |
                 * |-- -- -- --|-- -- --|-- -- --|--|-- -- -- -- --|
                 *
                 */
                if (BIT5(instr) == 0) { 
                    /* register mode */
                    registers[DR(instr)] = registers[SR1(instr)] + registers[SR2(instr)];
                } else {
                    /* immediate mode */
                    registers[DR(instr)] = registers[SR1(instr)] + IMM5(instr);
                }

                setcc(DR(instr));
                break;
            }
            case OP_AND: {
                /**
                 * AND (register mode)
                 * |15       12|11     9| 8     6| 5| 4  3| 2     0|
                 * |-- -- -- --|-- -- --|-- -- --|--|-- --|-- -- --|
                 * |   01 01   |   DR   |  SR1   | 0| 0  0|  SR2   |
                 * |-- -- -- --|-- -- --|-- -- --|--|-- --|-- -- --|
                 *
                 * AND (immedieate mode)
                 * |15       12|11     9| 8     6| 5| 4           0|
                 * |-- -- -- --|-- -- --|-- -- --|--|-- -- -- -- --|
                 * |   01 01   |   DR   |  SR1   | 1|     imm5     |
                 * |-- -- -- --|-- -- --|-- -- --|--|-- -- -- -- --|
                 */
                if (BIT5(instr) == 0) {
                    /* register mode */
                    registers[DR(instr)] = registers[SR1(instr)] & registers[SR2(instr)];
                } else {
                    /* immediate mode */
                    registers[DR(instr)] = registers[SR1(instr)] & IMM5(instr);
                }

                setcc(DR(instr));
                break;
            }
            case OP_BR: {
                /**
                 * BR
                 * |15       12|11|10| 9| 8                       0|
                 * |-- -- -- --|--|--|--|-- -- -- -- -- -- -- -- --|
                 * |   00 00   | n| z| p|         PCoffset9        |
                 * |-- -- -- --|--|--|--|-- -- -- -- -- -- -- -- --|
                 */
                uint16_t cond_flag = (instr >> 9) & 0x7;
                if (cond_flag & registers[R_COND])
                    registers[R_PC] += PC_OFFSET9(instr);
                break;
            }
            case OP_JMP: {
                /**
                 * JMP
                 * |15       12|11     9| 8     6| 5              0|
                 * |-- -- -- --|-- -- --|-- -- --|-- -- -- -- -- --|
                 * |   11 00   |  000   | BaseR  |     000000      |
                 * |-- -- -- --|-- -- --|-- -- --|-- -- -- -- -- --|
                 */
                registers[R_PC] = registers[BASE_R(instr)];
                break;
            }
            case OP_JSR: {
                /**
                 * JSR
                 * |15       12|11|10                             0|
                 * |-- -- -- --|--|-- -- -- -- -- -- -- -- -- -- --|
                 * |   01 00   | 1|           PCoffset11           |
                 * |-- -- -- --|--|-- -- -- -- -- -- -- -- -- -- --|
                 *
                 * JSRR
                 * |15       12|11|10  9| 8     6| 5              0|
                 * |-- -- -- --|--|-- --|-- -- --|-- -- -- -- -- --|
                 * |   01 00   | 0| 00  | BaseR  |     000000      |
                 * |-- -- -- --|--|-- --|-- -- --|-- -- -- -- -- --|
                 */
                registers[R_R7] = registers[R_PC];
                if (BIT11(instr) == 0) {
                    registers[R_PC] = registers[BASE_R(instr)]; /* JSRR */
                } else {
                    registers[R_PC] += PC_OFFSET11(instr);  /* JSR */
                }
                break;
            }
            case OP_LD: {
                /**
                 * LD
                 * |15       12|11     9| 8                       0|
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 * |   00 10   |   DR   |         PCoffset9        |
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 */
                registers[DR(instr)] = mem_read(registers[R_PC] + PC_OFFSET9(instr));
                setcc(DR(instr));
                break;
            }
            case OP_LDI: {
                /**
                 * LDI
                 * |15       12|11     9| 8                       0|
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 * |   10 10   |   DR   |         PCoffset9        |
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 */
                registers[DR(instr)] = mem_read(mem_read(registers[R_PC] + PC_OFFSET9(instr)));
                setcc(DR(instr));
                break;
            }
            case OP_LDR: {
                /**
                 * LDR
                 * |15       12|11     9| 8     6| 5              0|
                 * |-- -- -- --|-- -- --|-- -- --|-- -- -- -- -- --|
                 * |   01 10   |   DR   | BaseR  |     offset6     |
                 * |-- -- -- --|-- -- --|-- -- --|-- -- -- -- -- --|
                 */
                registers[DR(instr)] = mem_read(registers[BASE_R(instr)] + OFFSET6(instr));
                setcc(DR(instr));
                break;
            }
            case OP_LEA: {
                /**
                 * LEA
                 * |15       12|11     9| 8                       0|
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 * |   11 10   |   DR   |         PCoffset9        |
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 */
                registers[DR(instr)] = registers[R_PC] + PC_OFFSET9(instr);
                setcc(DR(instr));
                break;
            }
            case OP_NOT: {
                /**
                 * NOT
                 * |15       12|11     9| 8     6| 5| 4           0|
                 * |-- -- -- --|-- -- --|-- -- --|--|-- -- -- -- --|
                 * |   10 01   |   DR   |   SR   | 1|   11111      |
                 * |-- -- -- --|-- -- --|-- -- --|--|-- -- -- -- --|
                 */
                registers[DR(instr)] = ~registers[SR1(instr)];
                setcc(DR(instr));
                break;
            }
            case OP_ST: {
                /**
                 * ST
                 * |15       12|11     9| 8                       0|
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 * |   00 11   |   SR   |         PCoffset9        |
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 */
                mem_write(registers[R_PC] + PC_OFFSET9(instr), registers[SR(instr)]);
                break;
            }
            case OP_STI: {
                /**
                 * ST
                 * |15       12|11     9| 8                       0|
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 * |   10 11   |   SR   |         PCoffset9        |
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 */
                mem_write(mem_read(registers[R_PC] + PC_OFFSET9(instr)), registers[SR(instr)]);
                break;
            }
            case OP_STR: {
                /**
                 * ST
                 * |15       12|11     9| 8                       0|
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 * |   01 11   |   SR   | BaseR  |     offset6     |
                 * |-- -- -- --|-- -- --|-- -- -- -- -- -- -- -- --|
                 */
                mem_write(registers[BASE_R(instr)] + OFFSET6(instr), registers[SR(instr)]);
                break;
            }
            case OP_TRAP: {
                /**
                 * ST
                 * |15       12|11        8| 7                    0|
                 * |-- -- -- --|-- -- -- --|-- -- -- -- -- -- -- --|
                 * |   11 11   |   0000    |      trapvect8        |
                 * |-- -- -- --|-- -- -- --|-- -- -- -- -- -- -- --|
                 */
                switch (TRAP_VECT8(instr))
                {
                    case TRAP_GETC:  trap_getchar(); break;
                    case TRAP_OUT:   trap_out(); break;
                    case TRAP_PUTS:  trap_puts(); break;
                    case TRAP_IN:    trap_in(); break;
                    case TRAP_PUTSP: trap_putsp(); break;
                    case TRAP_HALT:  trap_halt(); break;
                }
                break;
            }
            case OP_RES:
            case OP_RTI:
            default:
                /* bad opcode */
                abort();
                break;
        }
    }
}

void vm_shutdown() {
    restore_input_buffering();
}

int read_image(const char* image_path) {
    FILE* file = fopen(image_path, "rb");
    if (!file) { return 0; };

    /* the origin tells us where in memory to place the image */
    uint16_t origin;
    fread(&origin, sizeof(origin), 1, file);
    origin = swap16(origin);

    /* we know the maximum file size so we only need one fread */
    uint16_t max_read = UINT16_MAX - origin;
    uint16_t* p = memory + origin;
    size_t read = fread(p, sizeof(uint16_t), max_read, file);

    /* swap to little endian */
    while (read-- > 0) {
        *p = swap16(*p);
        ++p;
    }

    fclose(file);
    return 1;
}

int main(int argc, const char* argv[]) {
    /* load arguments */
    if (argc < 2) {
        /* show usage string */
        printf("lc3 [image-file1] ...\n");
        exit(2);
    }

    for (int j = 1; j < argc; ++j) {
        if (!read_image(argv[j])) {
            printf("failed to load image: %s\n", argv[j]);
            exit(1);
        }
    }

    vm_setup();
    vm_run();
    vm_shutdown();

    return 0;
}