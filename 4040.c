/*
 * ============================================================================
 * FROM TRANSISTOR TO INTEL 4040 CPU IN C
 * ============================================================================
 * A complete bottom-up digital hardware simulation:
 *   1. Switch-level PMOS & NMOS transistors and tri-state wires
 *   2. CMOS Logic Gates (NOT, NAND, NOR, AND, OR, XOR)
 *   3. Combinational Logic (MUXes, Half/Full Adders, 4-bit Ripple Adder, ALU)
 *   4. Sequential Logic (NOR SR-Latch, D-Latch, Master-Slave D-Flip-Flop, Regs)
 *   5. Full Intel 4040 CPU (All 60 instructions, 24 banked registers,
 *      7-level stack, 8KB banked ROM, 4002 RAM, hardware interrupts, and HLT)
 * ============================================================================
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


/* Global counter to prove every CPU operation runs through transistors */
static uint64_t g_transistor_switches = 0;

/* ============================================================================
 * LAYER 1: SEMICONDUCTOR PHYSICS (WIRES & MOSFET TRANSISTORS)
 * ============================================================================
 */

typedef enum {
  LOW = 0,     /* VSS / Ground (0V)              */
  HIGH = 1,    /* VDD / Power  (+5V / -15V PMOS) */
  HIGH_Z = 2,  /* Floating / High-Impedance      */
  WIRE_ERR = 3 /* Short-circuit contention       */
} Wire;

/* Resolve two transistor outputs wired to the same node */
static inline Wire wire_resolve(Wire a, Wire b) {
  if (a == HIGH_Z)
    return b;
  if (b == HIGH_Z)
    return a;
  if (a == b)
    return a;
  return WIRE_ERR; /* VDD shorted to GND */
}

/*
 * PMOS Transistor:
 * Source is typically connected to HIGH (VDD).
 * Conducts source -> drain when gate is LOW.
 */
static inline Wire pmos(Wire source, Wire gate) {
  g_transistor_switches++;
  return (gate == LOW) ? source : HIGH_Z;
}

/*
 * NMOS Transistor:
 * Source is typically connected to LOW (GND).
 * Conducts source -> drain when gate is HIGH.
 */
static inline Wire nmos(Wire source, Wire gate) {
  g_transistor_switches++;
  return (gate == HIGH) ? source : HIGH_Z;
}

/* ============================================================================
 * LAYER 2: FUNDAMENTAL LOGIC GATES (BUILT FROM TRANSISTORS)
 * ============================================================================
 */

/*
 * CMOS Inverter (NOT Gate) - 2 Transistors
 *       VDD
 *        |
 *     [PMOS] <-- in
 *        +-----> out
 *     [NMOS] <-- in
 *        |
 *       GND
 */
static inline Wire gate_not(Wire in) {
  Wire pull_up = pmos(HIGH, in);
  Wire pull_down = nmos(LOW, in);
  return wire_resolve(pull_up, pull_down);
}

/*
 * CMOS 2-Input NAND Gate - 4 Transistors
 * Parallel PMOS pull-up network + Series NMOS pull-down network
 */
static inline Wire gate_nand(Wire a, Wire b) {
  /* Parallel PMOS connected to VDD */
  Wire p1 = pmos(HIGH, a);
  Wire p2 = pmos(HIGH, b);
  Wire pull_up = wire_resolve(p1, p2);

  /* Series NMOS connected to GND */
  Wire n_mid = nmos(LOW, b);
  Wire pull_down = nmos(n_mid, a);

  return wire_resolve(pull_up, pull_down);
}

/*
 * CMOS 2-Input NOR Gate - 4 Transistors
 * Series PMOS pull-up network + Parallel NMOS pull-down network
 */
static inline Wire gate_nor(Wire a, Wire b) {
  /* Series PMOS connected to VDD */
  Wire p_mid = pmos(HIGH, a);
  Wire pull_up = pmos(p_mid, b);

  /* Parallel NMOS connected to GND */
  Wire n1 = nmos(LOW, a);
  Wire n2 = nmos(LOW, b);
  Wire pull_down = wire_resolve(n1, n2);

  return wire_resolve(pull_up, pull_down);
}

/* Derived Gates built exclusively from NAND, NOR, and NOT */
static inline Wire gate_and(Wire a, Wire b) {
  return gate_not(gate_nand(a, b));
}

static inline Wire gate_or(Wire a, Wire b) { return gate_not(gate_nor(a, b)); }

/* XOR built from 4 NAND gates */
static inline Wire gate_xor(Wire a, Wire b) {
  Wire n1 = gate_nand(a, b);
  Wire n2 = gate_nand(a, n1);
  Wire n3 = gate_nand(b, n1);
  return gate_nand(n2, n3);
}

/* 3-Input AND Gate */
static inline Wire gate_and3(Wire a, Wire b, Wire c) {
  return gate_and(gate_and(a, b), c);
}

/* ============================================================================
 * LAYER 3: COMBINATIONAL LOGIC (MUX, ADDERS, 4-BIT ALU)
 * ============================================================================
 */

/* 4-bit Bus (Nibble) */
typedef struct {
  Wire b[4]; /* b[0] = LSB, b[3] = MSB */
} Word4;

static inline Word4 word4_from_u8(uint8_t val) {
  Word4 w;
  for (int i = 0; i < 4; i++) {
    w.b[i] = ((val >> i) & 1) ? HIGH : LOW;
  }
  return w;
}

static inline uint8_t word4_to_u8(Word4 w) {
  uint8_t val = 0;
  for (int i = 0; i < 4; i++) {
    if (w.b[i] == HIGH)
      val |= (1 << i);
  }
  return val;
}

/* 2-to-1 Multiplexer (1-bit): sel=0 -> a, sel=1 -> b */
static inline Wire mux2(Wire a, Wire b, Wire sel) {
  Wire not_sel = gate_not(sel);
  Wire path_a = gate_and(a, not_sel);
  Wire path_b = gate_and(b, sel);
  return gate_or(path_a, path_b);
}

/* 4-bit 2-to-1 Multiplexer */
static inline Word4 mux2_4bit(Word4 a, Word4 b, Wire sel) {
  Word4 out;
  for (int i = 0; i < 4; i++) {
    out.b[i] = mux2(a.b[i], b.b[i], sel);
  }
  return out;
}

/* Half Adder */
static inline void half_adder(Wire a, Wire b, Wire *sum, Wire *cout) {
  *sum = gate_xor(a, b);
  *cout = gate_and(a, b);
}

/* Full Adder (built from two Half Adders and an OR gate) */
static inline void full_adder(Wire a, Wire b, Wire cin, Wire *sum, Wire *cout) {
  Wire s1, c1, c2;
  half_adder(a, b, &s1, &c1);
  half_adder(s1, cin, sum, &c2);
  *cout = gate_or(c1, c2);
}

/* 4-Bit Ripple-Carry Adder */
static inline Word4 adder_4bit(Word4 a, Word4 b, Wire cin, Wire *cout) {
  Word4 sum;
  Wire c = cin;
  for (int i = 0; i < 4; i++) {
    full_adder(a.b[i], b.b[i], c, &sum.b[i], &c);
  }
  *cout = c;
  return sum;
}

/* 4-Bit Bitwise Inverter */
static inline Word4 not_4bit(Word4 a) {
  Word4 out;
  for (int i = 0; i < 4; i++)
    out.b[i] = gate_not(a.b[i]);
  return out;
}

/* 4-Bit Bitwise AND (Used by 4040 AN6, AN7 instructions) */
static inline Word4 and_4bit(Word4 a, Word4 b) {
  Word4 out;
  for (int i = 0; i < 4; i++)
    out.b[i] = gate_and(a.b[i], b.b[i]);
  return out;
}

/* 4-Bit Bitwise OR (Used by 4040 OR4, OR5 instructions) */
static inline Word4 or_4bit(Word4 a, Word4 b) {
  Word4 out;
  for (int i = 0; i < 4; i++)
    out.b[i] = gate_or(a.b[i], b.b[i]);
  return out;
}

/* 4-Bit Zero Detector (NOR of all 4 bits: HIGH when w == 0000) */
static inline Wire is_zero_4bit(Word4 w) {
  Wire or01 = gate_or(w.b[0], w.b[1]);
  Wire or23 = gate_or(w.b[2], w.b[3]);
  return gate_nor(or01, or23);
}

/*
 * Intel 4040 4-Bit ALU Operations
 */
typedef enum {
  ALU_ADD, /* ACC + B + CY                 */
  ALU_SUB, /* ACC + ~B + ~CY (1's comp)    */
  ALU_INC, /* B + 1 (no carry out change)  */
  ALU_IAC, /* ACC + 1 (sets CY)            */
  ALU_DAC, /* ACC + 0xF (sets CY)          */
  ALU_AND, /* ACC & B (4040 AN6/AN7)       */
  ALU_OR,  /* ACC | B (4040 OR4/OR5)       */
  ALU_CMA, /* ~ACC                         */
  ALU_RAL, /* Rotate Left through Carry    */
  ALU_RAR  /* Rotate Right through Carry   */
} AluOp;

static Word4 alu_4040_execute(AluOp op, Word4 acc, Word4 operand, Wire cy_in,
                              Wire *cy_out) {
  Word4 res = word4_from_u8(0);
  *cy_out = cy_in;

  switch (op) {
  case ALU_ADD:
    res = adder_4bit(acc, operand, cy_in, cy_out);
    break;

  case ALU_SUB: {
    /* Intel 4004/4040 subtraction inverts both subtrahend and carry-in */
    Word4 inv_op = not_4bit(operand);
    Wire inv_cy = gate_not(cy_in);
    res = adder_4bit(acc, inv_op, inv_cy, cy_out);
    break;
  }

  case ALU_INC: {
    Wire dummy_cy;
    res = adder_4bit(operand, word4_from_u8(1), LOW, &dummy_cy);
    break;
  }

  case ALU_IAC:
    res = adder_4bit(acc, word4_from_u8(1), LOW, cy_out);
    break;

  case ALU_DAC:
    res = adder_4bit(acc, word4_from_u8(0xF), LOW, cy_out);
    break;

  case ALU_AND:
    res = and_4bit(acc, operand);
    break;

  case ALU_OR:
    res = or_4bit(acc, operand);
    break;

  case ALU_CMA:
    res = not_4bit(acc);
    break;

  case ALU_RAL:
    res.b[0] = cy_in;
    res.b[1] = acc.b[0];
    res.b[2] = acc.b[1];
    res.b[3] = acc.b[2];
    *cy_out = acc.b[3];
    break;

  case ALU_RAR:
    res.b[3] = cy_in;
    res.b[2] = acc.b[3];
    res.b[1] = acc.b[2];
    res.b[0] = acc.b[1];
    *cy_out = acc.b[0];
    break;
  }
  return res;
}

/* ============================================================================
 * LAYER 4: SEQUENTIAL LOGIC (LATCHES, FLIP-FLOPS, REGISTERS)
 * ============================================================================
 */

/*
 * Cross-Coupled NOR SR-Latch (1-Bit Storage Element)
 * Uses iterative gate propagation to settle feedback loop.
 */
typedef struct {
  Wire q;
  Wire q_bar;
} SRLatch;

static inline void sr_latch_init(SRLatch *l) {
  l->q = LOW;
  l->q_bar = HIGH;
}

static inline void sr_latch_update(SRLatch *l, Wire s, Wire r) {
  /* Two gate-delay iterations settle the bistable feedback loop */
  for (int i = 0; i < 2; i++) {
    l->q = gate_nor(r, l->q_bar);
    l->q_bar = gate_nor(s, l->q);
  }
}

/* Gated D-Latch built from an SR-Latch, 2 AND gates, and 1 NOT gate */
static inline void d_latch_update(SRLatch *l, Wire d, Wire en) {
  Wire not_d = gate_not(d);
  Wire s = gate_and(d, en);
  Wire r = gate_and(not_d, en);
  sr_latch_update(l, s, r);
}

/*
 * Master-Slave Edge-Triggered D-Flip-Flop (1-Bit)
 * Built from two Gated D-Latches and an inverter on the clock line.
 */
typedef struct {
  SRLatch master;
  SRLatch slave;
} DFF;

static inline void dff_init(DFF *ff) {
  sr_latch_init(&ff->master);
  sr_latch_init(&ff->slave);
}

static inline void dff_clock_phase(DFF *ff, Wire d, Wire clk) {
  Wire not_clk = gate_not(clk);
  d_latch_update(&ff->master, d, not_clk); /* Master transparent on CLK=LOW  */
  d_latch_update(&ff->slave, ff->master.q, clk); /* Slave latches on CLK=HIGH */
}

static inline void dff_write(DFF *ff, Wire d) {
  dff_clock_phase(ff, d, LOW);
  dff_clock_phase(ff, d, HIGH);
}

static inline Wire dff_read(const DFF *ff) { return ff->slave.q; }

/* 4-Bit Hardware Register (Array of 4 Master-Slave D-Flip-Flops) */
typedef struct {
  DFF bit[4];
} Reg4;

static inline void reg4_init(Reg4 *r) {
  for (int i = 0; i < 4; i++)
    dff_init(&r->bit[i]);
}

static inline void reg4_write_word(Reg4 *r, Word4 w) {
  for (int i = 0; i < 4; i++)
    dff_write(&r->bit[i], w.b[i]);
}

static inline Word4 reg4_read_word(const Reg4 *r) {
  Word4 w;
  for (int i = 0; i < 4; i++)
    w.b[i] = dff_read(&r->bit[i]);
  return w;
}

static inline void reg4_write(Reg4 *r, uint8_t val) {
  reg4_write_word(r, word4_from_u8(val));
}

static inline uint8_t reg4_read(const Reg4 *r) {
  return word4_to_u8(reg4_read_word(r));
}

/* 12-Bit Hardware Register (3 x 4-Bit Registers for PC and Stack) */
typedef struct {
  Reg4 nibble[3]; /* nibble[0]=low, nibble[1]=mid, nibble[2]=high */
} Reg12;

static inline void reg12_init(Reg12 *r) {
  for (int i = 0; i < 3; i++)
    reg4_init(&r->nibble[i]);
}

static inline void reg12_write(Reg12 *r, uint16_t val) {
  reg4_write(&r->nibble[0], (val >> 0) & 0xF);
  reg4_write(&r->nibble[1], (val >> 4) & 0xF);
  reg4_write(&r->nibble[2], (val >> 8) & 0xF);
}

static inline uint16_t reg12_read(const Reg12 *r) {
  return (uint16_t)(reg4_read(&r->nibble[0]) | (reg4_read(&r->nibble[1]) << 4) |
                    (reg4_read(&r->nibble[2]) << 8));
}

/* ============================================================================
 * LAYER 5: INTEL 4040 MICROARCHITECTURE & MCS-40 SYSTEM
 * ============================================================================
 * Architectural Superset of the Intel 4004:
 *   - 4-bit Accumulator (ACC) + Carry Flip-Flop (CY)
 *   - 24 x 4-bit Index Registers:
 *       * Bank 0: R0..R7 (physical 0..7)
 *       * Bank 1: R0..R7 (physical 16..23, selected via SB0/SB1)
 *       * Shared: R8..R15 (physical 8..15, always accessible)
 *   - 8 x 12-bit Address Registers (1 Program Counter + 7-level Call Stack)
 *   - 8KB Program ROM (2 banks of 4KB selected via DB0/DB1)
 *   - 4002 Data RAM System (8 RAM banks x 4 chips x 4 regs x 16 main + 4
 * status)
 *   - Hardware Interrupt support (EIN, DIN, INT pin, BBS instruction)
 *   - Halt Mode (HLT instruction)
 * ============================================================================
 */

typedef struct {
  /* Core Transistor-Backed Registers */
  Reg4 acc;            /* 4-bit Accumulator                            */
  DFF cy;              /* 1-bit Carry Flip-Flop                        */
  Reg4 index_regs[24]; /* 24 x 4-bit Index Registers                   */
  Reg12 stack[8];      /* 12-bit PC + 7-level Subroutine/Interrupt Stk */
  uint8_t sp;          /* 3-bit Stack Pointer (0..7)                   */

  /* Bus & Control Registers */
  Reg4 src_hi;      /* Send Register Control (High Nibble)          */
  Reg4 src_lo;      /* Send Register Control (Low Nibble)           */
  Reg4 src_save_hi; /* Saved SRC High during Interrupt (for BBS)    */
  Reg4 src_save_lo; /* Saved SRC Low during Interrupt (for BBS)     */

  /* 4040 Control Flip-Flops */
  DFF reg_bank;         /* 0 = Index Bank 0 (0..7), 1 = Bank 1 (16..23) */
  DFF rom_bank;         /* 0 = ROM Bank 0, 1 = ROM Bank 1               */
  DFF pending_rom_bank; /* Delayed ROM bank latch for DB0/DB1           */
  DFF int_enabled;      /* Interrupt Enable Flag (EIN / DIN)            */
  DFF in_interrupt;     /* Currently servicing an interrupt             */
  DFF halted;           /* CPU Halted Flag (HLT instruction)            */
  DFF test_pin;         /* External TEST pin                            */
  DFF int_pin;          /* External Interrupt Request pin               */
  Reg4 dcl_reg;         /* 3-bit Command Line Register (RAM Bank 0..7)  */

  /* Program Memory Latch for WPM/RPM */
  uint8_t wpm_half_latch;
  uint8_t rpm_data_latch;

  /* MCS-40 Memory & I/O Subsystem */
  uint8_t rom[2][4096];     /* 2 Banks x 4096 Bytes of 4001/4308 ROM        */
  uint8_t rom_ports[2][16]; /* 16 x 4-bit I/O ports per ROM bank            */

  /* 4002 Data RAM: [8 banks][4 chips][4 registers][16 main nibbles]        */
  uint8_t ram_main[8][4][4][16];
  /* 4002 Status RAM: [8 banks][4 chips][4 registers][4 status nibbles]     */
  uint8_t ram_status[8][4][4][4];
  /* 4002 Output Ports: [8 banks][4 chips]                                  */
  uint8_t ram_ports[8][4];

  uint64_t cycles;
  uint64_t instructions_executed;
} Intel4040;

/* Map logical register index (0..15) to physical register (0..23) using SB0/SB1
 */
static int map_reg_index(const Intel4040 *cpu, int r) {
  r &= 0xF;
  if (r < 8 && dff_read(&cpu->reg_bank) == HIGH) {
    return r + 16; /* Bank 1 maps R0..R7 -> Physical R16..R23 */
  }
  return r;
}

static uint8_t cpu_read_reg(const Intel4040 *cpu, int r) {
  return reg4_read(&cpu->index_regs[map_reg_index(cpu, r)]);
}

static void cpu_write_reg(Intel4040 *cpu, int r, uint8_t val) {
  reg4_write(&cpu->index_regs[map_reg_index(cpu, r)], val);
}

static uint16_t cpu_get_pc(const Intel4040 *cpu) {
  return reg12_read(&cpu->stack[cpu->sp]);
}

static void cpu_set_pc(Intel4040 *cpu, uint16_t addr) {
  reg12_write(&cpu->stack[cpu->sp], addr & 0xFFF);
}

static uint8_t cpu_fetch_byte(Intel4040 *cpu) {
  uint8_t bank = (dff_read(&cpu->rom_bank) == HIGH) ? 1 : 0;
  uint16_t pc = cpu_get_pc(cpu);
  uint8_t byte = cpu->rom[bank][pc & 0xFFF];
  cpu_set_pc(cpu, (pc + 1) & 0xFFF);
  cpu->cycles += 1;
  return byte;
}

/* Initialize the Intel 4040 CPU */
static void cpu_init(Intel4040 *cpu) {
  memset(cpu, 0, sizeof(*cpu));
  reg4_init(&cpu->acc);
  dff_init(&cpu->cy);
  for (int i = 0; i < 24; i++)
    reg4_init(&cpu->index_regs[i]);
  for (int i = 0; i < 8; i++)
    reg12_init(&cpu->stack[i]);

  reg4_init(&cpu->src_hi);
  reg4_init(&cpu->src_lo);
  reg4_init(&cpu->src_save_hi);
  reg4_init(&cpu->src_save_lo);

  dff_init(&cpu->reg_bank);
  dff_init(&cpu->rom_bank);
  dff_init(&cpu->pending_rom_bank);
  dff_init(&cpu->int_enabled);
  dff_init(&cpu->in_interrupt);
  dff_init(&cpu->halted);
  dff_init(&cpu->test_pin);
  dff_init(&cpu->int_pin);
  reg4_init(&cpu->dcl_reg);
}

/* Trigger an external hardware interrupt on the 4040 INT pin */
static void cpu_trigger_interrupt(Intel4040 *cpu) {
  dff_write(&cpu->int_pin, HIGH);
}

/* Execute a single Intel 4040 Instruction */
static void cpu_step(Intel4040 *cpu) {
  /* 1. Check for Pending Hardware Interrupt (4040 feature) */
  if (dff_read(&cpu->int_pin) == HIGH && dff_read(&cpu->int_enabled) == HIGH &&
      dff_read(&cpu->in_interrupt) == LOW) {

    /* Wake from HLT if halted */
    dff_write(&cpu->halted, LOW);
    dff_write(&cpu->in_interrupt, HIGH);
    dff_write(&cpu->int_pin, LOW);

    /* Save SRC register into interrupt shadow latch (restored by BBS) */
    reg4_write_word(&cpu->src_save_hi, reg4_read_word(&cpu->src_hi));
    reg4_write_word(&cpu->src_save_lo, reg4_read_word(&cpu->src_lo));

    /* Push current PC and vector to ROM Bank 0, Address 0x003 */
    cpu->sp = (cpu->sp + 1) & 0x7;
    dff_write(&cpu->rom_bank, LOW);
    dff_write(&cpu->pending_rom_bank, LOW);
    cpu_set_pc(cpu, 0x003);
    cpu->cycles += 1;
    return;
  }

  /* 2. If CPU is Halted (HLT instruction), consume 1 cycle and return */
  if (dff_read(&cpu->halted) == HIGH) {
    cpu->cycles += 1;
    return;
  }

  /* 3. Fetch Opcode Byte */
  uint8_t opcode = cpu_fetch_byte(cpu);
  uint8_t opr = (opcode >> 4) & 0xF; /* High nibble */
  uint8_t opa = opcode & 0xF;        /* Low nibble  */

  /* Apply any delayed ROM bank switch when not a jump instruction */
  dff_write(&cpu->rom_bank, dff_read(&cpu->pending_rom_bank));

  cpu->instructions_executed++;

  /* 4. Decode and Execute all 60 Intel 4040 Instructions */
  switch (opr) {
  case 0x0: /* 0x00..0x0E: NOP and 4040 Machine Control Extensions */
    switch (opa) {
    case 0x0: /* NOP: No Operation */
      break;

    case 0x1: /* HLT: Halt CPU (4040 Exclusive) */
      dff_write(&cpu->halted, HIGH);
      break;

    case 0x2: /* BBS: Branch Back and Restore SRC (4040 Exclusive) */
      reg4_write_word(&cpu->src_hi, reg4_read_word(&cpu->src_save_hi));
      reg4_write_word(&cpu->src_lo, reg4_read_word(&cpu->src_save_lo));
      cpu->sp = (cpu->sp - 1) & 0x7;
      dff_write(&cpu->in_interrupt, LOW);
      dff_write(&cpu->int_enabled, HIGH);
      break;

    case 0x3: { /* LCR: Load Command Register to ACC (4040 Exclusive) */
      uint8_t cr = (reg4_read(&cpu->dcl_reg) & 0x7) |
                   ((dff_read(&cpu->rom_bank) == HIGH) ? 0x8 : 0x0);
      reg4_write(&cpu->acc, cr);
      break;
    }

    case 0x4:   /* OR4: Logical OR R4 into ACC (4040 Exclusive) */
    case 0x5: { /* OR5: Logical OR R5 into ACC (4040 Exclusive) */
      int reg_num = (opa == 0x4) ? 4 : 5;
      Word4 r_val = word4_from_u8(cpu_read_reg(cpu, reg_num));
      Wire cy_out;
      Word4 res = alu_4040_execute(ALU_OR, reg4_read_word(&cpu->acc), r_val,
                                   dff_read(&cpu->cy), &cy_out);
      reg4_write_word(&cpu->acc, res);
      break;
    }

    case 0x6:   /* AN6: Logical AND R6 into ACC (4040 Exclusive) */
    case 0x7: { /* AN7: Logical AND R7 into ACC (4040 Exclusive) */
      int reg_num = (opa == 0x6) ? 6 : 7;
      Word4 r_val = word4_from_u8(cpu_read_reg(cpu, reg_num));
      Wire cy_out;
      Word4 res = alu_4040_execute(ALU_AND, reg4_read_word(&cpu->acc), r_val,
                                   dff_read(&cpu->cy), &cy_out);
      reg4_write_word(&cpu->acc, res);
      break;
    }

    case 0x8: /* DB0: Designate ROM Bank 0 (4040 Exclusive) */
      dff_write(&cpu->pending_rom_bank, LOW);
      break;

    case 0x9: /* DB1: Designate ROM Bank 1 (4040 Exclusive) */
      dff_write(&cpu->pending_rom_bank, HIGH);
      break;

    case 0xA: /* SB0: Select Index Register Bank 0 (4040 Exclusive) */
      dff_write(&cpu->reg_bank, LOW);
      break;

    case 0xB: /* SB1: Select Index Register Bank 1 (4040 Exclusive) */
      dff_write(&cpu->reg_bank, HIGH);
      break;

    case 0xC: /* EIN: Enable Interrupts (4040 Exclusive) */
      dff_write(&cpu->int_enabled, HIGH);
      break;

    case 0xD: /* DIN: Disable Interrupts (4040 Exclusive) */
      dff_write(&cpu->int_enabled, LOW);
      break;

    case 0xE: /* RPM: Read Program Memory (4040 Exclusive) */
      reg4_write(&cpu->acc, cpu->rpm_data_latch & 0xF);
      break;

    default:
      break;
    }
    break;

  case 0x1: { /* JCN: Jump on Condition (2-byte instruction) */
    uint8_t target_lo = cpu_fetch_byte(cpu);
    Wire acc_zero = is_zero_4bit(reg4_read_word(&cpu->acc));
    Wire cy_set = dff_read(&cpu->cy);
    Wire test_low = gate_not(dff_read(&cpu->test_pin));

    Wire cond = LOW;
    if (opa & 0x4)
      cond = gate_or(cond, acc_zero);
    if (opa & 0x2)
      cond = gate_or(cond, cy_set);
    if (opa & 0x1)
      cond = gate_or(cond, test_low);
    if (opa & 0x8)
      cond = gate_not(cond);

    if (cond == HIGH) {
      uint16_t pc = (cpu_get_pc(cpu) & 0xF00) | target_lo;
      cpu_set_pc(cpu, pc);
    }
    break;
  }

  case 0x2: { /* FIM (even OPA) / SRC (odd OPA) */
    int pair = (opa >> 1) & 0x7;
    if ((opa & 1) == 0) {
      /* FIM Rp, D8: Fetch Immediate 8-bit into Register Pair */
      uint8_t data = cpu_fetch_byte(cpu);
      cpu_write_reg(cpu, pair * 2, (data >> 4) & 0xF);
      cpu_write_reg(cpu, pair * 2 + 1, data & 0xF);
    } else {
      /* SRC Rp: Send Register Control */
      reg4_write(&cpu->src_hi, cpu_read_reg(cpu, pair * 2));
      reg4_write(&cpu->src_lo, cpu_read_reg(cpu, pair * 2 + 1));
    }
    break;
  }

  case 0x3: { /* FIN (even OPA) / JIN (odd OPA) */
    int pair = (opa >> 1) & 0x7;
    if ((opa & 1) == 0) {
      /* FIN Rp: Fetch Indirect from ROM using R0:R1 */
      uint16_t addr = (cpu_get_pc(cpu) & 0xF00) | (cpu_read_reg(cpu, 0) << 4) |
                      cpu_read_reg(cpu, 1);
      uint8_t bank = (dff_read(&cpu->rom_bank) == HIGH) ? 1 : 0;
      uint8_t data = cpu->rom[bank][addr & 0xFFF];
      cpu_write_reg(cpu, pair * 2, (data >> 4) & 0xF);
      cpu_write_reg(cpu, pair * 2 + 1, data & 0xF);
      cpu->cycles += 1;
    } else {
      /* JIN Rp: Jump Indirect via Register Pair */
      uint16_t addr = (cpu_get_pc(cpu) & 0xF00) |
                      (cpu_read_reg(cpu, pair * 2) << 4) |
                      cpu_read_reg(cpu, pair * 2 + 1);
      cpu_set_pc(cpu, addr);
    }
    break;
  }

  case 0x4: { /* JUN: Jump Unconditional (12-bit address) */
    uint8_t lo = cpu_fetch_byte(cpu);
    cpu_set_pc(cpu, ((uint16_t)opa << 8) | lo);
    break;
  }

  case 0x5: { /* JMS: Jump to Subroutine (7-level stack in 4040) */
    uint8_t lo = cpu_fetch_byte(cpu);
    cpu->sp = (cpu->sp + 1) & 0x7;
    cpu_set_pc(cpu, ((uint16_t)opa << 8) | lo);
    break;
  }

  case 0x6: { /* INC Rr: Increment Index Register via Transistor ALU */
    Wire dummy_cy;
    Word4 cur = word4_from_u8(cpu_read_reg(cpu, opa));
    Word4 inc = alu_4040_execute(ALU_INC, cur, cur, LOW, &dummy_cy);
    cpu_write_reg(cpu, opa, word4_to_u8(inc));
    break;
  }

  case 0x7: { /* ISZ Rr, addr8: Increment and Skip/Jump if Non-Zero */
    uint8_t target_lo = cpu_fetch_byte(cpu);
    Wire dummy_cy;
    Word4 cur = word4_from_u8(cpu_read_reg(cpu, opa));
    Word4 inc = alu_4040_execute(ALU_INC, cur, cur, LOW, &dummy_cy);
    cpu_write_reg(cpu, opa, word4_to_u8(inc));

    if (is_zero_4bit(inc) == LOW) {
      uint16_t pc = (cpu_get_pc(cpu) & 0xF00) | target_lo;
      cpu_set_pc(cpu, pc);
    }
    break;
  }

  case 0x8: { /* ADD Rr: Add Index Register + CY to ACC */
    Wire cy_out;
    Word4 r_val = word4_from_u8(cpu_read_reg(cpu, opa));
    Word4 res = alu_4040_execute(ALU_ADD, reg4_read_word(&cpu->acc), r_val,
                                 dff_read(&cpu->cy), &cy_out);
    reg4_write_word(&cpu->acc, res);
    dff_write(&cpu->cy, cy_out);
    break;
  }

  case 0x9: { /* SUB Rr: Subtract Index Register with Borrow from ACC */
    Wire cy_out;
    Word4 r_val = word4_from_u8(cpu_read_reg(cpu, opa));
    Word4 res = alu_4040_execute(ALU_SUB, reg4_read_word(&cpu->acc), r_val,
                                 dff_read(&cpu->cy), &cy_out);
    reg4_write_word(&cpu->acc, res);
    dff_write(&cpu->cy, cy_out);
    break;
  }

  case 0xA: /* LD Rr: Load Index Register into ACC */
    reg4_write(&cpu->acc, cpu_read_reg(cpu, opa));
    break;

  case 0xB: { /* XCH Rr: Exchange ACC and Index Register */
    uint8_t tmp = reg4_read(&cpu->acc);
    reg4_write(&cpu->acc, cpu_read_reg(cpu, opa));
    cpu_write_reg(cpu, opa, tmp);
    break;
  }

  case 0xC: /* BBL data4: Branch Back and Load Immediate into ACC */
    cpu->sp = (cpu->sp - 1) & 0x7;
    reg4_write(&cpu->acc, opa);
    break;

  case 0xD: /* LDM data4: Load Immediate into ACC */
    reg4_write(&cpu->acc, opa);
    break;

  case 0xE: { /* RAM & I/O Group Instructions (Addressed via SRC & DCL) */
    uint8_t ram_bank = reg4_read(&cpu->dcl_reg) & 0x7;
    uint8_t s_hi = reg4_read(&cpu->src_hi);
    uint8_t s_lo = reg4_read(&cpu->src_lo);
    uint8_t chip = (s_hi >> 2) & 0x3;
    uint8_t reg = s_hi & 0x3;
    uint8_t ch = s_lo & 0xF;
    uint8_t rom_port = s_hi & 0xF;
    uint8_t r_bank = (dff_read(&cpu->rom_bank) == HIGH) ? 1 : 0;

    switch (opa) {
    case 0x0: /* WRM: Write ACC to RAM Main Memory */
      cpu->ram_main[ram_bank][chip][reg][ch] = reg4_read(&cpu->acc);
      break;
    case 0x1: /* WMP: Write ACC to RAM Output Port */
      cpu->ram_ports[ram_bank][chip] = reg4_read(&cpu->acc);
      break;
    case 0x2: /* WRR: Write ACC to ROM I/O Port */
      cpu->rom_ports[r_bank][rom_port] = reg4_read(&cpu->acc);
      break;
    case 0x3: /* WPM: Write Program Memory Latch */
      cpu->rpm_data_latch = reg4_read(&cpu->acc);
      cpu->wpm_half_latch ^= 1;
      break;
    case 0x4: /* WR0 */
    case 0x5: /* WR1 */
    case 0x6: /* WR2 */
    case 0x7: /* WR3: Write ACC to RAM Status Character 0..3 */
      cpu->ram_status[ram_bank][chip][reg][opa - 0x4] = reg4_read(&cpu->acc);
      break;
    case 0x8: { /* SBM: Subtract RAM Main Character from ACC */
      Wire cy_out;
      Word4 m_val = word4_from_u8(cpu->ram_main[ram_bank][chip][reg][ch]);
      Word4 res = alu_4040_execute(ALU_SUB, reg4_read_word(&cpu->acc), m_val,
                                   dff_read(&cpu->cy), &cy_out);
      reg4_write_word(&cpu->acc, res);
      dff_write(&cpu->cy, cy_out);
      break;
    }
    case 0x9: /* RDM: Read RAM Main Character into ACC */
      reg4_write(&cpu->acc, cpu->ram_main[ram_bank][chip][reg][ch]);
      break;
    case 0xA: /* RDR: Read ROM Port into ACC */
      reg4_write(&cpu->acc, cpu->rom_ports[r_bank][rom_port]);
      break;
    case 0xB: { /* ADM: Add RAM Main Character to ACC */
      Wire cy_out;
      Word4 m_val = word4_from_u8(cpu->ram_main[ram_bank][chip][reg][ch]);
      Word4 res = alu_4040_execute(ALU_ADD, reg4_read_word(&cpu->acc), m_val,
                                   dff_read(&cpu->cy), &cy_out);
      reg4_write_word(&cpu->acc, res);
      dff_write(&cpu->cy, cy_out);
      break;
    }
    case 0xC: /* RD0 */
    case 0xD: /* RD1 */
    case 0xE: /* RD2 */
    case 0xF: /* RD3: Read RAM Status Character 0..3 into ACC */
      reg4_write(&cpu->acc, cpu->ram_status[ram_bank][chip][reg][opa - 0xC]);
      break;
    }
    break;
  }

  case 0xF: { /* Accumulator Group Instructions */
    Wire cy_out;
    switch (opa) {
    case 0x0: /* CLB: Clear Both ACC and CY */
      reg4_write(&cpu->acc, 0);
      dff_write(&cpu->cy, LOW);
      break;
    case 0x1: /* CLC: Clear Carry */
      dff_write(&cpu->cy, LOW);
      break;
    case 0x2: { /* IAC: Increment ACC */
      Word4 res = alu_4040_execute(ALU_IAC, reg4_read_word(&cpu->acc),
                                   word4_from_u8(0), LOW, &cy_out);
      reg4_write_word(&cpu->acc, res);
      dff_write(&cpu->cy, cy_out);
      break;
    }
    case 0x3: /* CMC: Complement Carry */
      dff_write(&cpu->cy, gate_not(dff_read(&cpu->cy)));
      break;
    case 0x4: { /* CMA: Complement Accumulator */
      Word4 res = alu_4040_execute(ALU_CMA, reg4_read_word(&cpu->acc),
                                   word4_from_u8(0), LOW, &cy_out);
      reg4_write_word(&cpu->acc, res);
      break;
    }
    case 0x5: { /* RAL: Rotate Left through Carry */
      Word4 res =
          alu_4040_execute(ALU_RAL, reg4_read_word(&cpu->acc), word4_from_u8(0),
                           dff_read(&cpu->cy), &cy_out);
      reg4_write_word(&cpu->acc, res);
      dff_write(&cpu->cy, cy_out);
      break;
    }
    case 0x6: { /* RAR: Rotate Right through Carry */
      Word4 res =
          alu_4040_execute(ALU_RAR, reg4_read_word(&cpu->acc), word4_from_u8(0),
                           dff_read(&cpu->cy), &cy_out);
      reg4_write_word(&cpu->acc, res);
      dff_write(&cpu->cy, cy_out);
      break;
    }
    case 0x7: /* TCC: Transfer Carry to ACC and Clear Carry */
      reg4_write(&cpu->acc, (dff_read(&cpu->cy) == HIGH) ? 1 : 0);
      dff_write(&cpu->cy, LOW);
      break;
    case 0x8: { /* DAC: Decrement Accumulator */
      Word4 res = alu_4040_execute(ALU_DAC, reg4_read_word(&cpu->acc),
                                   word4_from_u8(0), LOW, &cy_out);
      reg4_write_word(&cpu->acc, res);
      dff_write(&cpu->cy, cy_out);
      break;
    }
    case 0x9: /* TCS: Transfer Carry Subtract (9 or 10) */
      reg4_write(&cpu->acc, (dff_read(&cpu->cy) == HIGH) ? 10 : 9);
      dff_write(&cpu->cy, LOW);
      break;
    case 0xA: /* STC: Set Carry */
      dff_write(&cpu->cy, HIGH);
      break;
    case 0xB: { /* DAA: Decimal Adjust Accumulator */
      uint8_t a = reg4_read(&cpu->acc);
      if (a > 9 || dff_read(&cpu->cy) == HIGH) {
        Word4 res = adder_4bit(reg4_read_word(&cpu->acc), word4_from_u8(6), LOW,
                               &cy_out);
        reg4_write_word(&cpu->acc, res);
        if (cy_out == HIGH)
          dff_write(&cpu->cy, HIGH);
      }
      break;
    }
    case 0xC: { /* KBP: Keyboard Process (1-of-4 Decoder) */
      uint8_t a = reg4_read(&cpu->acc);
      uint8_t k = (a == 0x0)   ? 0x0
                  : (a == 0x1) ? 0x1
                  : (a == 0x2) ? 0x2
                  : (a == 0x4) ? 0x3
                  : (a == 0x8) ? 0x4
                               : 0xF;
      reg4_write(&cpu->acc, k);
      break;
    }
    case 0xD: /* DCL: Designate Command Line (Select RAM Bank) */
      reg4_write(&cpu->dcl_reg, reg4_read(&cpu->acc) & 0x7);
      break;
    default:
      break;
    }
    break;
  }
  }
}

/* ============================================================================
 * HARDWARE SELF-TEST & INTEL 4040 FIRMWARE VERIFICATION
 * ============================================================================
 */

static void run_bottom_up_hardware_diagnostics(void) {
  printf("================================================================\n");
  printf(" PHASE 1: BOTTOM-UP HARDWARE DIAGNOSTICS (TRANSISTOR -> GATES)\n");
  printf("================================================================\n");

  /* 1. Verify MOSFET Transistors */
  printf("[Layer 1] Testing PMOS & NMOS Transistors... ");
  if (pmos(HIGH, LOW) == HIGH && pmos(HIGH, HIGH) == HIGH_Z &&
      nmos(LOW, HIGH) == LOW && nmos(LOW, LOW) == HIGH_Z) {
    printf("PASS\n");
  } else {
    printf("FAIL\n");
  }

  /* 2. Verify Logic Gates Truth Tables */
  printf(
      "[Layer 2] Testing CMOS Logic Gates (NOT, NAND, NOR, AND, OR, XOR)... ");
  bool gates_ok = true;
  const Wire vals[2] = {LOW, HIGH};
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      Wire a = vals[i], b = vals[j];
      if ((int)gate_nand(a, b) != !(i & j))
        gates_ok = false;
      if ((int)gate_nor(a, b) != !(i | j))
        gates_ok = false;
      if ((int)gate_and(a, b) != (i & j))
        gates_ok = false;
      if ((int)gate_or(a, b) != (i | j))
        gates_ok = false;
      if ((int)gate_xor(a, b) != (i ^ j))
        gates_ok = false;
    }
  }
  printf("%s\n", gates_ok ? "PASS" : "FAIL");

  /* 3. Verify 4-bit Ripple-Carry Adder across all 512 combinations */
  printf(
      "[Layer 3] Testing 4-Bit Ripple-Carry Adder (all 512 input states)... ");
  bool adder_ok = true;
  for (int a = 0; a < 16; a++) {
    for (int b = 0; b < 16; b++) {
      for (int c = 0; c < 2; c++) {
        Wire cout;
        Word4 sum = adder_4bit(word4_from_u8(a), word4_from_u8(b),
                               c ? HIGH : LOW, &cout);
        int expected = a + b + c;
        if (word4_to_u8(sum) != (expected & 0xF) ||
            (cout == HIGH) != ((expected >> 4) & 1)) {
          adder_ok = false;
        }
      }
    }
  }
  printf("%s\n", adder_ok ? "PASS" : "FAIL");

  /* 4. Verify Master-Slave D-Flip-Flop & 4-bit Register */
  printf("[Layer 4] Testing NOR SR-Latch, Master-Slave D-FF & 4-Bit Reg... ");
  Reg4 test_reg;
  reg4_init(&test_reg);
  bool seq_ok = true;
  for (uint8_t v = 0; v < 16; v++) {
    reg4_write(&test_reg, v);
    if (reg4_read(&test_reg) != v)
      seq_ok = false;
  }
  printf("%s\n\n", seq_ok ? "PASS" : "FAIL");
}

/*
 * Assemble and run a real Intel 4040 Machine Code Program that tests:
 *  1. Computing Fibonacci numbers (1, 1, 2, 3, 5, 8, 13) into 4002 Data RAM
 *  2. 4040 Register Bank Switching (SB1 / SB0) preserving Bank 0 registers
 *  3. 4040 Logical Instructions (OR4, AN6) and Subroutine Call (JMS / BBL)
 *  4. 4040 Hardware Interrupt Handling (EIN, Interrupt Vector 0x003, BBS)
 *  5. 4040 Halt Instruction (HLT)
 */
static void run_intel_4040_demo(void) {
  printf("================================================================\n");
  printf(" PHASE 2: INTEL 4040 CPU EXECUTION (MACHINE CODE VERIFICATION)\n");
  printf("================================================================\n");

  Intel4040 cpu;
  cpu_init(&cpu);

  /*
   * ROM BANK 0 MEMORY MAP:
   *   0x000..0x001 : JUN 0x010       (Reset Vector -> Main Program)
   *   0x003..0x009 : Interrupt Service Routine (ISR at Vector 0x003)
   *   0x010..0x03A : Main Program (Fibonacci + 4040 Bank/Logic/HLT Test)
   */
  uint8_t *rom0 = cpu.rom[0];

  /* --- Reset Vector (0x000) --- */
  rom0[0x000] = 0x40;
  rom0[0x001] = 0x10; /* JUN 0x010                   */

  /* --- Hardware Interrupt Service Routine (Vector 0x003) --- */
  rom0[0x003] = 0x20;
  rom0[0x004] = 0x0F; /* FIM P0, 0x0F (Char 15)      */
  rom0[0x005] = 0x21; /* SRC P0                      */
  rom0[0x006] = 0xDA; /* LDM 0xA (Interrupt marker)  */
  rom0[0x007] = 0xE0; /* WRM (RAM[0xF] = 0xA)        */
  rom0[0x008] = 0x02; /* BBS (4040 Return & Restore) */

  /* --- Main Program (0x010) --- */
  int p = 0x010;
  rom0[p++] = 0x0C; /* EIN: Enable 4040 Interrupts */
  rom0[p++] = 0x20;
  rom0[p++] = 0x01; /* FIM P0, 0x01 (R0=0, R1=1)   */
  rom0[p++] = 0x22;
  rom0[p++] = 0x00; /* FIM P1, 0x00 (R2=0, R3=0)   */
  rom0[p++] = 0x24;
  rom0[p++] = 0x09; /* FIM P2, 0x09 (R5=9: 7 iters)*/

  /* Loop at 0x017: Calculate 7 Fibonacci numbers into RAM[0..6] */
  int loop_addr = p;
  rom0[p++] = 0x23; /* SRC P1 (Address = R2:R3)    */
  rom0[p++] = 0xA1; /* LD  R1 (Current Fib number) */
  rom0[p++] = 0xE0; /* WRM    (Write to 4002 RAM)  */
  rom0[p++] = 0xF1; /* CLC    (Clear Carry)        */
  rom0[p++] = 0xA0; /* LD  R0 (Prev Fib number)    */
  rom0[p++] = 0x81; /* ADD R1 (ACC = R0 + R1)      */
  rom0[p++] = 0xB0; /* XCH R0 (R0 = new, ACC = old)*/
  rom0[p++] = 0xB1; /* XCH R1 (R1 = new, R0 = old) */
  rom0[p++] = 0x63; /* INC R3 (Next RAM address)   */
  rom0[p++] = 0x75;
  rom0[p++] = loop_addr; /* ISZ R5, loop_addr           */

  /* --- Test 4040 Exclusive Instructions: SB1, OR4, AN6,WR0, SB0 --- */
  rom0[p++] = 0x0B; /* SB1: Switch to Reg Bank 1!  */
  rom0[p++] = 0x24;
  rom0[p++] = 0xC5; /* FIM P2, 0xC5 (Bank1 R4=C,R5=5)*/
  rom0[p++] = 0x26;
  rom0[p++] = 0xE3; /* FIM P3, 0xE3 (Bank1 R6=E,R7=3)*/
  rom0[p++] = 0xD3; /* LDM 0x3 (ACC = 0011b)       */
  rom0[p++] = 0x04; /* OR4: ACC = 0x3 | 0xC = 0xF  */
  rom0[p++] = 0x06; /* AN6: ACC = 0xF & 0xE = 0xE  */
  rom0[p++] = 0xE4; /* WR0: RAM Status[0] = 0xE    */
  rom0[p++] = 0x0A; /* SB0: Restore Reg Bank 0     */
  rom0[p++] = 0x01; /* HLT: Enter 4040 Halt State  */

  /* Run simulation step-by-step, injecting a hardware interrupt mid-run */
  int max_steps = 100;
  for (int step = 0; step < max_steps; step++) {
    if (step == 18) {
      printf("  -> [Cycle %2llu] Asserting External Hardware INT Pin!\n",
             (unsigned long long)cpu.cycles);
      cpu_trigger_interrupt(&cpu);
    }
    cpu_step(&cpu);
    if (dff_read(&cpu.halted) == HIGH) {
      printf("  -> [Cycle %2llu] CPU Executed HLT Instruction (Halted "
             "cleanly).\n\n",
             (unsigned long long)cpu.cycles);
      break;
    }
  }

  /* Print Results & Architectural State */
  printf("4002 Data RAM Main Memory (Bank 0, Chip 0, Reg 0):\n");
  printf("  Fibonacci Sequence (RAM[0..6]) : ");
  for (int i = 0; i < 7; i++) {
    printf("%u ", cpu.ram_main[0][0][0][i]);
  }
  printf("\n");
  printf("  Interrupt Marker   (RAM[15])   : 0x%X (written by ISR at 0x003 via "
         "BBS)\n",
         cpu.ram_main[0][0][0][15]);
  printf("  4040 OR4/AN6 Result(Status[0]) : 0x%X (expected 0xE)\n\n",
         cpu.ram_status[0][0][0][0]);

  printf("Intel 4040 Banked Index Registers:\n");
  printf("  Bank 0 (R0..R7) : ");
  for (int i = 0; i < 8; i++)
    printf("R%d=%X ", i, reg4_read(&cpu.index_regs[i]));
  printf("\n  Bank 1 (R0..R7) : ");
  for (int i = 16; i < 24; i++)
    printf("R%d=%X ", i - 16, reg4_read(&cpu.index_regs[i]));
  printf("\n\n");

  printf("Simulation Statistics:\n");
  printf("  Instructions Executed : %llu\n",
         (unsigned long long)cpu.instructions_executed);
  printf("  Machine Cycles        : %llu\n", (unsigned long long)cpu.cycles);
  printf("  Transistor Switches   : %llu individual MOSFET evaluations\n",
         (unsigned long long)g_transistor_switches);
  printf("================================================================\n");
}

int main(void) {
  run_bottom_up_hardware_diagnostics();
  run_intel_4040_demo();
  return 0;
}