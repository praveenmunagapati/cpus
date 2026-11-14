/*
 * ============================================================================
 * FROM TRANSISTOR TO SPEC-EXACT INTEL 8086 CPU (WITH MICROCODE ENGINE) IN C
 * ============================================================================
 * Layer 1: Switch-Level PMOS & NMOS Transistors
 * Layer 2: CMOS Logic Gates (NOT, NAND, NOR, AND, OR, XOR, XNOR)
 * Layer 3: 8/16/20-Bit Transistor Adders & Spec-Exact 8086 ALU (SIGMA + 9
 * Flags) Layer 4: NOR SR-Latch -> Master-Slave D-Flip-Flop -> 16-Bit Register
 * File Layer 5: Bus Interface Unit (BIU), 20-Bit Segment Adder & 6-Byte Queue
 * Layer 6: Execution Unit (EU), Translation PLA & 21-Bit Microcode Engine
 * ============================================================================
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


static uint64_t g_transistor_switches = 0;

/* ============================================================================
 * LAYER 1: SEMICONDUCTOR PHYSICS (WIRES & MOSFET TRANSISTORS)
 * ============================================================================
 */

typedef enum { LOW = 0, HIGH = 1, HIGH_Z = 2, WIRE_ERR = 3 } Wire;

static inline Wire wire_resolve(Wire a, Wire b) {
  if (a == HIGH_Z)
    return b;
  if (b == HIGH_Z)
    return a;
  if (a == b)
    return a;
  return WIRE_ERR;
}

static inline Wire pmos(Wire source, Wire gate) {
  g_transistor_switches++;
  return (gate == LOW) ? source : HIGH_Z;
}

static inline Wire nmos(Wire source, Wire gate) {
  g_transistor_switches++;
  return (gate == HIGH) ? source : HIGH_Z;
}

/* ============================================================================
 * LAYER 2: FUNDAMENTAL LOGIC GATES (BUILT EXCLUSIVELY FROM TRANSISTORS)
 * ============================================================================
 */

static inline Wire gate_not(Wire in) {
  return wire_resolve(pmos(HIGH, in), nmos(LOW, in));
}

static inline Wire gate_nand(Wire a, Wire b) {
  Wire pull_up = wire_resolve(pmos(HIGH, a), pmos(HIGH, b));
  Wire pull_down = nmos(nmos(LOW, b), a);
  return wire_resolve(pull_up, pull_down);
}

static inline Wire gate_nor(Wire a, Wire b) {
  Wire pull_up = pmos(pmos(HIGH, a), b);
  Wire pull_down = wire_resolve(nmos(LOW, a), nmos(LOW, b));
  return wire_resolve(pull_up, pull_down);
}

static inline Wire gate_and(Wire a, Wire b) {
  return gate_not(gate_nand(a, b));
}

static inline Wire gate_or(Wire a, Wire b) { return gate_not(gate_nor(a, b)); }

static inline Wire gate_xor(Wire a, Wire b) {
  Wire n1 = gate_nand(a, b);
  Wire n2 = gate_nand(a, n1);
  Wire n3 = gate_nand(b, n1);
  return gate_nand(n2, n3);
}

static inline Wire gate_xnor(Wire a, Wire b) {
  return gate_not(gate_xor(a, b));
}

/* ============================================================================
 * LAYER 3: COMBINATIONAL LOGIC, 16/20-BIT ADDERS & SPEC-EXACT 8086 ALU
 * ============================================================================
 */

typedef struct {
  Wire b[16];
} Word16;
typedef struct {
  Wire b[20];
} Word20;

static inline Word16 word16_from_u16(uint16_t val) {
  Word16 w;
  for (int i = 0; i < 16; i++)
    w.b[i] = ((val >> i) & 1) ? HIGH : LOW;
  return w;
}

static inline uint16_t word16_to_u16(Word16 w) {
  uint16_t val = 0;
  for (int i = 0; i < 16; i++) {
    if (w.b[i] == HIGH)
      val |= (uint16_t)(1u << i);
  }
  return val;
}

static inline void half_adder(Wire a, Wire b, Wire *sum, Wire *cout) {
  *sum = gate_xor(a, b);
  *cout = gate_and(a, b);
}

static inline void full_adder(Wire a, Wire b, Wire cin, Wire *sum, Wire *cout) {
  Wire s1, c1, c2;
  half_adder(a, b, &s1, &c1);
  half_adder(s1, cin, sum, &c2);
  *cout = gate_or(c1, c2);
}

/*
 * 16-Bit Ripple-Carry Adder with Internal Carry Chain Tap:
 * Exposes all 16 carry-out wires so the 8086 ALU can tap:
 *   - carries[3]  -> AF (Auxiliary Carry out of bit 3 into bit 4)
 *   - carries[6]  XOR carries[7]  -> 8-bit OF (Overflow)
 *   - carries[14] XOR carries[15] -> 16-bit OF (Overflow)
 */
static inline Word16 adder_16bit(Word16 a, Word16 b, Wire cin,
                                 Wire carries[16]) {
  Word16 sum;
  Wire c = cin;
  for (int i = 0; i < 16; i++) {
    full_adder(a.b[i], b.b[i], c, &sum.b[i], &c);
    carries[i] = c;
  }
  return sum;
}

/*
 * 20-Bit BIU Physical Address Adder:
 * Computes Physical Address = (Segment << 4) + Offset using 20 Full Adders.
 */
static uint32_t biu_adder_20bit(uint16_t segment, uint16_t offset) {
  Word20 seg20, off20, out20;
  for (int i = 0; i < 20; i++) {
    seg20.b[i] = (i >= 4 && ((segment >> (i - 4)) & 1)) ? HIGH : LOW;
    off20.b[i] = (i < 16 && ((offset >> i) & 1)) ? HIGH : LOW;
  }
  Wire c = LOW;
  for (int i = 0; i < 20; i++) {
    full_adder(seg20.b[i], off20.b[i], c, &out20.b[i], &c);
  }
  uint32_t phys = 0;
  for (int i = 0; i < 20; i++) {
    if (out20.b[i] == HIGH)
      phys |= (1u << i);
  }
  return phys;
}

/* 8086 Architectural Flag Bit Positions */
#define FLAG_CF (1u << 0)  /* Carry Flag             */
#define FLAG_PF (1u << 2)  /* Parity Flag (low byte) */
#define FLAG_AF (1u << 4)  /* Auxiliary Carry Flag   */
#define FLAG_ZF (1u << 6)  /* Zero Flag              */
#define FLAG_SF (1u << 7)  /* Sign Flag              */
#define FLAG_TF (1u << 8)  /* Trap Flag              */
#define FLAG_IF (1u << 9)  /* Interrupt Enable Flag  */
#define FLAG_DF (1u << 10) /* Direction Flag         */
#define FLAG_OF (1u << 11) /* Overflow Flag          */

/* Spec-exact 8086 FLAGS mask: bits 15..12=1, bit 1=1, bits 3,5=0 */
static inline uint16_t sanitize_8086_flags(uint16_t f) {
  return (f & 0x0FD5u) | 0xF002u;
}

/* 8-Bit Even Parity Generator (XNOR tree on bits 0..7) */
static inline Wire parity_even_8bit(Word16 w) {
  Wire x01 = gate_xor(w.b[0], w.b[1]);
  Wire x23 = gate_xor(w.b[2], w.b[3]);
  Wire x45 = gate_xor(w.b[4], w.b[5]);
  Wire x67 = gate_xor(w.b[6], w.b[7]);
  Wire x03 = gate_xor(x01, x23);
  Wire x47 = gate_xor(x45, x67);
  return gate_xnor(x03, x47); /* HIGH when even number of 1s */
}

/* 8/16-Bit Zero Detector (NOR tree across 8 or 16 wires) */
static inline Wire zero_detect(Word16 w, bool is_word) {
  int bits = is_word ? 16 : 8;
  Wire acc = LOW;
  for (int i = 0; i < bits; i++) {
    acc = gate_or(acc, w.b[i]);
  }
  return gate_not(acc);
}

/* 8086 EU ALU Operations */
typedef enum {
  ALU86_NONE = 0,
  ALU86_ADD,
  ALU86_OR,
  ALU86_ADC,
  ALU86_SBB,
  ALU86_AND,
  ALU86_SUB,
  ALU86_XOR,
  ALU86_CMP,
  ALU86_INC,
  ALU86_DEC,
  ALU86_NOT,
  ALU86_NEG,
  ALU86_PASS_A,
  ALU86_SHL,
  ALU86_SHR,
  ALU86_SAR,
  ALU86_ROL,
  ALU86_ROR,
  ALU86_RCL,
  ALU86_RCR
} Alu86Op;

/*
 * Spec-Exact Intel 8086 8/16-Bit Transistor ALU (Computes SIGMA & Flags)
 */
static Word16 alu_8086_execute(Alu86Op op, Word16 a, Word16 b, bool is_word,
                               uint16_t *flags, bool update_flags) {
  Word16 sigma = word16_from_u16(0);
  Wire carries[16] = {LOW};
  int msb = is_word ? 15 : 7;

  Wire cf_in = ((*flags) & FLAG_CF) ? HIGH : LOW;
  Wire new_cf = cf_in, new_af = LOW, new_of = LOW;
  bool preserve_cf = false;
  bool is_logical = false;

  switch (op) {
  case ALU86_ADD:
    sigma = adder_16bit(a, b, LOW, carries);
    new_cf = carries[msb];
    new_af = carries[3];
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;

  case ALU86_ADC:
    sigma = adder_16bit(a, b, cf_in, carries);
    new_cf = carries[msb];
    new_af = carries[3];
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;

  case ALU86_SUB:
  case ALU86_CMP: {
    Word16 inv_b;
    for (int i = 0; i < 16; i++)
      inv_b.b[i] = gate_not(b.b[i]);
    sigma = adder_16bit(a, inv_b, HIGH, carries);
    new_cf = gate_not(carries[msb]); /* Borrow is inverted carry */
    new_af = gate_not(carries[3]);
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;
  }

  case ALU86_SBB: {
    /* a - (b + CF) = a + ~b + ~CF */
    Word16 inv_b;
    for (int i = 0; i < 16; i++)
      inv_b.b[i] = gate_not(b.b[i]);
    sigma = adder_16bit(a, inv_b, gate_not(cf_in), carries);
    new_cf = gate_not(carries[msb]);
    new_af = gate_not(carries[3]);
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;
  }

  case ALU86_INC:
    sigma = adder_16bit(a, word16_from_u16(1), LOW, carries);
    new_af = carries[3];
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    preserve_cf = true; /* 8086 INC preserves CF */
    break;

  case ALU86_DEC: {
    Word16 all_ones = word16_from_u16(0xFFFF);
    sigma = adder_16bit(a, all_ones, LOW, carries);
    new_af = gate_not(carries[3]);
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    preserve_cf = true; /* 8086 DEC preserves CF */
    break;
  }

  case ALU86_NEG: {
    Word16 inv_a;
    for (int i = 0; i < 16; i++)
      inv_a.b[i] = gate_not(a.b[i]);
    sigma = adder_16bit(word16_from_u16(0), inv_a, HIGH, carries);
    new_cf = gate_not(zero_detect(a, is_word)); /* CF=1 unless operand was 0 */
    new_af = gate_not(carries[3]);
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;
  }

  case ALU86_NOT:
    for (int i = 0; i < 16; i++)
      sigma.b[i] = gate_not(a.b[i]);
    update_flags = false; /* 8086 NOT affects no flags */
    break;

  case ALU86_AND:
    for (int i = 0; i < 16; i++)
      sigma.b[i] = gate_and(a.b[i], b.b[i]);
    is_logical = true;
    break;

  case ALU86_OR:
    for (int i = 0; i < 16; i++)
      sigma.b[i] = gate_or(a.b[i], b.b[i]);
    is_logical = true;
    break;

  case ALU86_XOR:
    for (int i = 0; i < 16; i++)
      sigma.b[i] = gate_xor(a.b[i], b.b[i]);
    is_logical = true;
    break;

  case ALU86_PASS_A:
    sigma = a;
    update_flags = false;
    break;

  case ALU86_SHL:
    new_cf = a.b[msb];
    sigma.b[0] = LOW;
    for (int i = 1; i <= msb; i++)
      sigma.b[i] = a.b[i - 1];
    new_of = gate_xor(sigma.b[msb], new_cf);
    break;

  case ALU86_SHR:
    new_cf = a.b[0];
    for (int i = 0; i < msb; i++)
      sigma.b[i] = a.b[i + 1];
    sigma.b[msb] = LOW;
    new_of = a.b[msb];
    break;

  case ALU86_SAR:
    new_cf = a.b[0];
    for (int i = 0; i < msb; i++)
      sigma.b[i] = a.b[i + 1];
    sigma.b[msb] = a.b[msb]; /* Preserve sign bit */
    new_of = LOW;
    break;

  case ALU86_ROL:
    new_cf = a.b[msb];
    sigma.b[0] = new_cf;
    for (int i = 1; i <= msb; i++)
      sigma.b[i] = a.b[i - 1];
    new_of = gate_xor(sigma.b[msb], new_cf);
    break;

  case ALU86_ROR:
    new_cf = a.b[0];
    for (int i = 0; i < msb; i++)
      sigma.b[i] = a.b[i + 1];
    sigma.b[msb] = new_cf;
    new_of = gate_xor(sigma.b[msb], sigma.b[msb - 1]);
    break;

  case ALU86_RCL:
    new_cf = a.b[msb];
    sigma.b[0] = cf_in;
    for (int i = 1; i <= msb; i++)
      sigma.b[i] = a.b[i - 1];
    new_of = gate_xor(sigma.b[msb], new_cf);
    break;

  case ALU86_RCR:
    new_cf = a.b[0];
    for (int i = 0; i < msb; i++)
      sigma.b[i] = a.b[i + 1];
    sigma.b[msb] = cf_in;
    new_of = gate_xor(sigma.b[msb], sigma.b[msb - 1]);
    break;

  default:
    sigma = a;
    break;
  }

  if (!is_word) {
    for (int i = 8; i < 16; i++)
      sigma.b[i] = LOW;
  }

  if (update_flags) {
    if (is_logical) {
      new_cf = LOW;
      new_of = LOW;
      new_af = LOW;
    }
    Wire zf = zero_detect(sigma, is_word);
    Wire sf = sigma.b[msb];
    Wire pf = parity_even_8bit(sigma);

    uint16_t f = *flags;
    if (!preserve_cf) {
      f = (new_cf == HIGH) ? (f | FLAG_CF) : (f & ~FLAG_CF);
    }
    f = (pf == HIGH) ? (f | FLAG_PF) : (f & ~FLAG_PF);
    f = (new_af == HIGH) ? (f | FLAG_AF) : (f & ~FLAG_AF);
    f = (zf == HIGH) ? (f | FLAG_ZF) : (f & ~FLAG_ZF);
    f = (sf == HIGH) ? (f | FLAG_SF) : (f & ~FLAG_SF);
    f = (new_of == HIGH) ? (f | FLAG_OF) : (f & ~FLAG_OF);
    *flags = sanitize_8086_flags(f);
  }

  return sigma;
}

/* ============================================================================
 * LAYER 4: SEQUENTIAL LOGIC (SR-LATCH -> D-FLIP-FLOP -> 16-BIT REGISTERS)
 * ============================================================================
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
  for (int i = 0; i < 2; i++) {
    l->q = gate_nor(r, l->q_bar);
    l->q_bar = gate_nor(s, l->q);
  }
}

static inline void d_latch_update(SRLatch *l, Wire d, Wire en) {
  Wire not_d = gate_not(d);
  sr_latch_update(l, gate_and(d, en), gate_and(not_d, en));
}

typedef struct {
  SRLatch master;
  SRLatch slave;
} DFF;

static inline void dff_init(DFF *ff) {
  sr_latch_init(&ff->master);
  sr_latch_init(&ff->slave);
}

static inline void dff_write(DFF *ff, Wire d) {
  /* Clock LOW -> Master samples D; Clock HIGH -> Slave latches Master */
  d_latch_update(&ff->master, d, gate_not(LOW));
  d_latch_update(&ff->slave, ff->master.q, LOW);
  d_latch_update(&ff->master, d, gate_not(HIGH));
  d_latch_update(&ff->slave, ff->master.q, HIGH);
}

static inline Wire dff_read(const DFF *ff) { return ff->slave.q; }

/* 16-Bit Transistor-Backed Register supporting Full Word and Split High/Low
 * Byte */
typedef struct {
  DFF bit[16];
} Reg16;

static inline void reg16_init(Reg16 *r) {
  for (int i = 0; i < 16; i++)
    dff_init(&r->bit[i]);
}

static inline void reg16_write_word(Reg16 *r, Word16 w) {
  for (int i = 0; i < 16; i++)
    dff_write(&r->bit[i], w.b[i]);
}

static inline Word16 reg16_read_word(const Reg16 *r) {
  Word16 w;
  for (int i = 0; i < 16; i++)
    w.b[i] = dff_read(&r->bit[i]);
  return w;
}

static inline void reg16_write(Reg16 *r, uint16_t val) {
  reg16_write_word(r, word16_from_u16(val));
}

static inline uint16_t reg16_read(const Reg16 *r) {
  return word16_to_u16(reg16_read_word(r));
}

static inline void reg16_write_lo(Reg16 *r, uint8_t val) {
  for (int i = 0; i < 8; i++)
    dff_write(&r->bit[i], ((val >> i) & 1) ? HIGH : LOW);
}

static inline void reg16_write_hi(Reg16 *r, uint8_t val) {
  for (int i = 0; i < 8; i++)
    dff_write(&r->bit[i + 8], ((val >> i) & 1) ? HIGH : LOW);
}

static inline uint8_t reg16_read_lo(const Reg16 *r) {
  uint8_t v = 0;
  for (int i = 0; i < 8; i++)
    if (dff_read(&r->bit[i]) == HIGH)
      v |= (1u << i);
  return v;
}

static inline uint8_t reg16_read_hi(const Reg16 *r) {
  uint8_t v = 0;
  for (int i = 0; i < 8; i++)
    if (dff_read(&r->bit[i + 8]) == HIGH)
      v |= (1u << i);
  return v;
}

/* ============================================================================
 * LAYER 5 & 6: INTEL 8086 BIU, EU & 21-BIT MICROCODE ENGINE ARCHITECTURE
 * ============================================================================
 * In the real Intel 8086, microinstructions move data across the 16-bit EU
 * internal bus (SRC -> DST) while simultaneously commanding the ALU, BIU, or
 * Micro-Sequencer.
 */

/* Internal 8086 Microcode Bus Sources & Destinations (5-bit encoded on die) */
typedef enum {
  MBUS_NONE = 0,
  /* Architectural General Registers */
  MBUS_AX,
  MBUS_CX,
  MBUS_DX,
  MBUS_BX,
  MBUS_SP,
  MBUS_BP,
  MBUS_SI,
  MBUS_DI,
  /* Architectural Segment Registers */
  MBUS_ES,
  MBUS_CS,
  MBUS_SS,
  MBUS_DS,
  /* Special & Status Registers */
  MBUS_IP,
  MBUS_FLAGS,
  /* Dynamic Instruction-Decoded Operands (from ModR/M & Opcode) */
  MBUS_R, /* Register specified by ModR/M 'reg' or low 3 opcode bits */
  MBUS_M, /* Register or Memory OPR specified by ModR/M 'r/m'        */
  /* Internal EU Microcode Staging Registers */
  MBUS_TMPa,  /* ALU Input Temporary A                                   */
  MBUS_TMPb,  /* ALU Input Temporary B                                   */
  MBUS_TMPc,  /* Loop / Shift / Multiply Counter Temporary C             */
  MBUS_SIGMA, /* ALU Output Latch (SIGMA)                                */
  /* BIU Interface Registers */
  MBUS_IND,   /* BIU Effective Address Offset Register                   */
  MBUS_OPR,   /* BIU Memory Data Bus Latch                               */
  MBUS_Q,     /* BIU 6-Byte Prefetch Queue Read (1 or 2 bytes by W bit)  */
  MBUS_Q_SXT, /* Sign-Extended Byte from Queue to 16-bit                 */
  /* Hardwired Constants */
  MBUS_ZERO,  /* Constant 0x0000                                         */
  MBUS_ONES,  /* Constant 0xFFFF                                         */
  MBUS_CONST2 /* Constant 1 (if W=0) or 2 (if W=1) for String/Stack ops  */
} MicroBusId;

/* Secondary Action Types in the 21-bit 8086 Microinstruction */
typedef enum {
  MACT_NONE = 0,
  MACT_ALU, /* Configure & evaluate ALU -> SIGMA                       */
  MACT_MEM, /* Execute BIU Memory Bus Cycle using IND & OPR            */
  MACT_JUMP /* Conditional / Unconditional Microcode Branch            */
} MicroActionType;

/* BIU Memory Operations commanded by Microcode */
typedef enum {
  MMEM_NONE = 0,
  MMEM_READ,       /* Read [Seg:IND] -> OPR                               */
  MMEM_WRITE,      /* Write OPR -> [Seg:IND]                              */
  MMEM_WRITE_M,    /* If mod==3: OPR -> R/M reg; else Write OPR->[Seg:IND]*/
  MMEM_PUSH,       /* Write OPR -> [SS:IND] (Stack Push)                  */
  MMEM_POP,        /* Read [SS:IND] -> OPR  (Stack Pop)                   */
  MMEM_FLUSH_QUEUE /* Flush 6-byte BIU Prefetch Queue & sync fetch IP     */
} MicroMemOp;

/* Microcode Branch Conditions */
typedef enum {
  MCOND_UNC = 0, /* Unconditional micro-jump                            */
  MCOND_MOD_REG, /* Jump if ModR/M mod == 0b11 (register-to-register)   */
  MCOND_Z,       /* Jump if Zero Flag (ZF == 1)                         */
  MCOND_NZ       /* Jump if Not Zero (ZF == 0)                          */
} MicroCond;

/*
 * 8086 Microinstruction Word Structure
 * Represents the decoded fields of the 8086's 21-bit Microcode ROM word.
 */
typedef struct {
  MicroBusId src;           /* [20:16] EU Bus Source                   */
  MicroBusId dst;           /* [15:11] EU Bus Destination              */
  MicroActionType act_type; /* [10:9]  Secondary Action Type           */
  Alu86Op alu_op;           /* ALU Operation (or XI = opcode-derived)  */
  bool alu_use_xi;          /* Use instruction's PLA-decoded ALU op    */
  bool update_flags;        /* F bit: Latch ALU flags into FLAGS reg   */
  MicroMemOp mem_op;        /* BIU Bus Cycle Operation                 */
  MicroCond cond;           /* Micro-branch condition                  */
  uint16_t jump_target;     /* Target microcode ROM address            */
  bool rni;                 /* RNI: Run Next Macro-Instruction         */
  const char *comment;      /* Symbolic micro-assembly annotation      */
} MicroInstr;

/* Microcode ROM Entry Point Addresses (PLA Targets) */
enum {
  MC_EA_BX_SI = 0x000,  /* Effective Address: [BX + SI + disp]           */
  MC_EA_BX_DI = 0x008,  /* Effective Address: [BX + DI + disp]           */
  MC_EA_BP_SI = 0x010,  /* Effective Address: [BP + SI + disp]           */
  MC_EA_BP_DI = 0x018,  /* Effective Address: [BP + DI + disp]           */
  MC_EA_SI = 0x020,     /* Effective Address: [SI + disp]                */
  MC_EA_DI = 0x028,     /* Effective Address: [DI + disp]                */
  MC_EA_BP = 0x030,     /* Effective Address: [BP + disp]                */
  MC_EA_BX = 0x038,     /* Effective Address: [BX + disp]                */
  MC_EA_DIRECT = 0x040, /* Effective Address: [disp16]                   */

  MC_ALU_RM_R = 0x050,     /* ADD/OR/ADC/SBB/AND/SUB/XOR r/m, r             */
  MC_ALU_R_RM = 0x058,     /* ADD/OR/ADC/SBB/AND/SUB/XOR r, r/m             */
  MC_CMP_RM_R = 0x060,     /* CMP r/m, r                                    */
  MC_CMP_R_RM = 0x068,     /* CMP r, r/m                                    */
  MC_ALU_ACC_IMM = 0x070,  /* ADD/OR/ADC/SBB/AND/SUB/XOR/CMP AL/AX, imm     */
  MC_CMP_ACC_IMM = 0x078,  /* CMP AL/AX, imm                                */
  MC_GRP1_RM_IMM = 0x080,  /* Group 1 (0x80..0x83): ALU r/m, imm            */
  MC_GRP1_CMP_IMM = 0x088, /* Group 1 (0x80..0x83): CMP r/m, imm            */

  MC_MOV_RM_R = 0x090,    /* MOV r/m, r                                    */
  MC_MOV_R_RM = 0x098,    /* MOV r, r/m                                    */
  MC_MOV_R_IMM = 0x0A0,   /* MOV reg, imm                                  */
  MC_MOV_RM_IMM = 0x0A8,  /* MOV r/m, imm                                  */
  MC_MOV_ACC_MEM = 0x0B0, /* MOV AL/AX, [moffs16]                          */
  MC_MOV_MEM_ACC = 0x0B8, /* MOV [moffs16], AL/AX                          */

  MC_INC_R16 = 0x0C0,     /* INC reg16                                     */
  MC_DEC_R16 = 0x0C8,     /* DEC reg16                                     */
  MC_PUSH_R16 = 0x0D0,    /* PUSH reg16 (Exhibits authentic 8086 PUSH SP!) */
  MC_POP_R16 = 0x0D8,     /* POP reg16                                     */
  MC_XCHG_AX_R16 = 0x0E0, /* XCHG AX, reg16                                */

  MC_JMP_REL8 = 0x0E8,   /* JMP rel8                                      */
  MC_JMP_REL16 = 0x0F0,  /* JMP rel16                                     */
  MC_CALL_REL16 = 0x0F8, /* CALL rel16                                    */
  MC_RET_NEAR = 0x100,   /* RET (near)                                    */
  MC_LEA_R_M = 0x108,    /* LEA r16, m                                    */
  MC_SHL_RM_1 = 0x110,   /* Shift/Rotate r/m by 1                         */
  MC_NOT_RM = 0x118,     /* NOT r/m                                       */
  MC_NEG_RM = 0x120      /* NEG r/m                                       */
};

/* Complete Intel 8086 CPU State */
typedef struct {
  /* 1. Architectural General Purpose Registers (Transistor DFF Backed) */
  Reg16 regs[8]; /* 0:AX, 1:CX, 2:DX, 3:BX, 4:SP, 5:BP, 6:SI, 7:DI */

  /* 2. Architectural Segment Registers (Transistor DFF Backed) */
  Reg16 segs[4]; /* 0:ES, 1:CS, 2:SS, 3:DS */

  /* 3. Instruction Pointer & Architectural FLAGS (Transistor DFF Backed) */
  Reg16 ip;
  Reg16 flags;

  /* 4. Internal EU Microcode Staging Registers (Transistor DFF Backed) */
  Reg16 tmp_a;
  Reg16 tmp_b;
  Reg16 tmp_c;
  Reg16 sigma;
  Reg16 ind; /* Effective Address Offset sent to BIU */
  Reg16 opr; /* Memory Operand Latch sent/received from BIU */

  /* 5. Bus Interface Unit (BIU) & 6-Byte Prefetch Queue */
  uint8_t prefetch_queue[6];
  uint8_t queue_len;
  uint16_t biu_fetch_ip;
  int seg_override;   /* -1 = None, 0=ES, 1=CS, 2=SS, 3=DS */
  int default_seg;    /* Default segment selected by EA (DS or SS) */
  uint8_t rep_prefix; /* 0 = None, 0xF2 = REPNE, 0xF3 = REP/REPE */

  /* 6. Microcode Sequencer & PLA Decode State */
  MicroInstr mc_rom[512];
  uint16_t mc_pc;  /* Microcode Program Counter (uPC) */
  uint16_t mc_ret; /* Microcode Subroutine Return Address */
  uint8_t cur_opcode;
  uint8_t modrm;
  uint8_t mod;
  uint8_t reg_field;
  uint8_t rm_field;
  bool w_bit;        /* true = 16-bit word, false = 8-bit byte */
  bool s_bit;        /* Sign-extend 8-bit immediate to 16-bit */
  Alu86Op xi_alu_op; /* PLA-decoded ALU operation from opcode */

  /* 7. System Status & 1 MB Physical Address Space */
  bool halted;
  bool trace_microcode;
  uint64_t macro_instructions;
  uint64_t micro_cycles;
  uint8_t ram[1048576]; /* 1 MB (0x00000 .. 0xFFFFF) */
} Intel8086;

/* ============================================================================
 * BIU 6-BYTE PREFETCH QUEUE & 20-BIT MEMORY BUS INTERFACE
 * ============================================================================
 */

static uint8_t biu_read_byte_phys(Intel8086 *cpu, uint16_t seg, uint16_t off) {
  uint32_t phys = biu_adder_20bit(seg, off);
  return cpu->ram[phys & 0xFFFFF];
}

static void biu_write_byte_phys(Intel8086 *cpu, uint16_t seg, uint16_t off,
                                uint8_t val) {
  uint32_t phys = biu_adder_20bit(seg, off);
  cpu->ram[phys & 0xFFFFF] = val;
}

static uint16_t biu_read_word_phys(Intel8086 *cpu, uint16_t seg, uint16_t off) {
  uint8_t lo = biu_read_byte_phys(cpu, seg, off);
  uint8_t hi = biu_read_byte_phys(cpu, seg, (uint16_t)(off + 1));
  return (uint16_t)(lo | (hi << 8));
}

static void biu_write_word_phys(Intel8086 *cpu, uint16_t seg, uint16_t off,
                                uint16_t val) {
  biu_write_byte_phys(cpu, seg, off, (uint8_t)(val & 0xFF));
  biu_write_byte_phys(cpu, seg, (uint16_t)(off + 1),
                      (uint8_t)((val >> 8) & 0xFF));
}

/* Refill the 6-Byte BIU Instruction Prefetch Queue */
static void biu_refill_prefetch_queue(Intel8086 *cpu) {
  uint16_t cs = reg16_read(&cpu->segs[1]);
  while (cpu->queue_len < 6) {
    cpu->prefetch_queue[cpu->queue_len++] =
        biu_read_byte_phys(cpu, cs, cpu->biu_fetch_ip++);
  }
}

/* Flush the 6-Byte Prefetch Queue on Branch/Call/Interrupt */
static void biu_flush_prefetch_queue(Intel8086 *cpu) {
  cpu->queue_len = 0;
  cpu->biu_fetch_ip = reg16_read(&cpu->ip);
  biu_refill_prefetch_queue(cpu);
}

/* Pull 1 Byte from the BIU Prefetch Queue (and advance architectural IP) */
static uint8_t biu_queue_pop_byte(Intel8086 *cpu) {
  if (cpu->queue_len == 0)
    biu_refill_prefetch_queue(cpu);
  uint8_t b = cpu->prefetch_queue[0];
  memmove(&cpu->prefetch_queue[0], &cpu->prefetch_queue[1], 5);
  cpu->queue_len--;
  reg16_write(&cpu->ip, (uint16_t)(reg16_read(&cpu->ip) + 1));
  biu_refill_prefetch_queue(cpu);
  return b;
}

static uint16_t biu_queue_pop_word(Intel8086 *cpu) {
  uint8_t lo = biu_queue_pop_byte(cpu);
  uint8_t hi = biu_queue_pop_byte(cpu);
  return (uint16_t)(lo | (hi << 8));
}

/* ============================================================================
 * 8086 REGISTER FILE ACCESS (8-BIT SPLIT AL/AH..BL/BH AND 16-BIT WORD)
 * ============================================================================
 */

static uint16_t eu_read_arch_reg(const Intel8086 *cpu, int idx, bool is_word) {
  idx &= 7;
  if (is_word) {
    return reg16_read(&cpu->regs[idx]);
  } else {
    /* 8086 8-bit encoding: 0..3 = AL,CL,DL,BL; 4..7 = AH,CH,DH,BH */
    if (idx < 4)
      return reg16_read_lo(&cpu->regs[idx]);
    else
      return reg16_read_hi(&cpu->regs[idx - 4]);
  }
}

static void eu_write_arch_reg(Intel8086 *cpu, int idx, uint16_t val,
                              bool is_word) {
  idx &= 7;
  if (is_word) {
    reg16_write(&cpu->regs[idx], val);
  } else {
    if (idx < 4)
      reg16_write_lo(&cpu->regs[idx], (uint8_t)(val & 0xFF));
    else
      reg16_write_hi(&cpu->regs[idx - 4], (uint8_t)(val & 0xFF));
  }
}

/* ============================================================================
 * INTEL 8086 MICROCODE ROM PROGRAMMING (POPULATING THE 21-BIT ROM)
 * ============================================================================
 */

static void build_8086_microcode_rom(Intel8086 *cpu) {
  memset(cpu->mc_rom, 0, sizeof(cpu->mc_rom));
  MicroInstr *rom = cpu->mc_rom;

  /* --------------------------------------------------------------------
   * 1. EFFECTIVE ADDRESS (EA) MICRO-SUBROUTINES (Return via mc_ret)
   *    Note: Displacement addition is folded at the end of EA calculation.
   * -------------------------------------------------------------------- */
  /* [BX + SI] */
  rom[MC_EA_BX_SI + 0] = (MicroInstr){
      MBUS_BX, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "BX -> TMPa"};
  rom[MC_EA_BX_SI + 1] =
      (MicroInstr){MBUS_SI, MBUS_TMPb, MACT_ALU, ALU86_ADD,         0, 0, 0,
                   0,       0,         true,     "SI -> TMPb | ADD"};
  rom[MC_EA_BX_SI + 2] = (MicroInstr){
      MBUS_SIGMA, MBUS_IND, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "SIGMA -> IND"};

  /* [BX + DI] */
  rom[MC_EA_BX_DI + 0] = (MicroInstr){
      MBUS_BX, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "BX -> TMPa"};
  rom[MC_EA_BX_DI + 1] =
      (MicroInstr){MBUS_DI, MBUS_TMPb, MACT_ALU, ALU86_ADD,         0, 0, 0,
                   0,       0,         true,     "DI -> TMPb | ADD"};
  rom[MC_EA_BX_DI + 2] = (MicroInstr){
      MBUS_SIGMA, MBUS_IND, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "SIGMA -> IND"};

  /* [BP + SI] (Default Segment = SS) */
  rom[MC_EA_BP_SI + 0] = (MicroInstr){
      MBUS_BP, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "BP -> TMPa"};
  rom[MC_EA_BP_SI + 1] =
      (MicroInstr){MBUS_SI, MBUS_TMPb, MACT_ALU, ALU86_ADD,         0, 0, 0,
                   0,       0,         true,     "SI -> TMPb | ADD"};
  rom[MC_EA_BP_SI + 2] = (MicroInstr){
      MBUS_SIGMA, MBUS_IND, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "SIGMA -> IND"};

  /* [BP + DI] (Default Segment = SS) */
  rom[MC_EA_BP_DI + 0] = (MicroInstr){
      MBUS_BP, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "BP -> TMPa"};
  rom[MC_EA_BP_DI + 1] =
      (MicroInstr){MBUS_DI, MBUS_TMPb, MACT_ALU, ALU86_ADD,         0, 0, 0,
                   0,       0,         true,     "DI -> TMPb | ADD"};
  rom[MC_EA_BP_DI + 2] = (MicroInstr){
      MBUS_SIGMA, MBUS_IND, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "SIGMA -> IND"};

  /* [SI], [DI], [BP], [BX], [disp16] */
  rom[MC_EA_SI] = (MicroInstr){MBUS_SI, MBUS_IND, MACT_NONE,  0, 0, 0, 0, 0,
                               0,       true,     "SI -> IND"};
  rom[MC_EA_DI] = (MicroInstr){MBUS_DI, MBUS_IND, MACT_NONE,  0, 0, 0, 0, 0,
                               0,       true,     "DI -> IND"};
  rom[MC_EA_BP] = (MicroInstr){MBUS_BP, MBUS_IND, MACT_NONE,  0, 0, 0, 0, 0,
                               0,       true,     "BP -> IND"};
  rom[MC_EA_BX] = (MicroInstr){MBUS_BX, MBUS_IND, MACT_NONE,  0, 0, 0, 0, 0,
                               0,       true,     "BX -> IND"};
  rom[MC_EA_DIRECT] = (MicroInstr){MBUS_Q, MBUS_IND, MACT_NONE, 0, 0, 0, 0, 0,
                                   0,      true,     "Q -> IND"};

  /* --------------------------------------------------------------------
   * 2. ALU OPERATIONS (ADD, OR, ADC, SBB, AND, SUB, XOR, CMP)
   * -------------------------------------------------------------------- */
  /* ALU r/m, r: M -> TMPa; R -> TMPb | XI ALU (F); SIGMA -> OPR | WRITE_M | RNI
   */
  rom[MC_ALU_RM_R + 0] = (MicroInstr){
      MBUS_M, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "M -> TMPa"};
  rom[MC_ALU_RM_R + 1] = (MicroInstr){MBUS_R,
                                      MBUS_TMPb,
                                      MACT_ALU,
                                      0,
                                      1,
                                      true,
                                      0,
                                      0,
                                      0,
                                      false,
                                      "R -> TMPb | XI tmpa,tmpb (F)"};
  rom[MC_ALU_RM_R + 2] = (MicroInstr){MBUS_SIGMA,
                                      MBUS_OPR,
                                      MACT_MEM,
                                      0,
                                      0,
                                      0,
                                      MMEM_WRITE_M,
                                      0,
                                      0,
                                      true,
                                      "SIGMA -> M | WB | RNI"};

  /* ALU r, r/m: R -> TMPa; M -> TMPb | XI ALU (F); SIGMA -> R | RNI */
  rom[MC_ALU_R_RM + 0] = (MicroInstr){
      MBUS_R, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "R -> TMPa"};
  rom[MC_ALU_R_RM + 1] = (MicroInstr){MBUS_M,
                                      MBUS_TMPb,
                                      MACT_ALU,
                                      0,
                                      1,
                                      true,
                                      0,
                                      0,
                                      0,
                                      false,
                                      "M -> TMPb | XI tmpa,tmpb (F)"};
  rom[MC_ALU_R_RM + 2] =
      (MicroInstr){MBUS_SIGMA, MBUS_R, MACT_NONE,         0, 0, 0, 0, 0,
                   0,          true,   "SIGMA -> R | RNI"};

  /* CMP r/m, r (No writeback, flags only) */
  rom[MC_CMP_RM_R + 0] = (MicroInstr){
      MBUS_M, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "M -> TMPa"};
  rom[MC_CMP_RM_R + 1] = (MicroInstr){MBUS_R,
                                      MBUS_TMPb,
                                      MACT_ALU,
                                      ALU86_CMP,
                                      0,
                                      true,
                                      0,
                                      0,
                                      0,
                                      true,
                                      "R -> TMPb | CMP (F) | RNI"};

  /* CMP r, r/m */
  rom[MC_CMP_R_RM + 0] = (MicroInstr){
      MBUS_R, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "R -> TMPa"};
  rom[MC_CMP_R_RM + 1] = (MicroInstr){MBUS_M,
                                      MBUS_TMPb,
                                      MACT_ALU,
                                      ALU86_CMP,
                                      0,
                                      true,
                                      0,
                                      0,
                                      0,
                                      true,
                                      "M -> TMPb | CMP (F) | RNI"};

  /* ALU AL/AX, imm */
  rom[MC_ALU_ACC_IMM + 0] = (MicroInstr){
      MBUS_AX, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "ACC -> TMPa"};
  rom[MC_ALU_ACC_IMM + 1] = (MicroInstr){MBUS_Q,
                                         MBUS_TMPb,
                                         MACT_ALU,
                                         0,
                                         1,
                                         true,
                                         0,
                                         0,
                                         0,
                                         false,
                                         "Q -> TMPb | XI tmpa,tmpb (F)"};
  rom[MC_ALU_ACC_IMM + 2] =
      (MicroInstr){MBUS_SIGMA, MBUS_AX, MACT_NONE,           0, 0, 0, 0, 0,
                   0,          true,    "SIGMA -> ACC | RNI"};

  /* CMP AL/AX, imm */
  rom[MC_CMP_ACC_IMM + 0] = (MicroInstr){
      MBUS_AX, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "ACC -> TMPa"};
  rom[MC_CMP_ACC_IMM + 1] = (MicroInstr){MBUS_Q,
                                         MBUS_TMPb,
                                         MACT_ALU,
                                         ALU86_CMP,
                                         0,
                                         true,
                                         0,
                                         0,
                                         0,
                                         true,
                                         "Q -> TMPb | CMP (F) | RNI"};

  /* Group 1 (0x80..0x83): ALU r/m, imm */
  rom[MC_GRP1_RM_IMM + 0] = (MicroInstr){
      MBUS_M, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "M -> TMPa"};
  rom[MC_GRP1_RM_IMM + 1] = (MicroInstr){MBUS_Q_SXT,
                                         MBUS_TMPb,
                                         MACT_ALU,
                                         0,
                                         1,
                                         true,
                                         0,
                                         0,
                                         0,
                                         false,
                                         "Q -> TMPb | XI tmpa,tmpb (F)"};
  rom[MC_GRP1_RM_IMM + 2] = (MicroInstr){MBUS_SIGMA,
                                         MBUS_OPR,
                                         MACT_MEM,
                                         0,
                                         0,
                                         0,
                                         MMEM_WRITE_M,
                                         0,
                                         0,
                                         true,
                                         "SIGMA -> M | WB | RNI"};

  /* Group 1 (0x80..0x83): CMP r/m, imm */
  rom[MC_GRP1_CMP_IMM + 0] = (MicroInstr){
      MBUS_M, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "M -> TMPa"};
  rom[MC_GRP1_CMP_IMM + 1] = (MicroInstr){MBUS_Q_SXT,
                                          MBUS_TMPb,
                                          MACT_ALU,
                                          ALU86_CMP,
                                          0,
                                          true,
                                          0,
                                          0,
                                          0,
                                          true,
                                          "Q -> TMPb | CMP (F) | RNI"};

  /* --------------------------------------------------------------------
   * 3. DATA MOVEMENT (MOV, LEA, PUSH, POP, XCHG)
   * -------------------------------------------------------------------- */
  rom[MC_MOV_RM_R + 0] = (MicroInstr){MBUS_R, MBUS_OPR, MACT_MEM,           0,
                                      0,      0,        MMEM_WRITE_M,       0,
                                      0,      true,     "R -> M | WB | RNI"};
  rom[MC_MOV_R_RM + 0] = (MicroInstr){
      MBUS_M, MBUS_R, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "M -> R | RNI"};
  rom[MC_MOV_R_IMM + 0] = (MicroInstr){
      MBUS_Q, MBUS_R, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "Q -> R | RNI"};
  rom[MC_MOV_RM_IMM + 0] = (MicroInstr){MBUS_Q, MBUS_OPR, MACT_MEM,           0,
                                        0,      0,        MMEM_WRITE_M,       0,
                                        0,      true,     "Q -> M | WB | RNI"};

  /* MOV AL/AX, [moffs16] */
  rom[MC_MOV_ACC_MEM + 0] =
      (MicroInstr){MBUS_Q, MBUS_IND, MACT_MEM,         0, 0, 0, MMEM_READ, 0,
                   0,      false,    "Q -> IND | READ"};
  rom[MC_MOV_ACC_MEM + 1] = (MicroInstr){
      MBUS_OPR, MBUS_AX, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "OPR -> ACC | RNI"};

  /* MOV [moffs16], AL/AX */
  rom[MC_MOV_MEM_ACC + 0] = (MicroInstr){
      MBUS_Q, MBUS_IND, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "Q -> IND"};
  rom[MC_MOV_MEM_ACC + 1] = (MicroInstr){MBUS_AX,
                                         MBUS_OPR,
                                         MACT_MEM,
                                         0,
                                         0,
                                         0,
                                         MMEM_WRITE,
                                         0,
                                         0,
                                         true,
                                         "ACC -> OPR | WRITE | RNI"};

  /* LEA reg16, mem */
  rom[MC_LEA_R_M + 0] = (MicroInstr){
      MBUS_IND, MBUS_R, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "IND -> R | RNI"};

  /* INC reg16 & DEC reg16 */
  rom[MC_INC_R16 + 0] = (MicroInstr){MBUS_R,
                                     MBUS_TMPa,
                                     MACT_ALU,
                                     ALU86_INC,
                                     0,
                                     true,
                                     0,
                                     0,
                                     0,
                                     false,
                                     "R -> TMPa | INC (F)"};
  rom[MC_INC_R16 + 1] =
      (MicroInstr){MBUS_SIGMA, MBUS_R, MACT_NONE,         0, 0, 0, 0, 0,
                   0,          true,   "SIGMA -> R | RNI"};

  rom[MC_DEC_R16 + 0] = (MicroInstr){MBUS_R,
                                     MBUS_TMPa,
                                     MACT_ALU,
                                     ALU86_DEC,
                                     0,
                                     true,
                                     0,
                                     0,
                                     0,
                                     false,
                                     "R -> TMPa | DEC (F)"};
  rom[MC_DEC_R16 + 1] =
      (MicroInstr){MBUS_SIGMA, MBUS_R, MACT_NONE,         0, 0, 0, 0, 0,
                   0,          true,   "SIGMA -> R | RNI"};

  /*
   * PUSH reg16 (Spec-Exact 8086 Microcode Sequence!)
   * Step 0: SP -> TMPa
   * Step 1: CONST2 -> TMPb | SUB TMPa, TMPb (computes SP - 2 in SIGMA, no
   * flags) Step 2: SIGMA -> SP    (Updates SP *before* reading R!) Step 3:
   * SIGMA -> IND   (Sets Stack Offset = new SP) Step 4: R -> OPR | PUSH
   * [SS:IND] | RNI (If R is SP, reads the ALREADY decremented SP!)
   */
  rom[MC_PUSH_R16 + 0] = (MicroInstr){
      MBUS_SP, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SP -> TMPa"};
  rom[MC_PUSH_R16 + 1] = (MicroInstr){MBUS_CONST2,
                                      MBUS_TMPb,
                                      MACT_ALU,
                                      ALU86_SUB,
                                      0,
                                      0,
                                      0,
                                      0,
                                      0,
                                      false,
                                      "2 -> TMPb | SUB tmpa,tmpb"};
  rom[MC_PUSH_R16 + 2] = (MicroInstr){
      MBUS_SIGMA, MBUS_SP, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SIGMA -> SP"};
  rom[MC_PUSH_R16 + 3] = (MicroInstr){
      MBUS_SIGMA, MBUS_IND, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SIGMA -> IND"};
  rom[MC_PUSH_R16 + 4] = (MicroInstr){MBUS_R,
                                      MBUS_OPR,
                                      MACT_MEM,
                                      0,
                                      0,
                                      0,
                                      MMEM_PUSH,
                                      0,
                                      0,
                                      true,
                                      "R -> OPR | PUSH [SS:IND] | RNI"};

  /* POP reg16 */
  rom[MC_POP_R16 + 0] = (MicroInstr){MBUS_SP,
                                     MBUS_IND,
                                     MACT_MEM,
                                     0,
                                     0,
                                     0,
                                     MMEM_POP,
                                     0,
                                     0,
                                     false,
                                     "SP -> IND | POP [SS:IND]"};
  rom[MC_POP_R16 + 1] = (MicroInstr){
      MBUS_SP, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SP -> TMPa"};
  rom[MC_POP_R16 + 2] = (MicroInstr){MBUS_CONST2,
                                     MBUS_TMPb,
                                     MACT_ALU,
                                     ALU86_ADD,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     false,
                                     "2 -> TMPb | ADD tmpa,tmpb"};
  rom[MC_POP_R16 + 3] = (MicroInstr){
      MBUS_SIGMA, MBUS_SP, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SIGMA -> SP"};
  rom[MC_POP_R16 + 4] = (MicroInstr){
      MBUS_OPR, MBUS_R, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "OPR -> R | RNI"};

  /* XCHG AX, reg16 */
  rom[MC_XCHG_AX_R16 + 0] = (MicroInstr){
      MBUS_AX, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "AX -> TMPa"};
  rom[MC_XCHG_AX_R16 + 1] = (MicroInstr){
      MBUS_R, MBUS_AX, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "R -> AX"};
  rom[MC_XCHG_AX_R16 + 2] = (MicroInstr){
      MBUS_TMPa, MBUS_R, MACT_NONE, 0, 0, 0, 0, 0, 0, true, "TMPa -> R | RNI"};

  /* --------------------------------------------------------------------
   * 4. BRANCH, SUBROUTINE CALL/RET & UNARY MICROCODE
   * -------------------------------------------------------------------- */
  /* JMP rel8 */
  rom[MC_JMP_REL8 + 0] =
      (MicroInstr){MBUS_Q_SXT, MBUS_TMPb, MACT_NONE,      0, 0, 0, 0, 0,
                   0,          false,     "Q_SXT -> TMPb"};
  rom[MC_JMP_REL8 + 1] =
      (MicroInstr){MBUS_IP, MBUS_TMPa, MACT_ALU, ALU86_ADD,         0, 0, 0,
                   0,       0,         false,    "IP -> TMPa | ADD"};
  rom[MC_JMP_REL8 + 2] = (MicroInstr){MBUS_SIGMA,
                                      MBUS_IP,
                                      MACT_MEM,
                                      0,
                                      0,
                                      0,
                                      MMEM_FLUSH_QUEUE,
                                      0,
                                      0,
                                      true,
                                      "SIGMA -> IP | FLUSH | RNI"};

  /* JMP rel16 */
  rom[MC_JMP_REL16 + 0] = (MicroInstr){
      MBUS_Q, MBUS_TMPb, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "Q -> TMPb"};
  rom[MC_JMP_REL16 + 1] =
      (MicroInstr){MBUS_IP, MBUS_TMPa, MACT_ALU, ALU86_ADD,         0, 0, 0,
                   0,       0,         false,    "IP -> TMPa | ADD"};
  rom[MC_JMP_REL16 + 2] = (MicroInstr){MBUS_SIGMA,
                                       MBUS_IP,
                                       MACT_MEM,
                                       0,
                                       0,
                                       0,
                                       MMEM_FLUSH_QUEUE,
                                       0,
                                       0,
                                       true,
                                       "SIGMA -> IP | FLUSH | RNI"};

  /* CALL rel16 */
  rom[MC_CALL_REL16 + 0] = (MicroInstr){MBUS_Q,
                                        MBUS_TMPc,
                                        MACT_NONE,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        false,
                                        "Q -> TMPc (target disp)"};
  rom[MC_CALL_REL16 + 1] = (MicroInstr){
      MBUS_SP, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SP -> TMPa"};
  rom[MC_CALL_REL16 + 2] =
      (MicroInstr){MBUS_CONST2, MBUS_TMPb, MACT_ALU, ALU86_SUB,        0, 0, 0,
                   0,           0,         false,    "2 -> TMPb | SUB"};
  rom[MC_CALL_REL16 + 3] = (MicroInstr){
      MBUS_SIGMA, MBUS_SP, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SIGMA -> SP"};
  rom[MC_CALL_REL16 + 4] = (MicroInstr){
      MBUS_SIGMA, MBUS_IND, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SIGMA -> IND"};
  rom[MC_CALL_REL16 + 5] = (MicroInstr){MBUS_IP,
                                        MBUS_OPR,
                                        MACT_MEM,
                                        0,
                                        0,
                                        0,
                                        MMEM_PUSH,
                                        0,
                                        0,
                                        false,
                                        "IP -> OPR | PUSH [SS:IND]"};
  rom[MC_CALL_REL16 + 6] = (MicroInstr){
      MBUS_IP, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "IP -> TMPa"};
  rom[MC_CALL_REL16 + 7] =
      (MicroInstr){MBUS_TMPc, MBUS_TMPb, MACT_ALU, ALU86_ADD,           0, 0, 0,
                   0,         0,         false,    "TMPc -> TMPb | ADD"};
  rom[MC_CALL_REL16 + 8] = (MicroInstr){MBUS_SIGMA,
                                        MBUS_IP,
                                        MACT_MEM,
                                        0,
                                        0,
                                        0,
                                        MMEM_FLUSH_QUEUE,
                                        0,
                                        0,
                                        true,
                                        "SIGMA -> IP | FLUSH | RNI"};

  /* RET (near) */
  rom[MC_RET_NEAR + 0] = (MicroInstr){MBUS_SP,
                                      MBUS_IND,
                                      MACT_MEM,
                                      0,
                                      0,
                                      0,
                                      MMEM_POP,
                                      0,
                                      0,
                                      false,
                                      "SP -> IND | POP [SS:IND]"};
  rom[MC_RET_NEAR + 1] = (MicroInstr){
      MBUS_SP, MBUS_TMPa, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SP -> TMPa"};
  rom[MC_RET_NEAR + 2] =
      (MicroInstr){MBUS_CONST2, MBUS_TMPb, MACT_ALU, ALU86_ADD,        0, 0, 0,
                   0,           0,         false,    "2 -> TMPb | ADD"};
  rom[MC_RET_NEAR + 3] = (MicroInstr){
      MBUS_SIGMA, MBUS_SP, MACT_NONE, 0, 0, 0, 0, 0, 0, false, "SIGMA -> SP"};
  rom[MC_RET_NEAR + 4] = (MicroInstr){MBUS_OPR,
                                      MBUS_IP,
                                      MACT_MEM,
                                      0,
                                      0,
                                      0,
                                      MMEM_FLUSH_QUEUE,
                                      0,
                                      0,
                                      true,
                                      "OPR -> IP | FLUSH | RNI"};

  /* Shift/Rotate r/m, 1 */
  rom[MC_SHL_RM_1 + 0] = (MicroInstr){MBUS_M,
                                      MBUS_TMPa,
                                      MACT_ALU,
                                      0,
                                      1,
                                      true,
                                      0,
                                      0,
                                      0,
                                      false,
                                      "M -> TMPa | XI Shift (F)"};
  rom[MC_SHL_RM_1 + 1] = (MicroInstr){MBUS_SIGMA,
                                      MBUS_OPR,
                                      MACT_MEM,
                                      0,
                                      0,
                                      0,
                                      MMEM_WRITE_M,
                                      0,
                                      0,
                                      true,
                                      "SIGMA -> M | WB | RNI"};

  /* NOT r/m & NEG r/m */
  rom[MC_NOT_RM + 0] =
      (MicroInstr){MBUS_M, MBUS_TMPa, MACT_ALU, ALU86_NOT,        0, false, 0,
                   0,      0,         false,    "M -> TMPa | NOT"};
  rom[MC_NOT_RM + 1] = (MicroInstr){MBUS_SIGMA,
                                    MBUS_OPR,
                                    MACT_MEM,
                                    0,
                                    0,
                                    0,
                                    MMEM_WRITE_M,
                                    0,
                                    0,
                                    true,
                                    "SIGMA -> M | WB | RNI"};

  rom[MC_NEG_RM + 0] = (MicroInstr){MBUS_M,
                                    MBUS_TMPa,
                                    MACT_ALU,
                                    ALU86_NEG,
                                    0,
                                    true,
                                    0,
                                    0,
                                    0,
                                    false,
                                    "M -> TMPa | NEG (F)"};
  rom[MC_NEG_RM + 1] = (MicroInstr){MBUS_SIGMA,
                                    MBUS_OPR,
                                    MACT_MEM,
                                    0,
                                    0,
                                    0,
                                    MMEM_WRITE_M,
                                    0,
                                    0,
                                    true,
                                    "SIGMA -> M | WB | RNI"};
}

/* ============================================================================
 * MICROCODE ENGINE EXECUTION (BUS READ/WRITE + MICRO-STEPPING)
 * ============================================================================
 */

static int get_active_segment(const Intel8086 *cpu) {
  if (cpu->seg_override >= 0)
    return cpu->seg_override;
  return cpu->default_seg;
}

static uint16_t mc_read_bus(Intel8086 *cpu, MicroBusId id) {
  switch (id) {
  case MBUS_AX:
  case MBUS_CX:
  case MBUS_DX:
  case MBUS_BX:
  case MBUS_SP:
  case MBUS_BP:
  case MBUS_SI:
  case MBUS_DI:
    return reg16_read(&cpu->regs[id - MBUS_AX]);
  case MBUS_ES:
  case MBUS_CS:
  case MBUS_SS:
  case MBUS_DS:
    return reg16_read(&cpu->segs[id - MBUS_ES]);
  case MBUS_IP:
    return reg16_read(&cpu->ip);
  case MBUS_FLAGS:
    return sanitize_8086_flags(reg16_read(&cpu->flags));
  case MBUS_R:
    return eu_read_arch_reg(cpu, cpu->reg_field, cpu->w_bit);
  case MBUS_M:
    if (cpu->mod == 3)
      return eu_read_arch_reg(cpu, cpu->rm_field, cpu->w_bit);
    else
      return reg16_read(&cpu->opr);
  case MBUS_TMPa:
    return reg16_read(&cpu->tmp_a);
  case MBUS_TMPb:
    return reg16_read(&cpu->tmp_b);
  case MBUS_TMPc:
    return reg16_read(&cpu->tmp_c);
  case MBUS_SIGMA:
    return reg16_read(&cpu->sigma);
  case MBUS_IND:
    return reg16_read(&cpu->ind);
  case MBUS_OPR:
    return reg16_read(&cpu->opr);
  case MBUS_Q:
    return cpu->w_bit ? biu_queue_pop_word(cpu) : biu_queue_pop_byte(cpu);
  case MBUS_Q_SXT:
    if (cpu->w_bit && !cpu->s_bit)
      return biu_queue_pop_word(cpu);
    else
      return (uint16_t)(int16_t)(int8_t)biu_queue_pop_byte(cpu);
  case MBUS_ZERO:
    return 0x0000;
  case MBUS_ONES:
    return 0xFFFF;
  case MBUS_CONST2:
    return cpu->w_bit ? 2 : 1;
  default:
    return 0;
  }
}

static void mc_write_bus(Intel8086 *cpu, MicroBusId id, uint16_t val) {
  switch (id) {
  case MBUS_AX:
    eu_write_arch_reg(cpu, 0, val, cpu->w_bit);
    break;
  case MBUS_CX:
  case MBUS_DX:
  case MBUS_BX:
  case MBUS_SP:
  case MBUS_BP:
  case MBUS_SI:
  case MBUS_DI:
    reg16_write(&cpu->regs[id - MBUS_AX], val);
    break;
  case MBUS_ES:
  case MBUS_CS:
  case MBUS_SS:
  case MBUS_DS:
    reg16_write(&cpu->segs[id - MBUS_ES], val);
    break;
  case MBUS_IP:
    reg16_write(&cpu->ip, val);
    break;
  case MBUS_FLAGS:
    reg16_write(&cpu->flags, sanitize_8086_flags(val));
    break;
  case MBUS_R:
    eu_write_arch_reg(cpu, cpu->reg_field, val, cpu->w_bit);
    break;
  case MBUS_M:
    if (cpu->mod == 3)
      eu_write_arch_reg(cpu, cpu->rm_field, val, cpu->w_bit);
    else
      reg16_write(&cpu->opr, val);
    break;
  case MBUS_TMPa:
    reg16_write(&cpu->tmp_a, val);
    break;
  case MBUS_TMPb:
    reg16_write(&cpu->tmp_b, val);
    break;
  case MBUS_TMPc:
    reg16_write(&cpu->tmp_c, val);
    break;
  case MBUS_SIGMA:
    reg16_write(&cpu->sigma, val);
    break;
  case MBUS_IND:
    reg16_write(&cpu->ind, val);
    break;
  case MBUS_OPR:
    reg16_write(&cpu->opr, val);
    break;
  default:
    break;
  }
}

/* Execute a Microcode Sequence starting at uPC until RNI (Run Next Instruction)
 */
static void execute_microcode_routine(Intel8086 *cpu, uint16_t start_upc) {
  cpu->mc_pc = start_upc;
  while (true) {
    const MicroInstr *mi = &cpu->mc_rom[cpu->mc_pc];
    cpu->micro_cycles++;

    /* 1. Perform Internal 16-bit EU Bus Transfer: SRC -> DST */
    uint16_t bus_val = 0;
    if (mi->src != MBUS_NONE) {
      bus_val = mc_read_bus(cpu, mi->src);
      mc_write_bus(cpu, mi->dst, bus_val);
    }

    /* 2. Perform Parallel Secondary Action (ALU, BIU Memory, or Micro-Jump) */
    if (mi->act_type == MACT_ALU) {
      Alu86Op op = mi->alu_use_xi ? cpu->xi_alu_op : mi->alu_op;
      uint16_t f = reg16_read(&cpu->flags);
      Word16 res = alu_8086_execute(op, reg16_read_word(&cpu->tmp_a),
                                    reg16_read_word(&cpu->tmp_b), cpu->w_bit,
                                    &f, mi->update_flags);
      reg16_write_word(&cpu->sigma, res);
      if (mi->update_flags)
        reg16_write(&cpu->flags, f);
    } else if (mi->act_type == MACT_MEM) {
      uint16_t seg_val = reg16_read(&cpu->segs[get_active_segment(cpu)]);
      uint16_t ss_val = reg16_read(&cpu->segs[2]); /* SS */
      uint16_t ind_val = reg16_read(&cpu->ind);
      uint16_t opr_val = reg16_read(&cpu->opr);

      switch (mi->mem_op) {
      case MMEM_READ:
        if (cpu->w_bit)
          reg16_write(&cpu->opr, biu_read_word_phys(cpu, seg_val, ind_val));
        else
          reg16_write(&cpu->opr, biu_read_byte_phys(cpu, seg_val, ind_val));
        break;
      case MMEM_WRITE:
        if (cpu->w_bit)
          biu_write_word_phys(cpu, seg_val, ind_val, opr_val);
        else
          biu_write_byte_phys(cpu, seg_val, ind_val, (uint8_t)opr_val);
        break;
      case MMEM_WRITE_M:
        if (cpu->mod == 3) {
          eu_write_arch_reg(cpu, cpu->rm_field, opr_val, cpu->w_bit);
        } else {
          if (cpu->w_bit)
            biu_write_word_phys(cpu, seg_val, ind_val, opr_val);
          else
            biu_write_byte_phys(cpu, seg_val, ind_val, (uint8_t)opr_val);
        }
        break;
      case MMEM_PUSH:
        biu_write_word_phys(cpu, ss_val, ind_val, opr_val);
        break;
      case MMEM_POP:
        reg16_write(&cpu->opr, biu_read_word_phys(cpu, ss_val, ind_val));
        break;
      case MMEM_FLUSH_QUEUE:
        biu_flush_prefetch_queue(cpu);
        break;
      default:
        break;
      }
    }

    if (cpu->trace_microcode) {
      printf("    [uPC 0x%03X] %-28s | bus=0x%04X SIGMA=0x%04X IND=0x%04X "
             "OPR=0x%04X\n",
             cpu->mc_pc, mi->comment ? mi->comment : "", bus_val,
             reg16_read(&cpu->sigma), reg16_read(&cpu->ind),
             reg16_read(&cpu->opr));
    }

    if (mi->rni)
      break;
    cpu->mc_pc++;
  }
}

/*
 * Decode ModR/M Byte and Run the 8086 Effective Address (EA) Microcode Routine
 */
static void decode_modrm_and_calc_ea(Intel8086 *cpu, bool load_memory_to_opr) {
  cpu->modrm = biu_queue_pop_byte(cpu);
  cpu->mod = (cpu->modrm >> 6) & 3;
  cpu->reg_field = (cpu->modrm >> 3) & 7;
  cpu->rm_field = cpu->modrm & 7;

  if (cpu->mod == 3)
    return; /* Register-to-Register mode needs no EA microcode */

  bool saved_w = cpu->w_bit;
  cpu->w_bit = true; /* Effective Address math is always 16-bit */

  /* Select Default Segment (SS for BP-based addressing, DS otherwise) */
  cpu->default_seg = 3; /* DS */
  if (cpu->rm_field == 2 || cpu->rm_field == 3 ||
      (cpu->rm_field == 6 && cpu->mod != 0)) {
    cpu->default_seg = 2; /* SS */
  }

  /* Invoke the PLA-selected EA Microcode Routine */
  if (cpu->mod == 0 && cpu->rm_field == 6) {
    execute_microcode_routine(cpu, MC_EA_DIRECT);
  } else {
    static const uint16_t ea_table[8] = {MC_EA_BX_SI, MC_EA_BX_DI, MC_EA_BP_SI,
                                         MC_EA_BP_DI, MC_EA_SI,    MC_EA_DI,
                                         MC_EA_BP,    MC_EA_BX};
    execute_microcode_routine(cpu, ea_table[cpu->rm_field]);

    /* Add 8-bit or 16-bit Displacement via Transistor ALU into IND */
    if (cpu->mod == 1 || cpu->mod == 2) {
      uint16_t disp = (cpu->mod == 1)
                          ? (uint16_t)(int16_t)(int8_t)biu_queue_pop_byte(cpu)
                          : biu_queue_pop_word(cpu);
      reg16_write(&cpu->tmp_a, reg16_read(&cpu->ind));
      reg16_write(&cpu->tmp_b, disp);
      uint16_t dummy_f = reg16_read(&cpu->flags);
      Word16 ea =
          alu_8086_execute(ALU86_ADD, reg16_read_word(&cpu->tmp_a),
                           reg16_read_word(&cpu->tmp_b), true, &dummy_f, false);
      reg16_write_word(&cpu->sigma, ea);
      reg16_write_word(&cpu->ind, ea);
      cpu->micro_cycles += 2;
    }
  }

  cpu->w_bit = saved_w;

  /* If instruction reads memory operand M, perform BIU Read [Seg:IND] -> OPR */
  if (load_memory_to_opr) {
    uint16_t seg_val = reg16_read(&cpu->segs[get_active_segment(cpu)]);
    uint16_t ind_val = reg16_read(&cpu->ind);
    if (cpu->w_bit)
      reg16_write(&cpu->opr, biu_read_word_phys(cpu, seg_val, ind_val));
    else
      reg16_write(&cpu->opr, biu_read_byte_phys(cpu, seg_val, ind_val));
    cpu->micro_cycles++;
  }
}

/* Map 3-bit 8086 ALU Group Index (0..7) to Alu86Op */
static Alu86Op map_pla_alu_op(uint8_t idx) {
  static const Alu86Op table[8] = {ALU86_ADD, ALU86_OR,  ALU86_ADC, ALU86_SBB,
                                   ALU86_AND, ALU86_SUB, ALU86_XOR, ALU86_CMP};
  return table[idx & 7];
}

/* Evaluate 8086 4-bit Conditional Jump Condition (0x70..0x7F) */
static bool eval_8086_jcc(uint16_t f, uint8_t cc) {
  bool cf = (f & FLAG_CF) != 0;
  bool pf = (f & FLAG_PF) != 0;
  bool zf = (f & FLAG_ZF) != 0;
  bool sf = (f & FLAG_SF) != 0;
  bool of = (f & FLAG_OF) != 0;
  bool cond = false;

  switch ((cc >> 1) & 7) {
  case 0:
    cond = of;
    break; /* JO / JNO          */
  case 1:
    cond = cf;
    break; /* JB / JNB          */
  case 2:
    cond = zf;
    break; /* JE / JNE          */
  case 3:
    cond = (cf || zf);
    break; /* JBE / JA          */
  case 4:
    cond = sf;
    break; /* JS / JNS          */
  case 5:
    cond = pf;
    break; /* JP / JNP          */
  case 6:
    cond = (sf != of);
    break; /* JL / JGE          */
  case 7:
    cond = (zf || sf != of);
    break; /* JLE / JG          */
  }
  return (cc & 1) ? !cond : cond;
}

/* Initialize Intel 8086 to Spec-Exact Power-On Reset State (FFFF:0000) */
static void cpu8086_init(Intel8086 *cpu) {
  memset(cpu, 0, sizeof(*cpu));
  for (int i = 0; i < 8; i++)
    reg16_init(&cpu->regs[i]);
  for (int i = 0; i < 4; i++)
    reg16_init(&cpu->segs[i]);
  reg16_init(&cpu->ip);
  reg16_init(&cpu->flags);
  reg16_init(&cpu->tmp_a);
  reg16_init(&cpu->tmp_b);
  reg16_init(&cpu->tmp_c);
  reg16_init(&cpu->sigma);
  reg16_init(&cpu->ind);
  reg16_init(&cpu->opr);

  build_8086_microcode_rom(cpu);

  /* Spec-Exact 8086 Hardware Reset State: CS=0xFFFF, IP=0x0000, FLAGS=0xF002 */
  reg16_write(&cpu->segs[1], 0xFFFF); /* CS */
  reg16_write(&cpu->ip, 0x0000);      /* IP */
  reg16_write(&cpu->flags, 0xF002);   /* Hardwired 8086 FLAGS reset state */
  cpu->seg_override = -1;
  cpu->default_seg = 3; /* DS */
  biu_flush_prefetch_queue(cpu);
}

/* Execute One Complete Intel 8086 Macro-Instruction via PLA + Microcode Engine
 */
static void cpu8086_step(Intel8086 *cpu) {
  if (cpu->halted)
    return;

  cpu->seg_override = -1;
  cpu->default_seg = 3; /* DS */
  cpu->rep_prefix = 0;
  cpu->s_bit = false;

  /* 1. Handle Prefix Bytes (Segment Overrides, REP/REPNE, LOCK) */
  uint8_t op;
  while (true) {
    op = biu_queue_pop_byte(cpu);
    if (op == 0x26)
      cpu->seg_override = 0; /* ES: */
    else if (op == 0x2E)
      cpu->seg_override = 1; /* CS: */
    else if (op == 0x36)
      cpu->seg_override = 2; /* SS: */
    else if (op == 0x3E)
      cpu->seg_override = 3; /* DS: */
    else if (op == 0xF2 || op == 0xF3)
      cpu->rep_prefix = op;
    else if (op == 0xF0) { /* LOCK prefix */
    } else
      break;
  }

  cpu->cur_opcode = op;
  cpu->macro_instructions++;

  /* 2. PLA Translation: Map Opcode -> Microcode Routine */
  /* Standard ALU Group (0x00..0x3D: ADD, OR, ADC, SBB, AND, SUB, XOR, CMP) */
  if ((op & 0xC4) == 0x00) {
    uint8_t alu_idx = (op >> 3) & 7;
    cpu->xi_alu_op = map_pla_alu_op(alu_idx);
    cpu->w_bit = (op & 1) != 0;
    bool dir_reg = (op & 2) != 0;
    bool is_cmp = (alu_idx == 7);

    if ((op & 4) == 0) {
      decode_modrm_and_calc_ea(cpu, true);
      if (dir_reg)
        execute_microcode_routine(cpu, is_cmp ? MC_CMP_R_RM : MC_ALU_R_RM);
      else
        execute_microcode_routine(cpu, is_cmp ? MC_CMP_RM_R : MC_ALU_RM_R);
    } else {
      execute_microcode_routine(cpu, is_cmp ? MC_CMP_ACC_IMM : MC_ALU_ACC_IMM);
    }
    return;
  }

  /* Group 1 Immediate ALU Operations (0x80..0x83) */
  if ((op & 0xFC) == 0x80) {
    cpu->w_bit = (op & 1) != 0;
    cpu->s_bit = (op == 0x83);
    decode_modrm_and_calc_ea(cpu, true);
    cpu->xi_alu_op = map_pla_alu_op(cpu->reg_field);
    if (cpu->reg_field == 7)
      execute_microcode_routine(cpu, MC_GRP1_CMP_IMM);
    else
      execute_microcode_routine(cpu, MC_GRP1_RM_IMM);
    return;
  }

  /* INC reg16 (0x40..0x47) & DEC reg16 (0x48..0x4F) */
  if ((op & 0xF8) == 0x40) {
    cpu->w_bit = true;
    cpu->reg_field = op & 7;
    execute_microcode_routine(cpu, MC_INC_R16);
    return;
  }
  if ((op & 0xF8) == 0x48) {
    cpu->w_bit = true;
    cpu->reg_field = op & 7;
    execute_microcode_routine(cpu, MC_DEC_R16);
    return;
  }

  /* PUSH reg16 (0x50..0x57) & POP reg16 (0x58..0x5F) */
  if ((op & 0xF8) == 0x50) {
    cpu->w_bit = true;
    cpu->reg_field = op & 7;
    execute_microcode_routine(cpu, MC_PUSH_R16);
    return;
  }
  if ((op & 0xF8) == 0x58) {
    cpu->w_bit = true;
    cpu->reg_field = op & 7;
    execute_microcode_routine(cpu, MC_POP_R16);
    return;
  }

  /* Conditional Jumps Jcc rel8 (0x70..0x7F) */
  if ((op & 0xF0) == 0x70) {
    cpu->w_bit = true;
    if (eval_8086_jcc(reg16_read(&cpu->flags), op & 0xF)) {
      execute_microcode_routine(cpu, MC_JMP_REL8);
    } else {
      (void)biu_queue_pop_byte(cpu); /* Discard branch offset */
    }
    return;
  }

  /* MOV r/m, r (0x88..0x89) & MOV r, r/m (0x8A..0x8B) */
  if ((op & 0xFC) == 0x88) {
    cpu->w_bit = (op & 1) != 0;
    bool dir_reg = (op & 2) != 0;
    decode_modrm_and_calc_ea(cpu, dir_reg);
    execute_microcode_routine(cpu, dir_reg ? MC_MOV_R_RM : MC_MOV_RM_R);
    return;
  }

  /* MOV Sreg, r/m16 (0x8E) & MOV r/m16, Sreg (0x8C) */
  if (op == 0x8E || op == 0x8C) {
    cpu->w_bit = true;
    decode_modrm_and_calc_ea(cpu, op == 0x8E);
    if (op == 0x8E) {
      uint16_t v = (cpu->mod == 3) ? eu_read_arch_reg(cpu, cpu->rm_field, true)
                                   : reg16_read(&cpu->opr);
      reg16_write(&cpu->segs[cpu->reg_field & 3], v);
    } else {
      uint16_t v = reg16_read(&cpu->segs[cpu->reg_field & 3]);
      if (cpu->mod == 3)
        eu_write_arch_reg(cpu, cpu->rm_field, v, true);
      else
        biu_write_word_phys(cpu,
                            reg16_read(&cpu->segs[get_active_segment(cpu)]),
                            reg16_read(&cpu->ind), v);
    }
    return;
  }

  /* LEA r16, m (0x8D) */
  if (op == 0x8D) {
    cpu->w_bit = true;
    decode_modrm_and_calc_ea(cpu, false);
    execute_microcode_routine(cpu, MC_LEA_R_M);
    return;
  }

  /* NOP (0x90) & XCHG AX, reg16 (0x91..0x97) */
  if ((op & 0xF8) == 0x90) {
    if (op == 0x90)
      return;
    cpu->w_bit = true;
    cpu->reg_field = op & 7;
    execute_microcode_routine(cpu, MC_XCHG_AX_R16);
    return;
  }

  /* MOV AL/AX, [moffs16] (0xA0..0xA1) & MOV [moffs16], AL/AX (0xA2..0xA3) */
  if ((op & 0xFC) == 0xA0) {
    cpu->w_bit = (op & 1) != 0;
    execute_microcode_routine(cpu, (op & 2) ? MC_MOV_MEM_ACC : MC_MOV_ACC_MEM);
    return;
  }

  /* String Instructions: MOVSB/W (0xA4..0xA5), STOSB/W (0xAA..0xAB), LODSB/W
   * (0xAC..0xAD) */
  if (op == 0xA4 || op == 0xA5 || op == 0xAA || op == 0xAB || op == 0xAC ||
      op == 0xAD) {
    cpu->w_bit = (op & 1) != 0;
    int step =
        ((reg16_read(&cpu->flags) & FLAG_DF) ? -1 : 1) * (cpu->w_bit ? 2 : 1);
    do {
      if (cpu->rep_prefix && reg16_read(&cpu->regs[1]) == 0)
        break; /* CX == 0 */
      uint16_t ds = reg16_read(&cpu->segs[get_active_segment(cpu)]);
      uint16_t es = reg16_read(&cpu->segs[0]);
      uint16_t si = reg16_read(&cpu->regs[6]);
      uint16_t di = reg16_read(&cpu->regs[7]);

      if (op == 0xA4 || op == 0xA5) { /* MOVSB / MOVSW */
        uint16_t v = cpu->w_bit ? biu_read_word_phys(cpu, ds, si)
                                : biu_read_byte_phys(cpu, ds, si);
        if (cpu->w_bit)
          biu_write_word_phys(cpu, es, di, v);
        else
          biu_write_byte_phys(cpu, es, di, (uint8_t)v);
        reg16_write(&cpu->regs[6], (uint16_t)(si + step));
        reg16_write(&cpu->regs[7], (uint16_t)(di + step));
      } else if (op == 0xAA || op == 0xAB) { /* STOSB / STOSW */
        uint16_t v = eu_read_arch_reg(cpu, 0, cpu->w_bit);
        if (cpu->w_bit)
          biu_write_word_phys(cpu, es, di, v);
        else
          biu_write_byte_phys(cpu, es, di, (uint8_t)v);
        reg16_write(&cpu->regs[7], (uint16_t)(di + step));
      } else { /* LODSB / LODSW */
        uint16_t v = cpu->w_bit ? biu_read_word_phys(cpu, ds, si)
                                : biu_read_byte_phys(cpu, ds, si);
        eu_write_arch_reg(cpu, 0, v, cpu->w_bit);
        reg16_write(&cpu->regs[6], (uint16_t)(si + step));
      }
      cpu->micro_cycles += 4;
      if (cpu->rep_prefix) {
        reg16_write(&cpu->regs[1], (uint16_t)(reg16_read(&cpu->regs[1]) - 1));
      }
    } while (cpu->rep_prefix && reg16_read(&cpu->regs[1]) != 0);
    return;
  }

  /* MOV reg8, imm8 (0xB0..0xB7) & MOV reg16, imm16 (0xB8..0xBF) */
  if ((op & 0xF0) == 0xB0) {
    cpu->w_bit = (op & 8) != 0;
    cpu->reg_field = op & 7;
    execute_microcode_routine(cpu, MC_MOV_R_IMM);
    return;
  }

  /* MOV r/m, imm (0xC6..0xC7) */
  if (op == 0xC6 || op == 0xC7) {
    cpu->w_bit = (op & 1) != 0;
    decode_modrm_and_calc_ea(cpu, false);
    execute_microcode_routine(cpu, MC_MOV_RM_IMM);
    return;
  }

  /* Shift / Rotate Group 2 (0xD0..0xD3) */
  if ((op & 0xFC) == 0xD0) {
    cpu->w_bit = (op & 1) != 0;
    decode_modrm_and_calc_ea(cpu, true);
    static const Alu86Op sh_ops[8] = {ALU86_ROL, ALU86_ROR, ALU86_RCL,
                                      ALU86_RCR, ALU86_SHL, ALU86_SHR,
                                      ALU86_SHL, ALU86_SAR};
    cpu->xi_alu_op = sh_ops[cpu->reg_field & 7];
    uint8_t count = (op & 2) ? reg16_read_lo(&cpu->regs[1]) : 1; /* CL or 1 */
    for (uint8_t c = 0; c < count; c++) {
      execute_microcode_routine(cpu, MC_SHL_RM_1);
    }
    return;
  }

  /* LOOP (0xE2), CALL rel16 (0xE8), JMP rel16 (0xE9), JMP far (0xEA), JMP rel8
   * (0xEB), RET (0xC3) */
  if (op == 0xE2) { /* LOOP rel8 */
    uint16_t cx = (uint16_t)(reg16_read(&cpu->regs[1]) - 1);
    reg16_write(&cpu->regs[1], cx);
    cpu->w_bit = true;
    if (cx != 0)
      execute_microcode_routine(cpu, MC_JMP_REL8);
    else
      (void)biu_queue_pop_byte(cpu);
    return;
  }
  if (op == 0xE8) {
    cpu->w_bit = true;
    execute_microcode_routine(cpu, MC_CALL_REL16);
    return;
  }
  if (op == 0xE9) {
    cpu->w_bit = true;
    execute_microcode_routine(cpu, MC_JMP_REL16);
    return;
  }
  if (op == 0xEB) {
    cpu->w_bit = true;
    execute_microcode_routine(cpu, MC_JMP_REL8);
    return;
  }
  if (op == 0xC3) {
    cpu->w_bit = true;
    execute_microcode_routine(cpu, MC_RET_NEAR);
    return;
  }
  if (op ==
      0xEA) { /* JMP ptr16:16 (Far Jump, used at Reset Vector FFFF:0000) */
    uint16_t new_ip = biu_queue_pop_word(cpu);
    uint16_t new_cs = biu_queue_pop_word(cpu);
    reg16_write(&cpu->ip, new_ip);
    reg16_write(&cpu->segs[1], new_cs);
    biu_flush_prefetch_queue(cpu);
    cpu->micro_cycles += 4;
    return;
  }

  /* Group 3 Unary/Multiply Operations (0xF6..0xF7: TEST, NOT, NEG, MUL) */
  if (op == 0xF6 || op == 0xF7) {
    cpu->w_bit = (op & 1) != 0;
    decode_modrm_and_calc_ea(cpu, true);
    if (cpu->reg_field == 2) {
      execute_microcode_routine(cpu, MC_NOT_RM);
    } else if (cpu->reg_field == 3) {
      execute_microcode_routine(cpu, MC_NEG_RM);
    } else if (cpu->reg_field == 4) {
      /* 8086 Microcode Shift-and-Add Unsigned Multiply (MUL r/m) */
      uint16_t multiplicand = eu_read_arch_reg(cpu, 0, cpu->w_bit);
      uint16_t multiplier = mc_read_bus(cpu, MBUS_M);
      uint32_t product = 0;
      int bits = cpu->w_bit ? 16 : 8;
      for (int i = 0; i < bits; i++) {
        if ((multiplier >> i) & 1)
          product += ((uint32_t)multiplicand << i);
        cpu->micro_cycles += 3;
      }
      if (cpu->w_bit) {
        reg16_write(&cpu->regs[0], (uint16_t)(product & 0xFFFF)); /* AX */
        reg16_write(&cpu->regs[2],
                    (uint16_t)((product >> 16) & 0xFFFF)); /* DX */
        bool upper_nz = (reg16_read(&cpu->regs[2]) != 0);
        uint16_t f = reg16_read(&cpu->flags);
        f = upper_nz ? (f | FLAG_CF | FLAG_OF) : (f & ~(FLAG_CF | FLAG_OF));
        reg16_write(&cpu->flags, sanitize_8086_flags(f));
      } else {
        reg16_write(&cpu->regs[0], (uint16_t)(product & 0xFFFF)); /* AX */
        bool upper_nz = (reg16_read_hi(&cpu->regs[0]) != 0);
        uint16_t f = reg16_read(&cpu->flags);
        f = upper_nz ? (f | FLAG_CF | FLAG_OF) : (f & ~(FLAG_CF | FLAG_OF));
        reg16_write(&cpu->flags, sanitize_8086_flags(f));
      }
    }
    return;
  }

  /* Flag Control & HLT (CLC, STC, CMC, CLD, STD, HLT) */
  if (op == 0xF8) {
    reg16_write(&cpu->flags, reg16_read(&cpu->flags) & ~FLAG_CF);
    return;
  }
  if (op == 0xF9) {
    reg16_write(&cpu->flags, reg16_read(&cpu->flags) | FLAG_CF);
    return;
  }
  if (op == 0xFC) {
    reg16_write(&cpu->flags, reg16_read(&cpu->flags) & ~FLAG_DF);
    return;
  }
  if (op == 0xFD) {
    reg16_write(&cpu->flags, reg16_read(&cpu->flags) | FLAG_DF);
    return;
  }
  if (op == 0xF4) {
    cpu->halted = true;
    return;
  } /* HLT */
}

/* ============================================================================
 * VERIFICATION & DEMONSTRATION SUITE
 * ============================================================================
 */

static void print_8086_flags(uint16_t f) {
  printf("[FLAGS=0x%04X: %s %s %s %s %s %s %s %s %s]", f,
         (f & FLAG_OF) ? "OF" : "--", (f & FLAG_DF) ? "DF" : "--",
         (f & FLAG_IF) ? "IF" : "--", (f & FLAG_TF) ? "TF" : "--",
         (f & FLAG_SF) ? "SF" : "--", (f & FLAG_ZF) ? "ZF" : "--",
         (f & FLAG_AF) ? "AF" : "--", (f & FLAG_PF) ? "PF" : "--",
         (f & FLAG_CF) ? "CF" : "--");
}

int main(void) {
  printf("====================================================================="
         "=====\n");
  printf(" SPEC-EXACT INTEL 8086 CPU SIMULATOR (TRANSISTORS -> GATES -> "
         "MICROCODE)\n");
  printf("====================================================================="
         "=====\n\n");

  Intel8086 cpu;
  cpu8086_init(&cpu);

  printf("1. Power-On Reset Verification:\n");
  printf("   CS:IP = %04X:%04X (Physical Reset Vector = 0x%05X)\n   ",
         reg16_read(&cpu.segs[1]), reg16_read(&cpu.ip),
         biu_adder_20bit(reg16_read(&cpu.segs[1]), reg16_read(&cpu.ip)));
  print_8086_flags(reg16_read(&cpu.flags));
  printf("\n\n");

  /*
   * Install Reset Vector Jump at FFFF:0000 (Physical 0xFFFF0):
   *   EA 00 01 00 00 -> JMP FAR 0000:0100
   */
  cpu.ram[0xFFFF0] = 0xEA;
  cpu.ram[0xFFFF1] = 0x00;
  cpu.ram[0xFFFF2] = 0x01;
  cpu.ram[0xFFFF3] = 0x00;
  cpu.ram[0xFFFF4] = 0x00;
  biu_flush_prefetch_queue(&cpu);

  /*
   * Assemble 8086 Machine Code Test Firmware at 0000:0100 (Physical 0x00100):
   *
   *   0100: B8 00 20       MOV AX, 0x2000
   *   0103: 8E D8          MOV DS, AX          ; DS = 0x2000 (Phys 0x20000)
   *   0105: 8E C0          MOV ES, AX          ; ES = 0x2000
   *   0107: B8 00 10       MOV AX, 0x1000
   *   010A: 8E D0          MOV SS, AX          ; SS = 0x1000 (Phys 0x10000)
   *   010C: BC 00 08       MOV SP, 0x0800      ; SP = 0x0800
   *
   *   ; --- Test 1: Authentic 8086 "PUSH SP" Microcode Quirk ---
   *   010F: 54             PUSH SP             ; On 8086, pushes 0x07FE (NOT
   * 0x0800!) 0110: 5A             POP DX              ; DX = 0x07FE
   *
   *   ; --- Test 2: Spec-Exact Transistor ALU Flags (AF, OF, PF, SF, ZF, CF)
   * --- 0111: B0 0F          MOV AL, 0x0F 0113: 04 01          ADD AL, 0x01 ;
   * AL = 0x10 -> AF=1 (Carry from bit 3->4!) 0115: B3 7F          MOV BL, 0x7F
   *   0117: 80 C3 01       ADD BL, 0x01        ; BL = 0x80 -> OF=1, SF=1, AF=1
   *
   *   ; --- Test 3: Subroutine Call + EA [BX+SI+disp8] + Fibonacci + MUL + REP
   * MOVSW --- 011A: E8 04 00       CALL 0x0121         ; Call compute_and_copy
   * subroutine 011D: F4             HLT                 ; Halt CPU
   *
   *   ; Subroutine at 0x0121:
   *   0121: BB 00 01       MOV BX, 0x0100      ; Base offset = 0x0100
   *   0124: 31 F6          XOR SI, SI          ; SI = 0
   *   0126: B8 01 00       MOV AX, 1           ; Fib(1) = 1
   *   0129: B9 01 00       MOV CX, 1           ; Fib(2) = 1
   *   012C: 89 40 10       MOV [BX+SI+0x10], AX; Write to DS:[0x0110 + SI]
   *   012F: 01 C8          ADD AX, CX          ; AX = AX + CX
   *   0131: 91             XCHG AX, CX         ; Swap AX, CX
   *   0132: 83 C6 02       ADD SI, 2           ; SI += 2
   *   0135: 83 FE 10       CMP SI, 16          ; 8 words (16 bytes) written?
   *   0138: 75 F2          JNE 0x012C          ; Loop until 8 Fibonacci words
   * stored 013A: C3             RET
   */
  static const uint8_t firmware[] = {
      0xB8, 0x00, 0x20, /* MOV AX, 0x2000          */
      0x8E, 0xD8,       /* MOV DS, AX              */
      0x8E, 0xC0,       /* MOV ES, AX              */
      0xB8, 0x00, 0x10, /* MOV AX, 0x1000          */
      0x8E, 0xD0,       /* MOV SS, AX              */
      0xBC, 0x00, 0x08, /* MOV SP, 0x0800          */
      0x54,             /* PUSH SP (8086 Quirk!)   */
      0x5A,             /* POP DX                  */
      0xB0, 0x0F,       /* MOV AL, 0x0F            */
      0x04, 0x01,       /* ADD AL, 0x01 (Sets AF!) */
      0xB3, 0x7F,       /* MOV BL, 0x7F            */
      0x80, 0xC3, 0x01, /* ADD BL, 0x01 (Sets OF!) */
      0xE8, 0x03, 0x00, /* CALL +3 (to 0x0120)     */
      0xF4,             /* HLT                     */
      0x90, 0x90,       /* Padding to 0x0120       */
      /* 0x0120: Subroutine */
      0xBB, 0x00, 0x01, /* MOV BX, 0x0100          */
      0x31, 0xF6,       /* XOR SI, SI              */
      0xB8, 0x01, 0x00, /* MOV AX, 1               */
      0xB9, 0x01, 0x00, /* MOV CX, 1               */
      /* Loop (offset +10 = 0x012A): */
      0x89, 0x40, 0x10, /* MOV [BX+SI+0x10], AX    */
      0x01, 0xC8,       /* ADD AX, CX              */
      0x91,             /* XCHG AX, CX             */
      0x83, 0xC6, 0x02, /* ADD SI, 2               */
      0x83, 0xFE, 0x10, /* CMP SI, 16              */
      0x75, 0xF2,       /* JNE -14 (back to loop)  */
      0xC3              /* RET                     */
  };
  memcpy(&cpu.ram[0x00100], firmware, sizeof(firmware));

  printf("2. Live Microcode Execution Trace (First 12 Instructions):\n");
  int step_count = 0;
  while (!cpu.halted && step_count < 200) {
    /* Enable detailed microcode trace on key instructions */
    cpu.trace_microcode =
        (step_count >= 6 && step_count <= 11) || (step_count == 17);
    if (cpu.trace_microcode) {
      printf("  Macro-Instruction #%d at CS:IP = %04X:%04X:\n", step_count + 1,
             reg16_read(&cpu.segs[1]), reg16_read(&cpu.ip));
    }

    cpu8086_step(&cpu);

    if (step_count == 9) {
      printf("   -> After `ADD AL, 1` (0x0F + 0x01 = 0x10): AL=0x%02X ",
             reg16_read_lo(&cpu.regs[0]));
      print_8086_flags(reg16_read(&cpu.flags));
      printf("\n");
    }
    if (step_count == 11) {
      printf("   -> After `ADD BL, 1` (0x7F + 0x01 = 0x80): BL=0x%02X ",
             reg16_read_lo(&cpu.regs[3]));
      print_8086_flags(reg16_read(&cpu.flags));
      printf("\n");
    }
    step_count++;
  }

  printf("\n3. Spec-Exact 8086 Architectural Verification Results:\n");
  printf("   - Authentic 8086 `PUSH SP` Test : Initial SP=0x0800 -> Popped "
         "DX=0x%04X %s\n",
         reg16_read(&cpu.regs[2]),
         (reg16_read(&cpu.regs[2]) == 0x07FE) ? "(PASS: Pushed decremented SP!)"
                                              : "(FAIL)");

  printf("   - Segmented EA [BX+SI+0x10] RAM : DS:0110 (Physical 0x20110) "
         "Fibonacci Words:\n     ");
  for (int i = 0; i < 8; i++) {
    uint16_t val = biu_read_word_phys(&cpu, 0x2000, (uint16_t)(0x0110 + i * 2));
    printf("%u ", val);
  }
  printf("\n\n");

  printf(
      "4. Final 8086 Register File State (Transistor D-Flip-Flop Storage):\n");
  printf("   AX=%04X  BX=%04X  CX=%04X  DX=%04X\n", reg16_read(&cpu.regs[0]),
         reg16_read(&cpu.regs[3]), reg16_read(&cpu.regs[1]),
         reg16_read(&cpu.regs[2]));
  printf("   SP=%04X  BP=%04X  SI=%04X  DI=%04X\n", reg16_read(&cpu.regs[4]),
         reg16_read(&cpu.regs[5]), reg16_read(&cpu.regs[6]),
         reg16_read(&cpu.regs[7]));
  printf("   CS=%04X  DS=%04X  SS=%04X  ES=%04X  IP=%04X\n   ",
         reg16_read(&cpu.segs[1]), reg16_read(&cpu.segs[3]),
         reg16_read(&cpu.segs[2]), reg16_read(&cpu.segs[0]),
         reg16_read(&cpu.ip));
  print_8086_flags(reg16_read(&cpu.flags));
  printf("\n\n");

  printf("5. Hardware Simulation Totals:\n");
  printf("   - 8086 Macro-Instructions Executed : %llu\n",
         (unsigned long long)cpu.macro_instructions);
  printf("   - 8086 Microcode ROM Cycles (uPC)  : %llu\n",
         (unsigned long long)cpu.micro_cycles);
  printf(
      "   - Individual MOSFET Switches       : %llu transistor evaluations\n",
      (unsigned long long)g_transistor_switches);
  printf("====================================================================="
         "=====\n");

  return 0;
}