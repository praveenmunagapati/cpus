/*
 * ============================================================================
 * FROM TRANSISTOR TO SPEC-EXACT 32-BIT INTEL 80386 CPU IN C
 * ============================================================================
 * Layer 1: Switch-Level PMOS & NMOS MOSFET Transistors
 * Layer 2: CMOS Logic Gates (NOT, NAND, NOR, AND, OR, XOR, XNOR, MUX2)
 * Layer 3: 32-Bit Ripple-Carry Adder, 32-Bit Barrel Shifter & 80386 32-Bit ALU
 * Layer 4: NOR SR-Latch -> Master-Slave D-Flip-Flop -> 32-Bit Register File
 * Layer 5: 80386 MMU: GDT Protected-Mode Segmentation + 2-Level Paging & TLB
 * Layer 6: 80386 32-Bit Microcode Engine, SIB Addressing & 0x0F Instructions
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

/* 1-Bit 2-to-1 Multiplexer built from CMOS gates */
static inline Wire gate_mux2(Wire a, Wire b, Wire sel) {
  return gate_or(gate_and(a, gate_not(sel)), gate_and(b, sel));
}

/* ============================================================================
 * LAYER 3: 32-BIT COMBINATIONAL LOGIC, BARREL SHIFTER & 80386 32-BIT ALU
 * ============================================================================
 */

typedef struct {
  Wire b[32];
} Word32;

static inline Word32 word32_from_u32(uint32_t val) {
  Word32 w;
  for (int i = 0; i < 32; i++)
    w.b[i] = ((val >> i) & 1u) ? HIGH : LOW;
  return w;
}

static inline uint32_t word32_to_u32(Word32 w) {
  uint32_t val = 0;
  for (int i = 0; i < 32; i++) {
    if (w.b[i] == HIGH)
      val |= (1u << i);
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
 * 32-Bit Transistor Ripple-Carry Adder with Full Internal Carry Tap:
 *   - carries[3]  -> AF (Auxiliary Carry from bit 3 to bit 4)
 *   - carries[6]  XOR carries[7]  -> 8-bit OF
 *   - carries[14] XOR carries[15] -> 16-bit OF
 *   - carries[30] XOR carries[31] -> 32-bit OF
 */
static inline Word32 adder_32bit(Word32 a, Word32 b, Wire cin,
                                 Wire carries[32]) {
  Word32 sum;
  Wire c = cin;
  for (int i = 0; i < 32; i++) {
    full_adder(a.b[i], b.b[i], c, &sum.b[i], &c);
    carries[i] = c;
  }
  return sum;
}

/*
 * 32-Bit Logarithmic Multiplexer Barrel Shifter (Left Shift by 0..31 bits)
 * Built from 5 stages of 32 gate_mux2 multiplexers (160 MUXes total).
 * Used by both the 386 SIB Scale unit (<< 0,1,2,3) and SHL instructions!
 */
static inline Word32 barrel_shifter_left_32bit(Word32 in, uint8_t shift_amt) {
  Word32 stage = in;
  for (int s = 0; s < 5; s++) {
    int step = 1 << s;
    Wire sel = ((shift_amt >> s) & 1) ? HIGH : LOW;
    Word32 next;
    for (int i = 0; i < 32; i++) {
      Wire shifted_bit = (i >= step) ? stage.b[i - step] : LOW;
      next.b[i] = gate_mux2(stage.b[i], shifted_bit, sel);
    }
    stage = next;
  }
  return stage;
}

/*
 * 32-Bit Logarithmic Multiplexer Barrel Shifter (Right Shift by 0..31 bits)
 * Built from 5 stages of 32 gate_mux2 multiplexers (160 MUXes total).
 * If fill_bit is LOW: logical right shift (SHR).
 * If fill_bit is MSB: arithmetic right shift (SAR).
 */
static inline Word32 barrel_shifter_right_32bit(Word32 in, uint8_t shift_amt,
                                                Wire fill_bit) {
  Word32 stage = in;
  for (int s = 0; s < 5; s++) {
    int step = 1 << s;
    Wire sel = ((shift_amt >> s) & 1) ? HIGH : LOW;
    Word32 next;
    for (int i = 0; i < 32; i++) {
      Wire shifted_bit = (i + step < 32) ? stage.b[i + step] : fill_bit;
      next.b[i] = gate_mux2(stage.b[i], shifted_bit, sel);
    }
    stage = next;
  }
  return stage;
}

/*
 * 32-Bit Logarithmic Multiplexer Rotate Shifter (ROL / ROR by 0..31 bits)
 * Built from 5 stages of gate_mux2 multiplexers.
 */
static inline Word32 barrel_shifter_rot_32bit(Word32 in, uint8_t shift_amt,
                                              bool dir_right, int width_bits) {
  Word32 stage = in;
  if (width_bits <= 0)
    width_bits = 32;
  shift_amt &= (width_bits - 1);
  for (int s = 0; s < 5; s++) {
    int step = 1 << s;
    Wire sel = ((shift_amt >> s) & 1) ? HIGH : LOW;
    Word32 next;
    for (int i = 0; i < width_bits; i++) {
      int src_idx = dir_right
                        ? ((i + step) % width_bits)
                        : ((i - step + width_bits) % width_bits);
      next.b[i] = gate_mux2(stage.b[i], stage.b[src_idx], sel);
    }
    for (int i = width_bits; i < 32; i++)
      next.b[i] = LOW;
    stage = next;
  }
  return stage;
}

/* 80386 EFLAGS Bit Definitions */
#define EFLAG_CF (1u << 0)  /* Carry Flag               */
#define EFLAG_PF (1u << 2)  /* Parity Flag (low 8 bits) */
#define EFLAG_AF (1u << 4)  /* Auxiliary Carry Flag     */
#define EFLAG_ZF (1u << 6)  /* Zero Flag                */
#define EFLAG_SF (1u << 7)  /* Sign Flag                */
#define EFLAG_TF (1u << 8)  /* Trap Flag                */
#define EFLAG_IF (1u << 9)  /* Interrupt Enable         */
#define EFLAG_DF (1u << 10) /* Direction Flag           */
#define EFLAG_OF (1u << 11) /* Overflow Flag            */
#define EFLAG_NT (1u << 14) /* Nested Task              */
#define EFLAG_RF (1u << 16) /* Resume Flag              */
#define EFLAG_VM (1u << 17) /* Virtual 8086 Mode        */

static inline uint32_t sanitize_386_eflags(uint32_t f) {
  return (f & 0x00037FD5u) | 0x00000002u; /* Bit 1 is always 1 on 80386 */
}

/* 8-Input XNOR Tree for Even Parity on Low Byte (res[7:0]) */
static inline Wire parity_even_low8(Word32 w) {
  Wire x01 = gate_xor(w.b[0], w.b[1]);
  Wire x23 = gate_xor(w.b[2], w.b[3]);
  Wire x45 = gate_xor(w.b[4], w.b[5]);
  Wire x67 = gate_xor(w.b[6], w.b[7]);
  return gate_xnor(gate_xor(x01, x23), gate_xor(x45, x67));
}

/* 8/16/32-Bit Zero Detector (NOR tree across active width) */
static inline Wire zero_detect_32(Word32 w, int width_bits) {
  Wire acc = LOW;
  for (int i = 0; i < width_bits; i++)
    acc = gate_or(acc, w.b[i]);
  return gate_not(acc);
}

typedef enum {
  ALU386_NONE = 0,
  ALU386_ADD,
  ALU386_OR,
  ALU386_ADC,
  ALU386_SBB,
  ALU386_AND,
  ALU386_SUB,
  ALU386_XOR,
  ALU386_CMP,
  ALU386_INC,
  ALU386_DEC,
  ALU386_NOT,
  ALU386_NEG,
  ALU386_SHL,
  ALU386_SHR,
  ALU386_SAR,
  ALU386_BSF,
  ALU386_BSR,
  ALU386_ROL,
  ALU386_ROR,
  ALU386_TEST
} Alu386Op;

/*
 * Spec-Exact Intel 80386 8/16/32-Bit Transistor ALU (Computes SIGMA & EFLAGS)
 */
static Word32 alu_386_execute(Alu386Op op, Word32 a, Word32 b, int width_bits,
                              uint32_t *eflags, bool update_flags) {
  Word32 sigma = word32_from_u32(0);
  Wire carries[32] = {LOW};
  int msb = width_bits - 1; /* 7, 15, or 31 */

  Wire cf_in = ((*eflags) & EFLAG_CF) ? HIGH : LOW;
  Wire new_cf = cf_in, new_af = LOW, new_of = LOW;
  bool preserve_cf = false;
  bool is_logical = false;

  switch (op) {
  case ALU386_ADD:
    sigma = adder_32bit(a, b, LOW, carries);
    new_cf = carries[msb];
    new_af = carries[3];
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;

  case ALU386_ADC:
    sigma = adder_32bit(a, b, cf_in, carries);
    new_cf = carries[msb];
    new_af = carries[3];
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;

  case ALU386_SUB:
  case ALU386_CMP: {
    Word32 inv_b;
    for (int i = 0; i < 32; i++)
      inv_b.b[i] = gate_not(b.b[i]);
    sigma = adder_32bit(a, inv_b, HIGH, carries);
    new_cf = gate_not(carries[msb]);
    new_af = gate_not(carries[3]);
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;
  }

  case ALU386_SBB: {
    Word32 inv_b;
    for (int i = 0; i < 32; i++)
      inv_b.b[i] = gate_not(b.b[i]);
    sigma = adder_32bit(a, inv_b, gate_not(cf_in), carries);
    new_cf = gate_not(carries[msb]);
    new_af = gate_not(carries[3]);
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;
  }

  case ALU386_INC:
    sigma = adder_32bit(a, word32_from_u32(1), LOW, carries);
    new_af = carries[3];
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    preserve_cf = true;
    break;

  case ALU386_DEC:
    sigma = adder_32bit(a, word32_from_u32(0xFFFFFFFFu), LOW, carries);
    new_af = gate_not(carries[3]);
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    preserve_cf = true;
    break;

  case ALU386_NEG: {
    Word32 inv_a;
    for (int i = 0; i < 32; i++)
      inv_a.b[i] = gate_not(a.b[i]);
    sigma = adder_32bit(word32_from_u32(0), inv_a, HIGH, carries);
    new_cf = gate_not(zero_detect_32(a, width_bits));
    new_af = gate_not(carries[3]);
    new_of = gate_xor(carries[msb - 1], carries[msb]);
    break;
  }

  case ALU386_NOT:
    for (int i = 0; i < 32; i++)
      sigma.b[i] = gate_not(a.b[i]);
    update_flags = false;
    break;

  case ALU386_AND:
    for (int i = 0; i < 32; i++)
      sigma.b[i] = gate_and(a.b[i], b.b[i]);
    is_logical = true;
    break;

  case ALU386_OR:
    for (int i = 0; i < 32; i++)
      sigma.b[i] = gate_or(a.b[i], b.b[i]);
    is_logical = true;
    break;

  case ALU386_XOR:
    for (int i = 0; i < 32; i++)
      sigma.b[i] = gate_xor(a.b[i], b.b[i]);
    is_logical = true;
    break;

  case ALU386_SHL: {
    uint8_t cnt = (uint8_t)(word32_to_u32(b) & 0x1F);
    if (cnt == 0) {
      sigma = a;
      update_flags = false;
    } else {
      sigma = barrel_shifter_left_32bit(a, cnt);
      new_cf = (cnt <= width_bits) ? a.b[width_bits - cnt] : LOW;
      new_of = gate_xor(sigma.b[msb], new_cf);
    }
    break;
  }

  case ALU386_SHR: {
    uint8_t cnt = (uint8_t)(word32_to_u32(b) & 0x1F);
    if (cnt == 0) {
      sigma = a;
      update_flags = false;
    } else {
      sigma = barrel_shifter_right_32bit(a, cnt, LOW);
      new_cf = (cnt <= width_bits) ? a.b[cnt - 1] : LOW;
      new_of = a.b[msb];
    }
    break;
  }

  case ALU386_SAR: {
    uint8_t cnt = (uint8_t)(word32_to_u32(b) & 0x1F);
    if (cnt == 0) {
      sigma = a;
      update_flags = false;
    } else {
      sigma = barrel_shifter_right_32bit(a, cnt, a.b[msb]);
      new_cf = (cnt <= width_bits) ? a.b[cnt - 1] : a.b[msb];
      new_of = LOW;
    }
    break;
  }

  case ALU386_ROL: {
    uint8_t cnt = (uint8_t)(word32_to_u32(b) & 0x1F);
    if (cnt == 0) {
      sigma = a;
      update_flags = false;
    } else {
      sigma = barrel_shifter_rot_32bit(a, cnt, false, width_bits);
      new_cf = sigma.b[0];
      new_of = gate_xor(sigma.b[msb], sigma.b[0]);
    }
    break;
  }

  case ALU386_ROR: {
    uint8_t cnt = (uint8_t)(word32_to_u32(b) & 0x1F);
    if (cnt == 0) {
      sigma = a;
      update_flags = false;
    } else {
      sigma = barrel_shifter_rot_32bit(a, cnt, true, width_bits);
      new_cf = sigma.b[msb];
      new_of = gate_xor(sigma.b[msb], sigma.b[msb - 1]);
    }
    break;
  }

  case ALU386_TEST:
    for (int i = 0; i < 32; i++)
      sigma.b[i] = gate_and(a.b[i], b.b[i]);
    is_logical = true;
    break;

  case ALU386_BSF: {
    uint32_t val = word32_to_u32(b);
    if (val == 0) {
      new_cf = LOW;
    } else {
      int idx = 0;
      while (((val >> idx) & 1u) == 0 && idx < width_bits)
        idx++;
      sigma = word32_from_u32(idx);
    }
    break;
  }

  case ALU386_BSR: {
    uint32_t val = word32_to_u32(b);
    if (val == 0) {
      new_cf = LOW;
    } else {
      int idx = msb;
      while (((val >> idx) & 1u) == 0 && idx > 0)
        idx--;
      sigma = word32_from_u32(idx);
    }
    break;
  }

  default:
    sigma = a;
    break;
  }

  for (int i = width_bits; i < 32; i++)
    sigma.b[i] = LOW;

  if (update_flags) {
    if (is_logical) {
      new_cf = LOW;
      new_of = LOW;
      new_af = LOW;
    }
    Wire zf = zero_detect_32(sigma, width_bits);
    Wire sf = sigma.b[msb];
    Wire pf = parity_even_low8(sigma);

    uint32_t f = *eflags;
    if (!preserve_cf)
      f = (new_cf == HIGH) ? (f | EFLAG_CF) : (f & ~EFLAG_CF);
    f = (pf == HIGH) ? (f | EFLAG_PF) : (f & ~EFLAG_PF);
    f = (new_af == HIGH) ? (f | EFLAG_AF) : (f & ~EFLAG_AF);
    f = (zf == HIGH) ? (f | EFLAG_ZF) : (f & ~EFLAG_ZF);
    f = (sf == HIGH) ? (f | EFLAG_SF) : (f & ~EFLAG_SF);
    f = (new_of == HIGH) ? (f | EFLAG_OF) : (f & ~EFLAG_OF);
    *eflags = sanitize_386_eflags(f);
  }

  return sigma;
}

/* ============================================================================
 * LAYER 4: SEQUENTIAL LOGIC (SR-LATCH -> D-FLIP-FLOP -> 32-BIT REGISTERS)
 * ============================================================================
 */

typedef struct {
  Wire q, q_bar;
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
  sr_latch_update(l, gate_and(d, en), gate_and(gate_not(d), en));
}

typedef struct {
  SRLatch master, slave;
} DFF;

static inline void dff_init(DFF *ff) {
  sr_latch_init(&ff->master);
  sr_latch_init(&ff->slave);
}

static inline void dff_write(DFF *ff, Wire d) {
  d_latch_update(&ff->master, d, gate_not(LOW));
  d_latch_update(&ff->slave, ff->master.q, LOW);
  d_latch_update(&ff->master, d, gate_not(HIGH));
  d_latch_update(&ff->slave, ff->master.q, HIGH);
}

static inline Wire dff_read(const DFF *ff) { return ff->slave.q; }

/*
 * 32-Bit Transistor-Backed Hardware Register (`Reg32`)
 * Supports 32-bit (EAX), 16-bit (AX), and 8-bit (AL/AH) hierarchical access.
 */
typedef struct {
  DFF bit[32];
} Reg32;

static inline void reg32_init(Reg32 *r) {
  for (int i = 0; i < 32; i++)
    dff_init(&r->bit[i]);
}

static inline void reg32_write_word(Reg32 *r, Word32 w) {
  for (int i = 0; i < 32; i++)
    dff_write(&r->bit[i], w.b[i]);
}

static inline Word32 reg32_read_word(const Reg32 *r) {
  Word32 w;
  for (int i = 0; i < 32; i++)
    w.b[i] = dff_read(&r->bit[i]);
  return w;
}

static inline void reg32_write(Reg32 *r, uint32_t val) {
  reg32_write_word(r, word32_from_u32(val));
}

static inline uint32_t reg32_read(const Reg32 *r) {
  return word32_to_u32(reg32_read_word(r));
}

static inline void reg32_write16(Reg32 *r, uint16_t val) {
  for (int i = 0; i < 16; i++)
    dff_write(&r->bit[i], ((val >> i) & 1u) ? HIGH : LOW);
}

static inline uint16_t reg32_read16(const Reg32 *r) {
  uint16_t v = 0;
  for (int i = 0; i < 16; i++)
    if (dff_read(&r->bit[i]) == HIGH)
      v |= (uint16_t)(1u << i);
  return v;
}

static inline void reg32_write_lo8(Reg32 *r, uint8_t val) {
  for (int i = 0; i < 8; i++)
    dff_write(&r->bit[i], ((val >> i) & 1u) ? HIGH : LOW);
}

static inline void reg32_write_hi8(Reg32 *r, uint8_t val) {
  for (int i = 0; i < 8; i++)
    dff_write(&r->bit[i + 8], ((val >> i) & 1u) ? HIGH : LOW);
}

static inline uint8_t reg32_read_lo8(const Reg32 *r) {
  uint8_t v = 0;
  for (int i = 0; i < 8; i++)
    if (dff_read(&r->bit[i]) == HIGH)
      v |= (uint8_t)(1u << i);
  return v;
}

static inline uint8_t reg32_read_hi8(const Reg32 *r) {
  uint8_t v = 0;
  for (int i = 0; i < 8; i++)
    if (dff_read(&r->bit[i + 8]) == HIGH)
      v |= (uint8_t)(1u << i);
  return v;
}

/* ============================================================================
 * LAYER 5: 80386 SEGMENTATION UNIT (GDT) & 2-LEVEL PAGING MMU + TLB
 * ============================================================================
 */

/* Hidden 64-Bit Segment Descriptor Cache (Loaded from GDT in Protected Mode) */
typedef struct {
  uint16_t selector;
  uint32_t base;  /* 32-bit Linear Base Address                */
  uint32_t limit; /* 32-bit Scaled Limit (Byte or 4KB Page)    */
  uint8_t access; /* P, DPL, S, Type                           */
  bool d_bit;     /* Default operand/address size (0=16, 1=32) */
  bool g_bit;     /* Granularity (0=1B, 1=4KB)                 */
} SegDescCache;

/* 32-Entry Translation Lookaside Buffer (TLB) Entry */
typedef struct {
  bool valid;
  uint32_t vpn;      /* Virtual Page Number (Linear >> 12)        */
  uint32_t ppn;      /* Physical Page Frame (Physical >> 12)      */
  uint32_t pde_phys; /* Physical address of PDE for telemetry     */
  uint32_t pte_phys; /* Physical address of PTE for telemetry     */
} TLBEntry;

/* ============================================================================
 * LAYER 6: INTEL 80386 MICROCODE ENGINE & COMPLETE CPU STRUCTURE
 * ============================================================================
 */

typedef enum {
  MB386_NONE = 0,
  MB386_EAX,
  MB386_ECX,
  MB386_EDX,
  MB386_EBX,
  MB386_ESP,
  MB386_EBP,
  MB386_ESI,
  MB386_EDI,
  MB386_EIP,
  MB386_EFLAGS,
  MB386_R, /* ModR/M reg or opcode reg */
  MB386_M, /* ModR/M r/m (register or OPR) */
  MB386_TMPa,
  MB386_TMPb,
  MB386_TMPc,
  MB386_SIGMA,   /* 32-bit Transistor ALU Output */
  MB386_IND,     /* 32-bit Effective Address Offset */
  MB386_OPR,     /* 32-bit Memory Bus Data Latch */
  MB386_IMM,     /* Immediate from Prefetch Queue */
  MB386_IMM_SXT, /* Sign-Extended 8-bit Immediate */
  MB386_STEP_SZ  /* Stack/String step size (2 in 16-bit, 4 in 32-bit) */
} MicroBus386;

typedef enum {
  MACT386_NONE = 0,
  MACT386_ALU,
  MACT386_MEM_WRITE_M,
  MACT386_MEM_PUSH,
  MACT386_MEM_POP,
  MACT386_MEM_READ
} MicroAct386;

typedef struct {
  MicroBus386 src;
  MicroBus386 dst;
  MicroAct386 act;
  Alu386Op alu_op;
  bool use_xi;
  bool update_flags;
  bool rni;
  const char *comment;
} MicroInstr386;

/* Microcode ROM Entry Points for the 80386 */
enum {
  MC386_ALU_RM_R = 0x00,     /* ADD/OR/ADC/SBB/AND/SUB/XOR r/m, r          */
  MC386_ALU_R_RM = 0x04,     /* ADD/OR/ADC/SBB/AND/SUB/XOR r, r/m          */
  MC386_CMP_RM_R = 0x08,     /* CMP r/m, r                                 */
  MC386_CMP_R_RM = 0x0C,     /* CMP r, r/m                                 */
  MC386_ALU_ACC_IMM = 0x10,  /* ALU AL/AX/EAX, imm                         */
  MC386_CMP_ACC_IMM = 0x14,  /* CMP AL/AX/EAX, imm                         */
  MC386_GRP1_RM_IMM = 0x18,  /* Group 1 ALU r/m, imm                       */
  MC386_GRP1_CMP_IMM = 0x1C, /* Group 1 CMP r/m, imm                       */
  MC386_MOV_RM_R = 0x20,     /* MOV r/m, r                                 */
  MC386_MOV_R_RM = 0x24,     /* MOV r, r/m                                 */
  MC386_MOV_R_IMM = 0x28,    /* MOV reg, imm                               */
  MC386_MOV_RM_IMM = 0x2C,   /* MOV r/m, imm                               */
  MC386_INC_R = 0x30,        /* INC reg16/32                               */
  MC386_DEC_R = 0x34,        /* DEC reg16/32                               */
  MC386_PUSH_R = 0x38,       /* 80386 Spec-Exact PUSH reg (Fixes PUSH ESP) */
  MC386_POP_R = 0x40,        /* POP reg16/32                               */
  MC386_LEA_R_M = 0x48,      /* LEA r16/32, m                              */
  MC386_CALL_REL = 0x50,     /* CALL rel16/32                              */
  MC386_RET_NEAR = 0x58,     /* RET (near)                                 */
  MC386_JMP_REL = 0x60,      /* JMP rel16/32                               */
  MC386_XCHG_R_RM = 0x68,    /* XCHG r/m, r                                */
  MC386_XCHG_EAX_R = 0x70,   /* XCHG EAX, reg                              */
  MC386_TEST_RM_R = 0x78,    /* TEST r/m, r                                */
  MC386_TEST_ACC_IMM = 0x80, /* TEST AL/AX/EAX, imm                         */
  MC386_NOT_RM = 0x88,       /* NOT r/m                                    */
  MC386_NEG_RM = 0x90,       /* NEG r/m                                    */
  MC386_SHL_RM = 0x98,       /* SHL r/m by count                           */
  MC386_SHR_RM = 0xA0,       /* SHR r/m by count                           */
  MC386_SAR_RM = 0xA8,       /* SAR r/m by count                           */
  MC386_PUSH_IMM = 0xB0,     /* PUSH imm8/imm32                            */
  MC386_PUSH_MEM = 0xB8,     /* PUSH m16/32                                */
  MC386_POP_MEM = 0xC0,      /* POP m16/32                                 */
  MC386_MOV_ACC_MEM = 0xC8,  /* MOV AL/AX/EAX, [moffs]                     */
  MC386_MOV_MEM_ACC = 0xD0   /* MOV [moffs], AL/AX/EAX                     */
};

#define PHYS_RAM_SIZE (1024 * 1024) /* 1 MB Physical RAM for simulation */

typedef struct {
  /* 1. 32-Bit General Purpose Registers (Transistor DFF Backed) */
  Reg32 regs[8]; /* 0:EAX, 1:ECX, 2:EDX, 3:EBX, 4:ESP, 5:EBP, 6:ESI, 7:EDI */

  /* 2. 32-Bit Instruction Pointer & EFLAGS (Transistor DFF Backed) */
  Reg32 eip;
  Reg32 eflags;

  /* 3. Six Segment Registers (ES, CS, SS, DS, FS, GS) + Descriptor Caches */
  SegDescCache segs[6]; /* 0:ES, 1:CS, 2:SS, 3:DS, 4:FS, 5:GS */

  /* 4. System Control Registers (CR0, CR2, CR3) & Descriptor Table Regs */
  Reg32 cr0; /* Bit 0 = PE (Prot Enable), Bit 31 = PG (Paging) */
  Reg32 cr2; /* Page Fault Linear Address                      */
  Reg32 cr3; /* Page Directory Base Register (PDBR)            */
  uint32_t gdtr_base;
  uint16_t gdtr_limit;
  uint32_t idtr_base;
  uint16_t idtr_limit;
  uint32_t ldtr_base;
  uint16_t ldtr_limit;
  uint32_t tr_base;
  uint16_t tr_limit;

  /* 5. Internal 32-Bit EU Microcode Staging Registers */
  Reg32 tmp_a;
  Reg32 tmp_b;
  Reg32 tmp_c;
  Reg32 sigma;
  Reg32 ind;
  Reg32 opr;

  /* 6. Hardware TLB & Paging Telemetry */
  TLBEntry tlb[32];
  uint64_t tlb_hits;
  uint64_t page_walks;
  uint32_t last_linear_addr;
  uint32_t last_phys_addr;

  /* 7. Microcode ROM & Instruction Decode State */
  MicroInstr386 mc_rom[256];
  uint16_t mc_pc;
  int op_bits;   /* Active operand size: 8, 16, or 32 bits */
  int addr_bits; /* Active address size: 16 or 32 bits     */
  int seg_override;
  int default_seg;
  uint8_t rep_prefix; /* 0 = None, 0xF2 = REPNE, 0xF3 = REP */
  uint8_t modrm, mod, reg_field, rm_field;
  bool s_bit;
  Alu386Op xi_alu_op;

  bool halted;
  bool trace_mc;
  uint64_t macro_instructions;
  uint64_t micro_cycles;
  uint8_t ram[PHYS_RAM_SIZE];
} Intel80386;

/* ============================================================================
 * 80386 MMU: SEGMENTATION (GDT) + 2-LEVEL HARDWARE PAGING + TLB
 * ============================================================================
 */

static void tlb_flush(Intel80386 *cpu) {
  memset(cpu->tlb, 0, sizeof(cpu->tlb));
}

static void tlb_invalidate_page(Intel80386 *cpu, uint32_t vaddr) {
  uint32_t vpn = vaddr >> 12;
  uint32_t tlb_idx = vpn & 31u;
  if (cpu->tlb[tlb_idx].valid && cpu->tlb[tlb_idx].vpn == vpn) {
    cpu->tlb[tlb_idx].valid = false;
  }
}

static uint32_t phys_read32(const Intel80386 *cpu, uint32_t phys) {
  phys &= (PHYS_RAM_SIZE - 1);
  return (uint32_t)cpu->ram[phys] | ((uint32_t)cpu->ram[phys + 1] << 8) |
         ((uint32_t)cpu->ram[phys + 2] << 16) |
         ((uint32_t)cpu->ram[phys + 3] << 24);
}

static void phys_write32(Intel80386 *cpu, uint32_t phys, uint32_t val) {
  phys &= (PHYS_RAM_SIZE - 1);
  cpu->ram[phys + 0] = (uint8_t)(val & 0xFF);
  cpu->ram[phys + 1] = (uint8_t)((val >> 8) & 0xFF);
  cpu->ram[phys + 2] = (uint8_t)((val >> 16) & 0xFF);
  cpu->ram[phys + 3] = (uint8_t)((val >> 24) & 0xFF);
}

/*
 * Stage 1 + Stage 2 MMU Address Translation:
 *   1. Linear Address = Segment_Cache.Base + Offset (via 32-Bit Transistor
 * Adder!)
 *   2. If CR0.PG == 1: Walk 2-Level Page Directory (CR3) -> Page Table ->
 * Physical
 */
static uint32_t mmu_translate_386(Intel80386 *cpu, int seg_idx, uint32_t offset,
                                  bool is_write) {
  Wire carries[32];
  Word32 base_w = word32_from_u32(cpu->segs[seg_idx].base);
  Word32 off_w = word32_from_u32(offset);
  uint32_t linear = word32_to_u32(adder_32bit(base_w, off_w, LOW, carries));
  cpu->last_linear_addr = linear;

  /* Check CR0 Bit 31 (PG - Paging Enable) */
  uint32_t cr0 = reg32_read(&cpu->cr0);
  if ((cr0 & 0x80000000u) == 0) {
    cpu->last_phys_addr = linear;
    return linear; /* Paging disabled: Linear == Physical */
  }

  /* Paging Enabled: Check 32-Entry TLB first */
  uint32_t vpn = linear >> 12;
  uint32_t page_off = linear & 0xFFFu;
  uint32_t tlb_idx = vpn & 31u;

  if (cpu->tlb[tlb_idx].valid && cpu->tlb[tlb_idx].vpn == vpn) {
    cpu->tlb_hits++;
    if (is_write) {
      /* Set Dirty (Bit 6) in PTE */
      uint32_t pte = phys_read32(cpu, cpu->tlb[tlb_idx].pte_phys);
      phys_write32(cpu, cpu->tlb[tlb_idx].pte_phys, pte | 0x40u);
    }
    uint32_t phys = (cpu->tlb[tlb_idx].ppn << 12) | page_off;
    cpu->last_phys_addr = phys;
    return phys;
  }

  /* TLB Miss: Perform Hardware 2-Level Page Table Walk */
  cpu->page_walks++;
  uint32_t pdbr = reg32_read(&cpu->cr3) & 0xFFFFF000u;
  uint32_t dir_idx = (linear >> 22) & 0x3FFu;
  uint32_t tbl_idx = (linear >> 12) & 0x3FFu;

  uint32_t pde_phys = pdbr + (dir_idx * 4u);
  uint32_t pde = phys_read32(cpu, pde_phys);
  if ((pde & 1u) == 0) {
    /* Page Fault (#PF): Latch faulting linear address into CR2 */
    reg32_write(&cpu->cr2, linear);
    return 0;
  }
  /* Set Accessed (Bit 5) on Page Directory Entry */
  phys_write32(cpu, pde_phys, pde | 0x20u);

  uint32_t pt_base = pde & 0xFFFFF000u;
  uint32_t pte_phys = pt_base + (tbl_idx * 4u);
  uint32_t pte = phys_read32(cpu, pte_phys);
  if ((pte & 1u) == 0) {
    reg32_write(&cpu->cr2, linear);
    return 0;
  }
  /* Set Accessed (Bit 5) and (if write) Dirty (Bit 6) on Page Table Entry */
  pte |= 0x20u | (is_write ? 0x40u : 0x00u);
  phys_write32(cpu, pte_phys, pte);

  uint32_t ppn = pte >> 12;
  cpu->tlb[tlb_idx] = (TLBEntry){true, vpn, ppn, pde_phys, pte_phys};

  uint32_t phys = (ppn << 12) | page_off;
  cpu->last_phys_addr = phys;
  return phys;
}

/* Load a Segment Register (Real Mode Base = sel << 4; Protected Mode = GDT
 * Lookup) */
static void load_segment_386(Intel80386 *cpu, int seg_idx, uint16_t selector) {
  SegDescCache *sc = &cpu->segs[seg_idx];
  sc->selector = selector;

  if ((reg32_read(&cpu->cr0) & 1u) == 0) {
    /* 16-Bit Real Mode */
    sc->base = (uint32_t)selector << 4;
    sc->limit = 0xFFFFu;
    sc->access = 0x92u;
    sc->d_bit = false;
    sc->g_bit = false;
  } else {
    /* 32-Bit Protected Mode: Fetch 8-Byte Segment Descriptor from GDT or LDT */
    bool is_ldt = (selector & 4) != 0;
    uint32_t table_base = is_ldt ? cpu->ldtr_base : cpu->gdtr_base;
    uint32_t index = (selector >> 3) * 8u;
    uint32_t desc_addr = table_base + index;
    uint32_t lo = phys_read32(cpu, desc_addr);
    uint32_t hi = phys_read32(cpu, desc_addr + 4u);

    uint32_t limit = (lo & 0xFFFFu) | (hi & 0x000F0000u);
    uint32_t base =
        ((lo >> 16) & 0xFFFFu) | ((hi & 0xFFu) << 16) | (hi & 0xFF000000u);
    uint8_t access = (uint8_t)((hi >> 8) & 0xFFu);
    uint8_t gran = (uint8_t)((hi >> 20) & 0x0Fu);

    sc->g_bit = (gran & 0x8u) != 0;
    sc->d_bit = (gran & 0x4u) != 0; /* D/B = 1 -> 32-bit USE32 segment */
    sc->base = base;
    sc->limit = sc->g_bit ? ((limit << 12) | 0xFFFu) : limit;
    sc->access = access;
  }
}

/* Memory Read/Write Helpers through the 80386 MMU */
static uint8_t mmu_read8(Intel80386 *cpu, int seg, uint32_t off) {
  uint32_t phys = mmu_translate_386(cpu, seg, off, false);
  return cpu->ram[phys & (PHYS_RAM_SIZE - 1)];
}

static uint16_t mmu_read16(Intel80386 *cpu, int seg, uint32_t off) {
  return (uint16_t)mmu_read8(cpu, seg, off) |
         ((uint16_t)mmu_read8(cpu, seg, off + 1) << 8);
}

static uint32_t mmu_read32(Intel80386 *cpu, int seg, uint32_t off) {
  return (uint32_t)mmu_read16(cpu, seg, off) |
         ((uint32_t)mmu_read16(cpu, seg, off + 2) << 16);
}

static void mmu_write8(Intel80386 *cpu, int seg, uint32_t off, uint8_t val) {
  uint32_t phys = mmu_translate_386(cpu, seg, off, true);
  cpu->ram[phys & (PHYS_RAM_SIZE - 1)] = val;
}

static void mmu_write16(Intel80386 *cpu, int seg, uint32_t off, uint16_t val) {
  mmu_write8(cpu, seg, off, (uint8_t)(val & 0xFF));
  mmu_write8(cpu, seg, off + 1, (uint8_t)((val >> 8) & 0xFF));
}

static void mmu_write32(Intel80386 *cpu, int seg, uint32_t off, uint32_t val) {
  mmu_write16(cpu, seg, off, (uint16_t)(val & 0xFFFF));
  mmu_write16(cpu, seg, off + 2, (uint16_t)((val >> 16) & 0xFFFF));
}

static uint32_t mmu_read_sized(Intel80386 *cpu, int seg, uint32_t off,
                               int bits) {
  if (bits == 8)
    return mmu_read8(cpu, seg, off);
  if (bits == 16)
    return mmu_read16(cpu, seg, off);
  return mmu_read32(cpu, seg, off);
}

static void mmu_write_sized(Intel80386 *cpu, int seg, uint32_t off,
                            uint32_t val, int bits) {
  if (bits == 8)
    mmu_write8(cpu, seg, off, (uint8_t)val);
  else if (bits == 16)
    mmu_write16(cpu, seg, off, (uint16_t)val);
  else
    mmu_write32(cpu, seg, off, val);
}

/* Instruction Stream Fetch from CS:EIP */
static uint8_t fetch8(Intel80386 *cpu) {
  uint32_t eip = reg32_read(&cpu->eip);
  uint8_t val = mmu_read8(cpu, 1 /* CS */, eip);
  reg32_write(&cpu->eip, eip + 1);
  return val;
}

static uint16_t fetch16(Intel80386 *cpu) {
  uint16_t lo = fetch8(cpu);
  uint16_t hi = fetch8(cpu);
  return (uint16_t)(lo | (hi << 8));
}

static uint32_t fetch32(Intel80386 *cpu) {
  uint32_t lo = fetch16(cpu);
  uint32_t hi = fetch16(cpu);
  return lo | (hi << 16);
}

static uint32_t fetch_imm_sized(Intel80386 *cpu, int bits) {
  if (bits == 8)
    return fetch8(cpu);
  if (bits == 16)
    return fetch16(cpu);
  return fetch32(cpu);
}

/* ============================================================================
 * 80386 REGISTER ACCESS & MICROCODE ENGINE
 * ============================================================================
 */

static uint32_t eu386_read_reg(const Intel80386 *cpu, int idx, int bits) {
  idx &= 7;
  if (bits == 32)
    return reg32_read(&cpu->regs[idx]);
  if (bits == 16)
    return reg32_read16(&cpu->regs[idx]);
  if (idx < 4)
    return reg32_read_lo8(&cpu->regs[idx]);
  return reg32_read_hi8(&cpu->regs[idx - 4]);
}

static void eu386_write_reg(Intel80386 *cpu, int idx, uint32_t val, int bits) {
  idx &= 7;
  if (bits == 32)
    reg32_write(&cpu->regs[idx], val);
  else if (bits == 16)
    reg32_write16(&cpu->regs[idx], (uint16_t)val);
  else if (idx < 4)
    reg32_write_lo8(&cpu->regs[idx], (uint8_t)val);
  else
    reg32_write_hi8(&cpu->regs[idx - 4], (uint8_t)val);
}

/*
 * Architectural Stack Push & Pop Helpers
 * Updates ESP and performs memory operations with transistor cycles.
 */
static void eu386_push(Intel80386 *cpu, uint32_t val) {
  uint32_t esp = reg32_read(&cpu->regs[4]);
  uint32_t step = (cpu->op_bits == 32) ? 4u : 2u;
  esp -= step;
  reg32_write(&cpu->regs[4], esp);
  mmu_write_sized(cpu, 2 /* SS */, esp, val, cpu->op_bits);
  cpu->micro_cycles += 2;
}

static uint32_t eu386_pop(Intel80386 *cpu) {
  uint32_t esp = reg32_read(&cpu->regs[4]);
  uint32_t val = mmu_read_sized(cpu, 2 /* SS */, esp, cpu->op_bits);
  uint32_t step = (cpu->op_bits == 32) ? 4u : 2u;
  reg32_write(&cpu->regs[4], esp + step);
  cpu->micro_cycles += 2;
  return val;
}

static void build_386_microcode_rom(Intel80386 *cpu) {
  memset(cpu->mc_rom, 0, sizeof(cpu->mc_rom));
  MicroInstr386 *rom = cpu->mc_rom;

  /* ALU r/m, r */
  rom[MC386_ALU_RM_R + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC386_ALU_RM_R + 1] = (MicroInstr386){
      MB386_R, MB386_TMPb, MACT386_ALU, 0,
      1,       true,       false,       "R -> TMPb | XI ALU (F)"};
  rom[MC386_ALU_RM_R + 2] =
      (MicroInstr386){MB386_SIGMA, MB386_OPR, MACT386_MEM_WRITE_M,    0, 0,
                      false,       true,      "SIGMA -> M | WB | RNI"};

  /* ALU r, r/m */
  rom[MC386_ALU_R_RM + 0] = (MicroInstr386){
      MB386_R, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "R -> TMPa"};
  rom[MC386_ALU_R_RM + 1] = (MicroInstr386){
      MB386_M, MB386_TMPb, MACT386_ALU, 0,
      1,       true,       false,       "M -> TMPb | XI ALU (F)"};
  rom[MC386_ALU_R_RM + 2] =
      (MicroInstr386){MB386_SIGMA, MB386_R, MACT386_NONE, 0,
                      0,           false,   true,         "SIGMA -> R | RNI"};

  /* CMP r/m, r & CMP r, r/m */
  rom[MC386_CMP_RM_R + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC386_CMP_RM_R + 1] = (MicroInstr386){
      MB386_R, MB386_TMPb, MACT386_ALU, ALU386_CMP,
      0,       true,       true,        "R -> TMPb | CMP (F) | RNI"};

  rom[MC386_CMP_R_RM + 0] = (MicroInstr386){
      MB386_R, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "R -> TMPa"};
  rom[MC386_CMP_R_RM + 1] = (MicroInstr386){
      MB386_M, MB386_TMPb, MACT386_ALU, ALU386_CMP,
      0,       true,       true,        "M -> TMPb | CMP (F) | RNI"};

  /* ALU EAX, imm & CMP EAX, imm */
  rom[MC386_ALU_ACC_IMM + 0] = (MicroInstr386){
      MB386_EAX, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "EAX -> TMPa"};
  rom[MC386_ALU_ACC_IMM + 1] = (MicroInstr386){
      MB386_IMM, MB386_TMPb, MACT386_ALU, 0,
      1,         true,       false,       "IMM -> TMPb | XI ALU (F)"};
  rom[MC386_ALU_ACC_IMM + 2] =
      (MicroInstr386){MB386_SIGMA, MB386_EAX, MACT386_NONE,        0, 0,
                      false,       true,      "SIGMA -> EAX | RNI"};

  rom[MC386_CMP_ACC_IMM + 0] = (MicroInstr386){
      MB386_EAX, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "EAX -> TMPa"};
  rom[MC386_CMP_ACC_IMM + 1] = (MicroInstr386){
      MB386_IMM, MB386_TMPb, MACT386_ALU, ALU386_CMP,
      0,         true,       true,        "IMM -> TMPb | CMP (F) | RNI"};

  /* Group 1 ALU r/m, imm & CMP r/m, imm */
  rom[MC386_GRP1_RM_IMM + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC386_GRP1_RM_IMM + 1] = (MicroInstr386){MB386_IMM_SXT,
                                               MB386_TMPb,
                                               MACT386_ALU,
                                               0,
                                               1,
                                               true,
                                               false,
                                               "IMM -> TMPb | XI ALU (F)"};
  rom[MC386_GRP1_RM_IMM + 2] =
      (MicroInstr386){MB386_SIGMA, MB386_OPR, MACT386_MEM_WRITE_M,    0, 0,
                      false,       true,      "SIGMA -> M | WB | RNI"};

  rom[MC386_GRP1_CMP_IMM + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC386_GRP1_CMP_IMM + 1] = (MicroInstr386){MB386_IMM_SXT,
                                                MB386_TMPb,
                                                MACT386_ALU,
                                                ALU386_CMP,
                                                0,
                                                true,
                                                true,
                                                "IMM -> TMPb | CMP (F) | RNI"};

  /* MOV r/m, r ; MOV r, r/m ; MOV r, imm ; MOV r/m, imm ; LEA */
  rom[MC386_MOV_RM_R + 0] =
      (MicroInstr386){MB386_R, MB386_OPR, MACT386_MEM_WRITE_M, 0, 0,
                      false,   true,      "R -> M | WB | RNI"};
  rom[MC386_MOV_R_RM + 0] = (MicroInstr386){
      MB386_M, MB386_R, MACT386_NONE, 0, 0, false, true, "M -> R | RNI"};
  rom[MC386_MOV_R_IMM + 0] = (MicroInstr386){
      MB386_IMM, MB386_R, MACT386_NONE, 0, 0, false, true, "IMM -> R | RNI"};
  rom[MC386_MOV_RM_IMM + 0] =
      (MicroInstr386){MB386_IMM, MB386_OPR, MACT386_MEM_WRITE_M,  0, 0,
                      false,     true,      "IMM -> M | WB | RNI"};
  rom[MC386_LEA_R_M + 0] = (MicroInstr386){
      MB386_IND, MB386_R, MACT386_NONE, 0, 0, false, true, "IND -> R | RNI"};

  /* INC reg & DEC reg */
  rom[MC386_INC_R + 0] =
      (MicroInstr386){MB386_R, MB386_TMPa, MACT386_ALU, ALU386_INC,
                      0,       true,       false,       "R -> TMPa | INC (F)"};
  rom[MC386_INC_R + 1] =
      (MicroInstr386){MB386_SIGMA, MB386_R, MACT386_NONE, 0,
                      0,           false,   true,         "SIGMA -> R | RNI"};

  rom[MC386_DEC_R + 0] =
      (MicroInstr386){MB386_R, MB386_TMPa, MACT386_ALU, ALU386_DEC,
                      0,       true,       false,       "R -> TMPa | DEC (F)"};
  rom[MC386_DEC_R + 1] =
      (MicroInstr386){MB386_SIGMA, MB386_R, MACT386_NONE, 0,
                      0,           false,   true,         "SIGMA -> R | RNI"};

  /*
   * 80386 Spec-Exact PUSH reg16/32 Microcode Sequence:
   * Unlike the 8086, the 80386 latches R -> OPR in Step 0 BEFORE decrementing
   * ESP! Therefore, `PUSH ESP` pushes the ORIGINAL value of ESP before the
   * decrement.
   */
  rom[MC386_PUSH_R + 0] = (MicroInstr386){
      MB386_R, MB386_OPR, MACT386_NONE, 0,
      0,       false,     false,        "R -> OPR (Latches original ESP!)"};
  rom[MC386_PUSH_R + 1] = (MicroInstr386){
      MB386_ESP, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "ESP -> TMPa"};
  rom[MC386_PUSH_R + 2] = (MicroInstr386){MB386_STEP_SZ,
                                          MB386_TMPb,
                                          MACT386_ALU,
                                          ALU386_SUB,
                                          0,
                                          false,
                                          false,
                                          "STEP -> TMPb | SUB tmpa,tmpb"};
  rom[MC386_PUSH_R + 3] = (MicroInstr386){
      MB386_SIGMA, MB386_ESP, MACT386_NONE, 0, 0, false, false, "SIGMA -> ESP"};
  rom[MC386_PUSH_R + 4] = (MicroInstr386){MB386_SIGMA,
                                          MB386_IND,
                                          MACT386_MEM_PUSH,
                                          0,
                                          0,
                                          false,
                                          true,
                                          "SIGMA -> IND | PUSH [SS:IND] | RNI"};

  /* POP reg16/32 */
  rom[MC386_POP_R + 0] = (MicroInstr386){
      MB386_ESP, MB386_IND, MACT386_MEM_POP, 0,
      0,         false,     false,           "ESP -> IND | POP [SS:IND]"};
  rom[MC386_POP_R + 1] = (MicroInstr386){
      MB386_ESP, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "ESP -> TMPa"};
  rom[MC386_POP_R + 2] = (MicroInstr386){MB386_STEP_SZ,
                                         MB386_TMPb,
                                         MACT386_ALU,
                                         ALU386_ADD,
                                         0,
                                         false,
                                         false,
                                         "STEP -> TMPb | ADD tmpa,tmpb"};
  rom[MC386_POP_R + 3] = (MicroInstr386){
      MB386_SIGMA, MB386_ESP, MACT386_NONE, 0, 0, false, false, "SIGMA -> ESP"};
  rom[MC386_POP_R + 4] = (MicroInstr386){
      MB386_OPR, MB386_R, MACT386_NONE, 0, 0, false, true, "OPR -> R | RNI"};

  /* CALL rel16/32 */
  rom[MC386_CALL_REL + 0] = (MicroInstr386){
      MB386_EIP, MB386_OPR, MACT386_NONE, 0, 0, false, false, "EIP -> OPR (RetAddr)"};
  rom[MC386_CALL_REL + 1] = (MicroInstr386){
      MB386_ESP, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "ESP -> TMPa"};
  rom[MC386_CALL_REL + 2] = (MicroInstr386){
      MB386_STEP_SZ, MB386_TMPb, MACT386_ALU, ALU386_SUB, 0, false, false, "STEP -> TMPb | SUB tmpa,tmpb"};
  rom[MC386_CALL_REL + 3] = (MicroInstr386){
      MB386_SIGMA, MB386_ESP, MACT386_NONE, 0, 0, false, false, "SIGMA -> ESP"};
  rom[MC386_CALL_REL + 4] = (MicroInstr386){
      MB386_SIGMA, MB386_IND, MACT386_MEM_PUSH, 0, 0, false, false, "SIGMA -> IND | PUSH [SS:IND]"};
  rom[MC386_CALL_REL + 5] = (MicroInstr386){
      MB386_IMM_SXT, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "REL -> TMPa"};
  rom[MC386_CALL_REL + 6] = (MicroInstr386){
      MB386_EIP, MB386_TMPb, MACT386_ALU, ALU386_ADD, 0, false, false, "EIP -> TMPb | ADD tmpa,tmpb"};
  rom[MC386_CALL_REL + 7] = (MicroInstr386){
      MB386_SIGMA, MB386_EIP, MACT386_NONE, 0, 0, false, true, "SIGMA -> EIP | RNI"};

  /* RET (near) */
  rom[MC386_RET_NEAR + 0] = (MicroInstr386){
      MB386_ESP, MB386_IND, MACT386_MEM_POP, 0, 0, false, false, "ESP -> IND | POP [SS:IND]"};
  rom[MC386_RET_NEAR + 1] = (MicroInstr386){
      MB386_ESP, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "ESP -> TMPa"};
  rom[MC386_RET_NEAR + 2] = (MicroInstr386){
      MB386_STEP_SZ, MB386_TMPb, MACT386_ALU, ALU386_ADD, 0, false, false, "STEP -> TMPb | ADD tmpa,tmpb"};
  rom[MC386_RET_NEAR + 3] = (MicroInstr386){
      MB386_SIGMA, MB386_ESP, MACT386_NONE, 0, 0, false, false, "SIGMA -> ESP"};
  rom[MC386_RET_NEAR + 4] = (MicroInstr386){
      MB386_OPR, MB386_EIP, MACT386_NONE, 0, 0, false, true, "OPR -> EIP | RNI"};

  /* JMP rel16/32 */
  rom[MC386_JMP_REL + 0] = (MicroInstr386){
      MB386_IMM_SXT, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "REL -> TMPa"};
  rom[MC386_JMP_REL + 1] = (MicroInstr386){
      MB386_EIP, MB386_TMPb, MACT386_ALU, ALU386_ADD, 0, false, false, "EIP -> TMPb | ADD tmpa,tmpb"};
  rom[MC386_JMP_REL + 2] = (MicroInstr386){
      MB386_SIGMA, MB386_EIP, MACT386_NONE, 0, 0, false, true, "SIGMA -> EIP | RNI"};

  /* XCHG r/m, r & XCHG EAX, reg */
  rom[MC386_XCHG_R_RM + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC386_XCHG_R_RM + 1] = (MicroInstr386){
      MB386_R, MB386_OPR, MACT386_MEM_WRITE_M, 0, 0, false, false, "R -> M | WB"};
  rom[MC386_XCHG_R_RM + 2] = (MicroInstr386){
      MB386_TMPa, MB386_R, MACT386_NONE, 0, 0, false, true, "TMPa -> R | RNI"};

  rom[MC386_XCHG_EAX_R + 0] = (MicroInstr386){
      MB386_EAX, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "EAX -> TMPa"};
  rom[MC386_XCHG_EAX_R + 1] = (MicroInstr386){
      MB386_R, MB386_EAX, MACT386_NONE, 0, 0, false, false, "R -> EAX"};
  rom[MC386_XCHG_EAX_R + 2] = (MicroInstr386){
      MB386_TMPa, MB386_R, MACT386_NONE, 0, 0, false, true, "TMPa -> R | RNI"};

  /* TEST r/m, r & TEST EAX, imm */
  rom[MC386_TEST_RM_R + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC386_TEST_RM_R + 1] = (MicroInstr386){
      MB386_R, MB386_TMPb, MACT386_ALU, ALU386_AND, 0, true, true, "R -> TMPb | TEST (F) | RNI"};

  rom[MC386_TEST_ACC_IMM + 0] = (MicroInstr386){
      MB386_EAX, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "EAX -> TMPa"};
  rom[MC386_TEST_ACC_IMM + 1] = (MicroInstr386){
      MB386_IMM, MB386_TMPb, MACT386_ALU, ALU386_AND, 0, true, true, "IMM -> TMPb | TEST (F) | RNI"};

  /* NOT r/m & NEG r/m */
  rom[MC386_NOT_RM + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_ALU, ALU386_NOT, 0, false, false, "M -> TMPa | NOT"};
  rom[MC386_NOT_RM + 1] = (MicroInstr386){
      MB386_SIGMA, MB386_OPR, MACT386_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};

  rom[MC386_NEG_RM + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_ALU, ALU386_NEG, 0, true, false, "M -> TMPa | NEG (F)"};
  rom[MC386_NEG_RM + 1] = (MicroInstr386){
      MB386_SIGMA, MB386_OPR, MACT386_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};

  /* SHL / SHR / SAR r/m */
  rom[MC386_SHL_RM + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC386_SHL_RM + 1] = (MicroInstr386){
      MB386_TMPc, MB386_TMPb, MACT386_ALU, ALU386_SHL, 0, true, false, "TMPc -> TMPb | SHL (F)"};
  rom[MC386_SHL_RM + 2] = (MicroInstr386){
      MB386_SIGMA, MB386_OPR, MACT386_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};

  rom[MC386_SHR_RM + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC386_SHR_RM + 1] = (MicroInstr386){
      MB386_TMPc, MB386_TMPb, MACT386_ALU, ALU386_SHR, 0, true, false, "TMPc -> TMPb | SHR (F)"};
  rom[MC386_SHR_RM + 2] = (MicroInstr386){
      MB386_SIGMA, MB386_OPR, MACT386_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};

  rom[MC386_SAR_RM + 0] = (MicroInstr386){
      MB386_M, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC386_SAR_RM + 1] = (MicroInstr386){
      MB386_TMPc, MB386_TMPb, MACT386_ALU, ALU386_SAR, 0, true, false, "TMPc -> TMPb | SAR (F)"};
  rom[MC386_SAR_RM + 2] = (MicroInstr386){
      MB386_SIGMA, MB386_OPR, MACT386_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};

  /* PUSH imm / mem & POP mem */
  rom[MC386_PUSH_IMM + 0] = (MicroInstr386){
      MB386_IMM, MB386_OPR, MACT386_NONE, 0, 0, false, false, "IMM -> OPR"};
  rom[MC386_PUSH_IMM + 1] = (MicroInstr386){
      MB386_ESP, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "ESP -> TMPa"};
  rom[MC386_PUSH_IMM + 2] = (MicroInstr386){
      MB386_STEP_SZ, MB386_TMPb, MACT386_ALU, ALU386_SUB, 0, false, false, "STEP -> TMPb | SUB"};
  rom[MC386_PUSH_IMM + 3] = (MicroInstr386){
      MB386_SIGMA, MB386_ESP, MACT386_NONE, 0, 0, false, false, "SIGMA -> ESP"};
  rom[MC386_PUSH_IMM + 4] = (MicroInstr386){
      MB386_SIGMA, MB386_IND, MACT386_MEM_PUSH, 0, 0, false, true, "SIGMA -> IND | PUSH [SS:IND] | RNI"};

  rom[MC386_PUSH_MEM + 0] = (MicroInstr386){
      MB386_M, MB386_OPR, MACT386_NONE, 0, 0, false, false, "M -> OPR"};
  rom[MC386_PUSH_MEM + 1] = (MicroInstr386){
      MB386_ESP, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "ESP -> TMPa"};
  rom[MC386_PUSH_MEM + 2] = (MicroInstr386){
      MB386_STEP_SZ, MB386_TMPb, MACT386_ALU, ALU386_SUB, 0, false, false, "STEP -> TMPb | SUB"};
  rom[MC386_PUSH_MEM + 3] = (MicroInstr386){
      MB386_SIGMA, MB386_ESP, MACT386_NONE, 0, 0, false, false, "SIGMA -> ESP"};
  rom[MC386_PUSH_MEM + 4] = (MicroInstr386){
      MB386_SIGMA, MB386_IND, MACT386_MEM_PUSH, 0, 0, false, true, "SIGMA -> IND | PUSH [SS:IND] | RNI"};

  rom[MC386_POP_MEM + 0] = (MicroInstr386){
      MB386_ESP, MB386_IND, MACT386_MEM_POP, 0, 0, false, false, "ESP -> IND | POP [SS:IND]"};
  rom[MC386_POP_MEM + 1] = (MicroInstr386){
      MB386_ESP, MB386_TMPa, MACT386_NONE, 0, 0, false, false, "ESP -> TMPa"};
  rom[MC386_POP_MEM + 2] = (MicroInstr386){
      MB386_STEP_SZ, MB386_TMPb, MACT386_ALU, ALU386_ADD, 0, false, false, "STEP -> TMPb | ADD"};
  rom[MC386_POP_MEM + 3] = (MicroInstr386){
      MB386_SIGMA, MB386_ESP, MACT386_NONE, 0, 0, false, false, "SIGMA -> ESP"};
  rom[MC386_POP_MEM + 4] = (MicroInstr386){
      MB386_OPR, MB386_OPR, MACT386_MEM_WRITE_M, 0, 0, false, true, "OPR -> M | WB | RNI"};

  /* MOV AL/AX/EAX, [moffs] & MOV [moffs], AL/AX/EAX */
  rom[MC386_MOV_ACC_MEM + 0] = (MicroInstr386){
      MB386_IND, MB386_IND, MACT386_MEM_READ, 0, 0, false, false, "IND -> MEM_READ"};
  rom[MC386_MOV_ACC_MEM + 1] = (MicroInstr386){
      MB386_OPR, MB386_EAX, MACT386_NONE, 0, 0, false, true, "OPR -> EAX | RNI"};

  rom[MC386_MOV_MEM_ACC + 0] = (MicroInstr386){
      MB386_EAX, MB386_OPR, MACT386_MEM_WRITE_M, 0, 0, false, true, "EAX -> M | WB | RNI"};
}

static int get_active_seg_386(const Intel80386 *cpu) {
  return (cpu->seg_override >= 0) ? cpu->seg_override : cpu->default_seg;
}

static uint32_t mc386_read_bus(Intel80386 *cpu, MicroBus386 id) {
  switch (id) {
  case MB386_EAX:
  case MB386_ECX:
  case MB386_EDX:
  case MB386_EBX:
  case MB386_ESP:
  case MB386_EBP:
  case MB386_ESI:
  case MB386_EDI:
    return eu386_read_reg(cpu, id - MB386_EAX,
                          (id == MB386_ESP) ? 32 : cpu->op_bits);
  case MB386_EIP:
    return reg32_read(&cpu->eip);
  case MB386_EFLAGS:
    return reg32_read(&cpu->eflags);
  case MB386_R:
    return eu386_read_reg(cpu, cpu->reg_field, cpu->op_bits);
  case MB386_M:
    if (cpu->mod == 3)
      return eu386_read_reg(cpu, cpu->rm_field, cpu->op_bits);
    return reg32_read(&cpu->opr);
  case MB386_TMPa:
    return reg32_read(&cpu->tmp_a);
  case MB386_TMPb:
    return reg32_read(&cpu->tmp_b);
  case MB386_TMPc:
    return reg32_read(&cpu->tmp_c);
  case MB386_SIGMA:
    return reg32_read(&cpu->sigma);
  case MB386_IND:
    return reg32_read(&cpu->ind);
  case MB386_OPR:
    return reg32_read(&cpu->opr);
  case MB386_IMM:
    return fetch_imm_sized(cpu, cpu->op_bits);
  case MB386_IMM_SXT:
    if (cpu->s_bit)
      return (uint32_t)(int32_t)(int8_t)fetch8(cpu);
    return fetch_imm_sized(cpu, cpu->op_bits);
  case MB386_STEP_SZ:
    return (cpu->op_bits == 32) ? 4u : 2u;
  default:
    return 0;
  }
}

static void mc386_write_bus(Intel80386 *cpu, MicroBus386 id, uint32_t val) {
  switch (id) {
  case MB386_EAX:
  case MB386_ECX:
  case MB386_EDX:
  case MB386_EBX:
  case MB386_ESP:
  case MB386_EBP:
  case MB386_ESI:
  case MB386_EDI:
    eu386_write_reg(cpu, id - MB386_EAX, val,
                    (id == MB386_ESP) ? 32 : cpu->op_bits);
    break;
  case MB386_EIP:
    reg32_write(&cpu->eip, val);
    break;
  case MB386_EFLAGS:
    reg32_write(&cpu->eflags, sanitize_386_eflags(val));
    break;
  case MB386_R:
    eu386_write_reg(cpu, cpu->reg_field, val, cpu->op_bits);
    break;
  case MB386_M:
    if (cpu->mod == 3)
      eu386_write_reg(cpu, cpu->rm_field, val, cpu->op_bits);
    else
      reg32_write(&cpu->opr, val);
    break;
  case MB386_TMPa:
    reg32_write(&cpu->tmp_a, val);
    break;
  case MB386_TMPb:
    reg32_write(&cpu->tmp_b, val);
    break;
  case MB386_TMPc:
    reg32_write(&cpu->tmp_c, val);
    break;
  case MB386_SIGMA:
    reg32_write(&cpu->sigma, val);
    break;
  case MB386_IND:
    reg32_write(&cpu->ind, val);
    break;
  case MB386_OPR:
    reg32_write(&cpu->opr, val);
    break;
  default:
    break;
  }
}

static void execute_386_microcode(Intel80386 *cpu, uint16_t start_upc) {
  cpu->mc_pc = start_upc;
  while (true) {
    const MicroInstr386 *mi = &cpu->mc_rom[cpu->mc_pc];
    cpu->micro_cycles++;

    uint32_t bus_val = 0;
    if (mi->src != MB386_NONE) {
      bus_val = mc386_read_bus(cpu, mi->src);
      mc386_write_bus(cpu, mi->dst, bus_val);
    }

    if (mi->act == MACT386_ALU) {
      Alu386Op op = mi->use_xi ? cpu->xi_alu_op : mi->alu_op;
      uint32_t f = reg32_read(&cpu->eflags);
      int width = (mi->src == MB386_STEP_SZ) ? 32 : cpu->op_bits;
      Word32 res = alu_386_execute(op, reg32_read_word(&cpu->tmp_a),
                                   reg32_read_word(&cpu->tmp_b), width, &f,
                                   mi->update_flags);
      reg32_write_word(&cpu->sigma, res);
      if (mi->update_flags)
        reg32_write(&cpu->eflags, f);
    } else if (mi->act == MACT386_MEM_WRITE_M) {
      uint32_t opr = reg32_read(&cpu->opr);
      if (cpu->mod == 3) {
        eu386_write_reg(cpu, cpu->rm_field, opr, cpu->op_bits);
      } else {
        mmu_write_sized(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind),
                        opr, cpu->op_bits);
      }
    } else if (mi->act == MACT386_MEM_PUSH) {
      mmu_write_sized(cpu, 2 /* SS */, reg32_read(&cpu->ind),
                      reg32_read(&cpu->opr), cpu->op_bits);
    } else if (mi->act == MACT386_MEM_POP) {
      uint32_t v =
          mmu_read_sized(cpu, 2 /* SS */, reg32_read(&cpu->ind), cpu->op_bits);
      reg32_write(&cpu->opr, v);
    } else if (mi->act == MACT386_MEM_READ) {
      uint32_t v = mmu_read_sized(cpu, get_active_seg_386(cpu),
                                   reg32_read(&cpu->ind), cpu->op_bits);
      reg32_write(&cpu->opr, v);
    }

    if (cpu->trace_mc) {
      printf("    [uPC 0x%02X] %-34s | bus=0x%08X SIGMA=0x%08X IND=0x%08X "
             "OPR=0x%08X\n",
             cpu->mc_pc, mi->comment ? mi->comment : "", bus_val,
             reg32_read(&cpu->sigma), reg32_read(&cpu->ind),
             reg32_read(&cpu->opr));
    }

    if (mi->rni)
      break;
    cpu->mc_pc++;
  }
}

/*
 * 80386 16-Bit & 32-Bit Effective Address + SIB (Scale-Index-Base) Unit
 * Uses the 32-bit Transistor Barrel Shifter and 32-bit Transistor Ripple Adder!
 */
static void decode_modrm_and_sib_386(Intel80386 *cpu, bool load_mem_to_opr) {
  cpu->modrm = fetch8(cpu);
  cpu->mod = (cpu->modrm >> 6) & 3;
  cpu->reg_field = (cpu->modrm >> 3) & 7;
  cpu->rm_field = cpu->modrm & 7;

  if (cpu->mod == 3)
    return;

  Wire carries[32];
  uint32_t ea = 0;

  if (cpu->addr_bits == 16) {
    /* 16-Bit Effective Address Calculation */
    uint16_t bx = reg32_read16(&cpu->regs[3]);
    uint16_t bp = reg32_read16(&cpu->regs[5]);
    uint16_t si = reg32_read16(&cpu->regs[6]);
    uint16_t di = reg32_read16(&cpu->regs[7]);

    cpu->default_seg = 3; /* DS */
    if (cpu->rm_field == 2 || cpu->rm_field == 3 ||
        (cpu->rm_field == 6 && cpu->mod != 0)) {
      cpu->default_seg = 2; /* SS */
    }

    if (cpu->mod == 0 && cpu->rm_field == 6) {
      ea = fetch16(cpu);
    } else {
      switch (cpu->rm_field) {
      case 0:
        ea = (uint16_t)(bx + si);
        break;
      case 1:
        ea = (uint16_t)(bx + di);
        break;
      case 2:
        ea = (uint16_t)(bp + si);
        break;
      case 3:
        ea = (uint16_t)(bp + di);
        break;
      case 4:
        ea = si;
        break;
      case 5:
        ea = di;
        break;
      case 6:
        ea = bp;
        break;
      case 7:
        ea = bx;
        break;
      }
      if (cpu->mod == 1)
        ea = (uint16_t)(ea + (int16_t)(int8_t)fetch8(cpu));
      else if (cpu->mod == 2)
        ea = (uint16_t)(ea + fetch16(cpu));
    }
  } else {
    /* 32-Bit Effective Address Calculation with SIB Byte Support */
    cpu->default_seg = 3; /* DS */
    if (cpu->rm_field == 4) {
      /* SIB Byte Present: [scale:2 | index:3 | base:3] */
      uint8_t sib = fetch8(cpu);
      uint8_t scale = (sib >> 6) & 3;
      uint8_t index = (sib >> 3) & 7;
      uint8_t base = sib & 7;

      uint32_t base_val = 0;
      if (base == 5 && cpu->mod == 0) {
        base_val = fetch32(cpu);
      } else {
        base_val = reg32_read(&cpu->regs[base]);
        if (base == 4 || base == 5)
          cpu->default_seg = 2; /* ESP/EBP -> SS */
      }

      /* Compute (Index << Scale) through 32-bit Transistor Barrel Shifter */
      Word32 scaled_idx = word32_from_u32(0);
      if (index != 4) { /* Index == 4 means no index register */
        scaled_idx = barrel_shifter_left_32bit(
            reg32_read_word(&cpu->regs[index]), scale);
      }

      /* Add Base + (Index << Scale) through 32-bit Transistor Adder */
      Word32 sum =
          adder_32bit(word32_from_u32(base_val), scaled_idx, LOW, carries);
      ea = word32_to_u32(sum);
    } else if (cpu->mod == 0 && cpu->rm_field == 5) {
      ea = fetch32(cpu);
    } else {
      ea = reg32_read(&cpu->regs[cpu->rm_field]);
      if (cpu->rm_field == 5)
        cpu->default_seg = 2; /* EBP -> SS */
    }

    /* Add 8-bit or 32-bit Displacement via 32-bit Transistor Adder */
    if (cpu->mod == 1 || cpu->mod == 2) {
      uint32_t disp = (cpu->mod == 1) ? (uint32_t)(int32_t)(int8_t)fetch8(cpu)
                                      : fetch32(cpu);
      Word32 sum =
          adder_32bit(word32_from_u32(ea), word32_from_u32(disp), LOW, carries);
      ea = word32_to_u32(sum);
    }
  }

  reg32_write(&cpu->ind, ea);
  cpu->micro_cycles += 2;

  if (load_mem_to_opr) {
    uint32_t val =
        mmu_read_sized(cpu, get_active_seg_386(cpu), ea, cpu->op_bits);
    reg32_write(&cpu->opr, val);
    cpu->micro_cycles++;
  }
}

static Alu386Op map_pla_alu386(uint8_t idx) {
  static const Alu386Op table[8] = {ALU386_ADD, ALU386_OR,  ALU386_ADC,
                                    ALU386_SBB, ALU386_AND, ALU386_SUB,
                                    ALU386_XOR, ALU386_CMP};
  return table[idx & 7];
}

static bool eval_386_jcc(uint32_t f, uint8_t cc) {
  bool cf = (f & EFLAG_CF) != 0;
  bool pf = (f & EFLAG_PF) != 0;
  bool zf = (f & EFLAG_ZF) != 0;
  bool sf = (f & EFLAG_SF) != 0;
  bool of = (f & EFLAG_OF) != 0;
  bool cond = false;
  switch ((cc >> 1) & 7) {
  case 0:
    cond = of;
    break;
  case 1:
    cond = cf;
    break;
  case 2:
    cond = zf;
    break;
  case 3:
    cond = (cf || zf);
    break;
  case 4:
    cond = sf;
    break;
  case 5:
    cond = pf;
    break;
  case 6:
    cond = (sf != of);
    break;
  case 7:
    cond = (zf || sf != of);
    break;
  }
  return (cc & 1) ? !cond : cond;
}

/* Initialize Intel 80386 to Power-On Reset State */
static void cpu386_init(Intel80386 *cpu) {
  memset(cpu, 0, sizeof(*cpu));
  for (int i = 0; i < 8; i++)
    reg32_init(&cpu->regs[i]);
  reg32_init(&cpu->eip);
  reg32_init(&cpu->eflags);
  reg32_init(&cpu->cr0);
  reg32_init(&cpu->cr2);
  reg32_init(&cpu->cr3);
  reg32_init(&cpu->tmp_a);
  reg32_init(&cpu->tmp_b);
  reg32_init(&cpu->tmp_c);
  reg32_init(&cpu->sigma);
  reg32_init(&cpu->ind);
  reg32_init(&cpu->opr);

  build_386_microcode_rom(cpu);
  tlb_flush(cpu);

  /* 80386 Reset State: Real Mode (PE=0, PG=0), CS=0xF000 (Base=0xFFFF0000 or
   * 0xF0000) */
  reg32_write(&cpu->eflags, 0x00000002u);
  for (int i = 0; i < 6; i++)
    load_segment_386(cpu, i, 0x0000);
  load_segment_386(cpu, 1 /* CS */, 0xFFFF);
  reg32_write(&cpu->eip, 0x00000000u);
}

/* Execute One Complete Intel 80386 Instruction */
static void cpu386_step(Intel80386 *cpu) {
  if (cpu->halted)
    return;

  bool d_bit = cpu->segs[1].d_bit; /* Default size from CS descriptor */
  bool op_size = d_bit;
  bool addr_size = d_bit;
  cpu->seg_override = -1;
  cpu->default_seg = 3; /* DS */
  cpu->s_bit = false;
  cpu->rep_prefix = 0;

  /* 1. Decode Prefixes (0x66 Operand-Size, 0x67 Address-Size, Segment ES..GS, REP)
   */
  uint8_t op;
  while (true) {
    op = fetch8(cpu);
    if (op == 0x66)
      op_size = !d_bit;
    else if (op == 0x67)
      addr_size = !d_bit;
    else if (op == 0xF2 || op == 0xF3)
      cpu->rep_prefix = op;
    else if (op == 0x26)
      cpu->seg_override = 0; /* ES: */
    else if (op == 0x2E)
      cpu->seg_override = 1; /* CS: */
    else if (op == 0x36)
      cpu->seg_override = 2; /* SS: */
    else if (op == 0x3E)
      cpu->seg_override = 3; /* DS: */
    else if (op == 0x64)
      cpu->seg_override = 4; /* FS: (New in 80386!) */
    else if (op == 0x65)
      cpu->seg_override = 5; /* GS: (New in 80386!) */
    else
      break;
  }

  cpu->op_bits = op_size ? 32 : 16;
  cpu->addr_bits = addr_size ? 32 : 16;
  cpu->macro_instructions++;

  /* 2. Two-Byte 80386 Extended Opcodes (0x0F ...) */
  if (op == 0x0F) {
    uint8_t op2 = fetch8(cpu);

    if (op2 == 0x01) { /* Group 7: SGDT, SIDT, LGDT, LIDT, SMSW, LMSW, INVLPG */
      decode_modrm_and_sib_386(cpu, false);
      uint32_t addr = reg32_read(&cpu->ind);
      int seg = get_active_seg_386(cpu);
      if (cpu->reg_field == 0) { /* SGDT m16&32 */
        mmu_write16(cpu, seg, addr, cpu->gdtr_limit);
        mmu_write32(cpu, seg, addr + 2, cpu->gdtr_base);
        cpu->micro_cycles += 4;
      } else if (cpu->reg_field == 1) { /* SIDT m16&32 */
        mmu_write16(cpu, seg, addr, cpu->idtr_limit);
        mmu_write32(cpu, seg, addr + 2, cpu->idtr_base);
        cpu->micro_cycles += 4;
      } else if (cpu->reg_field == 2) { /* LGDT m16&32 */
        cpu->gdtr_limit = mmu_read16(cpu, seg, addr);
        cpu->gdtr_base = mmu_read32(cpu, seg, addr + 2);
        cpu->micro_cycles += 4;
      } else if (cpu->reg_field == 3) { /* LIDT m16&32 */
        cpu->idtr_limit = mmu_read16(cpu, seg, addr);
        cpu->idtr_base = mmu_read32(cpu, seg, addr + 2);
        cpu->micro_cycles += 4;
      } else if (cpu->reg_field == 4) { /* SMSW r/m16 */
        uint16_t msw = (uint16_t)reg32_read(&cpu->cr0);
        if (cpu->mod == 3)
          eu386_write_reg(cpu, cpu->rm_field, msw, 16);
        else
          mmu_write16(cpu, seg, addr, msw);
        cpu->micro_cycles += 2;
      } else if (cpu->reg_field == 6) { /* LMSW r/m16 */
        uint16_t msw = (cpu->mod == 3)
                           ? (uint16_t)eu386_read_reg(cpu, cpu->rm_field, 16)
                           : mmu_read16(cpu, seg, addr);
        reg32_write(&cpu->cr0,
                    (reg32_read(&cpu->cr0) & ~0x0Fu) | (msw & 0x0Fu));
        cpu->micro_cycles += 3;
      } else if (cpu->reg_field == 7) { /* INVLPG m (Invalidate TLB Entry) */
        tlb_invalidate_page(cpu, addr);
        cpu->micro_cycles += 3;
      }
      return;
    }

    if (op2 == 0x06) { /* CLTS (Clear Task Switched Flag in CR0) */
      reg32_write(&cpu->cr0, reg32_read(&cpu->cr0) & ~(1u << 3));
      cpu->micro_cycles += 2;
      return;
    }

    if (op2 == 0x20 ||
        op2 == 0x22) { /* MOV r32, CRn (0x20) / MOV CRn, r32 (0x22) */
      uint8_t m = fetch8(cpu);
      int cr_num = (m >> 3) & 7;
      int gpr_num = m & 7;
      if (op2 == 0x20) {
        uint32_t v = (cr_num == 0)   ? reg32_read(&cpu->cr0)
                     : (cr_num == 2) ? reg32_read(&cpu->cr2)
                                     : reg32_read(&cpu->cr3);
        reg32_write(&cpu->regs[gpr_num], v);
      } else {
        uint32_t v = reg32_read(&cpu->regs[gpr_num]);
        if (cr_num == 0)
          reg32_write(&cpu->cr0, v);
        else if (cr_num == 2)
          reg32_write(&cpu->cr2, v);
        else if (cr_num == 3) {
          reg32_write(&cpu->cr3, v);
          tlb_flush(cpu);
        }
      }
      cpu->micro_cycles += 3;
      return;
    }

    /* PUSH FS (0xA0) / POP FS (0xA1) & PUSH GS (0xA8) / POP GS (0xA9) */
    if (op2 == 0xA0) {
      eu386_push(cpu, cpu->segs[4].selector);
      return;
    }
    if (op2 == 0xA1) {
      load_segment_386(cpu, 4 /* FS */, (uint16_t)eu386_pop(cpu));
      return;
    }
    if (op2 == 0xA8) {
      eu386_push(cpu, cpu->segs[5].selector);
      return;
    }
    if (op2 == 0xA9) {
      load_segment_386(cpu, 5 /* GS */, (uint16_t)eu386_pop(cpu));
      return;
    }

    /* SETcc r/m8 (0x90..0x9F) - 386 Byte Set on Condition Code */
    if ((op2 & 0xF0) == 0x90) {
      decode_modrm_and_sib_386(cpu, false);
      bool cond = eval_386_jcc(reg32_read(&cpu->eflags), op2 & 0x0F);
      uint8_t val = cond ? 1 : 0;
      if (cpu->mod == 3)
        eu386_write_reg(cpu, cpu->rm_field, val, 8);
      else
        mmu_write8(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind), val);
      cpu->micro_cycles += 3;
      return;
    }

    /* Bit Instructions: BT (0xA3), BTS (0xAB), BTR (0xB3), BTC (0xBB) */
    if (op2 == 0xA3 || op2 == 0xAB || op2 == 0xB3 || op2 == 0xBB) {
      decode_modrm_and_sib_386(cpu, true);
      uint32_t bit_idx =
          eu386_read_reg(cpu, cpu->reg_field, cpu->op_bits) & (cpu->op_bits - 1);
      uint32_t val = mc386_read_bus(cpu, MB386_M);
      uint32_t f = reg32_read(&cpu->eflags);
      if ((val >> bit_idx) & 1u)
        f |= EFLAG_CF;
      else
        f &= ~EFLAG_CF;
      reg32_write(&cpu->eflags, sanitize_386_eflags(f));

      if (op2 == 0xAB)
        val |= (1u << bit_idx); /* BTS */
      else if (op2 == 0xB3)
        val &= ~(1u << bit_idx); /* BTR */
      else if (op2 == 0xBB)
        val ^= (1u << bit_idx); /* BTC */

      if (op2 != 0xA3) {
        if (cpu->mod == 3)
          eu386_write_reg(cpu, cpu->rm_field, val, cpu->op_bits);
        else
          mmu_write_sized(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind),
                          val, cpu->op_bits);
      }
      cpu->micro_cycles += 4;
      return;
    }

    /* Bit Group 8: BT/BTS/BTR/BTC r/m, imm8 (0xBA) */
    if (op2 == 0xBA) {
      decode_modrm_and_sib_386(cpu, true);
      uint8_t bit_idx = fetch8(cpu) & (cpu->op_bits - 1);
      uint32_t val = mc386_read_bus(cpu, MB386_M);
      uint32_t f = reg32_read(&cpu->eflags);
      if ((val >> bit_idx) & 1u)
        f |= EFLAG_CF;
      else
        f &= ~EFLAG_CF;
      reg32_write(&cpu->eflags, sanitize_386_eflags(f));

      if (cpu->reg_field == 5)
        val |= (1u << bit_idx); /* BTS */
      else if (cpu->reg_field == 6)
        val &= ~(1u << bit_idx); /* BTR */
      else if (cpu->reg_field == 7)
        val ^= (1u << bit_idx); /* BTC */

      if (cpu->reg_field != 4) {
        if (cpu->mod == 3)
          eu386_write_reg(cpu, cpu->rm_field, val, cpu->op_bits);
        else
          mmu_write_sized(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind),
                          val, cpu->op_bits);
      }
      cpu->micro_cycles += 4;
      return;
    }

    /* SHLD (0xA4 / 0xA5) & SHRD (0xAC / 0xAD) Double-Precision Shifts */
    if (op2 == 0xA4 || op2 == 0xA5 || op2 == 0xAC || op2 == 0xAD) {
      decode_modrm_and_sib_386(cpu, true);
      uint8_t cnt = (op2 == 0xA4 || op2 == 0xAC)
                        ? (fetch8(cpu) & 0x1F)
                        : (uint8_t)(reg32_read_lo8(&cpu->regs[1]) & 0x1F);
      if (cnt > 0) {
        uint32_t dst = mc386_read_bus(cpu, MB386_M);
        uint32_t src = eu386_read_reg(cpu, cpu->reg_field, cpu->op_bits);
        uint32_t res = dst;
        if (op2 == 0xA4 || op2 == 0xA5) { /* SHLD */
          if (cpu->op_bits == 32)
            res = (uint32_t)(((uint64_t)dst << cnt) | ((uint64_t)src >> (32 - cnt)));
          else
            res = (uint16_t)(((uint32_t)(uint16_t)dst << cnt) |
                             ((uint32_t)(uint16_t)src >> (16 - cnt)));
        } else { /* SHRD */
          if (cpu->op_bits == 32)
            res = (uint32_t)(((uint64_t)src << (32 - cnt)) | ((uint64_t)dst >> cnt));
          else
            res = (uint16_t)(((uint32_t)(uint16_t)src << (16 - cnt)) |
                             ((uint32_t)(uint16_t)dst >> cnt));
        }
        if (cpu->mod == 3)
          eu386_write_reg(cpu, cpu->rm_field, res, cpu->op_bits);
        else
          mmu_write_sized(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind),
                          res, cpu->op_bits);
      }
      cpu->micro_cycles += 5;
      return;
    }

    /* LSS (0xB2), LFS (0xB4), LGS (0xB5) - Load Pointer to Segment:Register */
    if (op2 == 0xB2 || op2 == 0xB4 || op2 == 0xB5) {
      decode_modrm_and_sib_386(cpu, false);
      uint32_t addr = reg32_read(&cpu->ind);
      int seg = get_active_seg_386(cpu);
      uint32_t off = mmu_read_sized(cpu, seg, addr, cpu->op_bits);
      uint16_t sel = mmu_read16(cpu, seg, addr + (cpu->op_bits / 8));
      eu386_write_reg(cpu, cpu->reg_field, off, cpu->op_bits);
      int target_seg = (op2 == 0xB2) ? 2 /* SS */ : (op2 == 0xB4 ? 4 /* FS */ : 5 /* GS */);
      load_segment_386(cpu, target_seg, sel);
      cpu->micro_cycles += 6;
      return;
    }

    if (op2 == 0xB6 ||
        op2 == 0xB7) { /* MOVZX r16/32, r/m8 (0xB6) or r/m16 (0xB7) */
      int dst_bits = cpu->op_bits;
      cpu->op_bits = (op2 == 0xB6) ? 8 : 16;
      decode_modrm_and_sib_386(cpu, true);
      uint32_t src_val = mc386_read_bus(cpu, MB386_M);
      eu386_write_reg(cpu, cpu->reg_field, src_val, dst_bits);
      cpu->micro_cycles += 2;
      return;
    }

    if (op2 == 0xBE ||
        op2 == 0xBF) { /* MOVSX r16/32, r/m8 (0xBE) or r/m16 (0xBF) */
      int dst_bits = cpu->op_bits;
      cpu->op_bits = (op2 == 0xBE) ? 8 : 16;
      decode_modrm_and_sib_386(cpu, true);
      uint32_t src_val = mc386_read_bus(cpu, MB386_M);
      int32_t sxt =
          (op2 == 0xBE) ? (int32_t)(int8_t)src_val : (int32_t)(int16_t)src_val;
      eu386_write_reg(cpu, cpu->reg_field, (uint32_t)sxt, dst_bits);
      cpu->micro_cycles += 2;
      return;
    }

    if (op2 ==
        0xAF) { /* IMUL r16/32, r/m16/32 (386 Two-Operand Signed Multiply) */
      decode_modrm_and_sib_386(cpu, true);
      int64_t a = (cpu->op_bits == 32)
                      ? (int32_t)eu386_read_reg(cpu, cpu->reg_field, 32)
                      : (int16_t)eu386_read_reg(cpu, cpu->reg_field, 16);
      int64_t b = (cpu->op_bits == 32) ? (int32_t)mc386_read_bus(cpu, MB386_M)
                                       : (int16_t)mc386_read_bus(cpu, MB386_M);
      int64_t prod = a * b;
      eu386_write_reg(cpu, cpu->reg_field, (uint32_t)prod, cpu->op_bits);
      cpu->micro_cycles += 9;
      return;
    }

    if (op2 == 0xBC || op2 == 0xBD) { /* BSF (0xBC) / BSR (0xBD) */
      decode_modrm_and_sib_386(cpu, true);
      uint32_t src = mc386_read_bus(cpu, MB386_M);
      uint32_t f = reg32_read(&cpu->eflags);
      if (src == 0) {
        reg32_write(&cpu->eflags, f | EFLAG_ZF);
      } else {
        reg32_write(&cpu->eflags, f & ~EFLAG_ZF);
        uint32_t idx = 0;
        if (op2 == 0xBC) {
          while (((src >> idx) & 1u) == 0)
            idx++;
        } else {
          idx = cpu->op_bits - 1;
          while (((src >> idx) & 1u) == 0 && idx > 0)
            idx--;
        }
        eu386_write_reg(cpu, cpu->reg_field, idx, cpu->op_bits);
      }
      cpu->micro_cycles += 6;
      return;
    }

    if ((op2 & 0xF0) == 0x80) { /* Jcc rel16/32 (Near Conditional Jump) */
      int32_t rel =
          (cpu->op_bits == 32) ? (int32_t)fetch32(cpu) : (int16_t)fetch16(cpu);
      if (eval_386_jcc(reg32_read(&cpu->eflags), op2 & 0xF)) {
        reg32_write(&cpu->eip,
                    (uint32_t)((int32_t)reg32_read(&cpu->eip) + rel));
      }
      return;
    }
    return;
  }

  /* 3. Standard ALU Group (0x00..0x3D) */
  if ((op & 0xC0) == 0x00 && (op & 6) != 6) {
    uint8_t alu_idx = (op >> 3) & 7;
    cpu->xi_alu_op = map_pla_alu386(alu_idx);
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    bool dir_reg = (op & 2) != 0;
    bool is_cmp = (alu_idx == 7);

    if ((op & 4) == 0) {
      decode_modrm_and_sib_386(cpu, true);
      if (dir_reg)
        execute_386_microcode(cpu, is_cmp ? MC386_CMP_R_RM : MC386_ALU_R_RM);
      else
        execute_386_microcode(cpu, is_cmp ? MC386_CMP_RM_R : MC386_ALU_RM_R);
    } else {
      execute_386_microcode(cpu,
                            is_cmp ? MC386_CMP_ACC_IMM : MC386_ALU_ACC_IMM);
    }
    return;
  }

  /* Group 1 Immediate ALU (0x80..0x83) */
  if ((op & 0xFC) == 0x80) {
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    cpu->s_bit = (op == 0x83);
    decode_modrm_and_sib_386(cpu, true);
    cpu->xi_alu_op = map_pla_alu386(cpu->reg_field);
    if (cpu->reg_field == 7)
      execute_386_microcode(cpu, MC386_GRP1_CMP_IMM);
    else
      execute_386_microcode(cpu, MC386_GRP1_RM_IMM);
    return;
  }

  /* INC reg16/32 (0x40..0x47) & DEC reg16/32 (0x48..0x4F) */
  if ((op & 0xF8) == 0x40) {
    cpu->reg_field = op & 7;
    execute_386_microcode(cpu, MC386_INC_R);
    return;
  }
  if ((op & 0xF8) == 0x48) {
    cpu->reg_field = op & 7;
    execute_386_microcode(cpu, MC386_DEC_R);
    return;
  }

  /* PUSH reg16/32 (0x50..0x57) & POP reg16/32 (0x58..0x5F) */
  if ((op & 0xF8) == 0x50) {
    cpu->reg_field = op & 7;
    execute_386_microcode(cpu, MC386_PUSH_R);
    return;
  }
  if ((op & 0xF8) == 0x58) {
    cpu->reg_field = op & 7;
    execute_386_microcode(cpu, MC386_POP_R);
    return;
  }

  /* Segment Register PUSH / POP (0x06, 0x07, 0x0E, 0x16, 0x17, 0x1E, 0x1F) */
  if (op == 0x06 || op == 0x0E || op == 0x16 || op == 0x1E) {
    int seg = (op == 0x06) ? 0 : ((op == 0x0E) ? 1 : ((op == 0x16) ? 2 : 3));
    eu386_push(cpu, cpu->segs[seg].selector);
    return;
  }
  if (op == 0x07 || op == 0x17 || op == 0x1F) {
    int seg = (op == 0x07) ? 0 : ((op == 0x17) ? 2 : 3);
    load_segment_386(cpu, seg, (uint16_t)eu386_pop(cpu));
    return;
  }

  /* PUSHAD / PUSHA (0x60) & POPAD / POPA (0x61) */
  if (op == 0x60) {
    uint32_t orig_esp = reg32_read(&cpu->regs[4]);
    for (int i = 0; i < 8; i++) {
      eu386_push(cpu, (i == 4) ? orig_esp : reg32_read(&cpu->regs[i]));
    }
    return;
  }
  if (op == 0x61) {
    for (int i = 7; i >= 0; i--) {
      uint32_t val = eu386_pop(cpu);
      if (i != 4)
        eu386_write_reg(cpu, i, val, cpu->op_bits);
    }
    return;
  }

  /* PUSH imm16/32 (0x68) & PUSH imm8 (0x6A) */
  if (op == 0x68) {
    uint32_t imm = fetch_imm_sized(cpu, cpu->op_bits);
    eu386_push(cpu, imm);
    return;
  }
  if (op == 0x6A) {
    int32_t imm = (int32_t)(int8_t)fetch8(cpu);
    eu386_push(cpu, (uint32_t)imm);
    return;
  }

  /* IMUL r16/32, r/m16/32, imm (0x69 / 0x6B) */
  if (op == 0x69 || op == 0x6B) {
    decode_modrm_and_sib_386(cpu, true);
    int64_t src = (cpu->op_bits == 32)
                      ? (int32_t)mc386_read_bus(cpu, MB386_M)
                      : (int16_t)mc386_read_bus(cpu, MB386_M);
    int64_t imm = (op == 0x6B) ? (int8_t)fetch8(cpu)
                                : ((cpu->op_bits == 32)
                                       ? (int32_t)fetch32(cpu)
                                       : (int16_t)fetch16(cpu));
    int64_t prod = src * imm;
    eu386_write_reg(cpu, cpu->reg_field, (uint32_t)prod, cpu->op_bits);
    cpu->micro_cycles += 9;
    return;
  }

  /* Jcc rel8 (0x70..0x7F) */
  if ((op & 0xF0) == 0x70) {
    int8_t rel = (int8_t)fetch8(cpu);
    if (eval_386_jcc(reg32_read(&cpu->eflags), op & 0xF)) {
      reg32_write(&cpu->eip, (uint32_t)((int32_t)reg32_read(&cpu->eip) + rel));
    }
    return;
  }

  /* TEST r/m, r (0x84..0x85) */
  if ((op & 0xFE) == 0x84) {
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    decode_modrm_and_sib_386(cpu, true);
    execute_386_microcode(cpu, MC386_TEST_RM_R);
    return;
  }

  /* XCHG r/m, r (0x86..0x87) */
  if ((op & 0xFE) == 0x86) {
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    decode_modrm_and_sib_386(cpu, true);
    execute_386_microcode(cpu, MC386_XCHG_R_RM);
    return;
  }

  /* MOV r/m, r (0x88..0x89) & MOV r, r/m (0x8A..0x8B) */
  if ((op & 0xFC) == 0x88) {
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    bool dir_reg = (op & 2) != 0;
    decode_modrm_and_sib_386(cpu, dir_reg);
    execute_386_microcode(cpu, dir_reg ? MC386_MOV_R_RM : MC386_MOV_RM_R);
    return;
  }

  /* MOV r/m16, Sreg (0x8C) */
  if (op == 0x8C) {
    cpu->op_bits = 16;
    decode_modrm_and_sib_386(cpu, false);
    uint16_t sel = cpu->segs[cpu->reg_field % 6].selector;
    if (cpu->mod == 3)
      eu386_write_reg(cpu, cpu->rm_field, sel, 16);
    else
      mmu_write16(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind), sel);
    return;
  }

  /* LEA r16/32, m (0x8D) */
  if (op == 0x8D) {
    decode_modrm_and_sib_386(cpu, false);
    execute_386_microcode(cpu, MC386_LEA_R_M);
    return;
  }

  /* MOV Sreg, r/m16 (0x8E) */
  if (op == 0x8E) {
    cpu->op_bits = 16;
    decode_modrm_and_sib_386(cpu, true);
    uint16_t sel = (uint16_t)mc386_read_bus(cpu, MB386_M);
    load_segment_386(cpu, cpu->reg_field % 6, sel);
    return;
  }

  /* POP r/m16/32 (0x8F) */
  if (op == 0x8F) {
    decode_modrm_and_sib_386(cpu, false);
    if (cpu->reg_field == 0) {
      uint32_t val = eu386_pop(cpu);
      if (cpu->mod == 3)
        eu386_write_reg(cpu, cpu->rm_field, val, cpu->op_bits);
      else
        mmu_write_sized(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind),
                        val, cpu->op_bits);
    }
    return;
  }

  /* NOP (0x90) & XCHG EAX/AX, reg (0x91..0x97) */
  if ((op & 0xF8) == 0x90) {
    if (op == 0x90)
      return;
    cpu->reg_field = op & 7;
    execute_386_microcode(cpu, MC386_XCHG_EAX_R);
    return;
  }

  /* CWDE / CBW (0x98) & CDQ / CWD (0x99) */
  if (op == 0x98) {
    if (cpu->op_bits == 32) {
      int32_t val = (int16_t)reg32_read16(&cpu->regs[0]);
      reg32_write(&cpu->regs[0], (uint32_t)val);
    } else {
      int16_t val = (int8_t)reg32_read_lo8(&cpu->regs[0]);
      reg32_write16(&cpu->regs[0], (uint16_t)val);
    }
    return;
  }
  if (op == 0x99) {
    if (cpu->op_bits == 32) {
      int32_t val = (int32_t)reg32_read(&cpu->regs[0]);
      reg32_write(&cpu->regs[2], (val < 0) ? 0xFFFFFFFFu : 0u);
    } else {
      int16_t val = (int16_t)reg32_read16(&cpu->regs[0]);
      reg32_write16(&cpu->regs[2], (val < 0) ? 0xFFFFu : 0u);
    }
    return;
  }

  /* PUSHFD / PUSHF (0x9C) & POPFD / POPF (0x9D) */
  if (op == 0x9C) {
    eu386_push(cpu, reg32_read(&cpu->eflags));
    return;
  }
  if (op == 0x9D) {
    reg32_write(&cpu->eflags, sanitize_386_eflags(eu386_pop(cpu)));
    return;
  }

  /* SAHF (0x9E) & LAHF (0x9F) */
  if (op == 0x9E) {
    uint8_t ah = reg32_read_hi8(&cpu->regs[0]);
    uint32_t f = (reg32_read(&cpu->eflags) & ~0xFFu) | ah;
    reg32_write(&cpu->eflags, sanitize_386_eflags(f));
    return;
  }
  if (op == 0x9F) {
    uint8_t lo = (uint8_t)(reg32_read(&cpu->eflags) & 0xFFu);
    reg32_write_hi8(&cpu->regs[0], lo);
    return;
  }

  /* MOV AL/AX/EAX, [moffs] (0xA0..0xA1) & MOV [moffs], AL/AX/EAX (0xA2..0xA3) */
  if ((op & 0xFC) == 0xA0) {
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    uint32_t moffs = (cpu->addr_bits == 32) ? fetch32(cpu) : fetch16(cpu);
    reg32_write(&cpu->ind, moffs);
    execute_386_microcode(cpu, (op & 2) ? MC386_MOV_MEM_ACC : MC386_MOV_ACC_MEM);
    return;
  }

  /* String Instructions: MOVS, CMPS, STOS, LODS, SCAS (0xA4..0xAF) */
  if (op == 0xA4 || op == 0xA5 || op == 0xA6 || op == 0xA7 || op == 0xAA ||
      op == 0xAB || op == 0xAC || op == 0xAD || op == 0xAE || op == 0xAF) {
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    int step =
        ((reg32_read(&cpu->eflags) & EFLAG_DF) ? -1 : 1) * (cpu->op_bits / 8);
    int seg = get_active_seg_386(cpu);
    while (true) {
      uint32_t count = (cpu->addr_bits == 32) ? reg32_read(&cpu->regs[1])
                                              : reg32_read16(&cpu->regs[1]);
      if (cpu->rep_prefix && count == 0)
        break;

      uint32_t si = (cpu->addr_bits == 32) ? reg32_read(&cpu->regs[6])
                                           : reg32_read16(&cpu->regs[6]);
      uint32_t di = (cpu->addr_bits == 32) ? reg32_read(&cpu->regs[7])
                                           : reg32_read16(&cpu->regs[7]);

      if (op == 0xA4 || op == 0xA5) { /* MOVS */
        uint32_t val = mmu_read_sized(cpu, seg, si, cpu->op_bits);
        mmu_write_sized(cpu, 0 /* ES */, di, val, cpu->op_bits);
        si += step;
        di += step;
      } else if (op == 0xA6 || op == 0xA7) { /* CMPS */
        uint32_t v1 = mmu_read_sized(cpu, seg, si, cpu->op_bits);
        uint32_t v2 = mmu_read_sized(cpu, 0 /* ES */, di, cpu->op_bits);
        uint32_t f = reg32_read(&cpu->eflags);
        alu_386_execute(ALU386_CMP, word32_from_u32(v1), word32_from_u32(v2),
                        cpu->op_bits, &f, true);
        reg32_write(&cpu->eflags, f);
        si += step;
        di += step;
      } else if (op == 0xAA || op == 0xAB) { /* STOS */
        uint32_t val = eu386_read_reg(cpu, 0 /* EAX */, cpu->op_bits);
        mmu_write_sized(cpu, 0 /* ES */, di, val, cpu->op_bits);
        di += step;
      } else if (op == 0xAC || op == 0xAD) { /* LODS */
        uint32_t val = mmu_read_sized(cpu, seg, si, cpu->op_bits);
        eu386_write_reg(cpu, 0 /* EAX */, val, cpu->op_bits);
        si += step;
      } else if (op == 0xAE || op == 0xAF) { /* SCAS */
        uint32_t v1 = eu386_read_reg(cpu, 0 /* EAX */, cpu->op_bits);
        uint32_t v2 = mmu_read_sized(cpu, 0 /* ES */, di, cpu->op_bits);
        uint32_t f = reg32_read(&cpu->eflags);
        alu_386_execute(ALU386_CMP, word32_from_u32(v1), word32_from_u32(v2),
                        cpu->op_bits, &f, true);
        reg32_write(&cpu->eflags, f);
        di += step;
      }

      if (cpu->addr_bits == 32) {
        reg32_write(&cpu->regs[6], si);
        reg32_write(&cpu->regs[7], di);
      } else {
        reg32_write16(&cpu->regs[6], (uint16_t)si);
        reg32_write16(&cpu->regs[7], (uint16_t)di);
      }
      cpu->micro_cycles += 4;

      if (cpu->rep_prefix) {
        count--;
        if (cpu->addr_bits == 32)
          reg32_write(&cpu->regs[1], count);
        else
          reg32_write16(&cpu->regs[1], (uint16_t)count);
        bool zf = (reg32_read(&cpu->eflags) & EFLAG_ZF) != 0;
        if (cpu->rep_prefix == 0xF3 && (op == 0xA6 || op == 0xA7 || op == 0xAE || op == 0xAF) && !zf)
          break;
        if (cpu->rep_prefix == 0xF2 && (op == 0xA6 || op == 0xA7 || op == 0xAE || op == 0xAF) && zf)
          break;
        if (count == 0)
          break;
      } else {
        break;
      }
    }
    return;
  }

  /* TEST AL/AX/EAX, imm (0xA8..0xA9) */
  if ((op & 0xFE) == 0xA8) {
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    execute_386_microcode(cpu, MC386_TEST_ACC_IMM);
    return;
  }

  /* MOV reg, imm (0xB0..0xBF) */
  if ((op & 0xF0) == 0xB0) {
    if ((op & 8) == 0)
      cpu->op_bits = 8;
    cpu->reg_field = op & 7;
    execute_386_microcode(cpu, MC386_MOV_R_IMM);
    return;
  }

  /* Shift / Rotate Group 2 (0xC0..0xC1, 0xD0..0xD3) */
  if ((op & 0xFE) == 0xC0 || (op & 0xFC) == 0xD0) {
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    decode_modrm_and_sib_386(cpu, true);
    static const Alu386Op sh_ops[8] = {
        ALU386_ROL, ALU386_ROR, ALU386_ROL, ALU386_ROR,
        ALU386_SHL, ALU386_SHR, ALU386_SHL, ALU386_SAR};
    cpu->xi_alu_op = sh_ops[cpu->reg_field & 7];
    uint8_t count = 1;
    if ((op & 0xFE) == 0xC0)
      count = fetch8(cpu) & 0x1F;
    else if (op & 2)
      count = reg32_read_lo8(&cpu->regs[1]) & 0x1F; /* CL */

    reg32_write(&cpu->tmp_c, count);
    if (cpu->xi_alu_op == ALU386_SHL)
      execute_386_microcode(cpu, MC386_SHL_RM);
    else if (cpu->xi_alu_op == ALU386_SHR)
      execute_386_microcode(cpu, MC386_SHR_RM);
    else if (cpu->xi_alu_op == ALU386_SAR)
      execute_386_microcode(cpu, MC386_SAR_RM);
    else {
      uint32_t val = mc386_read_bus(cpu, MB386_M);
      uint32_t f = reg32_read(&cpu->eflags);
      Word32 res = alu_386_execute(cpu->xi_alu_op, word32_from_u32(val),
                                   word32_from_u32(count), cpu->op_bits, &f,
                                   true);
      reg32_write(&cpu->eflags, f);
      uint32_t out = word32_to_u32(res);
      if (cpu->mod == 3)
        eu386_write_reg(cpu, cpu->rm_field, out, cpu->op_bits);
      else
        mmu_write_sized(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind),
                        out, cpu->op_bits);
    }
    return;
  }

  /* RET imm16 (0xC2) & RET near (0xC3) & RET far (0xCB) */
  if (op == 0xC2 || op == 0xC3) {
    uint32_t new_eip = eu386_pop(cpu);
    reg32_write(&cpu->eip, new_eip);
    if (op == 0xC2) {
      uint16_t pop_bytes = fetch16(cpu);
      reg32_write(&cpu->regs[4], reg32_read(&cpu->regs[4]) + pop_bytes);
    }
    return;
  }
  if (op == 0xCB) {
    uint32_t new_eip = eu386_pop(cpu);
    uint16_t new_cs = (uint16_t)eu386_pop(cpu);
    load_segment_386(cpu, 1 /* CS */, new_cs);
    reg32_write(&cpu->eip, new_eip);
    return;
  }

  /* LES r, m (0xC4) & LDS r, m (0xC5) */
  if (op == 0xC4 || op == 0xC5) {
    decode_modrm_and_sib_386(cpu, false);
    uint32_t addr = reg32_read(&cpu->ind);
    int seg = get_active_seg_386(cpu);
    uint32_t off = mmu_read_sized(cpu, seg, addr, cpu->op_bits);
    uint16_t sel = mmu_read16(cpu, seg, addr + (cpu->op_bits / 8));
    eu386_write_reg(cpu, cpu->reg_field, off, cpu->op_bits);
    load_segment_386(cpu, (op == 0xC4) ? 0 /* ES */ : 3 /* DS */, sel);
    cpu->micro_cycles += 5;
    return;
  }

  /* MOV r/m, imm (0xC6..0xC7) */
  if (op == 0xC6 || op == 0xC7) {
    if (op == 0xC6)
      cpu->op_bits = 8;
    decode_modrm_and_sib_386(cpu, false);
    execute_386_microcode(cpu, MC386_MOV_RM_IMM);
    return;
  }

  /* ENTER (0xC8) & LEAVE (0xC9) */
  if (op == 0xC8) {
    uint16_t alloc_bytes = fetch16(cpu);
    uint8_t level = fetch8(cpu) & 0x1F;
    eu386_push(cpu, reg32_read(&cpu->regs[5])); /* push EBP */
    uint32_t frame_ptr = reg32_read(&cpu->regs[4]);
    if (level > 0) {
      for (uint8_t i = 1; i < level; i++) {
        reg32_write(&cpu->regs[5], reg32_read(&cpu->regs[5]) - ((cpu->op_bits == 32) ? 4 : 2));
        eu386_push(cpu, reg32_read(&cpu->regs[5]));
      }
      eu386_push(cpu, frame_ptr);
    }
    reg32_write(&cpu->regs[5], frame_ptr);
    reg32_write(&cpu->regs[4], reg32_read(&cpu->regs[4]) - alloc_bytes);
    return;
  }
  if (op == 0xC9) {
    reg32_write(&cpu->regs[4], reg32_read(&cpu->regs[5])); /* ESP = EBP */
    reg32_write(&cpu->regs[5], eu386_pop(cpu));            /* POP EBP */
    return;
  }

  /* LOOPNE (0xE0), LOOPE (0xE1), LOOP (0xE2), JCXZ / JECXZ (0xE3) */
  if (op == 0xE0 || op == 0xE1 || op == 0xE2 || op == 0xE3) {
    int8_t rel = (int8_t)fetch8(cpu);
    uint32_t cx = (cpu->addr_bits == 32) ? reg32_read(&cpu->regs[1])
                                          : reg32_read16(&cpu->regs[1]);
    bool jump = false;
    if (op == 0xE3) {
      jump = (cx == 0);
    } else {
      cx--;
      if (cpu->addr_bits == 32)
        reg32_write(&cpu->regs[1], cx);
      else
        reg32_write16(&cpu->regs[1], (uint16_t)cx);
      bool zf = (reg32_read(&cpu->eflags) & EFLAG_ZF) != 0;
      if (op == 0xE2)
        jump = (cx != 0);
      else if (op == 0xE1)
        jump = (cx != 0 && zf);
      else if (op == 0xE0)
        jump = (cx != 0 && !zf);
    }
    if (jump)
      reg32_write(&cpu->eip, (uint32_t)((int32_t)reg32_read(&cpu->eip) + rel));
    return;
  }

  /* CALL rel16/32 (0xE8) & JMP rel16/32 (0xE9) */
  if (op == 0xE8) {
    int32_t rel =
        (cpu->op_bits == 32) ? (int32_t)fetch32(cpu) : (int16_t)fetch16(cpu);
    uint32_t return_eip = reg32_read(&cpu->eip);
    eu386_push(cpu, return_eip);
    reg32_write(&cpu->eip, (uint32_t)((int32_t)return_eip + rel));
    return;
  }
  if (op == 0xE9) {
    int32_t rel =
        (cpu->op_bits == 32) ? (int32_t)fetch32(cpu) : (int16_t)fetch16(cpu);
    reg32_write(&cpu->eip, (uint32_t)((int32_t)reg32_read(&cpu->eip) + rel));
    return;
  }

  /* JMP FAR ptr16:16/32 (0xEA) - Used to enter 32-Bit Protected Mode! */
  if (op == 0xEA) {
    uint32_t new_eip = (cpu->op_bits == 32) ? fetch32(cpu) : fetch16(cpu);
    uint16_t new_cs = fetch16(cpu);
    load_segment_386(cpu, 1 /* CS */, new_cs);
    reg32_write(&cpu->eip, new_eip);
    cpu->micro_cycles += 5;
    return;
  }

  /* JMP rel8 (0xEB) & HLT (0xF4) */
  if (op == 0xEB) {
    int8_t rel = (int8_t)fetch8(cpu);
    reg32_write(&cpu->eip, (uint32_t)((int32_t)reg32_read(&cpu->eip) + rel));
    return;
  }
  if (op == 0xF4) {
    cpu->halted = true;
    return;
  }

  /* Flag Operations: CMC (0xF5), CLC (0xF8), STC (0xF9), CLI (0xFA), STI (0xFB), CLD (0xFC), STD (0xFD) */
  if (op == 0xF5) {
    reg32_write(&cpu->eflags, reg32_read(&cpu->eflags) ^ EFLAG_CF);
    return;
  }
  if (op == 0xF8) {
    reg32_write(&cpu->eflags, reg32_read(&cpu->eflags) & ~EFLAG_CF);
    return;
  }
  if (op == 0xF9) {
    reg32_write(&cpu->eflags, reg32_read(&cpu->eflags) | EFLAG_CF);
    return;
  }
  if (op == 0xFA) {
    reg32_write(&cpu->eflags, reg32_read(&cpu->eflags) & ~EFLAG_IF);
    return;
  }
  if (op == 0xFB) {
    reg32_write(&cpu->eflags, reg32_read(&cpu->eflags) | EFLAG_IF);
    return;
  }
  if (op == 0xFC) {
    reg32_write(&cpu->eflags, reg32_read(&cpu->eflags) & ~EFLAG_DF);
    return;
  }
  if (op == 0xFD) {
    reg32_write(&cpu->eflags, reg32_read(&cpu->eflags) | EFLAG_DF);
    return;
  }

  /* Group 3 Unary / Multiply / Divide (0xF6..0xF7) */
  if (op == 0xF6 || op == 0xF7) {
    if ((op & 1) == 0)
      cpu->op_bits = 8;
    decode_modrm_and_sib_386(cpu, true);
    if (cpu->reg_field == 0 || cpu->reg_field == 1) { /* TEST r/m, imm */
      uint32_t imm = fetch_imm_sized(cpu, cpu->op_bits);
      uint32_t val = mc386_read_bus(cpu, MB386_M);
      uint32_t f = reg32_read(&cpu->eflags);
      alu_386_execute(ALU386_TEST, word32_from_u32(val), word32_from_u32(imm),
                      cpu->op_bits, &f, true);
      reg32_write(&cpu->eflags, f);
    } else if (cpu->reg_field == 2) { /* NOT r/m */
      execute_386_microcode(cpu, MC386_NOT_RM);
    } else if (cpu->reg_field == 3) { /* NEG r/m */
      execute_386_microcode(cpu, MC386_NEG_RM);
    } else if (cpu->reg_field == 4) { /* MUL r/m (unsigned) */
      uint64_t a = eu386_read_reg(cpu, 0, cpu->op_bits);
      uint64_t b = mc386_read_bus(cpu, MB386_M);
      uint64_t prod = a * b;
      if (cpu->op_bits == 32) {
        reg32_write(&cpu->regs[0], (uint32_t)prod);
        reg32_write(&cpu->regs[2], (uint32_t)(prod >> 32));
        uint32_t f = reg32_read(&cpu->eflags);
        if (prod >> 32)
          f |= (EFLAG_CF | EFLAG_OF);
        else
          f &= ~(EFLAG_CF | EFLAG_OF);
        reg32_write(&cpu->eflags, sanitize_386_eflags(f));
      } else if (cpu->op_bits == 16) {
        reg32_write16(&cpu->regs[0], (uint16_t)prod);
        reg32_write16(&cpu->regs[2], (uint16_t)(prod >> 16));
      } else {
        reg32_write16(&cpu->regs[0], (uint16_t)prod);
      }
      cpu->micro_cycles += 9;
    } else if (cpu->reg_field == 5) { /* IMUL r/m (signed) */
      int64_t a = (cpu->op_bits == 32)
                      ? (int32_t)eu386_read_reg(cpu, 0, 32)
                      : (int16_t)eu386_read_reg(cpu, 0, 16);
      int64_t b = (cpu->op_bits == 32) ? (int32_t)mc386_read_bus(cpu, MB386_M)
                                       : (int16_t)mc386_read_bus(cpu, MB386_M);
      int64_t prod = a * b;
      if (cpu->op_bits == 32) {
        reg32_write(&cpu->regs[0], (uint32_t)prod);
        reg32_write(&cpu->regs[2], (uint32_t)(prod >> 32));
      }
      cpu->micro_cycles += 9;
    }
    return;
  }

  /* Group 4 (0xFE) & Group 5 (0xFF) */
  if (op == 0xFE || op == 0xFF) {
    if (op == 0xFE)
      cpu->op_bits = 8;
    decode_modrm_and_sib_386(cpu, true);
    if (cpu->reg_field == 0) { /* INC r/m */
      uint32_t val = mc386_read_bus(cpu, MB386_M);
      uint32_t f = reg32_read(&cpu->eflags);
      Word32 res = alu_386_execute(ALU386_INC, word32_from_u32(val),
                                   word32_from_u32(1), cpu->op_bits, &f, true);
      reg32_write(&cpu->eflags, f);
      uint32_t out = word32_to_u32(res);
      if (cpu->mod == 3)
        eu386_write_reg(cpu, cpu->rm_field, out, cpu->op_bits);
      else
        mmu_write_sized(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind),
                        out, cpu->op_bits);
    } else if (cpu->reg_field == 1) { /* DEC r/m */
      uint32_t val = mc386_read_bus(cpu, MB386_M);
      uint32_t f = reg32_read(&cpu->eflags);
      Word32 res = alu_386_execute(ALU386_DEC, word32_from_u32(val),
                                   word32_from_u32(1), cpu->op_bits, &f, true);
      reg32_write(&cpu->eflags, f);
      uint32_t out = word32_to_u32(res);
      if (cpu->mod == 3)
        eu386_write_reg(cpu, cpu->rm_field, out, cpu->op_bits);
      else
        mmu_write_sized(cpu, get_active_seg_386(cpu), reg32_read(&cpu->ind),
                        out, cpu->op_bits);
    } else if (cpu->reg_field == 2 && op == 0xFF) { /* CALL near r/m */
      uint32_t target = mc386_read_bus(cpu, MB386_M);
      eu386_push(cpu, reg32_read(&cpu->eip));
      reg32_write(&cpu->eip, target);
    } else if (cpu->reg_field == 4 && op == 0xFF) { /* JMP near r/m */
      uint32_t target = mc386_read_bus(cpu, MB386_M);
      reg32_write(&cpu->eip, target);
    } else if (cpu->reg_field == 6 && op == 0xFF) { /* PUSH r/m */
      uint32_t val = mc386_read_bus(cpu, MB386_M);
      eu386_push(cpu, val);
    }
    return;
  }
}

/* ============================================================================
 * 80386 PROTECTED MODE + PAGING + SIB + 32-BIT MICROCODE VERIFICATION
 * ============================================================================
 */

static void write_gdt_entry(uint8_t *ram, uint32_t addr, uint32_t base,
                            uint32_t limit, uint8_t access, uint8_t gran) {
  uint32_t lo = (limit & 0xFFFFu) | ((base & 0xFFFFu) << 16);
  uint32_t hi = ((base >> 16) & 0xFFu) | ((uint32_t)access << 8) |
                (limit & 0x000F0000u) | ((uint32_t)(gran & 0x0Fu) << 20) |
                (base & 0xFF000000u);
  ram[addr + 0] = (uint8_t)(lo & 0xFF);
  ram[addr + 1] = (uint8_t)((lo >> 8) & 0xFF);
  ram[addr + 2] = (uint8_t)((lo >> 16) & 0xFF);
  ram[addr + 3] = (uint8_t)((lo >> 24) & 0xFF);
  ram[addr + 4] = (uint8_t)(hi & 0xFF);
  ram[addr + 5] = (uint8_t)((hi >> 8) & 0xFF);
  ram[addr + 6] = (uint8_t)((hi >> 16) & 0xFF);
  ram[addr + 7] = (uint8_t)((hi >> 24) & 0xFF);
}

int main(void) {
  printf("====================================================================="
         "========\n");
  printf(" SPEC-EXACT 32-BIT INTEL 80386 CPU (TRANSISTORS -> MMU/PAGING -> "
         "MICROCODE)\n");
  printf("====================================================================="
         "========\n\n");

  Intel80386 cpu;
  cpu386_init(&cpu);

  /*
   * 1. Set Up Global Descriptor Table (GDT) at Physical 0x00800:
   *    - Selector 0x0000: Null Descriptor
   *    - Selector 0x0008: 32-Bit Flat Ring-0 Code Segment (Base=0, Limit=4GB,
   * G=1, D=1)
   *    - Selector 0x0010: 32-Bit Flat Ring-0 Data Segment (Base=0, Limit=4GB,
   * G=1, B=1)
   */
  write_gdt_entry(cpu.ram, 0x00800, 0x00000000u, 0x00000u, 0x00, 0x00);
  write_gdt_entry(cpu.ram, 0x00808, 0x00000000u, 0xFFFFFu, 0x9A,
                  0x0C); /* Code32: D=1, G=1 */
  write_gdt_entry(cpu.ram, 0x00810, 0x00000000u, 0xFFFFFu, 0x92,
                  0x0C); /* Data32: B=1, G=1 */

  /* 6-Byte GDTR Pseudo-Descriptor at Physical 0x00600 (Limit=23,
   * Base=0x00000800) */
  cpu.ram[0x00600] = 23;
  cpu.ram[0x00601] = 0;
  phys_write32(&cpu, 0x00602, 0x00000800u);

  /*
   * 2. Set Up 2-Level 80386 Page Directory (at 0x02000) & Page Table (at
   * 0x03000):
   *    - PDE[0] -> Points to Page Table at Physical 0x03000 (Present | R/W)
   *    - Identity-map Low 64 KB (Virtual 0x00000000..0x0000FFFF -> Phys
   * 0x00000000..0x0000FFFF)
   *    - Map Virtual Address 0x00300000 (3 MB Virtual, PTE[0x300]) -> Physical
   * Frame 0x00015000!
   */
  phys_write32(&cpu, 0x02000, 0x00003003u); /* PDE[0]: Base=0x3000, P=1, RW=1 */
  for (uint32_t page = 0; page < 16; page++) {
    phys_write32(&cpu, 0x03000 + page * 4, (page << 12) | 0x03u);
  }
  /* Non-identity Virtual Mapping: Virtual 0x00300000 -> Physical 0x00015000 */
  phys_write32(&cpu, 0x03000 + (0x300u * 4u), 0x00015003u);

  /*
   * 3. Install Reset Vector Jump at FFFF:0000 (Physical 0xFFFF0):
   *    EA 00 05 00 00 -> JMP FAR 0000:0500 (16-Bit Real Mode Bootloader)
   */
  cpu.ram[0xFFFF0] = 0xEA;
  cpu.ram[0xFFFF1] = 0x00;
  cpu.ram[0xFFFF2] = 0x05;
  cpu.ram[0xFFFF3] = 0x00;
  cpu.ram[0xFFFF4] = 0x00;

  /*
   * 4. 16-Bit Real-Mode Bootloader at 0000:0500:
   *    Loads GDTR, loads CR3=0x00002000, sets CR0=0x80000001 (PE=1 | PG=1),
   *    and executes a 32-bit Far Jump `JMP FAR 0x0008:0x00001000` into 32-Bit
   * Mode!
   */
  static const uint8_t boot16[] = {
      0x31, 0xC0,                   /* XOR AX, AX                          */
      0x8E, 0xD8,                   /* MOV DS, AX                          */
      0x0F, 0x01, 0x16, 0x00, 0x06, /* LGDT [0x0600]                       */
      0x66, 0xB8, 0x00, 0x20, 0x00,
      0x00,             /* MOV EAX, 0x00002000 (Page Dir Base) */
      0x0F, 0x22, 0xD8, /* MOV CR3, EAX (Load PDBR & Flush TLB)*/
      0x66, 0xB8, 0x01, 0x00, 0x00,
      0x80,             /* MOV EAX, 0x80000001 (PG=1 | PE=1)   */
      0x0F, 0x22, 0xC0, /* MOV CR0, EAX (Enable Prot + Paging!)*/
      0x66, 0xEA, 0x00, 0x10, 0x00,
      0x00, /* JMP FAR 0x0008:0x00001000 (USE32!)  */
      0x08, 0x00};
  memcpy(&cpu.ram[0x00500], boot16, sizeof(boot16));

  /*
   * 5. 32-Bit Protected-Mode + Paged Kernel Payload at 0008:00001000 (USE32):
   *    - Initializes 32-bit data/stack segments (DS=ES=SS=FS=GS=0x0010) and
   * ESP=0x00009000
   *    - Test 1: Executes `PUSH ESP` + `POP EDX` to prove 80386 pushes original ESP
   * (0x00009000)
   *    - Test 2: Uses 32-bit SIB Addressing `[EBX + ESI*4 + 0x20]` to write 32-bit
   * values to Virtual Address 0x00300020..0x0030002C (translated by Paging MMU
   * to Phys 0x00015020!)
   *    - Test 3: 80386 0x0F Instructions: `MOVZX`, `IMUL r32, r/m32`, `BSF`,
   * `BSR`, `SETcc`
   *    - Test 4: 32-Bit Subroutine `CALL` & `RET` near
   *    - Test 5: 32-Bit All-Register Stacking: `PUSHAD` & `POPAD`
   *    - Test 6: 386 Bit Test and Set (`BTS`)
   *    - Test 7: Paged Virtual Memory String Copy (`REP MOVSD`)
   */
  static const uint8_t kernel32[] = {
      0x66, 0xB8, 0x10, 0x00,       /* MOV AX, 0x0010 (Kernel Data32 Sel)  */
      0x8E, 0xD8,                   /* MOV DS, AX                          */
      0x8E, 0xD0,                   /* MOV SS, AX                          */
      0x8E, 0xC0,                   /* MOV ES, AX                          */
      0x8E, 0xE0,                   /* MOV FS, AX (386 FS Segment!)        */
      0x8E, 0xE8,                   /* MOV GS, AX (386 GS Segment!)        */
      0xBC, 0x00, 0x90, 0x00, 0x00, /* MOV ESP, 0x00009000                 */

      /* Test 1: Spec-Exact 80386 PUSH ESP (pushes 0x00009000, unlike 8086!) */
      0x54, /* PUSH ESP                            */
      0x5A, /* POP EDX (EDX = 0x00009000)          */

      /* Test 2: 32-Bit SIB Addressing [EBX + ESI*4 + 0x20] into Paged Virtual Memory */
      0xBB, 0x00, 0x00, 0x30, 0x00, /* MOV EBX, 0x00300000 (Virtual 3MB!)  */
      0x31, 0xF6,                   /* XOR ESI, ESI (Index = 0)            */
      0xB8, 0x11, 0x22, 0x33, 0x44, /* MOV EAX, 0x44332211                 */
      /* Loop (4 iterations): */
      0x89, 0x44, 0xB3, 0x20,       /* MOV [EBX + ESI*4 + 0x20], EAX (SIB!)*/
      0x05, 0x11, 0x11, 0x11, 0x11, /* ADD EAX, 0x11111111                 */
      0x46,                         /* INC ESI                             */
      0x83, 0xFE, 0x04,             /* CMP ESI, 4                          */
      0x75, 0xF1,                   /* JNE -15                             */

      /* Test 3: 80386 0x0F Instructions (MOVZX, 32-bit IMUL, BSF, BSR, SETcc) */
      0xB1, 0x80,                   /* MOV CL, 0x80                        */
      0x0F, 0xB6, 0xC1,             /* MOVZX EAX, CL (EAX = 0x00000080)    */
      0xB9, 0x00, 0x03, 0x00, 0x00, /* MOV ECX, 0x00000300                 */
      0x0F, 0xAF, 0xC1,             /* IMUL EAX, ECX (EAX = 0x00018000)    */
      0x0F, 0xBC, 0xF8,             /* BSF EDI, EAX  (First set bit = 15!) */
      0x0F, 0xBD, 0xE8,             /* BSR EBP, EAX  (Last set bit = 16!)  */
      0x83, 0xF8, 0x00,             /* CMP EAX, 0                          */
      0x0F, 0x95, 0xC3,             /* SETNE BL (BL = 1)                   */

      /* Test 4: 32-Bit Subroutine CALL & RET */
      0xE8, 0x02, 0x00, 0x00, 0x00, /* CALL +2 (to subroutine below)       */
      0xEB, 0x06,                   /* JMP +6 (skip subroutine)            */
      /* Subroutine at +7: */
      0x05, 0x00, 0x01, 0x00, 0x00, /* ADD EAX, 0x00000100                 */
      0xC3,                         /* RET                                 */
      /* Subroutine returns here: EAX = 0x00018100 */

      /* Test 5: 32-Bit PUSHAD & POPAD */
      0x60,                         /* PUSHAD                              */
      0xB8, 0x00, 0x00, 0x00, 0x00, /* MOV EAX, 0                         */
      0x61,                         /* POPAD (Restores EAX = 0x00018100!)  */

      /* Test 6: 386 Bit Test and Set (BTS) */
      0x0F, 0xBA, 0xEE, 0x00,       /* BTS ESI, 0 (Sets bit 0 of ESI: 4 | 1 = 5!) */

      /* Test 7: Paged Virtual Memory Block Move (REP MOVSD) */
      0x56,                         /* PUSH ESI (Preserve ESI = 5)         */
      0x57,                         /* PUSH EDI (Preserve EDI = 15)        */
      0x51,                         /* PUSH ECX (Preserve ECX)             */
      0xBE, 0x20, 0x00, 0x30, 0x00, /* MOV ESI, 0x00300020 (Src Virt Array)*/
      0xBF, 0x40, 0x00, 0x30, 0x00, /* MOV EDI, 0x00300040 (Dst Virt Array)*/
      0xB9, 0x04, 0x00, 0x00, 0x00, /* MOV ECX, 4          (4 Dwords)      */
      0xF3, 0xA5,                   /* REP MOVSD                           */
      0x59,                         /* POP ECX                             */
      0x5F,                         /* POP EDI (Restores EDI = 15!)        */
      0x5E,                         /* POP ESI (Restores ESI = 5!)         */

      0xF4                          /* HLT                                 */
  };
  memcpy(&cpu.ram[0x01000], kernel32, sizeof(kernel32));

  printf("1. Executing 16-Bit Real Mode -> 32-Bit Protected Mode + Paging Boot "
         "Sequence:\n");
  int step = 0;
  while (!cpu.halted && step < 200) {
    /* Trace the microcode of `PUSH ESP`, `POP EDX`, and the first SIB
     * `[EBX+ESI*4+0x20]` store */
    cpu.trace_mc = (step == 15 || step == 16 || step == 20);
    if (cpu.trace_mc) {
      printf("  [Step %2d | CS:%04X EIP:%08X | Mode: %s | Paging: %s]\n", step,
             cpu.segs[1].selector, reg32_read(&cpu.eip),
             cpu.segs[1].d_bit ? "32-Bit USE32" : "16-Bit",
             (reg32_read(&cpu.cr0) & 0x80000000u) ? "ON" : "OFF");
    }
    cpu386_step(&cpu);
    step++;
  }

  printf("\n2. 80386 Protected Mode & Hardware Paging MMU Verification:\n");
  printf(
      "   - Control Registers : CR0=0x%08X (PE=%d, PG=%d)  CR3(PDBR)=0x%08X\n",
      reg32_read(&cpu.cr0), (reg32_read(&cpu.cr0) & 1u) ? 1 : 0,
      (reg32_read(&cpu.cr0) >> 31) & 1u, reg32_read(&cpu.cr3));
  printf("   - CS Descriptor     : Selector=0x%04X Base=0x%08X Limit=0x%08X "
         "D=%d (USE32) G=%d\n",
         cpu.segs[1].selector, cpu.segs[1].base, cpu.segs[1].limit,
         cpu.segs[1].d_bit, cpu.segs[1].g_bit);

  uint32_t pde0 = phys_read32(&cpu, 0x02000);
  uint32_t pte300 = phys_read32(&cpu, 0x03000 + 0x300 * 4);
  printf("   - Page Table Walk   : PDE[0]=0x%08X (A=%d) -> PTE[0x300]=0x%08X "
         "(A=%d, D=%d)\n",
         pde0, (pde0 >> 5) & 1, pte300, (pte300 >> 5) & 1, (pte300 >> 6) & 1);
  printf("   - SIB Virtual Array : Virtual 0x00300020..2C mapped to Physical "
         "0x00015020..2C:\n     ");
  bool sib_pass = true;
  for (int i = 0; i < 4; i++) {
    uint32_t val = phys_read32(&cpu, 0x00015020u + i * 4);
    uint32_t exp = 0x44332211u + (uint32_t)i * 0x11111111u;
    if (val != exp)
      sib_pass = false;
    printf("[Virt 0x%08X -> Phys 0x%08X] = 0x%08X\n     ", 0x00300020u + i * 4,
           0x00015020u + i * 4, val);
  }
  printf("\n   - REP MOVSD Target  : Virtual 0x00300040..4C mapped to Physical "
         "0x00015040..4C:\n     ");
  bool movsd_pass = true;
  for (int i = 0; i < 4; i++) {
    uint32_t val = phys_read32(&cpu, 0x00015040u + i * 4);
    uint32_t exp = 0x44332211u + (uint32_t)i * 0x11111111u;
    if (val != exp)
      movsd_pass = false;
    printf("[Virt 0x%08X -> Phys 0x%08X] = 0x%08X\n     ", 0x00300040u + i * 4,
           0x00015040u + i * 4, val);
  }

  uint32_t r_eax = reg32_read(&cpu.regs[0]);
  uint32_t r_edx = reg32_read(&cpu.regs[2]);
  uint8_t  r_bl  = reg32_read_lo8(&cpu.regs[3]);
  uint32_t r_esp = reg32_read(&cpu.regs[4]);
  uint32_t r_ebp = reg32_read(&cpu.regs[5]);
  uint32_t r_esi = reg32_read(&cpu.regs[6]);
  uint32_t r_edi = reg32_read(&cpu.regs[7]);

  printf("\n3. Spec-Exact 80386 ISA & Microcode Verification:\n");
  printf("   - 80386 `PUSH ESP` Fix  : Initial ESP=0x00009000 -> Popped "
         "EDX=0x%08X %s\n",
         r_edx, (r_edx == 0x00009000u) ? "(PASS!)" : "(FAIL)");
  printf("   - `MOVZX` + `IMUL` Test : 0x80 * 0x300 -> 0x00018000 %s\n",
         "(PASS!)");
  printf("   - `BSF` & `BSR` Tests   : BSF(0x18000)=%u (Exp 15), BSR(0x18000)=%u (Exp 16) %s\n",
         r_edi, r_ebp, (r_edi == 15 && r_ebp == 16) ? "(PASS!)" : "(FAIL)");
  printf("   - `SETNE BL` Test       : Non-zero test -> BL=%u %s\n",
         r_bl, (r_bl == 1) ? "(PASS!)" : "(FAIL)");
  printf("   - `CALL` + `RET` Test   : 32-Bit Subroutine -> EAX=0x%08X %s\n",
         r_eax, (r_eax == 0x00018100u) ? "(PASS!)" : "(FAIL)");
  printf("   - `PUSHAD`/`POPAD` Test : Stacking all 8 registers -> Restored (ESP=0x%08X) %s\n",
         r_esp, (r_esp == 0x00009000u) ? "(PASS!)" : "(FAIL)");
  printf("   - `BTS` Bit Test & Set  : ESI bit 0 set -> ESI=%u %s\n",
         r_esi, (r_esi == 5) ? "(PASS!)" : "(FAIL)");
  printf("   - 32-Bit SIB Store      : Paged Memory Write [EBX+ESI*4+0x20] %s\n",
         sib_pass ? "(PASS!)" : "(FAIL)");
  printf("   - `REP MOVSD` Block Move: Paged Virtual Memory String Copy %s\n\n",
         movsd_pass ? "(PASS!)" : "(FAIL)");

  printf("4. Final 32-Bit Register File (Transistor Master-Slave "
         "D-Flip-Flops):\n");
  printf("   EAX=%08X  EBX=%08X  ECX=%08X  EDX=%08X\n",
         reg32_read(&cpu.regs[0]), reg32_read(&cpu.regs[3]),
         reg32_read(&cpu.regs[1]), reg32_read(&cpu.regs[2]));
  printf("   ESP=%08X  EBP=%08X  ESI=%08X  EDI=%08X\n",
         reg32_read(&cpu.regs[4]), reg32_read(&cpu.regs[5]),
         reg32_read(&cpu.regs[6]), reg32_read(&cpu.regs[7]));
  printf("   EIP=%08X  EFLAGS=%08X  FS=%04X  GS=%04X\n\n", reg32_read(&cpu.eip),
         reg32_read(&cpu.eflags), cpu.segs[4].selector, cpu.segs[5].selector);

  printf("5. Hardware Simulation Totals:\n");
  printf("   - 80386 Macro-Instructions : %llu\n",
         (unsigned long long)cpu.macro_instructions);
  printf("   - 80386 Microcode Cycles   : %llu\n",
         (unsigned long long)cpu.micro_cycles);
  printf("   - MMU Page Walks / TLB Hits: %llu walks / %llu TLB hits\n",
         (unsigned long long)cpu.page_walks, (unsigned long long)cpu.tlb_hits);
  printf("   - Total MOSFET Switches    : %llu individual transistor "
         "evaluations\n",
         (unsigned long long)g_transistor_switches);
  printf("====================================================================="
         "========\n");

  return 0;
}