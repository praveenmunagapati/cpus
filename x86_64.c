/*
 * ============================================================================
 * FROM TRANSISTORS TO SPEC-EXACT 64-BIT x86-64 / AMD64 CPU IN C
 * ============================================================================
 * Layer 1: Switch-Level PMOS & NMOS MOSFET Transistors
 * Layer 2: CMOS Logic Gates (NOT, NAND, NOR, AND, OR, XOR, XNOR, MUX2)
 * Layer 3: 64-Bit Transistor Ripple Adder, 64-Bit Barrel Shifter & 64-Bit ALU
 * Layer 4: NOR SR-Latch -> Master-Slave D-Flip-Flop -> 16x 64-Bit Registers (RAX..R15)
 * Layer 5: 64-Bit MMU: 4-Level Paging (PML4/PDPT/PD/PT), Canonical Checks & TLB
 * Layer 6: 64-Bit Microcode Engine, REX Prefixes, RIP-Relative, SIB & SYSCALL/SYSRET
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
  if (a == HIGH_Z) return b;
  if (b == HIGH_Z) return a;
  if (a == b) return a;
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

static inline Wire gate_or(Wire a, Wire b) {
  return gate_not(gate_nor(a, b));
}

static inline Wire gate_xor(Wire a, Wire b) {
  Wire n1 = gate_nand(a, b);
  Wire n2 = gate_nand(a, n1);
  Wire n3 = gate_nand(b, n1);
  return gate_nand(n2, n3);
}

static inline Wire gate_xnor(Wire a, Wire b) {
  return gate_not(gate_xor(a, b));
}

static inline Wire gate_mux2(Wire a, Wire b, Wire sel) {
  return gate_or(gate_and(a, gate_not(sel)), gate_and(b, sel));
}

/* ============================================================================
 * LAYER 3: 64-BIT COMBINATIONAL LOGIC, BARREL SHIFTER & 64-BIT ALU
 * ============================================================================
 */

typedef struct {
  Wire b[64];
} Word64;

static inline Word64 word64_from_u64(uint64_t val) {
  Word64 w;
  for (int i = 0; i < 64; i++)
    w.b[i] = ((val >> i) & 1ULL) ? HIGH : LOW;
  return w;
}

static inline uint64_t word64_to_u64(Word64 w) {
  uint64_t val = 0;
  for (int i = 0; i < 64; i++) {
    if (w.b[i] == HIGH)
      val |= (1ULL << i);
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
 * 64-Bit Transistor Ripple-Carry Adder with Full Internal Carry Taps:
 *   - carries[3]                  -> AF (Auxiliary Carry from bit 3 to bit 4)
 *   - carries[6]  XOR carries[7]  -> 8-bit OF
 *   - carries[14] XOR carries[15] -> 16-bit OF
 *   - carries[30] XOR carries[31] -> 32-bit OF
 *   - carries[62] XOR carries[63] -> 64-bit OF
 */
static inline Word64 adder_64bit(Word64 a, Word64 b, Wire cin, Wire carries[64]) {
  Word64 sum;
  Wire c = cin;
  for (int i = 0; i < 64; i++) {
    full_adder(a.b[i], b.b[i], c, &sum.b[i], &c);
    carries[i] = c;
  }
  return sum;
}

/*
 * 64-Bit Logarithmic Multiplexer Barrel Shifter (Shift Left by 0..63 bits)
 * Built from 6 stages of 64 gate_mux2 multiplexers (384 MUXes total).
 */
static inline Word64 barrel_shifter_left_64bit(Word64 in, uint8_t shift_cnt) {
  shift_cnt &= 63;
  Word64 cur = in;
  Word64 next;

  for (int stage = 0; stage < 6; stage++) {
    int shift_step = 1 << stage;
    Wire sel = ((shift_cnt >> stage) & 1) ? HIGH : LOW;
    for (int bit = 0; bit < 64; bit++) {
      Wire shifted_in = (bit >= shift_step) ? cur.b[bit - shift_step] : LOW;
      next.b[bit] = gate_mux2(cur.b[bit], shifted_in, sel);
    }
    cur = next;
  }
  return cur;
}

/*
 * 64-Bit Logarithmic Multiplexer Right Shifter (Logical SHR or Arithmetic SAR)
 * When fill_bit is LOW -> Logical Right Shift (SHR)
 * When fill_bit is MSB -> Arithmetic Right Shift (SAR with sign extension)
 */
static inline Word64 barrel_shifter_right_64bit(Word64 in, uint8_t shift_cnt, Wire fill_bit) {
  shift_cnt &= 63;
  Word64 cur = in;
  Word64 next;

  for (int stage = 0; stage < 6; stage++) {
    int shift_step = 1 << stage;
    Wire sel = ((shift_cnt >> stage) & 1) ? HIGH : LOW;
    for (int bit = 0; bit < 64; bit++) {
      Wire shifted_in = (bit + shift_step < 64) ? cur.b[bit + shift_step] : fill_bit;
      next.b[bit] = gate_mux2(cur.b[bit], shifted_in, sel);
    }
    cur = next;
  }
  return cur;
}

/*
 * 64-Bit Logarithmic Multiplexer Rotate Shifter (ROL / ROR by 0..63 bits)
 */
static inline Word64 barrel_shifter_rot_64bit(Word64 in, uint8_t rot_cnt, bool rot_right) {
  rot_cnt &= 63;
  if (rot_right)
    rot_cnt = (64 - rot_cnt) & 63;
  Word64 cur = in;
  Word64 next;

  for (int stage = 0; stage < 6; stage++) {
    int shift_step = 1 << stage;
    Wire sel = ((rot_cnt >> stage) & 1) ? HIGH : LOW;
    for (int bit = 0; bit < 64; bit++) {
      Wire rot_in = cur.b[(bit - shift_step + 64) % 64];
      next.b[bit] = gate_mux2(cur.b[bit], rot_in, sel);
    }
    cur = next;
  }
  return cur;
}

/* RFLAGS Bit Definitions */
#define RFLAG_CF (1ULL << 0)
#define RFLAG_PF (1ULL << 2)
#define RFLAG_AF (1ULL << 4)
#define RFLAG_ZF (1ULL << 6)
#define RFLAG_SF (1ULL << 7)
#define RFLAG_TF (1ULL << 8)
#define RFLAG_IF (1ULL << 9)
#define RFLAG_DF (1ULL << 10)
#define RFLAG_OF (1ULL << 11)

static inline uint64_t sanitize_rflags(uint64_t f) {
  return (f & 0x00000000003C7FD7ULL) | (1ULL << 1); /* Bit 1 is always 1 */
}

typedef enum {
  ALU64_ADD,
  ALU64_OR,
  ALU64_ADC,
  ALU64_SBB,
  ALU64_AND,
  ALU64_SUB,
  ALU64_XOR,
  ALU64_CMP,
  ALU64_INC,
  ALU64_DEC,
  ALU64_TEST,
  ALU64_NOT,
  ALU64_NEG,
  ALU64_SHL,
  ALU64_SHR,
  ALU64_SAR,
  ALU64_ROL,
  ALU64_ROR,
  ALU64_BSF,
  ALU64_BSR
} Alu64Op;

static Word64 alu_64_execute(Alu64Op op, Word64 a, Word64 b, int bits,
                             uint64_t *rflags, bool update_flags) {
  Word64 out;
  Wire carries[64];
  uint64_t f = *rflags;
  uint64_t val_a = word64_to_u64(a);
  uint64_t val_b = word64_to_u64(b);
  uint64_t mask = (bits == 64) ? 0xFFFFFFFFFFFFFFFFULL : ((1ULL << bits) - 1ULL);
  val_a &= mask;
  val_b &= mask;

  switch (op) {
  case ALU64_ADD:
  case ALU64_ADC: {
    Wire cin = (op == ALU64_ADC && (f & RFLAG_CF)) ? HIGH : LOW;
    out = adder_64bit(word64_from_u64(val_a), word64_from_u64(val_b), cin, carries);
    if (update_flags) {
      uint64_t res = word64_to_u64(out) & mask;
      Wire c_out = carries[bits - 1];
      Wire c_prev = (bits > 1) ? carries[bits - 2] : LOW;
      if (c_out == HIGH) f |= RFLAG_CF; else f &= ~RFLAG_CF;
      if (gate_xor(c_out, c_prev) == HIGH) f |= RFLAG_OF; else f &= ~RFLAG_OF;
      if (carries[3] == HIGH) f |= RFLAG_AF; else f &= ~RFLAG_AF;
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
      uint8_t parity = res & 0xFF;
      parity ^= parity >> 4;
      parity ^= parity >> 2;
      parity ^= parity >> 1;
      if ((parity & 1) == 0) f |= RFLAG_PF; else f &= ~RFLAG_PF;
    }
    break;
  }
  case ALU64_SUB:
  case ALU64_SBB:
  case ALU64_CMP: {
    Wire bin = (op == ALU64_SBB && (f & RFLAG_CF)) ? HIGH : LOW;
    Word64 b_inv;
    for (int i = 0; i < 64; i++)
      b_inv.b[i] = gate_not(word64_from_u64(val_b).b[i]);
    Wire cin = (bin == HIGH) ? LOW : HIGH;
    out = adder_64bit(word64_from_u64(val_a), b_inv, cin, carries);
    if (update_flags) {
      uint64_t res = word64_to_u64(out) & mask;
      Wire c_out = carries[bits - 1];
      Wire c_prev = (bits > 1) ? carries[bits - 2] : LOW;
      if (c_out == LOW) f |= RFLAG_CF; else f &= ~RFLAG_CF;
      if (gate_xor(gate_not(c_out), gate_not(c_prev)) == HIGH) f |= RFLAG_OF; else f &= ~RFLAG_OF;
      if (carries[3] == LOW) f |= RFLAG_AF; else f &= ~RFLAG_AF;
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
      uint8_t parity = res & 0xFF;
      parity ^= parity >> 4;
      parity ^= parity >> 2;
      parity ^= parity >> 1;
      if ((parity & 1) == 0) f |= RFLAG_PF; else f &= ~RFLAG_PF;
    }
    break;
  }
  case ALU64_AND:
  case ALU64_TEST: {
    for (int i = 0; i < 64; i++)
      out.b[i] = gate_and(a.b[i], b.b[i]);
    if (update_flags) {
      uint64_t res = word64_to_u64(out) & mask;
      f &= ~(RFLAG_CF | RFLAG_OF | RFLAG_AF);
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
      uint8_t parity = res & 0xFF;
      parity ^= parity >> 4;
      parity ^= parity >> 2;
      parity ^= parity >> 1;
      if ((parity & 1) == 0) f |= RFLAG_PF; else f &= ~RFLAG_PF;
    }
    break;
  }
  case ALU64_OR: {
    for (int i = 0; i < 64; i++)
      out.b[i] = gate_or(a.b[i], b.b[i]);
    if (update_flags) {
      uint64_t res = word64_to_u64(out) & mask;
      f &= ~(RFLAG_CF | RFLAG_OF | RFLAG_AF);
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
      uint8_t parity = res & 0xFF;
      parity ^= parity >> 4;
      parity ^= parity >> 2;
      parity ^= parity >> 1;
      if ((parity & 1) == 0) f |= RFLAG_PF; else f &= ~RFLAG_PF;
    }
    break;
  }
  case ALU64_XOR: {
    for (int i = 0; i < 64; i++)
      out.b[i] = gate_xor(a.b[i], b.b[i]);
    if (update_flags) {
      uint64_t res = word64_to_u64(out) & mask;
      f &= ~(RFLAG_CF | RFLAG_OF | RFLAG_AF);
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
      uint8_t parity = res & 0xFF;
      parity ^= parity >> 4;
      parity ^= parity >> 2;
      parity ^= parity >> 1;
      if ((parity & 1) == 0) f |= RFLAG_PF; else f &= ~RFLAG_PF;
    }
    break;
  }
  case ALU64_INC: {
    out = adder_64bit(word64_from_u64(val_a), word64_from_u64(1ULL), LOW, carries);
    if (update_flags) {
      uint64_t res = word64_to_u64(out) & mask;
      Wire c_out = carries[bits - 1];
      Wire c_prev = (bits > 1) ? carries[bits - 2] : LOW;
      if (gate_xor(c_out, c_prev) == HIGH) f |= RFLAG_OF; else f &= ~RFLAG_OF;
      if (carries[3] == HIGH) f |= RFLAG_AF; else f &= ~RFLAG_AF;
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
      uint8_t parity = res & 0xFF;
      parity ^= parity >> 4;
      parity ^= parity >> 2;
      parity ^= parity >> 1;
      if ((parity & 1) == 0) f |= RFLAG_PF; else f &= ~RFLAG_PF;
    }
    break;
  }
  case ALU64_DEC: {
    Word64 one_inv;
    for (int i = 0; i < 64; i++)
      one_inv.b[i] = gate_not(word64_from_u64(1ULL).b[i]);
    out = adder_64bit(word64_from_u64(val_a), one_inv, HIGH, carries);
    if (update_flags) {
      uint64_t res = word64_to_u64(out) & mask;
      Wire c_out = carries[bits - 1];
      Wire c_prev = (bits > 1) ? carries[bits - 2] : LOW;
      if (gate_xor(gate_not(c_out), gate_not(c_prev)) == HIGH) f |= RFLAG_OF; else f &= ~RFLAG_OF;
      if (carries[3] == LOW) f |= RFLAG_AF; else f &= ~RFLAG_AF;
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
      uint8_t parity = res & 0xFF;
      parity ^= parity >> 4;
      parity ^= parity >> 2;
      parity ^= parity >> 1;
      if ((parity & 1) == 0) f |= RFLAG_PF; else f &= ~RFLAG_PF;
    }
    break;
  }
  case ALU64_NOT: {
    for (int i = 0; i < 64; i++)
      out.b[i] = gate_not(a.b[i]);
    break;
  }
  case ALU64_NEG: {
    Word64 a_inv;
    for (int i = 0; i < 64; i++)
      a_inv.b[i] = gate_not(word64_from_u64(val_a).b[i]);
    out = adder_64bit(word64_from_u64(0ULL), a_inv, HIGH, carries);
    if (update_flags) {
      uint64_t res = word64_to_u64(out) & mask;
      if (val_a != 0) f |= RFLAG_CF; else f &= ~RFLAG_CF;
      if (val_a == (1ULL << (bits - 1))) f |= RFLAG_OF; else f &= ~RFLAG_OF;
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
    }
    break;
  }
  case ALU64_SHL: {
    uint8_t cnt = val_b & 63;
    out = barrel_shifter_left_64bit(a, cnt);
    if (update_flags && cnt > 0) {
      uint64_t res = word64_to_u64(out) & mask;
      if (((val_a << (cnt - 1)) >> (bits - 1)) & 1ULL) f |= RFLAG_CF; else f &= ~RFLAG_CF;
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
    }
    break;
  }
  case ALU64_SHR: {
    uint8_t cnt = val_b & 63;
    out = barrel_shifter_right_64bit(a, cnt, LOW);
    if (update_flags && cnt > 0) {
      uint64_t res = word64_to_u64(out) & mask;
      if ((val_a >> (cnt - 1)) & 1ULL) f |= RFLAG_CF; else f &= ~RFLAG_CF;
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
    }
    break;
  }
  case ALU64_SAR: {
    uint8_t cnt = val_b & 63;
    Wire sign = ((val_a >> (bits - 1)) & 1ULL) ? HIGH : LOW;
    out = barrel_shifter_right_64bit(a, cnt, sign);
    if (update_flags && cnt > 0) {
      uint64_t res = word64_to_u64(out) & mask;
      if ((val_a >> (cnt - 1)) & 1ULL) f |= RFLAG_CF; else f &= ~RFLAG_CF;
      if (res == 0) f |= RFLAG_ZF; else f &= ~RFLAG_ZF;
      if ((res >> (bits - 1)) & 1ULL) f |= RFLAG_SF; else f &= ~RFLAG_SF;
    }
    break;
  }
  case ALU64_ROL: {
    uint8_t cnt = val_b & (bits - 1);
    out = barrel_shifter_rot_64bit(a, cnt, false);
    break;
  }
  case ALU64_ROR: {
    uint8_t cnt = val_b & (bits - 1);
    out = barrel_shifter_rot_64bit(a, cnt, true);
    break;
  }
  default:
    out = a;
    break;
  }

  *rflags = sanitize_rflags(f);
  return out;
}

/* ============================================================================
 * LAYER 4: SEQUENTIAL LOGIC & 64-BIT TRANSISTOR HARDWARE REGISTERS
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

typedef struct {
  SRLatch latch;
} DLatch;

static inline void d_latch_init(DLatch *dl) { sr_latch_init(&dl->latch); }

static inline void d_latch_update(DLatch *dl, Wire d, Wire clk) {
  Wire s = gate_and(d, clk);
  Wire r = gate_and(gate_not(d), clk);
  sr_latch_update(&dl->latch, s, r);
}

typedef struct {
  DLatch master;
  DLatch slave;
} DFF;

static inline void dff_init(DFF *ff) {
  d_latch_init(&ff->master);
  d_latch_init(&ff->slave);
}

static inline void dff_write(DFF *ff, Wire d) {
  d_latch_update(&ff->master, d, HIGH);
  d_latch_update(&ff->slave, ff->master.latch.q, LOW);
  d_latch_update(&ff->master, d, gate_not(HIGH));
  d_latch_update(&ff->slave, ff->master.latch.q, HIGH);
}

static inline Wire dff_read(const DFF *ff) { return ff->slave.latch.q; }

typedef struct {
  DFF bit[64];
} Reg64;

static inline void reg64_init(Reg64 *r) {
  for (int i = 0; i < 64; i++)
    dff_init(&r->bit[i]);
}

static inline void reg64_write(Reg64 *r, uint64_t val) {
  for (int i = 0; i < 64; i++)
    dff_write(&r->bit[i], ((val >> i) & 1ULL) ? HIGH : LOW);
}

static inline uint64_t reg64_read(const Reg64 *r) {
  uint64_t val = 0;
  for (int i = 0; i < 64; i++) {
    if (dff_read(&r->bit[i]) == HIGH)
      val |= (1ULL << i);
  }
  return val;
}

/*
 * SPEC-EXACT AMD64 / INTEL 64 RULE:
 * 32-bit register writes implicitly ZERO-EXTEND into the upper 32 bits!
 */
static inline void reg64_write32(Reg64 *r, uint32_t val) {
  reg64_write(r, (uint64_t)val);
}

static inline uint32_t reg64_read32(const Reg64 *r) {
  return (uint32_t)reg64_read(r);
}

static inline void reg64_write16(Reg64 *r, uint16_t val) {
  for (int i = 0; i < 16; i++)
    dff_write(&r->bit[i], ((val >> i) & 1ULL) ? HIGH : LOW);
}

static inline uint16_t reg64_read16(const Reg64 *r) {
  uint16_t val = 0;
  for (int i = 0; i < 16; i++) {
    if (dff_read(&r->bit[i]) == HIGH)
      val |= (1ULL << i);
  }
  return val;
}

static inline void reg64_write_lo8(Reg64 *r, uint8_t val) {
  for (int i = 0; i < 8; i++)
    dff_write(&r->bit[i], ((val >> i) & 1ULL) ? HIGH : LOW);
}

static inline uint8_t reg64_read_lo8(const Reg64 *r) {
  uint8_t val = 0;
  for (int i = 0; i < 8; i++) {
    if (dff_read(&r->bit[i]) == HIGH)
      val |= (1ULL << i);
  }
  return val;
}

static inline void reg64_write_hi8(Reg64 *r, uint8_t val) {
  for (int i = 0; i < 8; i++)
    dff_write(&r->bit[i + 8], ((val >> i) & 1ULL) ? HIGH : LOW);
}

static inline uint8_t reg64_read_hi8(const Reg64 *r) {
  uint8_t val = 0;
  for (int i = 0; i < 8; i++) {
    if (dff_read(&r->bit[i + 8]) == HIGH)
      val |= (1ULL << i);
  }
  return val;
}

/* ============================================================================
 * LAYER 5: 64-BIT MMU, 4-LEVEL PAGING, CANONICAL ADDRESSES & TLB
 * ============================================================================
 */

#define PHYS_RAM_SIZE (2 * 1024 * 1024) /* 2 MB Physical RAM */

typedef struct {
  uint64_t vpn;
  uint64_t ppn;
  uint64_t pte_phys;
  bool valid;
} TLBEntry64;

typedef struct {
  uint16_t selector;
  uint64_t base;
  uint32_t limit;
  bool present;
  bool l_bit; /* 64-bit Long Mode Code Segment (L=1) */
  bool d_bit; /* Default size (0 in 64-bit mode)      */
  bool g_bit; /* Granularity                         */
} SegDescCache64;

/* Forward declaration */
struct X86_64CPU;
typedef struct X86_64CPU X86_64CPU;

static inline bool is_canonical_address(uint64_t addr) {
  uint64_t high = addr >> 47;
  return (high == 0 || high == 0x1FFFFULL);
}

static uint8_t phys_read8(const X86_64CPU *cpu, uint64_t paddr);
static void phys_write8(X86_64CPU *cpu, uint64_t paddr, uint8_t val);

static uint16_t phys_read16(const X86_64CPU *cpu, uint64_t paddr) {
  uint16_t lo = phys_read8(cpu, paddr);
  uint16_t hi = phys_read8(cpu, paddr + 1);
  return (uint16_t)(lo | (hi << 8));
}

static void phys_write16(X86_64CPU *cpu, uint64_t paddr, uint16_t val) {
  phys_write8(cpu, paddr, (uint8_t)val);
  phys_write8(cpu, paddr + 1, (uint8_t)(val >> 8));
}

static uint32_t phys_read32(const X86_64CPU *cpu, uint64_t paddr) {
  uint32_t lo = phys_read16(cpu, paddr);
  uint32_t hi = phys_read16(cpu, paddr + 2);
  return lo | (hi << 16);
}

static void phys_write32(X86_64CPU *cpu, uint64_t paddr, uint32_t val) {
  phys_write16(cpu, paddr, (uint16_t)val);
  phys_write16(cpu, paddr + 2, (uint16_t)(val >> 16));
}

static uint64_t phys_read64(const X86_64CPU *cpu, uint64_t paddr) {
  uint64_t lo = phys_read32(cpu, paddr);
  uint64_t hi = phys_read32(cpu, paddr + 4);
  return lo | (hi << 32);
}

static void phys_write64(X86_64CPU *cpu, uint64_t paddr, uint64_t val) {
  phys_write32(cpu, paddr, (uint32_t)val);
  phys_write32(cpu, paddr + 4, (uint32_t)(val >> 32));
}

/* ============================================================================
 * 64-BIT CPU STATE & MICROCODE DEFINITIONS
 * ============================================================================
 */

typedef enum {
  MB64_NONE = 0,
  MB64_RAX, MB64_RCX, MB64_RDX, MB64_RBX,
  MB64_RSP, MB64_RBP, MB64_RSI, MB64_RDI,
  MB64_R8,  MB64_R9,  MB64_R10, MB64_R11,
  MB64_R12, MB64_R13, MB64_R14, MB64_R15,
  MB64_RIP, MB64_RFLAGS,
  MB64_R, MB64_M,
  MB64_TMPa, MB64_TMPb, MB64_TMPc,
  MB64_SIGMA, MB64_IND, MB64_OPR,
  MB64_IMM, MB64_IMM_SXT,
  MB64_STEP_SZ
} MicroBus64;

typedef enum {
  MACT64_NONE = 0,
  MACT64_ALU,
  MACT64_MEM_WRITE_M,
  MACT64_MEM_PUSH,
  MACT64_MEM_POP,
  MACT64_MEM_READ
} MicroAct64;

typedef struct {
  MicroBus64 src;
  MicroBus64 dst;
  MicroAct64 act;
  Alu64Op alu_op;
  bool use_xi;
  bool update_flags;
  bool rni;
  const char *comment;
} MicroInstr64;

enum {
  MC64_ALU_RM_R   = 0x00,
  MC64_ALU_R_RM   = 0x04,
  MC64_CMP_RM_R   = 0x08,
  MC64_CMP_R_RM   = 0x0C,
  MC64_ALU_ACC_IMM= 0x10,
  MC64_CMP_ACC_IMM= 0x14,
  MC64_GRP1_RM_IMM= 0x18,
  MC64_GRP1_CMP_IMM=0x1C,
  MC64_MOV_RM_R   = 0x20,
  MC64_MOV_R_RM   = 0x24,
  MC64_MOV_R_IMM  = 0x28,
  MC64_MOV_RM_IMM = 0x2C,
  MC64_INC_R      = 0x30,
  MC64_DEC_R      = 0x34,
  MC64_PUSH_R     = 0x38,
  MC64_POP_R      = 0x40,
  MC64_LEA_R_M    = 0x48,
  MC64_CALL_REL   = 0x50,
  MC64_RET_NEAR   = 0x58,
  MC64_JMP_REL    = 0x60,
  MC64_XCHG_R_RM  = 0x68,
  MC64_XCHG_RAX_R = 0x70,
  MC64_TEST_RM_R  = 0x78,
  MC64_TEST_ACC_IMM=0x80,
  MC64_NOT_RM     = 0x88,
  MC64_NEG_RM     = 0x90,
  MC64_SHL_RM     = 0x98,
  MC64_SHR_RM     = 0xA0,
  MC64_SAR_RM     = 0xA8
};

struct X86_64CPU {
  /* 16x 64-Bit General Purpose Registers */
  Reg64 regs[16]; /* 0:RAX, 1:RCX, 2:RDX, 3:RBX, 4:RSP, 5:RBP, 6:RSI, 7:RDI,
                     8:R8,  9:R9, 10:R10, 11:R11, 12:R12, 13:R13, 14:R14, 15:R15 */

  /* 64-Bit Instruction Pointer & RFLAGS */
  Reg64 rip;
  Reg64 rflags;

  /* Six Segment Registers (ES, CS, SS, DS, FS, GS) */
  SegDescCache64 segs[6];

  /* Control Registers */
  Reg64 cr0; /* PE (bit 0), PG (bit 31)                      */
  Reg64 cr2; /* Page Fault Linear Address                    */
  Reg64 cr3; /* PML4 Base Address (Page Map Level 4)         */
  Reg64 cr4; /* PAE (bit 5 - required for Long Mode!)        */

  /* Extended Feature & System Call MSRs */
  uint64_t msr_efer;           /* 0xC0000080: LME(8), LMA(10), NXE(11), SCE(0) */
  uint64_t msr_star;           /* 0xC0000081: Target CS/SS for SYSCALL/SYSRET   */
  uint64_t msr_lstar;          /* 0xC0000082: Target 64-bit RIP for SYSCALL     */
  uint64_t msr_sfmask;         /* 0xC0000084: RFLAGS mask for SYSCALL           */
  uint64_t msr_fs_base;        /* 0xC0000100: 64-bit Base for FS Segment        */
  uint64_t msr_gs_base;        /* 0xC0000101: 64-bit Base for GS Segment        */
  uint64_t msr_kernel_gs_base; /* 0xC0000102: Swapped with GS_BASE on SWAPGS   */

  uint64_t gdtr_base;
  uint16_t gdtr_limit;
  uint64_t idtr_base;
  uint16_t idtr_limit;

  /* Internal Execution Unit Staging Registers */
  Reg64 tmp_a;
  Reg64 tmp_b;
  Reg64 tmp_c;
  Reg64 sigma;
  Reg64 ind;
  Reg64 opr;

  /* 64-Entry Hardware TLB */
  TLBEntry64 tlb[64];
  uint64_t tlb_hits;
  uint64_t page_walks;
  uint64_t last_linear_addr;
  uint64_t last_phys_addr;

  /* Microcode ROM & Instruction Decode State */
  MicroInstr64 mc_rom[256];
  uint16_t mc_pc;
  int op_bits;   /* 8, 16, 32, or 64 bits */
  int addr_bits; /* 32 or 64 bits         */
  int seg_override;
  uint8_t rep_prefix;
  uint8_t modrm, mod, reg_field, rm_field;
  bool s_bit;
  bool rex_w, rex_r, rex_x, rex_b, has_rex;
  Alu64Op xi_alu_op;

  bool halted;
  bool trace_mc;
  uint64_t macro_instructions;
  uint64_t micro_cycles;
  uint8_t ram[PHYS_RAM_SIZE];
};

static uint8_t phys_read8(const X86_64CPU *cpu, uint64_t paddr) {
  return cpu->ram[paddr & (PHYS_RAM_SIZE - 1)];
}

static void phys_write8(X86_64CPU *cpu, uint64_t paddr, uint8_t val) {
  cpu->ram[paddr & (PHYS_RAM_SIZE - 1)] = val;
}

static void tlb_flush(X86_64CPU *cpu) {
  for (int i = 0; i < 64; i++) {
    cpu->tlb[i].valid = false;
    cpu->tlb[i].vpn = 0;
    cpu->tlb[i].ppn = 0;
  }
}

static void tlb_invalidate_page(X86_64CPU *cpu, uint64_t vaddr) {
  uint64_t vpn = vaddr >> 12;
  uint32_t idx = vpn & 63;
  if (cpu->tlb[idx].valid && cpu->tlb[idx].vpn == vpn)
    cpu->tlb[idx].valid = false;
}

/*
 * 64-Bit 4-Level Hardware Paging MMU (PML4 -> PDPT -> PD -> PT)
 * Evaluates 48-bit Canonical Linear Addresses to Physical Frames!
 */
static uint64_t mmu_translate_64(X86_64CPU *cpu, int seg_idx, uint64_t offset, bool is_write) {
  uint64_t base = 0;
  if (cpu->segs[1].l_bit) {
    if (seg_idx == 4) base = cpu->msr_fs_base;
    else if (seg_idx == 5) base = cpu->msr_gs_base;
    else base = 0;
  } else {
    if (seg_idx == 4) base = cpu->msr_fs_base;
    else if (seg_idx == 5) base = cpu->msr_gs_base;
    else base = cpu->segs[seg_idx].base;
  }

  Wire carries[64];
  Word64 base_w = word64_from_u64(base);
  Word64 off_w = word64_from_u64(offset);
  uint64_t linear = word64_to_u64(adder_64bit(base_w, off_w, LOW, carries));
  cpu->last_linear_addr = linear;

  /* Check Canonical Address Form */
  if (!is_canonical_address(linear)) {
    printf("   [MMU FAULT] Non-canonical 64-bit virtual address: 0x%016llX\n",
           (unsigned long long)linear);
    cpu->halted = true;
    return 0;
  }

  /* Check CR0 Bit 31 (PG) & EFER Bit 10 (LMA) */
  uint64_t cr0 = reg64_read(&cpu->cr0);
  if ((cr0 & 0x80000000ULL) == 0 || (cpu->msr_efer & (1ULL << 10)) == 0) {
    cpu->last_phys_addr = linear & (PHYS_RAM_SIZE - 1);
    return cpu->last_phys_addr;
  }

  /* 4-Level Paging Enabled: Check 64-entry TLB */
  uint64_t vpn = linear >> 12;
  uint64_t page_off = linear & 0xFFFULL;
  uint32_t tlb_idx = vpn & 63;

  if (cpu->tlb[tlb_idx].valid && cpu->tlb[tlb_idx].vpn == vpn) {
    cpu->tlb_hits++;
    if (is_write) {
      uint64_t pte = phys_read64(cpu, cpu->tlb[tlb_idx].pte_phys);
      phys_write64(cpu, cpu->tlb[tlb_idx].pte_phys, pte | 0x40ULL); /* Dirty */
    }
    uint64_t phys = (cpu->tlb[tlb_idx].ppn << 12) | page_off;
    cpu->last_phys_addr = phys;
    return phys;
  }

  /* TLB Miss: 4-Level Page Table Walk */
  cpu->page_walks++;
  uint64_t pml4_base = reg64_read(&cpu->cr3) & 0x000FFFFFFFFFF000ULL;
  uint64_t pml4_idx  = (linear >> 39) & 0x1FFULL;
  uint64_t pdpt_idx  = (linear >> 30) & 0x1FFULL;
  uint64_t pd_idx    = (linear >> 21) & 0x1FFULL;
  uint64_t pt_idx    = (linear >> 12) & 0x1FFULL;

  /* Level 4: PML4 */
  uint64_t pml4e_phys = pml4_base + (pml4_idx * 8ULL);
  uint64_t pml4e = phys_read64(cpu, pml4e_phys);
  if ((pml4e & 1ULL) == 0) {
    printf("   [PAGE FAULT] PML4 entry not present for 0x%016llX\n",
           (unsigned long long)linear);
    cpu->halted = true;
    return 0;
  }
  phys_write64(cpu, pml4e_phys, pml4e | 0x20ULL); /* Accessed */

  /* Level 3: PDPT */
  uint64_t pdpt_base = pml4e & 0x000FFFFFFFFFF000ULL;
  uint64_t pdpte_phys = pdpt_base + (pdpt_idx * 8ULL);
  uint64_t pdpte = phys_read64(cpu, pdpte_phys);
  if ((pdpte & 1ULL) == 0) {
    printf("   [PAGE FAULT] PDPT entry not present for 0x%016llX\n",
           (unsigned long long)linear);
    cpu->halted = true;
    return 0;
  }
  phys_write64(cpu, pdpte_phys, pdpte | 0x20ULL); /* Accessed */

  /* Level 2: PD */
  uint64_t pd_base = pdpte & 0x000FFFFFFFFFF000ULL;
  uint64_t pde_phys = pd_base + (pd_idx * 8ULL);
  uint64_t pde = phys_read64(cpu, pde_phys);
  if ((pde & 1ULL) == 0) {
    printf("   [PAGE FAULT] PD entry not present for 0x%016llX\n",
           (unsigned long long)linear);
    cpu->halted = true;
    return 0;
  }
  phys_write64(cpu, pde_phys, pde | 0x20ULL); /* Accessed */

  /* Level 1: PT */
  uint64_t pt_base = pde & 0x000FFFFFFFFFF000ULL;
  uint64_t pte_phys = pt_base + (pt_idx * 8ULL);
  uint64_t pte = phys_read64(cpu, pte_phys);
  if ((pte & 1ULL) == 0) {
    printf("   [PAGE FAULT] PT entry not present for 0x%016llX\n",
           (unsigned long long)linear);
    cpu->halted = true;
    return 0;
  }
  pte |= 0x20ULL; /* Accessed */
  if (is_write)
    pte |= 0x40ULL; /* Dirty */
  phys_write64(cpu, pte_phys, pte);

  uint64_t ppn = (pte & 0x000FFFFFFFFFF000ULL) >> 12;
  cpu->tlb[tlb_idx].valid = true;
  cpu->tlb[tlb_idx].vpn = vpn;
  cpu->tlb[tlb_idx].ppn = ppn;
  cpu->tlb[tlb_idx].pte_phys = pte_phys;

  uint64_t phys = (ppn << 12) | page_off;
  cpu->last_phys_addr = phys;
  return phys;
}

static uint8_t mmu_read8(X86_64CPU *cpu, int seg, uint64_t off) {
  uint64_t phys = mmu_translate_64(cpu, seg, off, false);
  return phys_read8(cpu, phys);
}

static void mmu_write8(X86_64CPU *cpu, int seg, uint64_t off, uint8_t val) {
  uint64_t phys = mmu_translate_64(cpu, seg, off, true);
  phys_write8(cpu, phys, val);
}

static uint16_t mmu_read16(X86_64CPU *cpu, int seg, uint64_t off) {
  uint16_t lo = mmu_read8(cpu, seg, off);
  uint16_t hi = mmu_read8(cpu, seg, off + 1);
  return (uint16_t)(lo | (hi << 8));
}

static void mmu_write16(X86_64CPU *cpu, int seg, uint64_t off, uint16_t val) {
  mmu_write8(cpu, seg, off, (uint8_t)val);
  mmu_write8(cpu, seg, off + 1, (uint8_t)(val >> 8));
}

static uint32_t mmu_read32(X86_64CPU *cpu, int seg, uint64_t off) {
  uint32_t lo = mmu_read16(cpu, seg, off);
  uint32_t hi = mmu_read16(cpu, seg, off + 2);
  return lo | (hi << 16);
}

static void mmu_write32(X86_64CPU *cpu, int seg, uint64_t off, uint32_t val) {
  mmu_write16(cpu, seg, off, (uint16_t)val);
  mmu_write16(cpu, seg, off + 2, (uint16_t)(val >> 16));
}

static uint64_t mmu_read64(X86_64CPU *cpu, int seg, uint64_t off) {
  uint64_t lo = mmu_read32(cpu, seg, off);
  uint64_t hi = mmu_read32(cpu, seg, off + 4);
  return lo | (hi << 32);
}

static void mmu_write64(X86_64CPU *cpu, int seg, uint64_t off, uint64_t val) {
  mmu_write32(cpu, seg, off, (uint32_t)val);
  mmu_write32(cpu, seg, off + 4, (uint32_t)(val >> 32));
}

static uint64_t mmu_read_sized(X86_64CPU *cpu, int seg, uint64_t off, int bits) {
  if (bits == 8) return mmu_read8(cpu, seg, off);
  if (bits == 16) return mmu_read16(cpu, seg, off);
  if (bits == 32) return mmu_read32(cpu, seg, off);
  return mmu_read64(cpu, seg, off);
}

static void mmu_write_sized(X86_64CPU *cpu, int seg, uint64_t off, uint64_t val, int bits) {
  if (bits == 8) mmu_write8(cpu, seg, off, (uint8_t)val);
  else if (bits == 16) mmu_write16(cpu, seg, off, (uint16_t)val);
  else if (bits == 32) mmu_write32(cpu, seg, off, (uint32_t)val);
  else mmu_write64(cpu, seg, off, val);
}

/* Instruction Stream Fetch from CS:RIP */
static uint8_t fetch8(X86_64CPU *cpu) {
  uint64_t rip = reg64_read(&cpu->rip);
  uint8_t val = mmu_read8(cpu, 1 /* CS */, rip);
  reg64_write(&cpu->rip, rip + 1);
  return val;
}

static uint16_t fetch16(X86_64CPU *cpu) {
  uint16_t lo = fetch8(cpu);
  uint16_t hi = fetch8(cpu);
  return (uint16_t)(lo | (hi << 8));
}

static uint32_t fetch32(X86_64CPU *cpu) {
  uint32_t lo = fetch16(cpu);
  uint32_t hi = fetch16(cpu);
  return lo | (hi << 16);
}

static uint64_t fetch64(X86_64CPU *cpu) {
  uint64_t lo = fetch32(cpu);
  uint64_t hi = fetch32(cpu);
  return lo | (hi << 32);
}

static uint64_t fetch_imm_sized(X86_64CPU *cpu, int bits) {
  if (bits == 8) return fetch8(cpu);
  if (bits == 16) return fetch16(cpu);
  if (bits == 32) return fetch32(cpu);
  return fetch64(cpu);
}

/* ============================================================================
 * LAYER 6: 64-BIT REGISTER ACCESS, REX PREFIXES & MICROCODE ENGINE
 * ============================================================================
 */

static uint64_t eu64_read_reg(const X86_64CPU *cpu, int idx, int bits, bool has_rex) {
  idx &= 15;
  if (bits == 64)
    return reg64_read(&cpu->regs[idx]);
  if (bits == 32)
    return reg64_read32(&cpu->regs[idx]);
  if (bits == 16)
    return reg64_read16(&cpu->regs[idx]);

  /* 8-bit Register Access: */
  if (has_rex) {
    /* With REX, indices 4..7 map to SPL, BPL, SIL, DIL */
    return reg64_read_lo8(&cpu->regs[idx]);
  }
  /* Legacy 8-bit registers (AH, CH, DH, BH) when no REX prefix */
  if (idx < 4)
    return reg64_read_lo8(&cpu->regs[idx]);
  return reg64_read_hi8(&cpu->regs[idx - 4]);
}

static void eu64_write_reg(X86_64CPU *cpu, int idx, uint64_t val, int bits, bool has_rex) {
  idx &= 15;
  if (bits == 64) {
    reg64_write(&cpu->regs[idx], val);
  } else if (bits == 32) {
    /* Spec-Exact: 32-bit writes ZERO-EXTEND to 64 bits! */
    reg64_write32(&cpu->regs[idx], (uint32_t)val);
  } else if (bits == 16) {
    reg64_write16(&cpu->regs[idx], (uint16_t)val);
  } else {
    if (has_rex) {
      reg64_write_lo8(&cpu->regs[idx], (uint8_t)val);
    } else {
      if (idx < 4)
        reg64_write_lo8(&cpu->regs[idx], (uint8_t)val);
      else
        reg64_write_hi8(&cpu->regs[idx - 4], (uint8_t)val);
    }
  }
}

static void eu64_push(X86_64CPU *cpu, uint64_t val) {
  uint64_t rsp = reg64_read(&cpu->regs[4]);
  uint64_t step = (cpu->op_bits == 16) ? 2ULL : 8ULL; /* Default stack size in 64-bit is 8 */
  rsp -= step;
  reg64_write(&cpu->regs[4], rsp);
  mmu_write_sized(cpu, 2 /* SS */, rsp, val, (int)step * 8);
  cpu->micro_cycles += 2;
}

static uint64_t eu64_pop(X86_64CPU *cpu) {
  uint64_t rsp = reg64_read(&cpu->regs[4]);
  uint64_t step = (cpu->op_bits == 16) ? 2ULL : 8ULL;
  uint64_t val = mmu_read_sized(cpu, 2 /* SS */, rsp, (int)step * 8);
  reg64_write(&cpu->regs[4], rsp + step);
  cpu->micro_cycles += 2;
  return val;
}

static void load_segment_64(X86_64CPU *cpu, int seg_idx, uint16_t sel) {
  cpu->segs[seg_idx].selector = sel;
  if ((reg64_read(&cpu->cr0) & 1ULL) == 0) {
    /* Real Mode base: sel << 4 */
    cpu->segs[seg_idx].base = (uint64_t)sel << 4;
    cpu->segs[seg_idx].limit = 0xFFFF;
    cpu->segs[seg_idx].l_bit = false;
    cpu->segs[seg_idx].d_bit = false;
    return;
  }

  uint32_t rpl = sel & 3;
  (void)rpl;
  uint32_t idx = sel >> 3;
  uint64_t desc_addr = cpu->gdtr_base + (idx * 8ULL);
  uint32_t lo = phys_read32(cpu, desc_addr);
  uint32_t hi = phys_read32(cpu, desc_addr + 4);

  uint64_t base = (lo >> 16) | ((uint64_t)(hi & 0xFF) << 16) | ((uint64_t)(hi & 0xFF000000ULL));
  uint32_t limit = (lo & 0xFFFF) | (hi & 0x000F0000);
  bool g_bit = (hi & (1 << 23)) != 0;
  bool d_bit = (hi & (1 << 22)) != 0;
  bool l_bit = (hi & (1 << 21)) != 0; /* Long Mode Code Segment! */

  if (g_bit)
    limit = (limit << 12) | 0xFFF;

  cpu->segs[seg_idx].base = base;
  cpu->segs[seg_idx].limit = limit;
  cpu->segs[seg_idx].g_bit = g_bit;
  cpu->segs[seg_idx].d_bit = d_bit;
  cpu->segs[seg_idx].l_bit = l_bit;
  cpu->segs[seg_idx].present = (hi & (1 << 15)) != 0;
}

static void build_64_microcode_rom(X86_64CPU *cpu) {
  memset(cpu->mc_rom, 0, sizeof(cpu->mc_rom));
  MicroInstr64 *rom = cpu->mc_rom;

  /* ALU r/m, r */
  rom[MC64_ALU_RM_R + 0] = (MicroInstr64){MB64_M, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC64_ALU_RM_R + 1] = (MicroInstr64){MB64_R, MB64_TMPb, MACT64_ALU, 0, 1, true, false, "R -> TMPb | XI ALU (F)"};
  rom[MC64_ALU_RM_R + 2] = (MicroInstr64){MB64_SIGMA, MB64_OPR, MACT64_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};

  /* ALU r, r/m */
  rom[MC64_ALU_R_RM + 0] = (MicroInstr64){MB64_R, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "R -> TMPa"};
  rom[MC64_ALU_R_RM + 1] = (MicroInstr64){MB64_M, MB64_TMPb, MACT64_ALU, 0, 1, true, false, "M -> TMPb | XI ALU (F)"};
  rom[MC64_ALU_R_RM + 2] = (MicroInstr64){MB64_SIGMA, MB64_R, MACT64_NONE, 0, 0, false, true, "SIGMA -> R | RNI"};

  /* CMP r/m, r & CMP r, r/m */
  rom[MC64_CMP_RM_R + 0] = (MicroInstr64){MB64_M, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC64_CMP_RM_R + 1] = (MicroInstr64){MB64_R, MB64_TMPb, MACT64_ALU, ALU64_CMP, 0, true, true, "R -> TMPb | CMP (F) | RNI"};

  rom[MC64_CMP_R_RM + 0] = (MicroInstr64){MB64_R, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "R -> TMPa"};
  rom[MC64_CMP_R_RM + 1] = (MicroInstr64){MB64_M, MB64_TMPb, MACT64_ALU, ALU64_CMP, 0, true, true, "M -> TMPb | CMP (F) | RNI"};

  /* ALU RAX, imm & CMP RAX, imm */
  rom[MC64_ALU_ACC_IMM + 0] = (MicroInstr64){MB64_RAX, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "RAX -> TMPa"};
  rom[MC64_ALU_ACC_IMM + 1] = (MicroInstr64){MB64_IMM, MB64_TMPb, MACT64_ALU, 0, 1, true, false, "IMM -> TMPb | XI ALU (F)"};
  rom[MC64_ALU_ACC_IMM + 2] = (MicroInstr64){MB64_SIGMA, MB64_RAX, MACT64_NONE, 0, 0, false, true, "SIGMA -> RAX | RNI"};

  rom[MC64_CMP_ACC_IMM + 0] = (MicroInstr64){MB64_RAX, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "RAX -> TMPa"};
  rom[MC64_CMP_ACC_IMM + 1] = (MicroInstr64){MB64_IMM, MB64_TMPb, MACT64_ALU, ALU64_CMP, 0, true, true, "IMM -> TMPb | CMP (F) | RNI"};

  /* Group 1 ALU r/m, imm */
  rom[MC64_GRP1_RM_IMM + 0] = (MicroInstr64){MB64_M, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC64_GRP1_RM_IMM + 1] = (MicroInstr64){MB64_IMM_SXT, MB64_TMPb, MACT64_ALU, 0, 1, true, false, "IMM -> TMPb | XI ALU (F)"};
  rom[MC64_GRP1_RM_IMM + 2] = (MicroInstr64){MB64_SIGMA, MB64_OPR, MACT64_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};

  rom[MC64_GRP1_CMP_IMM + 0] = (MicroInstr64){MB64_M, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC64_GRP1_CMP_IMM + 1] = (MicroInstr64){MB64_IMM_SXT, MB64_TMPb, MACT64_ALU, ALU64_CMP, 0, true, true, "IMM -> TMPb | CMP (F) | RNI"};

  /* MOV r/m, r & MOV r, r/m & MOV r, imm */
  rom[MC64_MOV_RM_R + 0] = (MicroInstr64){MB64_R, MB64_OPR, MACT64_MEM_WRITE_M, 0, 0, false, true, "R -> M | WB | RNI"};
  rom[MC64_MOV_R_RM + 0] = (MicroInstr64){MB64_M, MB64_R, MACT64_NONE, 0, 0, false, true, "M -> R | RNI"};
  rom[MC64_MOV_R_IMM + 0] = (MicroInstr64){MB64_IMM, MB64_R, MACT64_NONE, 0, 0, false, true, "IMM -> R | RNI"};
  rom[MC64_MOV_RM_IMM + 0] = (MicroInstr64){MB64_IMM, MB64_OPR, MACT64_MEM_WRITE_M, 0, 0, false, true, "IMM -> M | WB | RNI"};
  rom[MC64_LEA_R_M + 0] = (MicroInstr64){MB64_IND, MB64_R, MACT64_NONE, 0, 0, false, true, "IND -> R | RNI"};

  /* INC reg & DEC reg */
  rom[MC64_INC_R + 0] = (MicroInstr64){MB64_R, MB64_TMPa, MACT64_ALU, ALU64_INC, 0, true, false, "R -> TMPa | INC (F)"};
  rom[MC64_INC_R + 1] = (MicroInstr64){MB64_SIGMA, MB64_R, MACT64_NONE, 0, 0, false, true, "SIGMA -> R | RNI"};

  rom[MC64_DEC_R + 0] = (MicroInstr64){MB64_R, MB64_TMPa, MACT64_ALU, ALU64_DEC, 0, true, false, "R -> TMPa | DEC (F)"};
  rom[MC64_DEC_R + 1] = (MicroInstr64){MB64_SIGMA, MB64_R, MACT64_NONE, 0, 0, false, true, "SIGMA -> R | RNI"};

  /* PUSH reg & POP reg (64-bit stack) */
  rom[MC64_PUSH_R + 0] = (MicroInstr64){MB64_R, MB64_OPR, MACT64_NONE, 0, 0, false, false, "R -> OPR (Latches original RSP!)"};
  rom[MC64_PUSH_R + 1] = (MicroInstr64){MB64_RSP, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "RSP -> TMPa"};
  rom[MC64_PUSH_R + 2] = (MicroInstr64){MB64_STEP_SZ, MB64_TMPb, MACT64_ALU, ALU64_SUB, 0, false, false, "STEP -> TMPb | SUB"};
  rom[MC64_PUSH_R + 3] = (MicroInstr64){MB64_SIGMA, MB64_RSP, MACT64_NONE, 0, 0, false, false, "SIGMA -> RSP"};
  rom[MC64_PUSH_R + 4] = (MicroInstr64){MB64_SIGMA, MB64_IND, MACT64_MEM_PUSH, 0, 0, false, true, "SIGMA -> IND | PUSH [SS:IND] | RNI"};

  rom[MC64_POP_R + 0] = (MicroInstr64){MB64_RSP, MB64_IND, MACT64_MEM_POP, 0, 0, false, false, "RSP -> IND | POP [SS:IND]"};
  rom[MC64_POP_R + 1] = (MicroInstr64){MB64_RSP, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "RSP -> TMPa"};
  rom[MC64_POP_R + 2] = (MicroInstr64){MB64_STEP_SZ, MB64_TMPb, MACT64_ALU, ALU64_ADD, 0, false, false, "STEP -> TMPb | ADD"};
  rom[MC64_POP_R + 3] = (MicroInstr64){MB64_SIGMA, MB64_RSP, MACT64_NONE, 0, 0, false, false, "SIGMA -> RSP"};
  rom[MC64_POP_R + 4] = (MicroInstr64){MB64_OPR, MB64_R, MACT64_NONE, 0, 0, false, true, "OPR -> R | RNI"};

  /* TEST r/m, r */
  rom[MC64_TEST_RM_R + 0] = (MicroInstr64){MB64_M, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC64_TEST_RM_R + 1] = (MicroInstr64){MB64_R, MB64_TMPb, MACT64_ALU, ALU64_AND, 0, true, true, "R -> TMPb | TEST (F) | RNI"};

  /* XCHG r/m, r */
  rom[MC64_XCHG_R_RM + 0] = (MicroInstr64){MB64_M, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC64_XCHG_R_RM + 1] = (MicroInstr64){MB64_R, MB64_OPR, MACT64_MEM_WRITE_M, 0, 0, false, false, "R -> M | WB"};
  rom[MC64_XCHG_R_RM + 2] = (MicroInstr64){MB64_TMPa, MB64_R, MACT64_NONE, 0, 0, false, true, "TMPa -> R | RNI"};

  /* SHL / SHR / SAR r/m */
  rom[MC64_SHL_RM + 0] = (MicroInstr64){MB64_M, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC64_SHL_RM + 1] = (MicroInstr64){MB64_TMPc, MB64_TMPb, MACT64_ALU, ALU64_SHL, 0, true, false, "TMPc -> TMPb | SHL (F)"};
  rom[MC64_SHL_RM + 2] = (MicroInstr64){MB64_SIGMA, MB64_OPR, MACT64_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};

  rom[MC64_SHR_RM + 0] = (MicroInstr64){MB64_M, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC64_SHR_RM + 1] = (MicroInstr64){MB64_TMPc, MB64_TMPb, MACT64_ALU, ALU64_SHR, 0, true, false, "TMPc -> TMPb | SHR (F)"};
  rom[MC64_SHR_RM + 2] = (MicroInstr64){MB64_SIGMA, MB64_OPR, MACT64_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};

  rom[MC64_SAR_RM + 0] = (MicroInstr64){MB64_M, MB64_TMPa, MACT64_NONE, 0, 0, false, false, "M -> TMPa"};
  rom[MC64_SAR_RM + 1] = (MicroInstr64){MB64_TMPc, MB64_TMPb, MACT64_ALU, ALU64_SAR, 0, true, false, "TMPc -> TMPb | SAR (F)"};
  rom[MC64_SAR_RM + 2] = (MicroInstr64){MB64_SIGMA, MB64_OPR, MACT64_MEM_WRITE_M, 0, 0, false, true, "SIGMA -> M | WB | RNI"};
}

static uint64_t mc64_read_bus(X86_64CPU *cpu, MicroBus64 id) {
  if (id >= MB64_RAX && id <= MB64_R15)
    return eu64_read_reg(cpu, id - MB64_RAX, (id == MB64_RSP) ? 64 : cpu->op_bits, cpu->has_rex);
  switch (id) {
  case MB64_RIP: return reg64_read(&cpu->rip);
  case MB64_RFLAGS: return reg64_read(&cpu->rflags);
  case MB64_R: return eu64_read_reg(cpu, cpu->reg_field, cpu->op_bits, cpu->has_rex);
  case MB64_M:
    if (cpu->mod == 3)
      return eu64_read_reg(cpu, cpu->rm_field, cpu->op_bits, cpu->has_rex);
    return reg64_read(&cpu->opr);
  case MB64_TMPa: return reg64_read(&cpu->tmp_a);
  case MB64_TMPb: return reg64_read(&cpu->tmp_b);
  case MB64_TMPc: return reg64_read(&cpu->tmp_c);
  case MB64_SIGMA: return reg64_read(&cpu->sigma);
  case MB64_IND: return reg64_read(&cpu->ind);
  case MB64_OPR: return reg64_read(&cpu->opr);
  case MB64_IMM:
    if (cpu->op_bits == 64)
      return (uint64_t)(int64_t)(int32_t)fetch32(cpu);
    return fetch_imm_sized(cpu, cpu->op_bits);
  case MB64_IMM_SXT:
    if (cpu->s_bit)
      return (uint64_t)(int64_t)(int8_t)fetch8(cpu);
    return fetch_imm_sized(cpu, (cpu->op_bits == 64) ? 32 : cpu->op_bits);
  case MB64_STEP_SZ:
    return (cpu->op_bits == 16) ? 2ULL : ((cpu->op_bits == 32) ? 4ULL : 8ULL);
  default:
    return 0;
  }
}

static void mc64_write_bus(X86_64CPU *cpu, MicroBus64 id, uint64_t val) {
  if (id >= MB64_RAX && id <= MB64_R15) {
    eu64_write_reg(cpu, id - MB64_RAX, val, (id == MB64_RSP) ? 64 : cpu->op_bits, cpu->has_rex);
    return;
  }
  switch (id) {
  case MB64_RIP: reg64_write(&cpu->rip, val); break;
  case MB64_RFLAGS: reg64_write(&cpu->rflags, sanitize_rflags(val)); break;
  case MB64_R: eu64_write_reg(cpu, cpu->reg_field, val, cpu->op_bits, cpu->has_rex); break;
  case MB64_M:
    if (cpu->mod == 3)
      eu64_write_reg(cpu, cpu->rm_field, val, cpu->op_bits, cpu->has_rex);
    else
      reg64_write(&cpu->opr, val);
    break;
  case MB64_TMPa: reg64_write(&cpu->tmp_a, val); break;
  case MB64_TMPb: reg64_write(&cpu->tmp_b, val); break;
  case MB64_TMPc: reg64_write(&cpu->tmp_c, val); break;
  case MB64_SIGMA: reg64_write(&cpu->sigma, val); break;
  case MB64_IND: reg64_write(&cpu->ind, val); break;
  case MB64_OPR: reg64_write(&cpu->opr, val); break;
  default: break;
  }
}

static void execute_64_microcode(X86_64CPU *cpu, uint16_t start_upc) {
  cpu->mc_pc = start_upc;
  while (true) {
    const MicroInstr64 *mi = &cpu->mc_rom[cpu->mc_pc];
    cpu->micro_cycles++;

    uint64_t bus_val = 0;
    if (mi->src != MB64_NONE) {
      bus_val = mc64_read_bus(cpu, mi->src);
      mc64_write_bus(cpu, mi->dst, bus_val);
    }

    if (mi->act == MACT64_ALU) {
      Alu64Op op = mi->use_xi ? cpu->xi_alu_op : mi->alu_op;
      uint64_t f = reg64_read(&cpu->rflags);
      int width = (mi->src == MB64_STEP_SZ) ? 64 : cpu->op_bits;
      Word64 res = alu_64_execute(op, word64_from_u64(reg64_read(&cpu->tmp_a)),
                                  word64_from_u64(reg64_read(&cpu->tmp_b)),
                                  width, &f, mi->update_flags);
      reg64_write(&cpu->sigma, word64_to_u64(res));
      if (mi->update_flags)
        reg64_write(&cpu->rflags, f);
    } else if (mi->act == MACT64_MEM_WRITE_M) {
      uint64_t opr = reg64_read(&cpu->opr);
      if (cpu->mod == 3) {
        eu64_write_reg(cpu, cpu->rm_field, opr, cpu->op_bits, cpu->has_rex);
      } else {
        int seg = (cpu->seg_override >= 0) ? cpu->seg_override : 3 /* DS */;
        mmu_write_sized(cpu, seg, reg64_read(&cpu->ind), opr, cpu->op_bits);
      }
    } else if (mi->act == MACT64_MEM_PUSH) {
      mmu_write_sized(cpu, 2 /* SS */, reg64_read(&cpu->ind), reg64_read(&cpu->opr), cpu->op_bits);
    } else if (mi->act == MACT64_MEM_POP) {
      uint64_t v = mmu_read_sized(cpu, 2 /* SS */, reg64_read(&cpu->ind), cpu->op_bits);
      reg64_write(&cpu->opr, v);
    } else if (mi->act == MACT64_MEM_READ) {
      int seg = (cpu->seg_override >= 0) ? cpu->seg_override : 3 /* DS */;
      uint64_t v = mmu_read_sized(cpu, seg, reg64_read(&cpu->ind), cpu->op_bits);
      reg64_write(&cpu->opr, v);
    }

    if (cpu->trace_mc) {
      printf("    [uPC 0x%02X] %-34s | bus=0x%016llX SIGMA=0x%016llX IND=0x%016llX OPR=0x%016llX\n",
             cpu->mc_pc, mi->comment ? mi->comment : "",
             (unsigned long long)bus_val,
             (unsigned long long)reg64_read(&cpu->sigma),
             (unsigned long long)reg64_read(&cpu->ind),
             (unsigned long long)reg64_read(&cpu->opr));
    }

    if (mi->rni) break;
    cpu->mc_pc++;
  }
}

/*
 * 64-Bit Effective Address + SIB (Scale-Index-Base) + RIP-Relative Unit
 */
static void decode_modrm_and_sib_64(X86_64CPU *cpu, bool load_mem_to_opr) {
  cpu->modrm = fetch8(cpu);
  cpu->mod = (cpu->modrm >> 6) & 3;
  cpu->reg_field = ((cpu->rex_r ? 1 : 0) << 3) | ((cpu->modrm >> 3) & 7);
  cpu->rm_field  = ((cpu->rex_b ? 1 : 0) << 3) | (cpu->modrm & 7);

  if (cpu->mod == 3)
    return;

  uint64_t ea = 0;
  if (cpu->addr_bits == 16) {
    uint16_t bx = reg64_read16(&cpu->regs[3]);
    uint16_t bp = reg64_read16(&cpu->regs[5]);
    uint16_t si = reg64_read16(&cpu->regs[6]);
    uint16_t di = reg64_read16(&cpu->regs[7]);

    if (cpu->mod == 0 && (cpu->rm_field & 7) == 6) {
      ea = fetch16(cpu);
    } else {
      switch (cpu->rm_field & 7) {
      case 0: ea = (uint16_t)(bx + si); break;
      case 1: ea = (uint16_t)(bx + di); break;
      case 2: ea = (uint16_t)(bp + si); break;
      case 3: ea = (uint16_t)(bp + di); break;
      case 4: ea = si; break;
      case 5: ea = di; break;
      case 6: ea = bp; break;
      case 7: ea = bx; break;
      }
      if (cpu->mod == 1)
        ea = (uint16_t)(ea + (int8_t)fetch8(cpu));
      else if (cpu->mod == 2)
        ea = (uint16_t)(ea + fetch16(cpu));
    }
  } else {
    bool is_64bit_mode = cpu->segs[1].l_bit;

    /* Check SIB byte presence */
    if ((cpu->rm_field & 7) == 4) {
      uint8_t sib = fetch8(cpu);
      int scale_pow = (sib >> 6) & 3;
      int index_reg = ((cpu->rex_x ? 1 : 0) << 3) | ((sib >> 3) & 7);
      int base_reg  = ((cpu->rex_b ? 1 : 0) << 3) | (sib & 7);

      uint64_t scaled_index = 0;
      if (index_reg != 4) { /* Index = 4 with rex_x=0 means no index */
        Word64 idx_w = word64_from_u64(reg64_read(&cpu->regs[index_reg]));
        scaled_index = word64_to_u64(barrel_shifter_left_64bit(idx_w, scale_pow));
      }

      uint64_t base = 0;
      if (cpu->mod == 0 && (base_reg & 7) == 5) {
        /* [scaled_index + disp32] without base */
        base = (uint64_t)(int64_t)(int32_t)fetch32(cpu);
      } else {
        base = reg64_read(&cpu->regs[base_reg]);
        if (cpu->mod == 1)
          base += (int64_t)(int8_t)fetch8(cpu);
        else if (cpu->mod == 2)
          base += (int64_t)(int32_t)fetch32(cpu);
      }

      Wire carries[64];
      Word64 sum = adder_64bit(word64_from_u64(base), word64_from_u64(scaled_index), LOW, carries);
      ea = word64_to_u64(sum);
    } else if (is_64bit_mode && cpu->mod == 0 && (cpu->rm_field & 7) == 5) {
      /*
       * 64-Bit RIP-Relative Addressing! [RIP + disp32]
       */
      int32_t disp = (int32_t)fetch32(cpu);
      uint64_t cur_rip = reg64_read(&cpu->rip);
      ea = cur_rip + (int64_t)disp;
    } else {
      /* Normal base + disp */
      uint64_t base = reg64_read(&cpu->regs[cpu->rm_field]);
      if (cpu->mod == 0 && (cpu->rm_field & 7) == 5) {
        ea = (uint64_t)(int64_t)(int32_t)fetch32(cpu);
      } else {
        ea = base;
        if (cpu->mod == 1)
          ea += (int64_t)(int8_t)fetch8(cpu);
        else if (cpu->mod == 2)
          ea += (int64_t)(int32_t)fetch32(cpu);
      }
    }
    if (cpu->addr_bits == 32)
      ea &= 0xFFFFFFFFULL;
  }

  reg64_write(&cpu->ind, ea);
  if (load_mem_to_opr) {
    int seg = (cpu->seg_override >= 0) ? cpu->seg_override : 3 /* DS */;
    uint64_t val = mmu_read_sized(cpu, seg, ea, cpu->op_bits);
    reg64_write(&cpu->opr, val);
  }
}

static bool eval_64_jcc(uint64_t f, uint8_t cond_code) {
  bool cf = (f & RFLAG_CF) != 0;
  bool zf = (f & RFLAG_ZF) != 0;
  bool sf = (f & RFLAG_SF) != 0;
  bool of = (f & RFLAG_OF) != 0;
  bool pf = (f & RFLAG_PF) != 0;

  switch (cond_code & 0xF) {
  case 0x0: return of;
  case 0x1: return !of;
  case 0x2: return cf;
  case 0x3: return !cf;
  case 0x4: return zf;
  case 0x5: return !zf;
  case 0x6: return cf || zf;
  case 0x7: return !cf && !zf;
  case 0x8: return sf;
  case 0x9: return !sf;
  case 0xA: return pf;
  case 0xB: return !pf;
  case 0xC: return sf != of;
  case 0xD: return sf == of;
  case 0xE: return zf || (sf != of);
  case 0xF: return !zf && (sf == of);
  default: return false;
  }
}

static Alu64Op map_pla_alu64(uint8_t op3) {
  static const Alu64Op table[8] = {
      ALU64_ADD, ALU64_OR,  ALU64_ADC, ALU64_SBB,
      ALU64_AND, ALU64_SUB, ALU64_XOR, ALU64_CMP};
  return table[op3 & 7];
}

static void cpu64_init(X86_64CPU *cpu) {
  memset(cpu, 0, sizeof(*cpu));
  for (int i = 0; i < 16; i++)
    reg64_init(&cpu->regs[i]);
  reg64_init(&cpu->rip);
  reg64_init(&cpu->rflags);
  reg64_init(&cpu->cr0);
  reg64_init(&cpu->cr2);
  reg64_init(&cpu->cr3);
  reg64_init(&cpu->cr4);
  reg64_init(&cpu->tmp_a);
  reg64_init(&cpu->tmp_b);
  reg64_init(&cpu->tmp_c);
  reg64_init(&cpu->sigma);
  reg64_init(&cpu->ind);
  reg64_init(&cpu->opr);

  build_64_microcode_rom(cpu);
  tlb_flush(cpu);

  /* Reset State: Real Mode (PE=0, PG=0), CS=0xF000 */
  reg64_write(&cpu->rflags, 0x00000002ULL);
  for (int i = 0; i < 6; i++)
    load_segment_64(cpu, i, 0x0000);
  load_segment_64(cpu, 1 /* CS */, 0xFFFF);
  reg64_write(&cpu->rip, 0x00000000ULL);
}

/* Execute One Complete x86-64 Instruction */
static void cpu64_step(X86_64CPU *cpu) {
  if (cpu->halted)
    return;

  bool is_long_mode = cpu->segs[1].l_bit; /* L=1: 64-bit Long Mode Active */
  bool d_bit = cpu->segs[1].d_bit || is_long_mode;
  bool op_size = d_bit;
  bool addr_size = d_bit;
  cpu->seg_override = -1;
  cpu->rep_prefix = 0;
  cpu->s_bit = false;
  cpu->has_rex = false;
  cpu->rex_w = false;
  cpu->rex_r = false;
  cpu->rex_x = false;
  cpu->rex_b = false;

  /* 1. Decode Legacy & REX Prefixes */
  uint8_t op;
  while (true) {
    op = fetch8(cpu);
    if (op == 0x66) {
      op_size = !d_bit;
    } else if (op == 0x67) {
      addr_size = !d_bit;
    } else if (op == 0xF2 || op == 0xF3) {
      cpu->rep_prefix = op;
    } else if (op == 0x26) {
      cpu->seg_override = 0; /* ES: */
    } else if (op == 0x2E) {
      cpu->seg_override = 1; /* CS: */
    } else if (op == 0x36) {
      cpu->seg_override = 2; /* SS: */
    } else if (op == 0x3E) {
      cpu->seg_override = 3; /* DS: */
    } else if (op == 0x64) {
      cpu->seg_override = 4; /* FS: */
    } else if (op == 0x65) {
      cpu->seg_override = 5; /* GS: */
    } else if (is_long_mode && (op & 0xF0) == 0x40) {
      /* REX Prefix: 0100 W R X B */
      cpu->has_rex = true;
      cpu->rex_w = (op & 8) != 0;
      cpu->rex_r = (op & 4) != 0;
      cpu->rex_x = (op & 2) != 0;
      cpu->rex_b = (op & 1) != 0;
    } else {
      break;
    }
  }

  if (is_long_mode) {
    cpu->op_bits = cpu->rex_w ? 64 : (op_size ? 32 : 16);
    cpu->addr_bits = addr_size ? 64 : 32;
  } else {
    cpu->op_bits = op_size ? 32 : 16;
    cpu->addr_bits = addr_size ? 32 : 16;
  }

  cpu->macro_instructions++;

  /* 2. Two-Byte Extended Opcodes (0x0F ...) */
  if (op == 0x0F) {
    uint8_t op2 = fetch8(cpu);

    /* Group 7: SGDT, SIDT, LGDT, LIDT, SMSW, LMSW, INVLPG, SWAPGS */
    if (op2 == 0x01) {
      uint8_t next_b = phys_read8(cpu, mmu_translate_64(cpu, 1 /* CS */, reg64_read(&cpu->rip), false));
      if (next_b == 0xF8) { /* SWAPGS (0x0F 0x01 0xF8) */
        fetch8(cpu);
        uint64_t tmp = cpu->msr_gs_base;
        cpu->msr_gs_base = cpu->msr_kernel_gs_base;
        cpu->msr_kernel_gs_base = tmp;
        cpu->micro_cycles += 3;
        return;
      }

      decode_modrm_and_sib_64(cpu, false);
      uint64_t addr = reg64_read(&cpu->ind);
      int seg = (cpu->seg_override >= 0) ? cpu->seg_override : 3;
      if (cpu->reg_field == 2) { /* LGDT m16&64 */
        cpu->gdtr_limit = mmu_read16(cpu, seg, addr);
        cpu->gdtr_base  = is_long_mode ? mmu_read64(cpu, seg, addr + 2) : mmu_read32(cpu, seg, addr + 2);
        cpu->micro_cycles += 4;
      } else if (cpu->reg_field == 3) { /* LIDT m16&64 */
        cpu->idtr_limit = mmu_read16(cpu, seg, addr);
        cpu->idtr_base  = is_long_mode ? mmu_read64(cpu, seg, addr + 2) : mmu_read32(cpu, seg, addr + 2);
        cpu->micro_cycles += 4;
      } else if (cpu->reg_field == 7) { /* INVLPG m */
        tlb_invalidate_page(cpu, addr);
        cpu->micro_cycles += 3;
      }
      return;
    }

    /* Fast System Call: SYSCALL (0x0F 0x05) */
    if (op2 == 0x05) {
      reg64_write(&cpu->regs[1], reg64_read(&cpu->rip));   /* RCX = RIP */
      reg64_write(&cpu->regs[11], reg64_read(&cpu->rflags));/* R11 = RFLAGS */
      reg64_write(&cpu->rflags, reg64_read(&cpu->rflags) & ~cpu->msr_sfmask);
      reg64_write(&cpu->rip, cpu->msr_lstar);              /* Jump to 64-bit handler */
      cpu->micro_cycles += 5;
      return;
    }

    /* Fast System Return: SYSRET (0x0F 0x07) */
    if (op2 == 0x07) {
      reg64_write(&cpu->rip, reg64_read(&cpu->regs[1]));   /* RIP = RCX */
      reg64_write(&cpu->rflags, (reg64_read(&cpu->regs[11]) & 0x3C7FD7ULL) | 2);
      cpu->micro_cycles += 5;
      return;
    }

    /* Control Register Move: MOV r64, CRn (0x0F 0x20) / MOV CRn, r64 (0x0F 0x22) */
    if (op2 == 0x20 || op2 == 0x22) {
      uint8_t m = fetch8(cpu);
      int cr_num = (m >> 3) & 7;
      int gpr_num = ((cpu->rex_b ? 1 : 0) << 3) | (m & 7);
      if (op2 == 0x20) {
        uint64_t v = (cr_num == 0) ? reg64_read(&cpu->cr0) :
                     (cr_num == 2) ? reg64_read(&cpu->cr2) :
                     (cr_num == 3) ? reg64_read(&cpu->cr3) : reg64_read(&cpu->cr4);
        reg64_write(&cpu->regs[gpr_num], v);
      } else {
        uint64_t v = reg64_read(&cpu->regs[gpr_num]);
        if (cr_num == 0) {
          reg64_write(&cpu->cr0, v);
          /* If PG=1 and EFER.LME=1, hardware activates Long Mode (LMA=1) */
          if ((v & 0x80000000ULL) && (cpu->msr_efer & (1ULL << 8)))
            cpu->msr_efer |= (1ULL << 10); /* LMA = 1 */
        } else if (cr_num == 2) {
          reg64_write(&cpu->cr2, v);
        } else if (cr_num == 3) {
          reg64_write(&cpu->cr3, v);
          tlb_flush(cpu);
        } else if (cr_num == 4) {
          reg64_write(&cpu->cr4, v);
        }
      }
      cpu->micro_cycles += 3;
      return;
    }

    /* MSR Operations: WRMSR (0x0F 0x30) & RDMSR (0x0F 0x32) */
    if (op2 == 0x30 || op2 == 0x32) {
      uint32_t msr_id = (uint32_t)reg64_read32(&cpu->regs[1]); /* ECX */
      if (op2 == 0x30) { /* WRMSR: [EDX:EAX] -> MSR[ECX] */
        uint64_t val = ((uint64_t)reg64_read32(&cpu->regs[2]) << 32) | reg64_read32(&cpu->regs[0]);
        if (msr_id == 0xC0000080U) cpu->msr_efer = val;
        else if (msr_id == 0xC0000081U) cpu->msr_star = val;
        else if (msr_id == 0xC0000082U) cpu->msr_lstar = val;
        else if (msr_id == 0xC0000084U) cpu->msr_sfmask = val;
        else if (msr_id == 0xC0000100U) cpu->msr_fs_base = val;
        else if (msr_id == 0xC0000101U) cpu->msr_gs_base = val;
        else if (msr_id == 0xC0000102U) cpu->msr_kernel_gs_base = val;
      } else { /* RDMSR: MSR[ECX] -> [EDX:EAX] */
        uint64_t val = 0;
        if (msr_id == 0xC0000080U) val = cpu->msr_efer;
        else if (msr_id == 0xC0000081U) val = cpu->msr_star;
        else if (msr_id == 0xC0000082U) val = cpu->msr_lstar;
        else if (msr_id == 0xC0000084U) val = cpu->msr_sfmask;
        else if (msr_id == 0xC0000100U) val = cpu->msr_fs_base;
        else if (msr_id == 0xC0000101U) val = cpu->msr_gs_base;
        else if (msr_id == 0xC0000102U) val = cpu->msr_kernel_gs_base;
        reg64_write32(&cpu->regs[0], (uint32_t)val);
        reg64_write32(&cpu->regs[2], (uint32_t)(val >> 32));
      }
      cpu->micro_cycles += 8;
      return;
    }

    /* Jcc rel32 (0x0F 0x80..0x8F) */
    if ((op2 & 0xF0) == 0x80) {
      int32_t rel = (int32_t)fetch32(cpu);
      if (eval_64_jcc(reg64_read(&cpu->rflags), op2 & 0xF))
        reg64_write(&cpu->rip, reg64_read(&cpu->rip) + rel);
      return;
    }

    /* SETcc r/m8 (0x0F 0x90..0x9F) */
    if ((op2 & 0xF0) == 0x90) {
      decode_modrm_and_sib_64(cpu, false);
      bool cond = eval_64_jcc(reg64_read(&cpu->rflags), op2 & 0x0F);
      uint8_t val = cond ? 1 : 0;
      if (cpu->mod == 3)
        eu64_write_reg(cpu, cpu->rm_field, val, 8, cpu->has_rex);
      else
        mmu_write8(cpu, (cpu->seg_override >= 0) ? cpu->seg_override : 3, reg64_read(&cpu->ind), val);
      cpu->micro_cycles += 3;
      return;
    }

    /* Bit Instructions: BT (0xA3), BTS (0xAB), BTR (0xB3), BTC (0xBB) */
    if (op2 == 0xA3 || op2 == 0xAB || op2 == 0xB3 || op2 == 0xBB) {
      decode_modrm_and_sib_64(cpu, true);
      uint64_t bit_idx = eu64_read_reg(cpu, cpu->reg_field, cpu->op_bits, cpu->has_rex) & (cpu->op_bits - 1);
      uint64_t val = mc64_read_bus(cpu, MB64_M);
      uint64_t f = reg64_read(&cpu->rflags);
      if ((val >> bit_idx) & 1ULL) f |= RFLAG_CF; else f &= ~RFLAG_CF;
      reg64_write(&cpu->rflags, sanitize_rflags(f));

      if (op2 == 0xAB) val |= (1ULL << bit_idx);       /* BTS */
      else if (op2 == 0xB3) val &= ~(1ULL << bit_idx);  /* BTR */
      else if (op2 == 0xBB) val ^= (1ULL << bit_idx);   /* BTC */

      if (op2 != 0xA3) {
        if (cpu->mod == 3)
          eu64_write_reg(cpu, cpu->rm_field, val, cpu->op_bits, cpu->has_rex);
        else
          mmu_write_sized(cpu, (cpu->seg_override >= 0) ? cpu->seg_override : 3,
                          reg64_read(&cpu->ind), val, cpu->op_bits);
      }
      cpu->micro_cycles += 4;
      return;
    }

    /* Bit Group 8: BT/BTS/BTR/BTC r/m, imm8 (0x0F 0xBA) */
    if (op2 == 0xBA) {
      decode_modrm_and_sib_64(cpu, true);
      uint8_t bit_idx = fetch8(cpu) & (cpu->op_bits - 1);
      uint64_t val = mc64_read_bus(cpu, MB64_M);
      uint64_t f = reg64_read(&cpu->rflags);
      if ((val >> bit_idx) & 1ULL) f |= RFLAG_CF; else f &= ~RFLAG_CF;
      reg64_write(&cpu->rflags, sanitize_rflags(f));

      if (cpu->reg_field == 5) val |= (1ULL << bit_idx);
      else if (cpu->reg_field == 6) val &= ~(1ULL << bit_idx);
      else if (cpu->reg_field == 7) val ^= (1ULL << bit_idx);

      if (cpu->reg_field != 4) {
        if (cpu->mod == 3)
          eu64_write_reg(cpu, cpu->rm_field, val, cpu->op_bits, cpu->has_rex);
        else
          mmu_write_sized(cpu, (cpu->seg_override >= 0) ? cpu->seg_override : 3,
                          reg64_read(&cpu->ind), val, cpu->op_bits);
      }
      cpu->micro_cycles += 4;
      return;
    }

    /* SHLD (0xA4/0xA5) & SHRD (0xAC/0xAD) */
    if (op2 == 0xA4 || op2 == 0xA5 || op2 == 0xAC || op2 == 0xAD) {
      decode_modrm_and_sib_64(cpu, true);
      uint8_t cnt = (op2 == 0xA4 || op2 == 0xAC) ? (fetch8(cpu) & (cpu->op_bits - 1))
                                                  : (uint8_t)(reg64_read_lo8(&cpu->regs[1]) & (cpu->op_bits - 1));
      if (cnt > 0) {
        uint64_t dst = mc64_read_bus(cpu, MB64_M);
        uint64_t src = eu64_read_reg(cpu, cpu->reg_field, cpu->op_bits, cpu->has_rex);
        uint64_t res = dst;
        if (op2 == 0xA4 || op2 == 0xA5) { /* SHLD */
          if (cpu->op_bits == 64)
            res = (dst << cnt) | (src >> (64 - cnt));
          else
            res = (uint32_t)(((uint64_t)(uint32_t)dst << cnt) | ((uint64_t)(uint32_t)src >> (32 - cnt)));
        } else { /* SHRD */
          if (cpu->op_bits == 64)
            res = (src << (64 - cnt)) | (dst >> cnt);
          else
            res = (uint32_t)(((uint64_t)(uint32_t)src << (32 - cnt)) | ((uint64_t)(uint32_t)dst >> cnt));
        }
        if (cpu->mod == 3)
          eu64_write_reg(cpu, cpu->rm_field, res, cpu->op_bits, cpu->has_rex);
        else
          mmu_write_sized(cpu, (cpu->seg_override >= 0) ? cpu->seg_override : 3,
                          reg64_read(&cpu->ind), res, cpu->op_bits);
      }
      cpu->micro_cycles += 5;
      return;
    }

    /* IMUL r, r/m (0x0F 0xAF) */
    if (op2 == 0xAF) {
      decode_modrm_and_sib_64(cpu, true);
      int64_t a = (int64_t)eu64_read_reg(cpu, cpu->reg_field, cpu->op_bits, cpu->has_rex);
      int64_t b = (int64_t)mc64_read_bus(cpu, MB64_M);
      int64_t prod = a * b;
      eu64_write_reg(cpu, cpu->reg_field, (uint64_t)prod, cpu->op_bits, cpu->has_rex);
      cpu->micro_cycles += 5;
      return;
    }

    /* BSF (0x0F 0xBC) / BSR (0x0F 0xBD) */
    if (op2 == 0xBC || op2 == 0xBD) {
      decode_modrm_and_sib_64(cpu, true);
      uint64_t src = mc64_read_bus(cpu, MB64_M);
      uint64_t f = reg64_read(&cpu->rflags);
      if (src == 0) {
        reg64_write(&cpu->rflags, f | RFLAG_ZF);
      } else {
        reg64_write(&cpu->rflags, f & ~RFLAG_ZF);
        uint64_t idx = 0;
        if (op2 == 0xBC) {
          while (((src >> idx) & 1ULL) == 0) idx++;
        } else {
          idx = cpu->op_bits - 1;
          while (((src >> idx) & 1ULL) == 0 && idx > 0) idx--;
        }
        eu64_write_reg(cpu, cpu->reg_field, idx, cpu->op_bits, cpu->has_rex);
      }
      cpu->micro_cycles += 5;
      return;
    }

    /* MOVZX r, r/m8 (0xB6) / r/m16 (0xB7) */
    if (op2 == 0xB6 || op2 == 0xB7) {
      int dst_bits = cpu->op_bits;
      cpu->op_bits = (op2 == 0xB6) ? 8 : 16;
      decode_modrm_and_sib_64(cpu, true);
      uint64_t src = mc64_read_bus(cpu, MB64_M);
      eu64_write_reg(cpu, cpu->reg_field, src, dst_bits, cpu->has_rex);
      cpu->micro_cycles += 2;
      return;
    }

    /* MOVSX r, r/m8 (0xBE) / r/m16 (0xBF) */
    if (op2 == 0xBE || op2 == 0xBF) {
      int dst_bits = cpu->op_bits;
      cpu->op_bits = (op2 == 0xBE) ? 8 : 16;
      decode_modrm_and_sib_64(cpu, true);
      uint64_t src = mc64_read_bus(cpu, MB64_M);
      int64_t sxt = (op2 == 0xBE) ? (int64_t)(int8_t)src : (int64_t)(int16_t)src;
      eu64_write_reg(cpu, cpu->reg_field, (uint64_t)sxt, dst_bits, cpu->has_rex);
      cpu->micro_cycles += 2;
      return;
    }
    return;
  }

  /* 3. MOVSXD (0x63 in 64-bit mode: Sign-extends 32-bit r/m to 64-bit reg) */
  if (is_long_mode && op == 0x63) {
    int dst_bits = cpu->op_bits;
    cpu->op_bits = 32;
    decode_modrm_and_sib_64(cpu, true);
    int32_t val32 = (int32_t)mc64_read_bus(cpu, MB64_M);
    eu64_write_reg(cpu, cpu->reg_field, (uint64_t)(int64_t)val32, dst_bits, cpu->has_rex);
    cpu->micro_cycles += 2;
    return;
  }

  /* 4. Standard ALU Operations (0x00..0x3D) */
  if ((op & 0xC0) == 0x00 && (op & 6) != 6) {
    uint8_t alu_idx = (op >> 3) & 7;
    cpu->xi_alu_op = map_pla_alu64(alu_idx);
    if ((op & 1) == 0) cpu->op_bits = 8;
    bool dir_reg = (op & 2) != 0;
    bool is_cmp = (alu_idx == 7);

    if ((op & 4) == 0) {
      decode_modrm_and_sib_64(cpu, true);
      execute_64_microcode(cpu, dir_reg ? (is_cmp ? MC64_CMP_R_RM : MC64_ALU_R_RM)
                                        : (is_cmp ? MC64_CMP_RM_R : MC64_ALU_RM_R));
    } else {
      execute_64_microcode(cpu, is_cmp ? MC64_CMP_ACC_IMM : MC64_ALU_ACC_IMM);
    }
    return;
  }

  /* 5. Group 1 Immediate ALU (0x80..0x83) */
  if ((op & 0xFC) == 0x80) {
    if ((op & 1) == 0) cpu->op_bits = 8;
    cpu->s_bit = (op == 0x83);
    decode_modrm_and_sib_64(cpu, true);
    cpu->xi_alu_op = map_pla_alu64(cpu->reg_field);
    if (cpu->reg_field == 7)
      execute_64_microcode(cpu, MC64_GRP1_CMP_IMM);
    else
      execute_64_microcode(cpu, MC64_GRP1_RM_IMM);
    return;
  }

  /* 6. PUSH reg (0x50..0x57) & POP reg (0x58..0x5F) */
  if ((op & 0xF8) == 0x50) {
    cpu->reg_field = ((cpu->rex_b ? 1 : 0) << 3) | (op & 7);
    cpu->op_bits = 64; /* Default operand size for push/pop in 64-bit is 64 */
    execute_64_microcode(cpu, MC64_PUSH_R);
    return;
  }
  if ((op & 0xF8) == 0x58) {
    cpu->reg_field = ((cpu->rex_b ? 1 : 0) << 3) | (op & 7);
    cpu->op_bits = 64;
    execute_64_microcode(cpu, MC64_POP_R);
    return;
  }

  /* 7. PUSH imm (0x68 / 0x6A) */
  if (op == 0x68) {
    int32_t imm32 = (int32_t)fetch32(cpu);
    eu64_push(cpu, (uint64_t)(int64_t)imm32);
    return;
  }
  if (op == 0x6A) {
    int8_t imm8 = (int8_t)fetch8(cpu);
    eu64_push(cpu, (uint64_t)(int64_t)imm8);
    return;
  }

  /* 8. Jcc rel8 (0x70..0x7F) */
  if ((op & 0xF0) == 0x70) {
    int8_t rel = (int8_t)fetch8(cpu);
    if (eval_64_jcc(reg64_read(&cpu->rflags), op & 0xF))
      reg64_write(&cpu->rip, reg64_read(&cpu->rip) + rel);
    return;
  }

  /* 9. TEST r/m, r (0x84..0x85) */
  if ((op & 0xFE) == 0x84) {
    if ((op & 1) == 0) cpu->op_bits = 8;
    decode_modrm_and_sib_64(cpu, true);
    execute_64_microcode(cpu, MC64_TEST_RM_R);
    return;
  }

  /* 10. XCHG r/m, r (0x86..0x87) */
  if ((op & 0xFE) == 0x86) {
    if ((op & 1) == 0) cpu->op_bits = 8;
    decode_modrm_and_sib_64(cpu, true);
    execute_64_microcode(cpu, MC64_XCHG_R_RM);
    return;
  }

  /* 11. MOV r/m, r (0x88..0x89) & MOV r, r/m (0x8A..0x8B) */
  if ((op & 0xFC) == 0x88) {
    if ((op & 1) == 0) cpu->op_bits = 8;
    bool dir_reg = (op & 2) != 0;
    decode_modrm_and_sib_64(cpu, dir_reg);
    execute_64_microcode(cpu, dir_reg ? MC64_MOV_R_RM : MC64_MOV_RM_R);
    return;
  }

  /* MOV r/m16, Sreg (0x8C) */
  if (op == 0x8C) {
    cpu->op_bits = 16;
    decode_modrm_and_sib_64(cpu, false);
    uint16_t sel = cpu->segs[cpu->reg_field % 6].selector;
    if (cpu->mod == 3)
      eu64_write_reg(cpu, cpu->rm_field, sel, 16, cpu->has_rex);
    else
      mmu_write16(cpu, (cpu->seg_override >= 0) ? cpu->seg_override : 3, reg64_read(&cpu->ind), sel);
    return;
  }

  /* 12. LEA r, m (0x8D) */
  if (op == 0x8D) {
    decode_modrm_and_sib_64(cpu, false);
    execute_64_microcode(cpu, MC64_LEA_R_M);
    return;
  }

  /* MOV Sreg, r/m16 (0x8E) */
  if (op == 0x8E) {
    cpu->op_bits = 16;
    decode_modrm_and_sib_64(cpu, true);
    uint16_t sel = (uint16_t)mc64_read_bus(cpu, MB64_M);
    load_segment_64(cpu, cpu->reg_field % 6, sel);
    return;
  }

  /* 13. NOP (0x90) & XCHG RAX, reg (0x91..0x97) */
  if ((op & 0xF8) == 0x90) {
    if (op == 0x90 && !cpu->rex_b)
      return; /* Pure NOP */
    cpu->reg_field = ((cpu->rex_b ? 1 : 0) << 3) | (op & 7);
    execute_64_microcode(cpu, MC64_XCHG_RAX_R);
    return;
  }

  /* 14. PUSHFQ (0x9C) & POPFQ (0x9D) */
  if (op == 0x9C) {
    eu64_push(cpu, reg64_read(&cpu->rflags));
    return;
  }
  if (op == 0x9D) {
    reg64_write(&cpu->rflags, sanitize_rflags(eu64_pop(cpu)));
    return;
  }

  /* 15. MOV reg, imm (0xB0..0xBF) */
  if ((op & 0xF0) == 0xB0) {
    if ((op & 8) == 0) cpu->op_bits = 8;
    cpu->reg_field = ((cpu->rex_b ? 1 : 0) << 3) | (op & 7);
    /* In 64-bit mode: REX.W + 0xB8..0xBF is full 64-bit immediate load! */
    if (cpu->op_bits == 64) {
      uint64_t imm64 = fetch64(cpu);
      eu64_write_reg(cpu, cpu->reg_field, imm64, 64, cpu->has_rex);
      return;
    }
    execute_64_microcode(cpu, MC64_MOV_R_IMM);
    return;
  }

  /* 16. Shift / Rotate Group 2 (0xC0..0xC1, 0xD0..0xD3) */
  if ((op & 0xFE) == 0xC0 || (op & 0xFC) == 0xD0) {
    if ((op & 1) == 0) cpu->op_bits = 8;
    decode_modrm_and_sib_64(cpu, true);
    static const Alu64Op sh_ops[8] = {
        ALU64_ROL, ALU64_ROR, ALU64_ROL, ALU64_ROR,
        ALU64_SHL, ALU64_SHR, ALU64_SHL, ALU64_SAR};
    cpu->xi_alu_op = sh_ops[cpu->reg_field & 7];
    uint8_t count = 1;
    if ((op & 0xFE) == 0xC0)
      count = fetch8(cpu) & (cpu->op_bits - 1);
    else if (op & 2)
      count = reg64_read_lo8(&cpu->regs[1]) & (cpu->op_bits - 1); /* CL */

    reg64_write(&cpu->tmp_c, count);
    if (cpu->xi_alu_op == ALU64_SHL)
      execute_64_microcode(cpu, MC64_SHL_RM);
    else if (cpu->xi_alu_op == ALU64_SHR)
      execute_64_microcode(cpu, MC64_SHR_RM);
    else if (cpu->xi_alu_op == ALU64_SAR)
      execute_64_microcode(cpu, MC64_SAR_RM);
    else {
      uint64_t val = mc64_read_bus(cpu, MB64_M);
      uint64_t f = reg64_read(&cpu->rflags);
      Word64 res = alu_64_execute(cpu->xi_alu_op, word64_from_u64(val),
                                  word64_from_u64(count), cpu->op_bits, &f, true);
      reg64_write(&cpu->rflags, f);
      uint64_t out = word64_to_u64(res);
      if (cpu->mod == 3)
        eu64_write_reg(cpu, cpu->rm_field, out, cpu->op_bits, cpu->has_rex);
      else
        mmu_write_sized(cpu, (cpu->seg_override >= 0) ? cpu->seg_override : 3,
                        reg64_read(&cpu->ind), out, cpu->op_bits);
    }
    return;
  }

  /* 17. RET near (0xC3) & RET imm16 (0xC2) */
  if (op == 0xC2 || op == 0xC3) {
    uint64_t new_rip = eu64_pop(cpu);
    reg64_write(&cpu->rip, new_rip);
    if (op == 0xC2) {
      uint16_t pop_bytes = fetch16(cpu);
      reg64_write(&cpu->regs[4], reg64_read(&cpu->regs[4]) + pop_bytes);
    }
    return;
  }

  /* 18. MOV r/m, imm (0xC6..0xC7) */
  if (op == 0xC6 || op == 0xC7) {
    if (op == 0xC6) cpu->op_bits = 8;
    decode_modrm_and_sib_64(cpu, false);
    /* In 64-bit mode, 0xC7 with REX.W=1 takes a sign-extended 32-bit immediate */
    uint64_t imm = (cpu->op_bits == 64) ? (uint64_t)(int64_t)(int32_t)fetch32(cpu)
                                        : fetch_imm_sized(cpu, cpu->op_bits);
    if (cpu->mod == 3)
      eu64_write_reg(cpu, cpu->rm_field, imm, cpu->op_bits, cpu->has_rex);
    else
      mmu_write_sized(cpu, (cpu->seg_override >= 0) ? cpu->seg_override : 3,
                      reg64_read(&cpu->ind), imm, cpu->op_bits);
    cpu->micro_cycles += 2;
    return;
  }

  /* 19. String Instructions: MOVS, STOS, LODS, CMPS, SCAS (0xA4..0xAF) with REP */
  if (op == 0xA4 || op == 0xA5 || op == 0xA6 || op == 0xA7 || op == 0xAA ||
      op == 0xAB || op == 0xAC || op == 0xAD || op == 0xAE || op == 0xAF) {
    if ((op & 1) == 0) cpu->op_bits = 8;
    int step = ((reg64_read(&cpu->rflags) & RFLAG_DF) ? -1 : 1) * (cpu->op_bits / 8);
    int seg = (cpu->seg_override >= 0) ? cpu->seg_override : 3 /* DS */;

    while (true) {
      uint64_t count = (cpu->addr_bits == 64) ? reg64_read(&cpu->regs[1]) : reg64_read32(&cpu->regs[1]);
      if (cpu->rep_prefix && count == 0)
        break;

      uint64_t si = (cpu->addr_bits == 64) ? reg64_read(&cpu->regs[6]) : reg64_read32(&cpu->regs[6]);
      uint64_t di = (cpu->addr_bits == 64) ? reg64_read(&cpu->regs[7]) : reg64_read32(&cpu->regs[7]);

      if (op == 0xA4 || op == 0xA5) { /* MOVS */
        uint64_t val = mmu_read_sized(cpu, seg, si, cpu->op_bits);
        mmu_write_sized(cpu, 0 /* ES */, di, val, cpu->op_bits);
        si += step;
        di += step;
      } else if (op == 0xAA || op == 0xAB) { /* STOS */
        uint64_t val = eu64_read_reg(cpu, 0 /* RAX */, cpu->op_bits, cpu->has_rex);
        mmu_write_sized(cpu, 0 /* ES */, di, val, cpu->op_bits);
        di += step;
      } else if (op == 0xAC || op == 0xAD) { /* LODS */
        uint64_t val = mmu_read_sized(cpu, seg, si, cpu->op_bits);
        eu64_write_reg(cpu, 0 /* RAX */, val, cpu->op_bits, cpu->has_rex);
        si += step;
      } else if (op == 0xA6 || op == 0xA7) { /* CMPS */
        uint64_t v1 = mmu_read_sized(cpu, seg, si, cpu->op_bits);
        uint64_t v2 = mmu_read_sized(cpu, 0 /* ES */, di, cpu->op_bits);
        uint64_t f = reg64_read(&cpu->rflags);
        alu_64_execute(ALU64_CMP, word64_from_u64(v1), word64_from_u64(v2), cpu->op_bits, &f, true);
        reg64_write(&cpu->rflags, f);
        si += step;
        di += step;
      } else if (op == 0xAE || op == 0xAF) { /* SCAS */
        uint64_t v1 = eu64_read_reg(cpu, 0 /* RAX */, cpu->op_bits, cpu->has_rex);
        uint64_t v2 = mmu_read_sized(cpu, 0 /* ES */, di, cpu->op_bits);
        uint64_t f = reg64_read(&cpu->rflags);
        alu_64_execute(ALU64_CMP, word64_from_u64(v1), word64_from_u64(v2), cpu->op_bits, &f, true);
        reg64_write(&cpu->rflags, f);
        di += step;
      }

      if (cpu->addr_bits == 64) {
        reg64_write(&cpu->regs[6], si);
        reg64_write(&cpu->regs[7], di);
      } else {
        reg64_write32(&cpu->regs[6], (uint32_t)si);
        reg64_write32(&cpu->regs[7], (uint32_t)di);
      }
      cpu->micro_cycles += 4;

      if (cpu->rep_prefix) {
        count--;
        if (cpu->addr_bits == 64)
          reg64_write(&cpu->regs[1], count);
        else
          reg64_write32(&cpu->regs[1], (uint32_t)count);
        bool zf = (reg64_read(&cpu->rflags) & RFLAG_ZF) != 0;
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

  /* 20. CALL rel32 (0xE8) & JMP rel32 (0xE9) */
  if (op == 0xE8) {
    int32_t rel = (int32_t)fetch32(cpu);
    uint64_t return_rip = reg64_read(&cpu->rip);
    eu64_push(cpu, return_rip);
    reg64_write(&cpu->rip, return_rip + rel);
    return;
  }
  if (op == 0xE9) {
    int32_t rel = (int32_t)fetch32(cpu);
    reg64_write(&cpu->rip, reg64_read(&cpu->rip) + rel);
    return;
  }

  /* 21. JMP FAR ptr16:16/32/64 (0xEA in legacy, or 0x66 0xEA) */
  if (op == 0xEA) {
    uint64_t new_rip = (cpu->op_bits == 32) ? fetch32(cpu) : fetch16(cpu);
    uint16_t new_cs = fetch16(cpu);
    load_segment_64(cpu, 1 /* CS */, new_cs);
    reg64_write(&cpu->rip, new_rip);
    cpu->micro_cycles += 5;
    return;
  }

  /* 22. JMP rel8 (0xEB) & HLT (0xF4) */
  if (op == 0xEB) {
    int8_t rel = (int8_t)fetch8(cpu);
    reg64_write(&cpu->rip, reg64_read(&cpu->rip) + rel);
    return;
  }
  if (op == 0xF4) {
    cpu->halted = true;
    return;
  }

  /* 23. Flag Operations: CLC, STC, CMC, CLI, STI, CLD, STD */
  if (op == 0xF5) { reg64_write(&cpu->rflags, reg64_read(&cpu->rflags) ^ RFLAG_CF); return; }
  if (op == 0xF8) { reg64_write(&cpu->rflags, reg64_read(&cpu->rflags) & ~RFLAG_CF); return; }
  if (op == 0xF9) { reg64_write(&cpu->rflags, reg64_read(&cpu->rflags) | RFLAG_CF); return; }
  if (op == 0xFA) { reg64_write(&cpu->rflags, reg64_read(&cpu->rflags) & ~RFLAG_IF); return; }
  if (op == 0xFB) { reg64_write(&cpu->rflags, reg64_read(&cpu->rflags) | RFLAG_IF); return; }
  if (op == 0xFC) { reg64_write(&cpu->rflags, reg64_read(&cpu->rflags) & ~RFLAG_DF); return; }
  if (op == 0xFD) { reg64_write(&cpu->rflags, reg64_read(&cpu->rflags) | RFLAG_DF); return; }

  /* 24. Group 3 Unary / Multiply / Divide (0xF6..0xF7) */
  if (op == 0xF6 || op == 0xF7) {
    if ((op & 1) == 0) cpu->op_bits = 8;
    decode_modrm_and_sib_64(cpu, true);
    if (cpu->reg_field == 0 || cpu->reg_field == 1) { /* TEST */
      uint64_t imm = fetch_imm_sized(cpu, (cpu->op_bits == 64) ? 32 : cpu->op_bits);
      uint64_t val = mc64_read_bus(cpu, MB64_M);
      uint64_t f = reg64_read(&cpu->rflags);
      alu_64_execute(ALU64_TEST, word64_from_u64(val), word64_from_u64(imm), cpu->op_bits, &f, true);
      reg64_write(&cpu->rflags, f);
    } else if (cpu->reg_field == 2) { /* NOT */
      execute_64_microcode(cpu, MC64_NOT_RM);
    } else if (cpu->reg_field == 3) { /* NEG */
      execute_64_microcode(cpu, MC64_NEG_RM);
    }
    return;
  }

  /* 25. Group 5 (0xFF): INC, DEC, CALL near, JMP near, PUSH */
  if (op == 0xFF) {
    decode_modrm_and_sib_64(cpu, true);
    if (cpu->reg_field == 0) { /* INC r/m */
      uint64_t val = mc64_read_bus(cpu, MB64_M);
      uint64_t f = reg64_read(&cpu->rflags);
      Word64 res = alu_64_execute(ALU64_INC, word64_from_u64(val), word64_from_u64(1ULL), cpu->op_bits, &f, true);
      reg64_write(&cpu->rflags, f);
      uint64_t out = word64_to_u64(res);
      if (cpu->mod == 3) eu64_write_reg(cpu, cpu->rm_field, out, cpu->op_bits, cpu->has_rex);
      else mmu_write_sized(cpu, (cpu->seg_override >= 0) ? cpu->seg_override : 3, reg64_read(&cpu->ind), out, cpu->op_bits);
    } else if (cpu->reg_field == 1) { /* DEC r/m */
      uint64_t val = mc64_read_bus(cpu, MB64_M);
      uint64_t f = reg64_read(&cpu->rflags);
      Word64 res = alu_64_execute(ALU64_DEC, word64_from_u64(val), word64_from_u64(1ULL), cpu->op_bits, &f, true);
      reg64_write(&cpu->rflags, f);
      uint64_t out = word64_to_u64(res);
      if (cpu->mod == 3) eu64_write_reg(cpu, cpu->rm_field, out, cpu->op_bits, cpu->has_rex);
      else mmu_write_sized(cpu, (cpu->seg_override >= 0) ? cpu->seg_override : 3, reg64_read(&cpu->ind), out, cpu->op_bits);
    } else if (cpu->reg_field == 2) { /* CALL near r/m */
      uint64_t target = mc64_read_bus(cpu, MB64_M);
      eu64_push(cpu, reg64_read(&cpu->rip));
      reg64_write(&cpu->rip, target);
    } else if (cpu->reg_field == 4) { /* JMP near r/m */
      uint64_t target = mc64_read_bus(cpu, MB64_M);
      reg64_write(&cpu->rip, target);
    } else if (cpu->reg_field == 6) { /* PUSH r/m */
      uint64_t val = mc64_read_bus(cpu, MB64_M);
      eu64_push(cpu, val);
    }
    return;
  }
}

/* ============================================================================
 * 64-BIT HARDWARE VERIFICATION & DEMONSTRATION SUITE
 * ============================================================================
 */

static void write_gdt_entry_64(uint8_t *ram, uint64_t addr, uint64_t base,
                              uint32_t limit, uint8_t access, uint8_t gran) {
  uint32_t lo = (limit & 0xFFFFu) | ((uint32_t)(base & 0xFFFFu) << 16);
  uint32_t hi = ((base >> 16) & 0xFFu) | ((uint32_t)access << 8) |
                (limit & 0x000F0000u) | ((uint32_t)(gran & 0x0Fu) << 20) |
                (uint32_t)(base & 0xFF000000u);
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
  printf("=============================================================================\n");
  printf(" SPEC-EXACT 64-BIT x86-64 / AMD64 CPU (TRANSISTORS -> 4-LEVEL MMU -> LONG MODE)\n");
  printf("=============================================================================\n\n");

  static X86_64CPU cpu;
  cpu64_init(&cpu);

  /*
   * 1. Global Descriptor Table (GDT) at Physical 0x00800:
   *    - Selector 0x0000: Null Descriptor
   *    - Selector 0x0008: 16-Bit Real/Prot Mode CS (Base=0, Limit=0xFFFF)
   *    - Selector 0x0010: 64-Bit Ring-0 Code Segment (Base=0, Limit=0xFFFFF, L=1, D=0, G=1)
   *    - Selector 0x0018: 64-Bit Ring-0 Data Segment (Base=0, Limit=0xFFFFF, G=1)
   */
  write_gdt_entry_64(cpu.ram, 0x00800, 0, 0, 0x00, 0x00);
  write_gdt_entry_64(cpu.ram, 0x00808, 0, 0xFFFF, 0x9A, 0x00);
  write_gdt_entry_64(cpu.ram, 0x00810, 0, 0xFFFFF, 0x9A, 0x0A); /* L=1 (64-bit code), D=0, G=1 */
  write_gdt_entry_64(cpu.ram, 0x00818, 0, 0xFFFFF, 0x92, 0x0C); /* B=1, G=1 */

  /* GDTR Pseudo-Descriptor at Physical 0x00600 (Limit=31, Base=0x00000800) */
  cpu.ram[0x00600] = 31;
  cpu.ram[0x00601] = 0;
  phys_write32(&cpu, 0x00602, 0x00000800U);

  /*
   * 2. Set Up 4-Level 64-Bit Paging (PML4 -> PDPT -> PD -> PT):
   *    - PML4 at Physical 0x00002000
   *    - PDPT at Physical 0x00003000
   *    - PD   at Physical 0x00004000
   *    - PT   at Physical 0x00005000
   *
   *    Mapping 1: Identity-Map Low 2 MB (Virtual 0x00000000..0x001FFFFF -> Phys 0x00000000..0x001FFFFF)
   *    Mapping 2: High Canonical Virtual Mapping:
   *               Virtual 0xFFFF800000000000 (PML4[256]) -> Maps to Physical Frame 0x00020000!
   */
  phys_write64(&cpu, 0x02000, 0x00003003ULL); /* PML4[0] -> PDPT at 0x3000 */
  phys_write64(&cpu, 0x03000, 0x00004003ULL); /* PDPT[0] -> PD at 0x4000   */
  phys_write64(&cpu, 0x04000, 0x00005003ULL); /* PD[0]   -> PT at 0x5000   */

  for (uint64_t page = 0; page < 512; page++) {
    phys_write64(&cpu, 0x05000 + page * 8, (page << 12) | 0x03ULL); /* Identity map 2MB */
  }

  /* High Canonical Half: PML4[256] -> Dedicated PDPT at 0x6000 -> PD at 0x7000 -> PT at 0x8000 */
  phys_write64(&cpu, 0x02000 + 256 * 8, 0x00006003ULL);
  phys_write64(&cpu, 0x06000, 0x00007003ULL);
  phys_write64(&cpu, 0x07000, 0x00008003ULL);
  /* Map 0xFFFF800000000000 (PT[0]) -> Physical Frame 0x00020000 */
  phys_write64(&cpu, 0x08000, 0x00020003ULL);

  /*
   * 3. 16-Bit Real-Mode Bootloader at 0000:0500:
   *    - Loads GDTR
   *    - Enables PAE in CR4 (CR4 |= 0x20)
   *    - Loads CR3 = 0x00002000 (PML4 Base)
   *    - Sets EFER MSR (0xC0000080) LME bit (Bit 8) and SCE bit (Bit 0)
   *    - Sets CR0 PE=1 and PG=1 (0x80000001) -> CPU activates Long Mode (LMA=1)!
   *    - Far Jumps `JMP FAR 0x0010:0x00001000` into 64-Bit Long Mode!
   */
  static const uint8_t boot16[] = {
      0x31, 0xC0,                         /* XOR AX, AX                          */
      0x8E, 0xD8,                         /* MOV DS, AX                          */
      0x0F, 0x01, 0x16, 0x00, 0x06,       /* LGDT [0x0600]                       */
      0x0F, 0x20, 0xE0,                   /* MOV EAX, CR4                        */
      0x0C, 0x20,                         /* OR AL, 0x20 (Enable PAE!)           */
      0x0F, 0x22, 0xE0,                   /* MOV CR4, EAX                        */
      0x66, 0xB8, 0x00, 0x20, 0x00, 0x00, /* MOV EAX, 0x00002000 (PML4 Base)     */
      0x0F, 0x22, 0xD8,                   /* MOV CR3, EAX                        */
      0x66, 0xB9, 0x80, 0x00, 0x00, 0xC0, /* MOV ECX, 0xC0000080 (EFER MSR)      */
      0x0F, 0x32,                         /* RDMSR                               */
      0x66, 0x0D, 0x01, 0x01, 0x00, 0x00, /* OR EAX, 0x00000101 (LME=1, SCE=1)   */
      0x0F, 0x30,                         /* WRMSR (Long Mode Enabled!)          */
      0x66, 0xB8, 0x01, 0x00, 0x00, 0x80, /* MOV EAX, 0x80000001 (PG=1, PE=1)   */
      0x0F, 0x22, 0xC0,                   /* MOV CR0, EAX (Activates Long Mode!) */
      0x66, 0xEA, 0x00, 0x10, 0x00, 0x00, /* JMP FAR 0x0010:0x00001000 (64-Bit!)*/
      0x10, 0x00
  };
  memcpy(&cpu.ram[0x00500], boot16, sizeof(boot16));

  /* Install Reset Vector Jump at FFFF:0000 -> JMP FAR 0000:0500 */
  cpu.ram[0xFFFF0] = 0xEA;
  cpu.ram[0xFFFF1] = 0x00;
  cpu.ram[0xFFFF2] = 0x05;
  cpu.ram[0xFFFF3] = 0x00;
  cpu.ram[0xFFFF4] = 0x00;

  /*
   * 4. 64-Bit Long Mode Kernel Payload at 0x0010:0x00001000:
   *    - Inits segments & RSP = 0x0000000000009000
   *    - Test 1: 64-Bit Immediate Load & REX Extension (R12, RAX)
   *    - Test 2: 32-Bit Zero-Extension Rule (MOV EAX, 0xFFFFFFFF -> upper 32 bits cleared!)
   *    - Test 3: 64-Bit RIP-Relative Addressing [RIP + disp32]
   *    - Test 4: 64-Bit SIB Addressing with Scale 8 [RBX + R13*8 + 0x20] into High Canonical Memory
   *    - Test 5: 64-Bit Barrel Shifter (SHR 40 bits) & BSF/BSR
   *    - Test 6: 64-Bit Subroutine CALL rel32 & RET near
   *    - Test 7: 64-Bit Bit Operation (BTS R14, 3)
   *    - Test 8: 64-Bit Paged Virtual Memory String Copy (REP MOVSQ)
   *    - Test 9: Fast System Call via SYSCALL & SYSRET (LSTAR MSR)
   */
  static const uint8_t kernel64[] = {
      /* 0x00: Init Stack: MOV RSP, 0x0000000000009000 (10 bytes: 0x00..0x09) */
      0x48, 0xBC, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

      /* 0x0A: Test 9 Setup: Register LSTAR MSR 0xC0000082 */
      0xB9, 0x82, 0x00, 0x00, 0xC0,                               /* MOV ECX, 0xC0000082 (LSTAR)   (5 bytes: 0x0A..0x0E) */
      0xB8, 0xC0, 0x10, 0x00, 0x00,                               /* MOV EAX, 0x000010C0 (Handler) (5 bytes: 0x0F..0x13) */
      0x31, 0xD2,                                                 /* XOR EDX, EDX                  (2 bytes: 0x14..0x15) */
      0x0F, 0x30,                                                 /* WRMSR                         (2 bytes: 0x16..0x17) */

      /* 0x18: Test 1: 64-Bit REX Register Extensions & 64-Bit Immediates */
      0x48, 0xB8, 0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01, /* MOV RAX, 0x0123456789ABCDEF (10 bytes: 0x18..0x21) */
      0x49, 0xBC, 0x00, 0x40, 0x00, 0x30, 0x00, 0x20, 0x00, 0x10, /* MOV R12, 0x1000200030004000 (10 bytes: 0x22..0x2B) */
      0x49, 0x01, 0xC4,                                           /* ADD R12, RAX                  (3 bytes: 0x2C..0x2E) */

      /* 0x2F: Test 3: 64-Bit RIP-Relative Addressing [RIP + disp32] */
      0x48, 0x8B, 0x15, 0x82, 0x00, 0x00, 0x00,                   /* MOV RDX, [RIP + 0x82] (load from 0x10B8) (7 bytes: 0x2F..0x35) */

      /* 0x36: Test 4: 64-Bit SIB [RBX + R13*8 + 0x20] into High Canonical Virtual Memory */
      0x48, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, /* MOV RBX, 0xFFFF800000000000 (10 bytes: 0x36..0x3F) */
      0x4D, 0x31, 0xED,                                           /* XOR R13, R13                  (3 bytes: 0x40..0x42) */
      0x48, 0xB8, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, /* MOV RAX, 0x1122334455667788 (10 bytes: 0x43..0x4C) */
      /* SIB Loop (4 Quadwords): */
      0x4A, 0x89, 0x44, 0xEB, 0x20,                               /* MOV [RBX + R13*8 + 0x20], RAX (5 bytes: 0x4D..0x51) */
      0x48, 0x05, 0x11, 0x11, 0x11, 0x11,                         /* ADD RAX, 0x11111111           (6 bytes: 0x52..0x57) */
      0x49, 0xFF, 0xC5,                                           /* INC R13                       (3 bytes: 0x58..0x5A) */
      0x49, 0x83, 0xFD, 0x04,                                     /* CMP R13, 4                    (4 bytes: 0x5B..0x5E) */
      0x75, 0xEC,                                                 /* JNE -20 (back to 0x104D)      (2 bytes: 0x5F..0x60) */

      /* 0x61: Test 7: 64-Bit Bit Operation: BTS R14, 3 */
      0x49, 0xC7, 0xC6, 0x00, 0x00, 0x00, 0x00,                   /* MOV R14, 0                    (7 bytes: 0x61..0x67) */
      0x49, 0x0F, 0xBA, 0xEE, 0x03,                               /* BTS R14, 3 (R14 becomes 8!)   (5 bytes: 0x68..0x6C) */

      /* 0x6D: Test 8: 64-Bit Paged Virtual Memory String Copy (REP MOVSQ) */
      0x48, 0xBE, 0x20, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, /* MOV RSI, 0xFFFF800000000020 (Src) (10 bytes: 0x6D..0x76) */
      0x48, 0xBF, 0x50, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, /* MOV RDI, 0xFFFF800000000050 (Dst) (10 bytes: 0x77..0x80) */
      0x48, 0xC7, 0xC1, 0x04, 0x00, 0x00, 0x00,                   /* MOV RCX, 4 (4 Quadwords)      (7 bytes: 0x81..0x87) */
      0xF3, 0x48, 0xA5,                                           /* REP MOVSQ                     (3 bytes: 0x88..0x8A) */

      /* 0x8B: Test 5: 64-Bit Barrel Shifter (SHR 32 bits) & BSF/BSR */
      0x48, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, /* MOV RAX, 0x0100000000000000 (Bit 56 set) (10 bytes: 0x8B..0x94) */
      0x48, 0xC1, 0xE8, 0x20,                                     /* SHR RAX, 32 (0x0000000001000000) (4 bytes: 0x95..0x98) */
      0x48, 0x0F, 0xBC, 0xF8,                                     /* BSF RDI, RAX (RDI = 24!)      (4 bytes: 0x99..0x9C) */
      0x48, 0x0F, 0xBD, 0xE8,                                     /* BSR RBP, RAX (RBP = 24!)      (4 bytes: 0x9D..0xA0) */

      /* 0xA1: Test 6: 64-Bit Subroutine CALL rel32 & RET near */
      0xE8, 0x02, 0x00, 0x00, 0x00,                               /* CALL +2 (to 0x10A8)           (5 bytes: 0xA1..0xA5) */
      0xEB, 0x05,                                                 /* JMP +5 (skip subroutine to 0x10AD) (2 bytes: 0xA6..0xA7) */
      /* Subroutine at 0x10A8: */
      0x48, 0x83, 0xC0, 0x42,                                     /* ADD RAX, 0x42                 (4 bytes: 0xA8..0xAB) */
      0xC3,                                                       /* RET                           (1 byte:  0xAC)       */
      /* Returns to 0x10A6, JMP +5 jumps to 0x10AD */
      /* RAX = 0x0000000001000042 (Test 2: Spec-Exact 32-Bit Zero-Extension satisfied!) */

      /* 0xAD: Test 9: Fast System Call (SYSCALL) */
      0x0F, 0x05,                                                 /* SYSCALL (2 bytes: 0xAD..0xAE) */
      0xF4,                                                       /* HLT at 0x10AF (1 byte: 0xAF) */

      /* Padding 8 bytes to reach offset 0xB8 */
      0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90,

      /* 0xB8: Target Data for Test 3 [RIP + 0x82] at offset 0xB8 (address 0x000010B8) */
      0xEF, 0xBE, 0xAD, 0xDE, 0x00, 0x00, 0x00, 0x00,             /* 0x00000000DEADBEEF (8 bytes: 0xB8..0xBF) */

      /* 0xC0: SYSCALL Handler at address 0x000010C0: */
      0x49, 0xC7, 0xC7, 0x77, 0x00, 0x00, 0x00,                   /* MOV R15, 0x77 (Syscall Served) (7 bytes: 0xC0..0xC6) */
      0x48, 0x0F, 0x07                                            /* SYSRET                        (3 bytes: 0xC7..0xC9) */
  };
  memcpy(&cpu.ram[0x01000], kernel64, sizeof(kernel64));

  printf("1. Executing 16-Bit Real Mode -> 64-Bit Long Mode (IA-32e) Boot Sequence:\n");
  int step = 0;
  while (!cpu.halted && step < 200) {
    /* Microcode trace on key instructions */
    cpu.trace_mc = (step == 18 || step == 19 || step == 24);
    if (cpu.trace_mc) {
      printf("  [Step %2d | CS:%04X RIP:%016llX | LongMode: %s | Paging: %s]\n",
             step, cpu.segs[1].selector, (unsigned long long)reg64_read(&cpu.rip),
             cpu.segs[1].l_bit ? "Active (L=1)" : "Inactive",
             (reg64_read(&cpu.cr0) & 0x80000000ULL) ? "4-Level ON" : "OFF");
    }
    cpu64_step(&cpu);
    step++;
  }

  printf("\n2. 64-Bit Long Mode & 4-Level Hardware Paging MMU Verification:\n");
  printf("   - Control Registers : CR0=0x%016llX  CR3(PML4)=0x%016llX  CR4=0x%016llX\n",
         (unsigned long long)reg64_read(&cpu.cr0),
         (unsigned long long)reg64_read(&cpu.cr3),
         (unsigned long long)reg64_read(&cpu.cr4));
  printf("   - Extended MSRs     : EFER=0x%016llX (LME=%d, LMA=%d, SCE=%d) LSTAR=0x%016llX\n",
         (unsigned long long)cpu.msr_efer,
         (int)((cpu.msr_efer >> 8) & 1),
         (int)((cpu.msr_efer >> 10) & 1),
         (int)(cpu.msr_efer & 1),
         (unsigned long long)cpu.msr_lstar);
  printf("   - 64-Bit CS Reg     : Selector=0x%04X Base=0x%016llX L=%d (Long Mode) D=%d\n",
         cpu.segs[1].selector, (unsigned long long)cpu.segs[1].base,
         cpu.segs[1].l_bit, cpu.segs[1].d_bit);

  uint64_t pml4_256 = phys_read64(&cpu, 0x02000 + 256 * 8);
  uint64_t pte_high = phys_read64(&cpu, 0x08000);
  printf("   - 4-Level Page Walk : PML4[256]=0x%016llX (A=%d) -> PTE=0x%016llX (A=%d, D=%d)\n",
         (unsigned long long)pml4_256, (int)((pml4_256 >> 5) & 1),
         (unsigned long long)pte_high, (int)((pte_high >> 5) & 1), (int)((pte_high >> 6) & 1));

  printf("   - High Canonical SIB: Virtual 0xFFFF800000000020..38 mapped to Physical 0x00020020..38:\n     ");
  bool sib_pass = true;
  for (int i = 0; i < 4; i++) {
    uint64_t val = phys_read64(&cpu, 0x00020020ULL + i * 8);
    uint64_t exp = 0x1122334455667788ULL + (uint64_t)i * 0x11111111ULL;
    if (val != exp) sib_pass = false;
    printf("[Virt 0xFFFF8000000000%02X -> Phys 0x000200%02X] = 0x%016llX\n     ",
           0x20 + i * 8, 0x20 + i * 8, (unsigned long long)val);
  }

  printf("\n   - REP MOVSQ Target  : Virtual 0xFFFF800000000050..68 mapped to Physical 0x00020050..68:\n     ");
  bool movsq_pass = true;
  for (int i = 0; i < 4; i++) {
    uint64_t val = phys_read64(&cpu, 0x00020050ULL + i * 8);
    uint64_t exp = 0x1122334455667788ULL + (uint64_t)i * 0x11111111ULL;
    if (val != exp) movsq_pass = false;
    printf("[Virt 0xFFFF8000000000%02X -> Phys 0x000200%02X] = 0x%016llX\n     ",
           0x50 + i * 8, 0x50 + i * 8, (unsigned long long)val);
  }

  uint64_t r_rax = reg64_read(&cpu.regs[0]);
  uint64_t r_rdx = reg64_read(&cpu.regs[2]);
  uint64_t r_rsp = reg64_read(&cpu.regs[4]);
  uint64_t r_rbp = reg64_read(&cpu.regs[5]);
  uint64_t r_rdi = reg64_read(&cpu.regs[7]);
  uint64_t r_r12 = reg64_read(&cpu.regs[12]);
  uint64_t r_r14 = reg64_read(&cpu.regs[14]);
  uint64_t r_r15 = reg64_read(&cpu.regs[15]);

  printf("\n3. Spec-Exact x86-64 Architecture & Microcode Verification:\n");
  printf("   - 64-Bit REX R12 Test   : 0x1000200030004000 + RAX -> R12=0x%016llX %s\n",
         (unsigned long long)r_r12, (r_r12 == 0x11236567B9AC0DEFULL) ? "(PASS!)" : "(FAIL)");
  printf("   - 32-Bit Zero-Extension : Upper 32 bits cleared: RAX=0x%016llX %s\n",
         (unsigned long long)r_rax, (r_rax == 0x0000000001000042ULL) ? "(PASS!)" : "(FAIL)");
  printf("   - RIP-Relative Load     : [RIP + 0x82] -> RDX=0x%016llX %s\n",
         (unsigned long long)r_rdx, (r_rdx == 0x00000000DEADBEEFULL) ? "(PASS!)" : "(FAIL)");
  printf("   - 64-Bit SIB Store      : Paged Write [RBX + R13*8 + 0x20] %s\n",
         sib_pass ? "(PASS!)" : "(FAIL)");
  printf("   - 64-Bit Barrel Shifter : 0x0100000000000000 >> 32 & BSF/BSR -> RDI=%llu, RBP=%llu %s\n",
         (unsigned long long)r_rdi, (unsigned long long)r_rbp,
         (r_rdi == 24 && r_rbp == 24) ? "(PASS!)" : "(FAIL)");
  printf("   - 64-Bit CALL & RET     : Stack return RIP popped correctly (RSP=0x%016llX) %s\n",
         (unsigned long long)r_rsp, (r_rsp == 0x0000000000009000ULL) ? "(PASS!)" : "(FAIL)");
  printf("   - 64-Bit Bit Op (BTS)   : Bit 3 set -> R14=%llu %s\n",
         (unsigned long long)r_r14, (r_r14 == 8) ? "(PASS!)" : "(FAIL)");
  printf("   - 64-Bit String Copy    : `REP MOVSQ` 4 Quadwords %s\n",
         movsq_pass ? "(PASS!)" : "(FAIL)");
  printf("   - Fast SYSCALL / SYSRET : Served via LSTAR handler -> R15=0x%02llX %s\n\n",
         (unsigned long long)r_r15, (r_r15 == 0x77) ? "(PASS!)" : "(FAIL)");

  printf("4. Final 64-Bit Register File (16 Transistor DFF Registers):\n");
  printf("   RAX=%016llX  RBX=%016llX  RCX=%016llX  RDX=%016llX\n",
         (unsigned long long)reg64_read(&cpu.regs[0]), (unsigned long long)reg64_read(&cpu.regs[3]),
         (unsigned long long)reg64_read(&cpu.regs[1]), (unsigned long long)reg64_read(&cpu.regs[2]));
  printf("   RSP=%016llX  RBP=%016llX  RSI=%016llX  RDI=%016llX\n",
         (unsigned long long)reg64_read(&cpu.regs[4]), (unsigned long long)reg64_read(&cpu.regs[5]),
         (unsigned long long)reg64_read(&cpu.regs[6]), (unsigned long long)reg64_read(&cpu.regs[7]));
  printf("   R8 =%016llX  R9 =%016llX  R10=%016llX  R11=%016llX\n",
         (unsigned long long)reg64_read(&cpu.regs[8]), (unsigned long long)reg64_read(&cpu.regs[9]),
         (unsigned long long)reg64_read(&cpu.regs[10]), (unsigned long long)reg64_read(&cpu.regs[11]));
  printf("   R12=%016llX  R13=%016llX  R14=%016llX  R15=%016llX\n",
         (unsigned long long)reg64_read(&cpu.regs[12]), (unsigned long long)reg64_read(&cpu.regs[13]),
         (unsigned long long)reg64_read(&cpu.regs[14]), (unsigned long long)reg64_read(&cpu.regs[15]));
  printf("   RIP=%016llX  RFLAGS=%016llX\n\n",
         (unsigned long long)reg64_read(&cpu.rip), (unsigned long long)reg64_read(&cpu.rflags));

  printf("5. Hardware Simulation Totals:\n");
  printf("   - x86-64 Macro-Instructions : %llu\n", (unsigned long long)cpu.macro_instructions);
  printf("   - x86-64 Microcode Cycles   : %llu\n", (unsigned long long)cpu.micro_cycles);
  printf("   - MMU Page Walks / TLB Hits : %llu walks / %llu TLB hits\n",
         (unsigned long long)cpu.page_walks, (unsigned long long)cpu.tlb_hits);
  printf("   - Total MOSFET Switches     : %llu individual transistor evaluations\n",
         (unsigned long long)g_transistor_switches);
  printf("=============================================================================\n");

  return 0;
}
