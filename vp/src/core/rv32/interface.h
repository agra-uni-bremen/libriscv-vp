#include <stdlib.h>
#include <stdint.h>

#include "iss.h"

////
// Register File
////

static inline uint32_t
read_register(void *core, unsigned idx)
{
	return ((struct rv32::ISS*)core)->regs[idx];
}

static inline void
write_register(void *core, unsigned idx, uint32_t value)
{
	((struct rv32::ISS*)core)->regs[idx] = value;
}

////
// Program Counter
////

static inline uint32_t
read_next_pc(void *core)
{
	return ((struct rv32::ISS*)core)->pc;
}

static inline void
write_pc(void *core, uint32_t newPC)
{
	((struct rv32::ISS*)core)->pc = newPC;
}

////
// Data Memory
////

static inline uint8_t
load_byte(void *core, uint32_t addr)
{
	return ((struct rv32::ISS*)core)->mem->load_byte(addr);
}

static inline uint16_t
load_half(void *core, uint32_t addr)
{
	return ((struct rv32::ISS*)core)->mem->load_half(addr);
}

static inline uint32_t
load_word(void *core, uint32_t addr)
{
	return ((struct rv32::ISS*)core)->mem->load_word(addr);
}

static inline void
store_byte(void *core, uint32_t addr, uint8_t value)
{
	return ((struct rv32::ISS*)core)->mem->store_byte(addr, value);
}

static inline void
store_half(void *core, uint32_t addr, uint16_t value)
{
	return ((struct rv32::ISS*)core)->mem->store_half(addr, value);
}

static inline void
store_word(void *core, uint32_t addr, uint32_t value)
{
	return ((struct rv32::ISS*)core)->mem->store_word(addr, value);
}

////
// Instruction Decoder
////

static inline uint32_t
instr_rs1(void *instr)
{
	return ((struct Instruction*)instr)->rs1();
}

static inline uint32_t
instr_rs2(void *instr)
{
	return ((struct Instruction*)instr)->rs2();
}

static inline uint32_t
instr_rd(void *instr)
{
	return ((struct Instruction*)instr)->rd();
}

static inline uint32_t
instr_immI(void *instr)
{
	return ((struct Instruction*)instr)->I_imm();
}

static inline uint32_t
instr_immS(void *instr)
{
	return ((struct Instruction*)instr)->S_imm();
}

static inline uint32_t
instr_immB(void *instr)
{
	return ((struct Instruction*)instr)->B_imm();
}

static inline uint32_t
instr_immU(void *instr)
{
	return ((struct Instruction*)instr)->U_imm();
}

static inline uint32_t
instr_immJ(void *instr)
{
	return ((struct Instruction*)instr)->J_imm();
}

static inline uint32_t
instr_shamt(void *instr)
{
	return ((struct Instruction*)instr)->shamt();
}
