#include <stdint.h>

#include "iss.h"
#include "interface.h"

static struct rv32::ISS *core;

void init_core(struct rv32::ISS *_core) {
	core = _core;
}

////
// Register File
////

uint32_t
read_register(unsigned idx)
{
	return core->regs[idx];
}

void
write_register(unsigned idx, uint32_t value)
{
	core->regs[idx] = value;
}

////
// Program Counter
////

uint32_t
read_next_pc(void)
{
	return core->pc;
}

void
write_pc(uint32_t newPC)
{
	core->pc = newPC;
}

////
// Data Memory
////

uint8_t
load_byte(uint32_t addr)
{
	return (uint8_t)core->mem->load_byte(addr);
}

uint16_t
load_half(uint32_t addr)
{
	return (uint8_t)core->mem->load_half(addr);
}

uint32_t
load_word(uint32_t addr)
{
	return core->mem->load_word(addr);
}

void
store_byte(uint32_t addr, uint8_t value)
{
	return core->mem->store_byte(addr, value);
}

void
store_half(uint32_t addr, uint16_t value)
{
	return core->mem->store_half(addr, value);
}

void
store_word(uint32_t addr, uint32_t value)
{
	return core->mem->store_word(addr, value);
}

////
// Instruction Decoder
////

uint32_t
instr_rs1(void *instr)
{
	return ((struct Instruction*)instr)->rs1();
}

uint32_t
instr_rs2(void *instr)
{
	return ((struct Instruction*)instr)->rs2();
}

uint32_t
instr_rd(void *instr)
{
	return ((struct Instruction*)instr)->rd();
}

uint32_t
instr_immI(void *instr)
{
	return ((struct Instruction*)instr)->I_imm();
}

uint32_t
instr_immS(void *instr)
{
	return ((struct Instruction*)instr)->S_imm();
}

uint32_t
instr_immB(void *instr)
{
	return ((struct Instruction*)instr)->B_imm();
}

uint32_t
instr_immU(void *instr)
{
	return ((struct Instruction*)instr)->U_imm();
}

uint32_t
instr_immJ(void *instr)
{
	return ((struct Instruction*)instr)->J_imm();
}

uint32_t
instr_shamt(void *instr)
{
	return ((struct Instruction*)instr)->shamt();
}
