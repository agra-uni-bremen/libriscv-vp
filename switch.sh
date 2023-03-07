#!/bin/sh

awk -F '[ (]' '
/void exec/ {
	exec=$2
	inst=substr(exec, length("exec_")+1)
	opcode=toupper(inst)

	printf("\t\tcase Opcode::%s:\n\t\t\t%s(last_pc, &instr);\n\t\t\tbreak;\n", opcode, exec);
}' vp/src/core/rv32/libriscv_generated.h
