#ifndef RISCV_VP_ISS_LIBRISCV
#define RISCV_VP_ISS_LIBRISCV

extern "C" {
// Implementation of the formal-iss interface model.
#include "interface.h"

// Instruction executors, generated from LibRISCV.
#include "libriscv_generated.h"
}

#endif
