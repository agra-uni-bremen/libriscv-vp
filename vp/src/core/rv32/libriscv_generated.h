void exec_auipc(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr), instrPC + instr_immU(instr));
}
void exec_lui(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr), instr_immU(instr));
}
void exec_jalr(uint32_t instrPC, void * instr)
{
    uint32_t link = read_next_pc();
    write_pc(read_register(instr_rs1(instr)) + instr_immI(instr) & 0xfffffffe);
    write_register(instr_rd(instr), link);
}
void exec_jal(uint32_t instrPC, void * instr)
{
    uint32_t link = read_next_pc();
    write_pc(instrPC + instr_immJ(instr));
    write_register(instr_rd(instr), link);
}
void exec_sw(uint32_t instrPC, void * instr)
{
    store_word(read_register(instr_rs1(instr)) + instr_immS(instr),
               read_register(instr_rs2(instr)));
}
void exec_lw(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   load_word(read_register(instr_rs1(instr)) + instr_immI(instr)));
}
void exec_addi(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) + instr_immI(instr));
}
void exec_add(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) + read_register(instr_rs2(instr)));
}
