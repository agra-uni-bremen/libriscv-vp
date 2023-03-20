static inline void exec_xori(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) ^ instr_immI(instr));
}
static inline void exec_xor(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) ^ read_register(instr_rs2(instr)));
}
static inline void exec_sw(uint32_t instrPC, void * instr)
{
    store_word(read_register(instr_rs1(instr)) + instr_immS(instr),
               read_register(instr_rs2(instr)));
}
static inline void exec_sub(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) - read_register(instr_rs2(instr)));
}
static inline void exec_srli(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) >> instr_shamt(instr));
}
static inline void exec_srl(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) >> (read_register(instr_rs2(instr)) & 0x1f));
}
static inline void exec_srai(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   (int32_t) read_register(instr_rs1(instr)) >> instr_shamt(instr));
}
static inline void exec_sra(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   (int32_t) read_register(instr_rs1(instr)) >> (read_register(instr_rs2(instr)) & 0x1f));
}
static inline void exec_sltu(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) < read_register(instr_rs2(instr)));
}
static inline void exec_sltiu(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) < instr_immI(instr));
}
static inline void exec_slti(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   (int32_t) read_register(instr_rs1(instr)) < (int32_t) instr_immI(instr));
}
static inline void exec_slt(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   (int32_t) read_register(instr_rs1(instr)) < (int32_t) read_register(instr_rs2(instr)));
}
static inline void exec_slli(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) << instr_shamt(instr));
}
static inline void exec_sll(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) << (read_register(instr_rs2(instr)) & 0x1f));
}
static inline void exec_sh(uint32_t instrPC, void * instr)
{
    store_half(read_register(instr_rs1(instr)) + instr_immS(instr),
               read_register(instr_rs2(instr)));
}
static inline void exec_sb(uint32_t instrPC, void * instr)
{
    store_byte(read_register(instr_rs1(instr)) + instr_immS(instr),
               read_register(instr_rs2(instr)));
}
static inline void exec_ori(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) | instr_immI(instr));
}
static inline void exec_or(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) | read_register(instr_rs2(instr)));
}
static inline void exec_lw(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   load_word(read_register(instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_lui(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr), instr_immU(instr));
}
static inline void exec_lhu(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   (uint32_t) (uint16_t) load_half(read_register(instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_lh(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   (uint32_t) (int32_t) (int16_t) load_half(read_register(instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_lbu(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   (uint32_t) (uint8_t) load_byte(read_register(instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_lb(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   (uint32_t) (int32_t) (int8_t) load_byte(read_register(instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_jalr(uint32_t instrPC, void * instr)
{
    uint32_t link = read_next_pc();
    write_pc(read_register(instr_rs1(instr)) + instr_immI(instr) & 0xfffffffe);
    if ((read_register(instr_rs1(instr)) + instr_immI(instr) & 0xfffffffe & 0x3) >= 0x1)
    {
        abort();
    }
    write_register(instr_rd(instr), link);
}
static inline void exec_jal(uint32_t instrPC, void * instr)
{
    uint32_t link = read_next_pc();
    write_pc(instrPC + instr_immJ(instr));
    if ((instrPC + instr_immJ(instr) & 0x3) >= 0x1)
    {
        abort();
    }
    write_register(instr_rd(instr), link);
}
static inline void exec_fence(uint32_t instrPC, void * instr)
{
}
static inline void exec_bne(uint32_t instrPC, void * instr)
{
    if (read_register(instr_rs1(instr)) == read_register(instr_rs2(instr)))
    {
    }
    else
    {
        write_pc(instrPC + instr_immB(instr));
    }
}
static inline void exec_bltu(uint32_t instrPC, void * instr)
{
    if (read_register(instr_rs1(instr)) < read_register(instr_rs2(instr)))
    {
        write_pc(instrPC + instr_immB(instr));
        if ((instrPC + instr_immB(instr) & 0x3) >= 0x1)
        {
            abort();
        }
    }
}
static inline void exec_blt(uint32_t instrPC, void * instr)
{
    if ((int32_t) read_register(instr_rs1(instr)) < (int32_t) read_register(instr_rs2(instr)))
    {
        write_pc(instrPC + instr_immB(instr));
        if ((instrPC + instr_immB(instr) & 0x3) >= 0x1)
        {
            abort();
        }
    }
}
static inline void exec_bgeu(uint32_t instrPC, void * instr)
{
    if (read_register(instr_rs1(instr)) >= read_register(instr_rs2(instr)))
    {
        write_pc(instrPC + instr_immB(instr));
        if ((instrPC + instr_immB(instr) & 0x3) >= 0x1)
        {
            abort();
        }
    }
}
static inline void exec_bge(uint32_t instrPC, void * instr)
{
    if ((int32_t) read_register(instr_rs1(instr)) >= (int32_t) read_register(instr_rs2(instr)))
    {
        write_pc(instrPC + instr_immB(instr));
        if ((instrPC + instr_immB(instr) & 0x3) >= 0x1)
        {
            abort();
        }
    }
}
static inline void exec_beq(uint32_t instrPC, void * instr)
{
    if (read_register(instr_rs1(instr)) == read_register(instr_rs2(instr)))
    {
        write_pc(instrPC + instr_immB(instr));
    }
}
static inline void exec_auipc(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr), instrPC + instr_immU(instr));
}
static inline void exec_andi(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) & instr_immI(instr));
}
static inline void exec_and(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) & read_register(instr_rs2(instr)));
}
static inline void exec_addi(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) + instr_immI(instr));
}
static inline void exec_add(uint32_t instrPC, void * instr)
{
    write_register(instr_rd(instr),
                   read_register(instr_rs1(instr)) + read_register(instr_rs2(instr)));
}