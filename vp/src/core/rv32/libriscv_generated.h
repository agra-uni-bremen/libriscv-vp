static inline void exec_xori(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) ^ instr_immI(instr));
}
static inline void exec_xor(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) ^ read_register(core,
                                                                         instr_rs2(instr)));
}
static inline void exec_sw(void * core,
                           uint32_t instrPC,
                           void * instr)
{
    store_word(core,
               read_register(core, instr_rs1(instr)) + instr_immS(instr),
               read_register(core, instr_rs2(instr)));
}
static inline void exec_sub(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) - read_register(core,
                                                                         instr_rs2(instr)));
}
static inline void exec_srli(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) >> instr_shamt(instr));
}
static inline void exec_srl(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) >> (read_register(core,
                                                                           instr_rs2(instr)) & 0x1f));
}
static inline void exec_srai(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   (int32_t) read_register(core,
                                           instr_rs1(instr)) >> instr_shamt(instr));
}
static inline void exec_sra(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   (int32_t) read_register(core,
                                           instr_rs1(instr)) >> (read_register(core,
                                                                               instr_rs2(instr)) & 0x1f));
}
static inline void exec_sltu(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) < read_register(core,
                                                                         instr_rs2(instr)));
}
static inline void exec_sltiu(void * core,
                              uint32_t instrPC,
                              void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) < instr_immI(instr));
}
static inline void exec_slti(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   (int32_t) read_register(core,
                                           instr_rs1(instr)) < (int32_t) instr_immI(instr));
}
static inline void exec_slt(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   (int32_t) read_register(core,
                                           instr_rs1(instr)) < (int32_t) read_register(core,
                                                                                       instr_rs2(instr)));
}
static inline void exec_slli(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) << instr_shamt(instr));
}
static inline void exec_sll(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) << (read_register(core,
                                                                           instr_rs2(instr)) & 0x1f));
}
static inline void exec_sh(void * core,
                           uint32_t instrPC,
                           void * instr)
{
    store_half(core,
               read_register(core, instr_rs1(instr)) + instr_immS(instr),
               read_register(core, instr_rs2(instr)));
}
static inline void exec_sb(void * core,
                           uint32_t instrPC,
                           void * instr)
{
    store_byte(core,
               read_register(core, instr_rs1(instr)) + instr_immS(instr),
               read_register(core, instr_rs2(instr)));
}
static inline void exec_ori(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) | instr_immI(instr));
}
static inline void exec_or(void * core,
                           uint32_t instrPC,
                           void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) | read_register(core,
                                                                         instr_rs2(instr)));
}
static inline void exec_lw(void * core,
                           uint32_t instrPC,
                           void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   load_word(core,
                             read_register(core, instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_lui(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core, instr_rd(instr), instr_immU(instr));
}
static inline void exec_lhu(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   (uint32_t) (uint16_t) load_half(core,
                                                   read_register(core,
                                                                 instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_lh(void * core,
                           uint32_t instrPC,
                           void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   (uint32_t) (int32_t) (int16_t) load_half(core,
                                                            read_register(core,
                                                                          instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_lbu(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   (uint32_t) (uint8_t) load_byte(core,
                                                  read_register(core,
                                                                instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_lb(void * core,
                           uint32_t instrPC,
                           void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   (uint32_t) (int32_t) (int8_t) load_byte(core,
                                                           read_register(core,
                                                                         instr_rs1(instr)) + instr_immI(instr)));
}
static inline void exec_jalr(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    uint32_t link = read_next_pc(core);
    write_pc(core,
             read_register(core,
                           instr_rs1(instr)) + instr_immI(instr) & 0xfffffffe);
    if ((read_register(core,
                       instr_rs1(instr)) + instr_immI(instr) & 0xfffffffe & 0x3) >= 0x1)
    {
        abort();
    }
    write_register(core, instr_rd(instr), link);
}
static inline void exec_jal(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    uint32_t link = read_next_pc(core);
    write_pc(core, instrPC + instr_immJ(instr));
    if ((instrPC + instr_immJ(instr) & 0x3) >= 0x1)
    {
        abort();
    }
    write_register(core, instr_rd(instr), link);
}
static inline void exec_fence(void * core,
                              uint32_t instrPC,
                              void * instr)
{
}
static inline void exec_bne(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    if (read_register(core, instr_rs1(instr)) == read_register(core,
                                                               instr_rs2(instr)))
    {
    }
    else
    {
        write_pc(core, instrPC + instr_immB(instr));
    }
}
static inline void exec_bltu(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    if (read_register(core, instr_rs1(instr)) < read_register(core,
                                                              instr_rs2(instr)))
    {
        write_pc(core, instrPC + instr_immB(instr));
        if ((instrPC + instr_immB(instr) & 0x3) >= 0x1)
        {
            abort();
        }
    }
}
static inline void exec_blt(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    if ((int32_t) read_register(core,
                                instr_rs1(instr)) < (int32_t) read_register(core,
                                                                            instr_rs2(instr)))
    {
        write_pc(core, instrPC + instr_immB(instr));
        if ((instrPC + instr_immB(instr) & 0x3) >= 0x1)
        {
            abort();
        }
    }
}
static inline void exec_bgeu(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    if (read_register(core, instr_rs1(instr)) >= read_register(core,
                                                               instr_rs2(instr)))
    {
        write_pc(core, instrPC + instr_immB(instr));
        if ((instrPC + instr_immB(instr) & 0x3) >= 0x1)
        {
            abort();
        }
    }
}
static inline void exec_bge(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    if ((int32_t) read_register(core,
                                instr_rs1(instr)) >= (int32_t) read_register(core,
                                                                             instr_rs2(instr)))
    {
        write_pc(core, instrPC + instr_immB(instr));
        if ((instrPC + instr_immB(instr) & 0x3) >= 0x1)
        {
            abort();
        }
    }
}
static inline void exec_beq(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    if (read_register(core, instr_rs1(instr)) == read_register(core,
                                                               instr_rs2(instr)))
    {
        write_pc(core, instrPC + instr_immB(instr));
    }
}
static inline void exec_auipc(void * core,
                              uint32_t instrPC,
                              void * instr)
{
    write_register(core, instr_rd(instr), instrPC + instr_immU(instr));
}
static inline void exec_andi(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) & instr_immI(instr));
}
static inline void exec_and(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) & read_register(core,
                                                                         instr_rs2(instr)));
}
static inline void exec_addi(void * core,
                             uint32_t instrPC,
                             void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) + instr_immI(instr));
}
static inline void exec_add(void * core,
                            uint32_t instrPC,
                            void * instr)
{
    write_register(core,
                   instr_rd(instr),
                   read_register(core, instr_rs1(instr)) + read_register(core,
                                                                         instr_rs2(instr)));
}