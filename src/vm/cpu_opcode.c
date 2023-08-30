#include "./cpu.h"

static opcode_t cpu_op_table[256] = {
        { OPCODE_BRK, ADDR_IMP, 7, cpu_op_brk },
        { OPCODE_ORA, ADDR_INX, 6, cpu_op_ora },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ORA, ADDR_ZP, 3, cpu_op_ora },
        { OPCODE_ASL, ADDR_ZP, 5, cpu_op_asl },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_PHP, ADDR_IMP, 3, cpu_op_php },
        { OPCODE_ORA, ADDR_IMM, 2, cpu_op_ora },
        { OPCODE_ASL, ADDR_ACC, 2, cpu_op_asl },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ORA, ADDR_ABS, 4, cpu_op_ora },
        { OPCODE_ASL, ADDR_ABS, 6, cpu_op_asl },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BPL, ADDR_REL, 2, cpu_op_bpl },
        { OPCODE_ORA, ADDR_INY, 5, cpu_op_ora },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ORA, ADDR_ZPX, 4, cpu_op_ora },
        { OPCODE_ASL, ADDR_ZPX, 6, cpu_op_asl },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CLC, ADDR_IMP, 2, cpu_op_clc },
        { OPCODE_ORA, ADDR_ABY, 4, cpu_op_ora },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ORA, ADDR_ABX, 4, cpu_op_ora },
        { OPCODE_ASL, ADDR_ABX, 7, cpu_op_asl },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_JSR, ADDR_ABS, 6, cpu_op_jsr },
        { OPCODE_AND, ADDR_INX, 6, cpu_op_and },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BIT, ADDR_ZP, 3, cpu_op_bit },
        { OPCODE_AND, ADDR_ZP, 3, cpu_op_and },
        { OPCODE_ROL, ADDR_ZP, 5, cpu_op_rol },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_PLP, ADDR_IMP, 4, cpu_op_plp },
        { OPCODE_AND, ADDR_IMM, 2, cpu_op_and },
        { OPCODE_ROL, ADDR_ACC, 2, cpu_op_rol },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BIT, ADDR_ABS, 4, cpu_op_bit },
        { OPCODE_AND, ADDR_ABS, 4, cpu_op_and },
        { OPCODE_ROL, ADDR_ABS, 6, cpu_op_rol },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BMI, ADDR_REL, 2, cpu_op_bmi },
        { OPCODE_AND, ADDR_INY, 5, cpu_op_and },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_AND, ADDR_ZPX, 4, cpu_op_and },
        { OPCODE_ROL, ADDR_ZPX, 6, cpu_op_rol },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_SEC, ADDR_IMP, 2, cpu_op_sec },
        { OPCODE_AND, ADDR_ABY, 4, cpu_op_and },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_AND, ADDR_ABX, 4, cpu_op_and },
        { OPCODE_ROL, ADDR_ABX, 7, cpu_op_rol },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_RTI, ADDR_IMP, 6, cpu_op_rti },
        { OPCODE_EOR, ADDR_INX, 6, cpu_op_eor },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_EOR, ADDR_ZP, 3, cpu_op_eor },
        { OPCODE_LSR, ADDR_ZP, 5, cpu_op_lsr },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_PHA, ADDR_IMP, 3, cpu_op_pha },
        { OPCODE_EOR, ADDR_IMM, 2, cpu_op_eor },
        { OPCODE_LSR, ADDR_ACC, 2, cpu_op_lsr },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_JMP, ADDR_ABS, 3, cpu_op_jmp },
        { OPCODE_EOR, ADDR_ABS, 4, cpu_op_eor },
        { OPCODE_LSR, ADDR_ABS, 6, cpu_op_lsr },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BVC, ADDR_REL, 2, cpu_op_bvc },
        { OPCODE_EOR, ADDR_INY, 5, cpu_op_eor },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_EOR, ADDR_ZPX, 4, cpu_op_eor },
        { OPCODE_LSR, ADDR_ZPX, 6, cpu_op_lsr },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CLI, ADDR_IMP, 2, cpu_op_cli },
        { OPCODE_EOR, ADDR_ABY, 4, cpu_op_eor },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_EOR, ADDR_ABX, 4, cpu_op_eor },
        { OPCODE_LSR, ADDR_ABX, 7, cpu_op_lsr },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_RTS, ADDR_IMP, 6, cpu_op_rts },
        { OPCODE_ADC, ADDR_INX, 6, cpu_op_adc },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ADC, ADDR_ZP, 3, cpu_op_adc },
        { OPCODE_ROR, ADDR_ZP, 5, cpu_op_ror },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_PLA, ADDR_IMP, 4, cpu_op_pla },
        { OPCODE_ADC, ADDR_IMM, 2, cpu_op_adc },
        { OPCODE_ROR, ADDR_ACC, 2, cpu_op_ror },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_JMP, ADDR_IND, 5, cpu_op_jmp },
        { OPCODE_ADC, ADDR_ABS, 4, cpu_op_adc },
        { OPCODE_ROR, ADDR_ABS, 6, cpu_op_ror },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BVS, ADDR_REL, 2, cpu_op_bvs },
        { OPCODE_ADC, ADDR_INY, 5, cpu_op_adc },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_IMP, 0, cpu_op_ill },
        { OPCODE_ADC, ADDR_ZPX, 4, cpu_op_adc },
        { OPCODE_ROR, ADDR_ZPX, 6, cpu_op_ror },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_SEI, ADDR_IMP, 2, cpu_op_sei },
        { OPCODE_ADC, ADDR_ABY, 4, cpu_op_adc },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ADC, ADDR_ABX, 4, cpu_op_adc },
        { OPCODE_ROR, ADDR_ABX, 7, cpu_op_ror },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_STA, ADDR_INX, 6, cpu_op_sta },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_STY, ADDR_ZP, 3, cpu_op_sty },
        { OPCODE_STA, ADDR_ZP, 3, cpu_op_sta },
        { OPCODE_STX, ADDR_ZP, 3, cpu_op_stx },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_DEY, ADDR_IMP, 2, cpu_op_dey },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_TXA, ADDR_IMP, 2, cpu_op_txa },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_STY, ADDR_ABS, 4, cpu_op_sty },
        { OPCODE_STA, ADDR_ABS, 4, cpu_op_sta },
        { OPCODE_STX, ADDR_ABS, 4, cpu_op_stx },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BCC, ADDR_REL, 2, cpu_op_bcc },
        { OPCODE_STA, ADDR_INY, 6, cpu_op_sta },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_STY, ADDR_ZPX, 4, cpu_op_sty },
        { OPCODE_STA, ADDR_ZPX, 4, cpu_op_sta },
        { OPCODE_STX, ADDR_ZPY, 4, cpu_op_stx },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_TYA, ADDR_IMP, 2, cpu_op_tya },
        { OPCODE_STA, ADDR_ABY, 5, cpu_op_sta },
        { OPCODE_TXS, ADDR_IMP, 2, cpu_op_txs },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_STA, ADDR_ABX, 5, cpu_op_sta },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_LDY, ADDR_IMM, 2, cpu_op_ldy },
        { OPCODE_LDA, ADDR_INX, 6, cpu_op_lda },
        { OPCODE_LDX, ADDR_IMM, 2, cpu_op_ldx },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_LDY, ADDR_ZP, 3, cpu_op_ldy },
        { OPCODE_LDA, ADDR_ZP, 3, cpu_op_lda },
        { OPCODE_LDX, ADDR_ZP, 3, cpu_op_ldx },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_TAY, ADDR_IMP, 2, cpu_op_tay },
        { OPCODE_LDA, ADDR_IMM, 2, cpu_op_lda },
        { OPCODE_TAX, ADDR_IMP, 2, cpu_op_tax },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_LDY, ADDR_ABS, 4, cpu_op_ldy },
        { OPCODE_LDA, ADDR_ABS, 4, cpu_op_lda },
        { OPCODE_LDX, ADDR_ABS, 4, cpu_op_ldx },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BCS, ADDR_REL, 2, cpu_op_bcs },
        { OPCODE_LDA, ADDR_INY, 5, cpu_op_lda },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_LDY, ADDR_ZPX, 4, cpu_op_ldy },
        { OPCODE_LDA, ADDR_ZPX, 4, cpu_op_lda },
        { OPCODE_LDX, ADDR_ZPY, 4, cpu_op_ldx },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CLV, ADDR_IMP, 2, cpu_op_clv },
        { OPCODE_LDA, ADDR_ABY, 4, cpu_op_lda },
        { OPCODE_TSX, ADDR_IMP, 2, cpu_op_tsx },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_LDY, ADDR_ABX, 4, cpu_op_ldy },
        { OPCODE_LDA, ADDR_ABX, 4, cpu_op_lda },
        { OPCODE_LDX, ADDR_ABY, 4, cpu_op_ldx },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CPY, ADDR_IMM, 2, cpu_op_cpy },
        { OPCODE_CMP, ADDR_INX, 6, cpu_op_cmp },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CPY, ADDR_ZP, 3, cpu_op_cpy },
        { OPCODE_CMP, ADDR_ZP, 3, cpu_op_cmp },
        { OPCODE_DEC, ADDR_ZP, 5, cpu_op_dec },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_INY, ADDR_IMP, 2, cpu_op_iny },
        { OPCODE_CMP, ADDR_IMM, 2, cpu_op_cmp },
        { OPCODE_DEX, ADDR_IMP, 2, cpu_op_dex },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CPY, ADDR_ABS, 4, cpu_op_cpy },
        { OPCODE_CMP, ADDR_ABS, 4, cpu_op_cmp },
        { OPCODE_DEC, ADDR_ABS, 6, cpu_op_dec },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BNE, ADDR_REL, 2, cpu_op_bne },
        { OPCODE_CMP, ADDR_INY, 5, cpu_op_cmp },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CMP, ADDR_ZPX, 4, cpu_op_cmp },
        { OPCODE_DEC, ADDR_ZPX, 6, cpu_op_dec },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CLD, ADDR_IMP, 2, cpu_op_cld },
        { OPCODE_CMP, ADDR_ABY, 4, cpu_op_cmp },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CMP, ADDR_ABX, 4, cpu_op_cmp },
        { OPCODE_DEC, ADDR_ABX, 7, cpu_op_dec },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CPX, ADDR_IMM, 2, cpu_op_cpx },
        { OPCODE_SBC, ADDR_INX, 6, cpu_op_sbc },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CPX, ADDR_ZP, 3, cpu_op_cpx },
        { OPCODE_SBC, ADDR_ZP, 3, cpu_op_sbc },
        { OPCODE_INC, ADDR_ZP, 5, cpu_op_inc },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_INX, ADDR_IMP, 2, cpu_op_inx },
        { OPCODE_SBC, ADDR_IMM, 2, cpu_op_sbc },
        { OPCODE_NOP, ADDR_IMP, 2, cpu_op_nop },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_CPX, ADDR_ABS, 4, cpu_op_cpx },
        { OPCODE_SBC, ADDR_ABS, 4, cpu_op_sbc },
        { OPCODE_INC, ADDR_ABS, 6, cpu_op_inc },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_BEQ, ADDR_REL, 2, cpu_op_beq },
        { OPCODE_SBC, ADDR_INY, 5, cpu_op_sbc },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_SBC, ADDR_ZPX, 4, cpu_op_sbc },
        { OPCODE_INC, ADDR_ZPX, 6, cpu_op_inc },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_SED, ADDR_IMP, 2, cpu_op_sed },
        { OPCODE_SBC, ADDR_ABY, 4, cpu_op_sbc },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
        { OPCODE_SBC, ADDR_ABX, 4, cpu_op_sbc },
        { OPCODE_INC, ADDR_ABX, 7, cpu_op_inc },
        { OPCODE_ILL, ADDR_ACC, 0, cpu_op_ill },
};

opcode_t cpu_get_opcode(uint8_t opcode) {
    return cpu_op_table[opcode];
}

/**
 * @brief Unknown
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_ill(cpu_t *cpu, uint16_t in) {
    cpu->illegal = 1;
}

/**
 * @brief Add with Carry
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_adc(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t c = cpu_get_flag(cpu, FLAG_CARRY);

    uint16_t res = cpu->a + m + c;

    cpu_set_flag(cpu, FLAG_CARRY, res > 0xff);
    cpu_set_flag(cpu, FLAG_ZERO, (res & 0xff) == 0);
    cpu_set_flag(cpu, FLAG_OVERFLOW, (~(cpu->a ^ m) & (cpu->a ^ res)) & 0x80);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);

    cpu->a = res & 0xff;
}

/**
 * @brief AND (with accumulator)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_and(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = cpu->a & m;

    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);
    cpu_set_flag(cpu, FLAG_ZERO, res == 0);

    cpu->a = res;
}

/**
 * @brief Arithmetic Shift Left
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_asl(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint16_t res = m << 1;

    cpu_set_flag(cpu, FLAG_CARRY, res > 0xff);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);
    cpu_set_flag(cpu, FLAG_ZERO, (res & 0xff) == 0);

    if (in == 0x0a) {
        cpu->a = res & 0xff;
    } else {
        cpu_write(cpu, in, res & 0xff);
    }
}

/**
 * @brief Branch on Carry Clear
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bcc(cpu_t *cpu, uint16_t in) {
    if (!cpu_get_flag(cpu, FLAG_CARRY)) {
        cpu->pc = in;
    }
}

/**
 * @brief Branch on Carry Set
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bcs(cpu_t *cpu, uint16_t in) {
    if (cpu_get_flag(cpu, FLAG_CARRY)) {
        cpu->pc = in;
    }
}

/**
 * @brief Branch on equal (zero set)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_beq(cpu_t *cpu, uint16_t in) {
    if (cpu_get_flag(cpu, FLAG_ZERO)) {
        cpu->pc = in;
    }
}

/**
 * @brief Bit Test
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bit(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = cpu->a & m;

    cpu_set_flag(cpu, FLAG_ZERO, res == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, m & 0x80);
    cpu_set_flag(cpu, FLAG_OVERFLOW, m & 0x40);
}

/**
 * @brief Branch on Minus (negative set)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bmi(cpu_t *cpu, uint16_t in) {
    if (cpu_get_flag(cpu, FLAG_NEGATIVE)) {
        cpu->pc = in;
    }
}

/**
 * @brief Branch on Not Equal (zero clear)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bne(cpu_t *cpu, uint16_t in) {
    if (!cpu_get_flag(cpu, FLAG_ZERO)) {
        cpu->pc = in;
    }
}

/**
 * @brief Branch on Plus (negative clear)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bpl(cpu_t *cpu, uint16_t in) {
    if (!cpu_get_flag(cpu, FLAG_NEGATIVE)) {
        cpu->pc = in;
    }
}

/**
 * @brief Force Break
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_brk(cpu_t *cpu, uint16_t in) {
    cpu->pc++;

    cpu_push(cpu, cpu->pc >> 8);
    cpu_push(cpu, cpu->pc & 0xff);
    cpu_push(cpu, cpu->status | FLAG_BREAK);

    cpu_set_flag(cpu, FLAG_INTERRUPT, 1);

    cpu->pc = cpu_read(cpu, 0xfffe) | (cpu_read(cpu, 0xffff) << 8);
}

/**
 * @brief Branch on Overflow Clear
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bvc(cpu_t *cpu, uint16_t in) {
    if (!cpu_get_flag(cpu, FLAG_OVERFLOW)) {
        cpu->pc = in;
    }
}

/**
 * @brief Branch on Overflow Set
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bvs(cpu_t *cpu, uint16_t in) {
    if (cpu_get_flag(cpu, FLAG_OVERFLOW)) {
        cpu->pc = in;
    }
}

/**
 * @brief Clear Carry
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_clc(cpu_t *cpu, uint16_t in) {
    cpu_set_flag(cpu, FLAG_CARRY, 0);
}

/**
 * @brief Clear Decimal
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cld(cpu_t *cpu, uint16_t in) {
    cpu_set_flag(cpu, FLAG_DECIMAL, 0);
}

/**
 * @brief Clear Interrupt
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cli(cpu_t *cpu, uint16_t in) {
    cpu_set_flag(cpu, FLAG_INTERRUPT, 0);
}

/**
 * @brief Clear Overflow
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_clv(cpu_t *cpu, uint16_t in) {
    cpu_set_flag(cpu, FLAG_OVERFLOW, 0);
}

/**
 * @brief Compare (with accumulator)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cmp(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = cpu->a - m;

    cpu_set_flag(cpu, FLAG_CARRY, cpu->a >= m);
    cpu_set_flag(cpu, FLAG_ZERO, cpu->a == m);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);
}

/**
 * @brief Compare (with x)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cpx(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = cpu->x - m;

    cpu_set_flag(cpu, FLAG_CARRY, cpu->x >= m);
    cpu_set_flag(cpu, FLAG_ZERO, cpu->x == m);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);
}

/**
 * @brief Compare (with y)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cpy(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = cpu->y - m;

    cpu_set_flag(cpu, FLAG_CARRY, cpu->y >= m);
    cpu_set_flag(cpu, FLAG_ZERO, cpu->y == m);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);
}

/**
 * @brief Decrement
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_dec(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = m - 1;

    cpu_set_flag(cpu, FLAG_ZERO, res == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);

    cpu_write(cpu, in, res);
}

/**
 * @brief Decrement X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_dex(cpu_t *cpu, uint16_t in) {
    cpu->x--;

    cpu_set_flag(cpu, FLAG_ZERO, cpu->x == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->x & 0x80);
}

/**
 * @brief Decrement Y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_dey(cpu_t *cpu, uint16_t in) {
    cpu->y--;

    cpu_set_flag(cpu, FLAG_ZERO, cpu->y == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->y & 0x80);
}

/**
 * @brief Exclusive OR (with accumulator)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_eor(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = cpu->a ^ m;

    cpu_set_flag(cpu, FLAG_ZERO, res == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);

    cpu->a = res;
}

/**
 * @brief Increment
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_inc(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = m + 1;

    cpu_set_flag(cpu, FLAG_ZERO, res == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);

    cpu_write(cpu, in, res);
}

/**
 * @brief Increment X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_inx(cpu_t *cpu, uint16_t in) {
    cpu->x++;

    cpu_set_flag(cpu, FLAG_ZERO, cpu->x == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->x & 0x80);
}

/**
 * @brief Increment y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_iny(cpu_t *cpu, uint16_t in) {
    cpu->y++;

    cpu_set_flag(cpu, FLAG_ZERO, cpu->y == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->y & 0x80);
}

/**
 * @brief Jump
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_jmp(cpu_t *cpu, uint16_t in) {
    cpu->pc = in;
}

/**
 * @brief Jump to Subroutine
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_jsr(cpu_t *cpu, uint16_t in) {
    cpu->pc--;

    cpu_push(cpu, cpu->pc >> 8);
    cpu_push(cpu, cpu->pc & 0xff);

    cpu->pc = in;
}

/**
 * @brief Load Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_lda(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);

    cpu_set_flag(cpu, FLAG_ZERO, m == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, m & 0x80);

    cpu->a = m;
}

/**
 * @brief Load X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_ldx(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);

    cpu_set_flag(cpu, FLAG_ZERO, m == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, m & 0x80);

    cpu->x = m;
}

/**
 * @brief Load Y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_ldy(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);

    cpu_set_flag(cpu, FLAG_ZERO, m == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, m & 0x80);

    cpu->y = m;
}

/**
 * @brief Logical Shift Right
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_lsr(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = m >> 1;

    cpu_set_flag(cpu, FLAG_CARRY, m & 0x01);
    cpu_set_flag(cpu, FLAG_ZERO, res == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);

    if (in == 0x4a) {
        cpu->a = res;
    } else {
        cpu_write(cpu, in, res);
    }
}

/**
 * @brief No Operation
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_nop(cpu_t *cpu, uint16_t in) {
    /* no implementation */
}

/**
 * @brief Or with Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_ora(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t res = cpu->a | m;

    cpu_set_flag(cpu, FLAG_ZERO, res == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);

    cpu->a = res;
}

/**
 * @brief Push Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_pha(cpu_t *cpu, uint16_t in) {
    cpu_push(cpu, cpu->a);
}

/**
 * @brief Push Processor Status
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_php(cpu_t *cpu, uint16_t in) {
    cpu_push(cpu, cpu->status | FLAG_BREAK);
}

/**
 * @brief Pull Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_pla(cpu_t *cpu, uint16_t in) {
    cpu->a = cpu_pop(cpu);

    cpu_set_flag(cpu, FLAG_ZERO, cpu->a == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->a & 0x80);
}

/**
 * @brief Pull Processor Status
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_plp(cpu_t *cpu, uint16_t in) {
    cpu->status = cpu_pop(cpu) | FLAG_CONSTANT;
}

/**
 * @brief Rotate Left
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_rol(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t c = cpu_get_flag(cpu, FLAG_CARRY);

    uint16_t res = (m << 1) | c;

    cpu_set_flag(cpu, FLAG_CARRY, res > 0xff);
    cpu_set_flag(cpu, FLAG_ZERO, (res & 0xff) == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);

    if (in == 0x2a) {
        cpu->a = res & 0xff;
    } else {
        cpu_write(cpu, in, res & 0xff);
    }
}

/**
 * @brief Rotate Right
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_ror(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t c = cpu_get_flag(cpu, FLAG_CARRY);

    uint16_t res = (m >> 1) | (c << 7);

    cpu_set_flag(cpu, FLAG_CARRY, m & 0x01);
    cpu_set_flag(cpu, FLAG_ZERO, (res & 0xff) == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);

    if (in == 0x6a) {
        cpu->a = res & 0xff;
    } else {
        cpu_write(cpu, in, res & 0xff);
    }
}

/**
 * @brief Return from Interrupt
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_rti(cpu_t *cpu, uint16_t in) {
    cpu->status = cpu_pop(cpu);
    cpu->pc = cpu_pop(cpu);
    cpu->pc |= cpu_pop(cpu) << 8;
}

/**
 * @brief Return from Subroutine
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_rts(cpu_t *cpu, uint16_t in) {
    cpu->pc = cpu_pop(cpu);
    cpu->pc |= cpu_pop(cpu) << 8;

    cpu->pc++;
}

/**
 * @brief Subtract with Carry
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sbc(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu->bus->read(cpu->bus, in);
    uint8_t c = cpu_get_flag(cpu, FLAG_CARRY);

    uint16_t res = cpu->a - m - (1 - c);

    cpu_set_flag(cpu, FLAG_CARRY, res < 0x100);
    cpu_set_flag(cpu, FLAG_ZERO, (res & 0xff) == 0);
    cpu_set_flag(cpu, FLAG_OVERFLOW, (res ^ cpu->a) & (res ^ m) & 0x80);
    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);

    cpu->a = res & 0xff;
}

/**
 * @brief Set Carry
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sec(cpu_t *cpu, uint16_t in) {
    cpu_set_flag(cpu, FLAG_CARRY, 1);
}

/**
 * @brief Set Decimal
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sed(cpu_t *cpu, uint16_t in) {
    cpu_set_flag(cpu, FLAG_DECIMAL, 1);
}

/**
 * @brief Set Interrupt Disable
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sei(cpu_t *cpu, uint16_t in) {
    cpu_set_flag(cpu, FLAG_INTERRUPT, 1);
}

/**
 * @brief Store Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sta(cpu_t *cpu, uint16_t in) {
    cpu_write(cpu, in, cpu->a);
}

/**
 * @brief Store X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_stx(cpu_t *cpu, uint16_t in) {
    cpu_write(cpu, in, cpu->x);
}

/**
 * @brief Store Y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sty(cpu_t *cpu, uint16_t in) {
    cpu_write(cpu, in, cpu->y);
}

/**
 * @brief Transfer Accumulator to X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_tax(cpu_t *cpu, uint16_t in) {
    cpu->x = cpu->a;

    cpu_set_flag(cpu, FLAG_ZERO, cpu->x == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->x & 0x80);
}

/**
 * @brief Transfer Accumulator to Y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_tay(cpu_t *cpu, uint16_t in) {
    cpu->y = cpu->a;

    cpu_set_flag(cpu, FLAG_ZERO, cpu->y == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->y & 0x80);
}

/**
 * @brief Transfer Stack Pointer to X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_tsx(cpu_t *cpu, uint16_t in) {
    cpu->x = cpu->sp;

    cpu_set_flag(cpu, FLAG_ZERO, cpu->x == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->x & 0x80);
}

/**
 * @brief Transfer X to Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_txa(cpu_t *cpu, uint16_t in) {
    cpu->a = cpu->x;

    cpu_set_flag(cpu, FLAG_ZERO, cpu->a == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->a & 0x80);
}

/**
 * @brief Transfer X to Stack Pointer
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_txs(cpu_t *cpu, uint16_t in) {
    cpu->sp = cpu->x;
}

/**
 * @brief Transfer Y to Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_tya(cpu_t *cpu, uint16_t in) {
    cpu->a = cpu->y;

    cpu_set_flag(cpu, FLAG_ZERO, cpu->a == 0);
    cpu_set_flag(cpu, FLAG_NEGATIVE, cpu->a & 0x80);
}
