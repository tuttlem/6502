#include "./cpu.h"

cpu_op_t op_table[] = {
    cpu_op_brk,cpu_op_ora,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_ora,cpu_op_asl,cpu_op_unk,
    cpu_op_php,cpu_op_ora,cpu_op_asl,cpu_op_unk,cpu_op_unk,cpu_op_ora,cpu_op_asl,cpu_op_unk,
    cpu_op_bpl,cpu_op_ora,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_ora,cpu_op_asl,cpu_op_unk,
    cpu_op_clc, cpu_op_ora,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_ora,cpu_op_asl,cpu_op_unk,
    cpu_op_jsr,cpu_op_and,cpu_op_unk,cpu_op_unk,cpu_op_bit,cpu_op_and,cpu_op_rol,cpu_op_unk,
    cpu_op_plp,cpu_op_and,cpu_op_rol,cpu_op_unk,cpu_op_bit,cpu_op_and,cpu_op_rol,cpu_op_unk,
    cpu_op_bmi,cpu_op_and,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_and,cpu_op_rol,cpu_op_unk,
    cpu_op_sec,cpu_op_and,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_and,cpu_op_rol,cpu_op_unk,
    cpu_op_rti,cpu_op_eor,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_eor,cpu_op_lsr,cpu_op_unk,
    cpu_op_pha,cpu_op_eor,cpu_op_lsr,cpu_op_unk,cpu_op_jmp,cpu_op_eor,cpu_op_lsr,cpu_op_unk,
    cpu_op_bvc,cpu_op_eor,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_eor,cpu_op_lsr,cpu_op_unk,
    cpu_op_cli,cpu_op_eor,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_eor,cpu_op_lsr,cpu_op_unk,
    cpu_op_rts,cpu_op_adc,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_adc,cpu_op_ror,cpu_op_unk,
    cpu_op_pla,cpu_op_adc,cpu_op_ror,cpu_op_unk,cpu_op_jmp,cpu_op_adc,cpu_op_ror,cpu_op_unk,
    cpu_op_bvs,cpu_op_adc,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_adc,cpu_op_ror,cpu_op_unk,
    cpu_op_sei,cpu_op_adc,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_adc,cpu_op_ror,cpu_op_unk,
    cpu_op_unk,cpu_op_sta,cpu_op_unk,cpu_op_unk,cpu_op_sty,cpu_op_sta,cpu_op_stx,cpu_op_unk,
    cpu_op_dey,cpu_op_unk,cpu_op_txa,cpu_op_unk,cpu_op_sty,cpu_op_sta,cpu_op_stx,cpu_op_unk,
    cpu_op_bcc,cpu_op_sta,cpu_op_unk,cpu_op_unk,cpu_op_sty,cpu_op_sta,cpu_op_stx,cpu_op_unk,
    cpu_op_tya,cpu_op_sta,cpu_op_txs,cpu_op_unk,cpu_op_unk,cpu_op_sta,cpu_op_unk,cpu_op_unk,
    cpu_op_ldy,cpu_op_lda,cpu_op_ldx,cpu_op_unk,cpu_op_ldy,cpu_op_lda,cpu_op_ldx,cpu_op_unk,
    cpu_op_tay,cpu_op_lda,cpu_op_tax,cpu_op_unk,cpu_op_ldy,cpu_op_lda,cpu_op_ldx,cpu_op_unk,
    cpu_op_bcs,cpu_op_lda,cpu_op_unk,cpu_op_unk,cpu_op_ldy,cpu_op_lda,cpu_op_ldx,cpu_op_unk,
    cpu_op_clv,cpu_op_lda,cpu_op_tsx,cpu_op_unk,cpu_op_ldy,cpu_op_lda,cpu_op_ldx,cpu_op_unk,
    cpu_op_cpy,cpu_op_cmp,cpu_op_unk,cpu_op_unk,cpu_op_cpy,cpu_op_cmp,cpu_op_dec,cpu_op_unk,
    cpu_op_iny,cpu_op_cmp,cpu_op_dex,cpu_op_unk,cpu_op_cpy,cpu_op_cmp,cpu_op_dec,cpu_op_unk,
    cpu_op_bne,cpu_op_cmp,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_cmp,cpu_op_dec,cpu_op_unk,
    cpu_op_cld,cpu_op_cmp,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_cmp,cpu_op_dec,cpu_op_unk,
    cpu_op_cpx,cpu_op_sbc,cpu_op_unk,cpu_op_unk,cpu_op_cpx,cpu_op_sbc,cpu_op_inc,cpu_op_unk,
    cpu_op_inx,cpu_op_sbc,cpu_op_nop,cpu_op_unk,cpu_op_cpx,cpu_op_sbc,cpu_op_inc,cpu_op_unk,
    cpu_op_beq,cpu_op_sbc,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_sbc,cpu_op_inc,cpu_op_unk,
    cpu_op_sed,cpu_op_sbc,cpu_op_unk,cpu_op_unk,cpu_op_unk,cpu_op_sbc,cpu_op_inc,cpu_op_unk,
};

/**
 * @brief Unknown
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_unk(cpu_t *cpu, uint16_t in) {
    /* no implementation */
}

/**
 * @brief AND (with accumulator)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_and(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu_addr(cpu, in);
    uint8_t res = cpu->a & m;

    cpu_set_flag(cpu, FLAG_NEGATIVE, res & 0x80);
    cpu_set_flag(cpu, FLAG_ZERO, res == 0);

    cpu->a = res;
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