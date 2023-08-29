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
 * @brief Add with Carry
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_adc(cpu_t *cpu, uint16_t in) {
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);

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
    uint8_t m = cpu_addr(cpu, in);

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
    uint8_t m = cpu_addr(cpu, in);

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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
    uint8_t m = cpu_addr(cpu, in);
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
