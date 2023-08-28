#include "./cpu.h"

cpu_addr_t cpu_addr_table[] = {
        cpu_addr_acc,
        cpu_addr_imm,
        cpu_addr_abs,
        cpu_addr_abx,
        cpu_addr_aby,
        cpu_addr_zp,
        cpu_addr_zpx,
        cpu_addr_zpy,
        cpu_addr_ind,
        cpu_addr_inx,
        cpu_addr_iny,
        cpu_addr_rel,
        cpu_addr_imp,
};

/**
 * @brief Get the address of the argument for the given opcode
 * @param cpu The executing cpu
 * @param opcode The opcode to get the argument address for
 * @return The address of the argument
 */
uint16_t cpu_addr(cpu_t *cpu, uint8_t opcode) {
    return cpu_addr_table[opcode >> 2](cpu);
}

/**
 * @brief Argument addressing mode: accumulator
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_acc(cpu_t *cpu) {
    return 0;
}

/**
 * @brief Argument addressing mode: implied
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_imp(cpu_t *cpu) {
    return 0;
}

/**
 * @brief Argument addressing mode: immediate
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_imm(cpu_t *cpu) {
    return cpu->pc++;
}

/**
 * @brief Argument addressing mode: zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zp(cpu_t *cpu) {
    return cpu_read(cpu, cpu->pc++);
}

/**
 * @brief Argument addressing mode: indexed-x zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zpx(cpu_t *cpu) {
    return (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xff;
}

/**
 * @brief Argument addressing mode: indexed-y zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zpy(cpu_t *cpu) {
    return (cpu_read(cpu, cpu->pc++) + cpu->y) & 0xff;
}

/**
 * @brief Argument addressing mode: relative
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_rel(cpu_t *cpu) {
    return cpu_read(cpu, cpu->pc++);
}

/**
 * @brief Argument addressing mode: absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_abs(cpu_t *cpu) {
    uint16_t lo = cpu_read(cpu, cpu->pc++);
    uint16_t hi = cpu_read(cpu, cpu->pc++);

    return (hi << 8) | lo;
}

/**
 * @brief Argument addressing mode: indexed-x absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_abx(cpu_t *cpu) {
    uint16_t lo = cpu_read(cpu, cpu->pc++);
    uint16_t hi = cpu_read(cpu, cpu->pc++);

    return ((hi << 8) | lo) + cpu->x;
}

/**
 * @brief Argument addressing mode: indexed-y absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_aby(cpu_t *cpu) {
    uint16_t lo = cpu_read(cpu, cpu->pc++);
    uint16_t hi = cpu_read(cpu, cpu->pc++);

    return ((hi << 8) | lo) + cpu->y;
}

/**
 * @brief Argument addressing mode: absolute indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_ind(cpu_t *cpu) {
    uint16_t lo = cpu_read(cpu, cpu->pc++);
    uint16_t hi = cpu_read(cpu, cpu->pc++);

    uint16_t ptr = (hi << 8) | lo;

    lo = cpu_read(cpu, ptr);
    hi = cpu_read(cpu, ptr + 1);

    return (hi << 8) | lo;
}

/**
 * @brief Argument addressing mode: indexed-x indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_inx(cpu_t *cpu) {
    uint16_t ptr = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xff;

    uint16_t lo = cpu_read(cpu, ptr);
    uint16_t hi = cpu_read(cpu, ptr + 1);

    return (hi << 8) | lo;
}

/**
 * @brief Argument addressing mode: indexed-y indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_iny(cpu_t *cpu) {
    uint16_t ptr = cpu_read(cpu, cpu->pc++);

    uint16_t lo = cpu_read(cpu, ptr);
    uint16_t hi = cpu_read(cpu, ptr + 1);

    return ((hi << 8) | lo) + cpu->y;
}
