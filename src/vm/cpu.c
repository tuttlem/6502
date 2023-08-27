
#include "./cpu.h"


/**
 * @brief Create a new cpu
 * @return A pointer to the new cpu
 */
cpu_t *cpu_create() {
    cpu_t *cpu = malloc(sizeof(cpu_t));
    assert(cpu != NULL);

    cpu->bus = NULL;

    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;

    cpu->sp = 0;
    cpu->pc = 0;

    cpu->status = 0xff;

    return cpu;
}

/**
 * @brief Destroy a cpu
 * @param cpu The cpu to destroy
 */
void cpu_destroy(cpu_t *cpu) {
    assert(cpu != NULL);

    free(cpu);
}

/**
 * @brief Set a flag on the cpu
 * @param cpu The cpu to set the flag on
 * @param flag The flag to set
 * @param value The value to set the flag to
 */
void cpu_set_flag(cpu_t *cpu, uint8_t flag, uint8_t value) {
    assert(cpu != NULL);

    if (value) {
        cpu->status |= flag;
    } else {
        cpu->status &= ~flag;
    }
}

/**
 * @brief Reset a cpu
 * @param cpu The cpu to reset
 */

void cpu_reset(cpu_t *cpu) {
    assert(cpu != NULL);

    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;

    cpu->sp = 0xFD; /* hardcoded stack vector post-reset */
    cpu->pc = 0xFCE2; /* hardcoded start vector post-reset */

    cpu_set_flag(cpu, FLAG_INTERRUPT, 1);
    cpu_set_flag(cpu, FLAG_DECIMAL, 0);
    cpu_set_flag(cpu, FLAG_BREAK, 1);
    cpu_set_flag(cpu, FLAG_CONSTANT, 1);
}

/**
 * @brief Run a cpu
 * @param cpu The cpu to test
 */
void cpu_test(cpu_t *cpu) {
    assert(cpu != NULL);

    assert(cpu->pc == 0xFCE2);
    assert(cpu->sp == 0x01FD);
    assert(cpu->status == (FLAG_INTERRUPT | FLAG_BREAK | FLAG_CONSTANT));
}

/**
 * @brief Writes a value to the bus
 * @param cpu The cpu wanting to write
 * @param address The address to write to
 * @param data The data to write
 */
void cpu_bus_write(cpu_t *cpu, uint16_t address, uint8_t data) {
    assert(cpu != NULL);
    assert(cpu->bus != NULL);

    cpu->bus->write(cpu->bus, address, data);
}

/**
 * @brief Reads a value from the bus
 * @param cpu The cpu wanting to read
 * @param address The address to read from
 * @return The data read from the bus
 */
uint8_t cpu_bus_read(cpu_t *cpu, uint16_t address) {
    assert(cpu != NULL);
    assert(cpu->bus != NULL);

    return cpu->bus->read(cpu->bus, address);
}

typedef uint16_t (*cpu_addr_t)(cpu_t *cpu);

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
    return cpu_bus_read(cpu, cpu->pc++);
}

/**
 * @brief Argument addressing mode: indexed-x zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zpx(cpu_t *cpu) {
    return (cpu_bus_read(cpu, cpu->pc++) + cpu->x) & 0xff;
}

/**
 * @brief Argument addressing mode: indexed-y zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zpy(cpu_t *cpu) {
    return (cpu_bus_read(cpu, cpu->pc++) + cpu->y) & 0xff;
}

/**
 * @brief Argument addressing mode: relative
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_rel(cpu_t *cpu) {
    return cpu_bus_read(cpu, cpu->pc++);
}

/**
 * @brief Argument addressing mode: absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_abs(cpu_t *cpu) {
    uint16_t lo = cpu_bus_read(cpu, cpu->pc++);
    uint16_t hi = cpu_bus_read(cpu, cpu->pc++);

    return (hi << 8) | lo;
}

/**
 * @brief Argument addressing mode: indexed-x absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_abx(cpu_t *cpu) {
    uint16_t lo = cpu_bus_read(cpu, cpu->pc++);
    uint16_t hi = cpu_bus_read(cpu, cpu->pc++);

    return ((hi << 8) | lo) + cpu->x;
}

/**
 * @brief Argument addressing mode: indexed-y absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_aby(cpu_t *cpu) {
    uint16_t lo = cpu_bus_read(cpu, cpu->pc++);
    uint16_t hi = cpu_bus_read(cpu, cpu->pc++);

    return ((hi << 8) | lo) + cpu->y;
}

/**
 * @brief Argument addressing mode: absolute indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_ind(cpu_t *cpu) {
    uint16_t lo = cpu_bus_read(cpu, cpu->pc++);
    uint16_t hi = cpu_bus_read(cpu, cpu->pc++);

    uint16_t ptr = (hi << 8) | lo;

    lo = cpu_bus_read(cpu, ptr);
    hi = cpu_bus_read(cpu, ptr + 1);

    return (hi << 8) | lo;
}

/**
 * @brief Argument addressing mode: indexed-x indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_inx(cpu_t *cpu) {
    uint16_t ptr = (cpu_bus_read(cpu, cpu->pc++) + cpu->x) & 0xff;

    uint16_t lo = cpu_bus_read(cpu, ptr);
    uint16_t hi = cpu_bus_read(cpu, ptr + 1);

    return (hi << 8) | lo;
}

/**
 * @brief Argument addressing mode: indexed-y indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_iny(cpu_t *cpu) {
    uint16_t ptr = cpu_bus_read(cpu, cpu->pc++);

    uint16_t lo = cpu_bus_read(cpu, ptr);
    uint16_t hi = cpu_bus_read(cpu, ptr + 1);

    return ((hi << 8) | lo) + cpu->y;
}

