
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

    cpu->pc = (cpu_read(cpu, RST_VECTOR_H) << 8) | cpu_read(cpu, RST_VECTOR_L);
    cpu->sp = 0xFD;

    cpu->status = FLAG_CONSTANT | FLAG_BREAK;
}

/**
 * @brief Run a cpu
 * @param cpu The cpu to test
 */
void cpu_test(cpu_t *cpu) {
    assert(cpu != NULL);

    assert(cpu->pc == 0xFCE2);
    assert(cpu->sp == 0xFD);
    assert(cpu->status == (FLAG_INTERRUPT | FLAG_BREAK | FLAG_CONSTANT));
}

/**
 * @brief Writes a value to the bus
 * @param cpu The cpu wanting to write
 * @param address The address to write to
 * @param data The data to write
 */
void cpu_write(cpu_t *cpu, uint16_t address, uint8_t data) {
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
uint8_t cpu_read(cpu_t *cpu, uint16_t address) {
    assert(cpu != NULL);
    assert(cpu->bus != NULL);

    return cpu->bus->read(cpu->bus, address);
}

/**
 * @brief Pops a value off of the stack
 * @param cpu The cpu wanting to pop
 * @return The value popped off of the stack
 */
uint8_t cpu_pop(cpu_t *cpu) {
    assert(cpu != NULL);
    return cpu_read(cpu, 0x100 | ++cpu->sp);
}

/**
 * @brief Pushes a value onto the stack
 * @param cpu The cpu wanting to push
 * @param value The value to push onto the stack
 */
void cpu_push(cpu_t *cpu, uint8_t value) {
    assert(cpu != NULL);
    cpu_write(cpu, 0x100 | cpu->sp--, value);
}


/**
 * @brief Interrupts the cpu to execute a request
 * @param cpu The cpu to interrupt
 */
void cpu_irq(cpu_t *cpu) {
    assert(cpu != NULL);

    if (cpu->status & FLAG_INTERRUPT) {
        cpu_push(cpu, (cpu->pc >> 8) & 0xff);
        cpu_push(cpu, cpu->pc & 0xff);

        cpu_push(cpu, cpu->status);

        cpu_set_flag(cpu, FLAG_INTERRUPT, 1);
        cpu_set_flag(cpu, FLAG_BREAK, 0);

        cpu->pc = (cpu_read(cpu, IRQ_VECTOR_H) << 8) | cpu_read(cpu, IRQ_VECTOR_L);
    }
}

/**
 * @brief Non-maskable interrupts request
 * @param cpu The cpu to interrupt
 */
void cpu_nmi(cpu_t *cpu) {
    assert(cpu != NULL);

    cpu_push(cpu, (cpu->pc >> 8) & 0xff);
    cpu_push(cpu, cpu->pc & 0xff);

    cpu_push(cpu, cpu->status);

    cpu_set_flag(cpu, FLAG_INTERRUPT, 1);
    cpu_set_flag(cpu, FLAG_BREAK, 0);

    cpu->pc = (cpu_read(cpu, NMI_VECTOR_H) << 8) | cpu_read(cpu, NMI_VECTOR_L);
}


