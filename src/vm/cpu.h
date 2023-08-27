#ifndef __6502_vm_cpu_h__

#define __6502_vm_cpu_h__

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

#include "bus.h"
#include "opcode.h"

#define FLAG_CARRY      0x01
#define FLAG_ZERO       0x02
#define FLAG_INTERRUPT  0x04
#define FLAG_DECIMAL    0x08
#define FLAG_BREAK      0x10
#define FLAG_CONSTANT   0x20
#define FLAG_OVERFLOW   0x40
#define FLAG_NEGATIVE   0x80

#define IF_NEGATIVE(status) ((status & FLAG_NEGATIVE))
#define IF_OVERFLOW(status) ((status & FLAG_OVERFLOW))
#define IF_CONSTANT(status) ((status & FLAG_CONSTANT))
#define IF_BREAK(status) ((status & FLAG_BREAK))
#define IF_DECIMAL(status) ((status & FLAG_DECIMAL))
#define IF_INTERRUPT(status) ((status & FLAG_INTERRUPT))
#define IF_ZERO(status) ((status & FLAG_ZERO))
#define IF_CARRY(status) ((status & FLAG_CARRY))

typedef struct {
    uint8_t a;      /* accumulator */
    uint8_t x;      /* index register x */
    uint8_t y;      /* index register y */
    uint8_t sp;     /* stack pointer */
    uint16_t pc;    /* program counter */
    uint8_t status; /* status register */

    bus_t *bus;     /* communication bus */
} cpu_t;

/**
 * @brief Create a new cpu
 * @return A pointer to the new cpu
 */
cpu_t *cpu_create(void);

/**
 * @brief Destroy a cpu
 * @param cpu The cpu to destroy
 */
void cpu_destroy(cpu_t *cpu);

/**
 * @brief Reset a cpu
 * @param cpu The cpu to reset
 * @param flag The flag to set
 * @param value The value to set the flag to
 */
void cpu_set_flag(cpu_t *cpu, uint8_t flag, uint8_t value);

/**
 * @brief Reset a cpu
 * @param cpu The cpu to reset
 */
void cpu_reset(cpu_t *cpu);

/**
 * @brief Run a cpu
 * @param cpu The cpu to test
 */
void cpu_test(cpu_t *cpu);

/**
 * @brief Writes a value to the bus
 * @param cpu The cpu wanting to write
 * @param address The address to write to
 * @param data The data to write
 */
void cpu_bus_write(cpu_t *cpu, uint16_t address, uint8_t data);

/**
 * @brief Reads a value from the bus
 * @param cpu The cpu wanting to read
 * @param address The address to read from
 * @return The data read from the bus
 */
uint8_t cpu_bus_read(cpu_t *cpu, uint16_t address);

/**
 * @brief Argument addressing mode: accumulator
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_acc(cpu_t *cpu);

/**
 * @brief Argument addressing mode: implied
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_imp(cpu_t *cpu);

/**
 * @brief Argument addressing mode: immediate
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_imm(cpu_t *cpu);

/**
 * @brief Argument addressing mode: zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zp(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-x zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zpx(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-y zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zpy(cpu_t *cpu);

/**
 * @brief Argument addressing mode: relative
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_rel(cpu_t *cpu);

/**
 * @brief Argument addressing mode: absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_abs(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-x absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_abx(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-y absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_aby(cpu_t *cpu);

/**
 * @brief Argument addressing mode: absolute indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_ind(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-x indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_inx(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-y indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_iny(cpu_t *cpu);


#endif /* __6502_vm_cpu_h__ */