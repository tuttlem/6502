#ifndef __6502_vm_system_h__

#define __6502_vm_system_h__

#include "cpu.h"

#include <assert.h>
#include <stdlib.h>

#define SYSTEM_MEMORY_SIZE 0x10000

#define SYSTEM_IRQ_VECTOR_H 0xFFFF
#define SYSTEM_IRQ_VECTOR_L 0xFFFE
#define SYSTEM_RESET_VECTOR_H 0xFFFD
#define SYSTEM_RESET_VECTOR_L 0xFFFC
#define SYSTEM_NMI_VECTOR_H 0xFFFB
#define SYSTEM_NMI_VECTOR_L 0xFFFA


typedef struct {
    cpu_t *cpu;                 /* system processor */
    bus_t *bus;                 /* communication bus */

    uint8_t *memory;
    uint16_t memory_size;
} system_t;

/**
 * @brief Create a new system
 * @return A pointer to the new system
 */
system_t *system_create(void);

/**
 * @brief Destroy a system
 * @param system The system to destroy
 */
void system_destroy(system_t *system);

/**
 * @brief Reset a system
 * @param system The system to reset
 */
void system_reset(system_t *system);

/**
 * @brief brief Test a system
 * @param system The system to test
 */
void system_test(system_t *system);

/**
 * @brief Load a program into a system
 * @param system The system to load the program into
 * @param program The program to load
 * @param program_size The size of the program
 */
void system_load(system_t *system, uint8_t *program, uint16_t program_size);

/**
 * @brief Run a system
 * @param system The system to run
 */
void system_run(system_t *system);

/**
 * @brief Sets a system flag
 * @param system The system to set the flag on
 * @param flag The flag to set
 * @param value The value to set the flag to
 */
void system_set_flag(system_t *system, uint8_t flag, uint8_t value);

#endif /* __6502_vm_system_h__ */