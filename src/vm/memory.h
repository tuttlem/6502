#ifndef __6502_vm_memory_h__

#define __6502_vm_memory_h__

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct {
    uint8_t *data;
    uint16_t size;
} memory_t;

/**
 * @brief Create a new memory module
 * @param memory_size The size of the memory module
 * @return A pointer to the new memory module
 */
memory_t *memory_create(uint16_t memory_size);

/**
 * @brief Destroy a memory module
 * @param memory The memory module to destroy
 */
void memory_destroy(memory_t *memory);

#endif /* __6502_vm_memory_h__ */