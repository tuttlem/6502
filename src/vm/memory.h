#ifndef __6502_vm_memory_h__

#define __6502_vm_memory_h__

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct {
    uint8_t *memory;
    uint16_t memory_size;
} memory_t;

#endif /* __6502_vm_memory_h__ */