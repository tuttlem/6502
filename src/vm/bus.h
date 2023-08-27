#ifndef __6502_vm_bus_h__

#define __6502_vm_bus_h__

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

typedef void (*bus_write_t)(void*, uint16_t, uint8_t);
typedef uint8_t (*bus_read_t)(void*, uint16_t);

typedef struct {
    bus_write_t write;
    bus_read_t read;

    void *system;
} bus_t;

/**
 * @brief Create a new bus
 * @return A pointer to the new bus
 */
bus_t *bus_create(void);

/**
 * @brief Destroy a bus
 * @param bus The bus to destroy
 */
void bus_destroy(bus_t *bus);

#endif /* __6502_vm_bus_h__ */