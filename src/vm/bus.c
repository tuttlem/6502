
#include "./bus.h"

/**
 * @brief Create a new bus
 * @return A pointer to the new bus
 */
bus_t *bus_create(void) {
    bus_t *bus = malloc(sizeof(bus_t));
    assert(bus != NULL);

    return bus;
}

/**
 * @brief Destroy a bus
 * @param bus The bus to destroy
 */
void bus_destroy(bus_t *bus) {
    assert(bus != NULL);

    free(bus);
}
