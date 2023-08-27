
#include "system.h"

uint8_t system_reader(void *ptr, uint16_t address);
void system_writer(void *ptr, uint16_t address, uint8_t value);


/**
 * @brief Create a new system
 * @return A pointer to the new system
 */
system_t *system_create() {
    system_t *system = malloc(sizeof(system_t));
    assert(system != NULL);

    system->cpu = cpu_create();

    system->bus = bus_create();
    system->bus->system = system;
    system->bus->read = system_reader;
    system->bus->write = system_writer;

    system->cpu->bus = system->bus;

    system->memory_size = SYSTEM_MEMORY_SIZE;
    system->memory = malloc(system->memory_size);
    assert(system->memory != NULL);

    system->cpu->a = 0;
    system->cpu->x = 0;
    system->cpu->y = 0;

    system->cpu->sp = 0;
    system->cpu->pc = 0;

    system->cpu->status = 0xff;

    return system;
}

/**
 * @brief Destroy a system
 * @param system The system to destroy
 */
void system_destroy(system_t *system) {
    assert(system != NULL);

    cpu_destroy(system->cpu);
    bus_destroy(system->bus);
    free(system->memory);
    free(system);

    system = NULL;
}

/**
 * @brief Reset a system
 * @param system The system to reset
 */
void system_reset(system_t *system) {
    assert(system != NULL);

    system->cpu->a = 0;
    system->cpu->x = 0;
    system->cpu->y = 0;

    system->cpu->sp = 0xFD; /* hardcoded stack vector post-reset */
    system->cpu->pc = 0xFCE2; /* hardcoded start vector post-reset */

    cpu_set_flag(system->cpu, FLAG_INTERRUPT, 1);
    cpu_set_flag(system->cpu, FLAG_DECIMAL, 0);
    cpu_set_flag(system->cpu, FLAG_BREAK, 1);
    cpu_set_flag(system->cpu, FLAG_CONSTANT, 1);
}

/**
 * @brief brief Test a system
 * @param system The system to test
 */
void system_test(system_t *system) {
    assert(system != NULL);

    assert(system->cpu->pc == 0xFCE2);
    assert(system->cpu->sp == 0xFD);
//    assert(system->cpu->status == (FLAG_INTERRUPT | FLAG_BREAK | FLAG_CONSTANT));
}

/**
 * @brief Read from the system
 * @param ptr The bus structure to read from
 * @param address The address to read from
 * @return The value at the address
 */
uint8_t system_reader(void *ptr, uint16_t address) {
    assert(ptr != NULL);

    bus_t *b = (bus_t *)ptr;
    assert(b->system != NULL);
    system_t *s = (system_t *)b->system;

    return s->memory[address];
}

/**
 * @brief Write to the system
 * @param ptr The bus structure to write to
 * @param address The address to write to
 * @param value The value to write
 */
void system_writer(void *ptr, uint16_t address, uint8_t value) {
    assert(ptr != NULL);

    bus_t *b = (bus_t *)ptr;
    assert(b->system != NULL);
    system_t *s = (system_t *)b->system;

    s->memory[address] = value;
}

