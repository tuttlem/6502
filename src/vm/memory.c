#include "./memory.h"

/**
 * @brief Create a new memory module
 * @param memory_size The size of the memory module
 * @return A pointer to the new memory module
 */
memory_t *memory_create(uint16_t memory_size) {
    memory_t *memory = malloc(sizeof(memory_t));
    assert(memory != NULL);

    memory->size = memory_size;
    memory->data = malloc(memory->size);
    assert(memory->data != NULL);

    return memory;
}

/**
 * @brief Destroy a memory module
 * @param memory The memory module to destroy
 */
void memory_destroy(memory_t *memory) {
    assert(memory != NULL);

    free(memory->data);
    free(memory);

    memory = NULL;
}
