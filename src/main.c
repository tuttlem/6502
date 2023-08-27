#include <stdio.h>

#include "vm/system.h"

int main() {
    system_t *system = system_create();
    system_reset(system);
    system_test(system);

    uint8_t b = cpu_bus_read(system->cpu, 0x0000);

    printf("Value at 0x0000 is %X\n", b);

    system_destroy(system);

    return 0;
}
