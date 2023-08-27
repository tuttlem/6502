#include <stdio.h>

#include "vm/system.h"

void diag(cpu_t *cpu) {
    printf("A: %X X: %x Y: %X\n", cpu->a, cpu->x, cpu->y);
    printf("PC: %X SP: %X\n", cpu->pc, cpu->sp);
    printf("Status: %X\n", cpu->status);
}

int main() {
    system_t *system = system_create();
    system_reset(system);
    system_test(system);

    diag(system->cpu);

    cpu_stack_push(system->cpu, 0xDE);
    cpu_stack_push(system->cpu, 0xAD);
    cpu_stack_push(system->cpu, 0xBE);
    cpu_stack_push(system->cpu, 0xEF);

    for (int i = 0; i < 4; i ++) {
        uint8_t v = cpu_stack_pop(system->cpu);
        printf("Popped %X\n", v);
    }

    diag(system->cpu);
    system_destroy(system);

    return 0;
}
