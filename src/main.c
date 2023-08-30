#include <stdio.h>

#include "vm/system.h"

void diag(cpu_t *cpu) {
    printf("A: %X X: %x Y: %X\n", cpu->a, cpu->x, cpu->y);
    printf("PC: %X SP: %X\n", cpu->pc, cpu->sp);
    printf("Status: %X\n", cpu->status);
    printf("==========\n");
}

int main() {

    printf("%x\n", cpu_get_opcode(0).cycles);

    system_t *system = system_create();
    system_reset(system);
    system_test(system);

    diag(system->cpu);

    // LDA #6
    system->memory->data[system->cpu->pc] = 169;
    system->memory->data[system->cpu->pc + 1] = 6;

    // AND #1
    system->memory->data[system->cpu->pc + 2] = 41;
    system->memory->data[system->cpu->pc + 3] = 1;


    cpu_step(system->cpu);
    diag(system->cpu);

    cpu_step(system->cpu);
    diag(system->cpu);

    system_destroy(system);

    return 0;
}
