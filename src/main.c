#include <stdio.h>

#include "vm/system.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

void diag(cpu_t *cpu) {
    printf("A:  0x%X X: 0x%x Y: 0x%X PC: 0x%X SP: 0x%X (%d)\n", cpu->a, cpu->x, cpu->y, cpu->pc, cpu->sp, cpu->cycles);
    printf("NO_BDIZC\n", cpu->status);
    printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(cpu->status));
    printf("\n====================================================\n");
}

int main() {

    system_t *system = system_create();
    system_reset(system);
    system_test(system);

    diag(system->cpu);

    // LDA $06
    system->memory->data[system->cpu->pc] = 169;
    system->memory->data[system->cpu->pc + 1] = 6;

    // AND $02
    system->memory->data[system->cpu->pc + 2] = 41;
    system->memory->data[system->cpu->pc + 3] = 2;

    // illegal opcode
    system->memory->data[system->cpu->pc + 4] = 0x02;


    // cpu_step(system->cpu);
    // diag(system->cpu);
    // cpu_step(system->cpu);

    cpu_run(system->cpu);
    diag(system->cpu);

    system_destroy(system);

    return 0;
}
