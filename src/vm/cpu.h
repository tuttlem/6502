#ifndef __6502_vm_cpu_h__

#define __6502_vm_cpu_h__

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

#include "bus.h"
#include "opcode.h"

#define FLAG_CARRY      0x01
#define FLAG_ZERO       0x02
#define FLAG_INTERRUPT  0x04
#define FLAG_DECIMAL    0x08
#define FLAG_BREAK      0x10
#define FLAG_CONSTANT   0x20
#define FLAG_OVERFLOW   0x40
#define FLAG_NEGATIVE   0x80

#define IF_NEGATIVE(status) ((status & FLAG_NEGATIVE))
#define IF_OVERFLOW(status) ((status & FLAG_OVERFLOW))
#define IF_CONSTANT(status) ((status & FLAG_CONSTANT))
#define IF_BREAK(status) ((status & FLAG_BREAK))
#define IF_DECIMAL(status) ((status & FLAG_DECIMAL))
#define IF_INTERRUPT(status) ((status & FLAG_INTERRUPT))
#define IF_ZERO(status) ((status & FLAG_ZERO))
#define IF_CARRY(status) ((status & FLAG_CARRY))

#define IRQ_VECTOR_H 0xFFFF
#define IRQ_VECTOR_L 0xFFFE
#define NMI_VECTOR_H 0xFFFB
#define NMI_VECTOR_L 0xFFFA
#define RST_VECTOR_H 0xFFFD
#define RST_VECTOR_L 0xFFFC

typedef struct {
    uint8_t a;      /* accumulator */
    uint8_t x;      /* index register x */
    uint8_t y;      /* index register y */
    uint8_t sp;     /* stack pointer */
    uint16_t pc;    /* program counter */
    uint8_t status; /* status register */

    bus_t *bus;     /* communication bus */
} cpu_t;

typedef uint16_t (*cpu_addr_t)(cpu_t*);
typedef void (*cpu_op_t)(cpu_t*, uint16_t);

/**
 * @brief Create a new cpu
 * @return A pointer to the new cpu
 */
cpu_t *cpu_create(void);

/**
 * @brief Destroy a cpu
 * @param cpu The cpu to destroy
 */
void cpu_destroy(cpu_t *cpu);

/**
 * @brief Reset a cpu
 * @param cpu The cpu to reset
 * @param flag The flag to set
 * @param value The value to set the flag to
 */
void cpu_set_flag(cpu_t *cpu, uint8_t flag, uint8_t value);

/**
 * @brief Get a flag from the cpu
 * @param cpu The cpu to get the flag from
 * @param flag The flag to get
 * @return The value of the flag
 */
uint8_t cpu_get_flag(cpu_t *cpu, uint8_t flag);

/**
 * @brief Reset a cpu
 * @param cpu The cpu to reset
 */
void cpu_reset(cpu_t *cpu);

/**
 * @brief Run a cpu
 * @param cpu The cpu to test
 */
void cpu_test(cpu_t *cpu);

/**
 * @brief Writes a value to the bus
 * @param cpu The cpu wanting to write
 * @param address The address to write to
 * @param data The data to write
 */
void cpu_write(cpu_t *cpu, uint16_t address, uint8_t data);

/**
 * @brief Reads a value from the bus
 * @param cpu The cpu wanting to read
 * @param address The address to read from
 * @return The data read from the bus
 */
uint8_t cpu_read(cpu_t *cpu, uint16_t address);

/**
 * @brief Pops a value off of the stack
 * @param cpu The cpu wanting to pop
 * @return The value popped off of the stack
 */
uint8_t cpu_pop(cpu_t *cpu);

/**
 * @brief Pushes a value onto the stack
 * @param cpu The cpu wanting to push
 * @param value The value to push onto the stack
 */
void cpu_push(cpu_t *cpu, uint8_t value);

/**
 * @brief Interrupts the cpu to execute a request
 * @param cpu The cpu to interrupt
 */
void cpu_irq(cpu_t *cpu);

/**
 * @brief Non-maskable interrupts request
 * @param cpu The cpu to interrupt
 */
void cpu_nmi(cpu_t *cpu);

/**
 * @brief Get the address of the argument for the given opcode
 * @param cpu The executing cpu
 * @param opcode The opcode to get the argument address for
 * @return The address of the argument
 */
uint16_t cpu_addr(cpu_t *cpu, uint8_t opcode);

/**
 * @brief Argument addressing mode: accumulator
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_acc(cpu_t *cpu);

/**
 * @brief Argument addressing mode: implied
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_imp(cpu_t *cpu);

/**
 * @brief Argument addressing mode: immediate
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_imm(cpu_t *cpu);

/**
 * @brief Argument addressing mode: zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zp(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-x zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zpx(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-y zero page
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_zpy(cpu_t *cpu);

/**
 * @brief Argument addressing mode: relative
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_rel(cpu_t *cpu);

/**
 * @brief Argument addressing mode: absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_abs(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-x absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_abx(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-y absolute
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_aby(cpu_t *cpu);

/**
 * @brief Argument addressing mode: absolute indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_ind(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-x indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_inx(cpu_t *cpu);

/**
 * @brief Argument addressing mode: indexed-y indirect
 * @param cpu The executing cpu
 * @return The value
 */
uint16_t cpu_addr_iny(cpu_t *cpu);

/**
 * @brief Unknown
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_unk(cpu_t *cpu, uint16_t in);

/**
 * @brief Add with Carry
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_adc(cpu_t *cpu, uint16_t in);

/**
 * @brief AND (with accumulator)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_and(cpu_t *cpu, uint16_t in);

/**
 * @brief Arithmetic Shift Left
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_asl(cpu_t *cpu, uint16_t in);

/**
 * @brief Branch on Carry Clear
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bcc(cpu_t *cpu, uint16_t in);

/**
 * @brief Branch on Carry Set
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bcs(cpu_t *cpu, uint16_t in);

/**
 * @brief Branch on equal (zero set)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_beq(cpu_t *cpu, uint16_t in);

/**
 * @brief Bit Test
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bit(cpu_t *cpu, uint16_t in);

/**
 * @brief Branch on Minus (negative set)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bmi(cpu_t *cpu, uint16_t in);

/**
 * @brief Branch on Not Equal (zero clear)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bne(cpu_t *cpu, uint16_t in);

/**
 * @brief Branch on Plus (negative clear)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bpl(cpu_t *cpu, uint16_t in);

/**
 * @brief Force Break
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_brk(cpu_t *cpu, uint16_t in);

/**
 * @brief Branch on Overflow Clear
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bvc(cpu_t *cpu, uint16_t in);

/**
 * @brief Branch on Overflow Set
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_bvs(cpu_t *cpu, uint16_t in);

/**
 * @brief Clear Carry
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_clc(cpu_t *cpu, uint16_t in);

/**
 * @brief Clear Decimal
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cld(cpu_t *cpu, uint16_t in);

/**
 * @brief Clear Interrupt
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cli(cpu_t *cpu, uint16_t in);

/**
 * @brief Clear Overflow
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_clv(cpu_t *cpu, uint16_t in);

/**
 * @brief Compare (with accumulator)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cmp(cpu_t *cpu, uint16_t in);

/**
 * @brief Compare (with x)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cpx(cpu_t *cpu, uint16_t in);

/**
 * @brief Compare (with y)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_cpy(cpu_t *cpu, uint16_t in);

/**
 * @brief Decrement
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_dec(cpu_t *cpu, uint16_t in);

/**
 * @brief Decrement X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_dex(cpu_t *cpu, uint16_t in);

/**
 * @brief Decrement Y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_dey(cpu_t *cpu, uint16_t in);

/**
 * @brief Exclusive OR (with accumulator)
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_eor(cpu_t *cpu, uint16_t in);

/**
 * @brief Increment
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_inc(cpu_t *cpu, uint16_t in);

/**
 * @brief Increment X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_inx(cpu_t *cpu, uint16_t in);

/**
 * @brief Increment y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_iny(cpu_t *cpu, uint16_t in);

/**
 * @brief Jump
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_jmp(cpu_t *cpu, uint16_t in);

/**
 * @brief Jump to Subroutine
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_jsr(cpu_t *cpu, uint16_t in);

/**
 * @brief Load Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_lda(cpu_t *cpu, uint16_t in);

/**
 * @brief Load X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_ldx(cpu_t *cpu, uint16_t in);

/**
 * @brief Load Y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_ldy(cpu_t *cpu, uint16_t in);

/**
 * @brief Logical Shift Right
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_lsr(cpu_t *cpu, uint16_t in);

/**
 * @brief No Operation
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_nop(cpu_t *cpu, uint16_t in);

/**
 * @brief Or with Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_ora(cpu_t *cpu, uint16_t in);

/**
 * @brief Push Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_pha(cpu_t *cpu, uint16_t in);

/**
 * @brief Push Processor Status
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_php(cpu_t *cpu, uint16_t in);

/**
 * @brief Pull Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_pla(cpu_t *cpu, uint16_t in);

/**
 * @brief Pull Processor Status
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_plp(cpu_t *cpu, uint16_t in);

/**
 * @brief Rotate Left
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_rol(cpu_t *cpu, uint16_t in);

/**
 * @brief Rotate Right
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_ror(cpu_t *cpu, uint16_t in);

/**
 * @brief Return from Interrupt
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_rti(cpu_t *cpu, uint16_t in);

/**
 * @brief Return from Subroutine
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_rts(cpu_t *cpu, uint16_t in);

/**
 * @brief Subtract with Carry
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sbc(cpu_t *cpu, uint16_t in);

/**
 * @brief Set Carry
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sec(cpu_t *cpu, uint16_t in);

/**
 * @brief Set Decimal
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sed(cpu_t *cpu, uint16_t in);

/**
 * @brief Set Interrupt Disable
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sei(cpu_t *cpu, uint16_t in);

/**
 * @brief Store Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sta(cpu_t *cpu, uint16_t in);

/**
 * @brief Store X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_stx(cpu_t *cpu, uint16_t in);

/**
 * @brief Store Y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_sty(cpu_t *cpu, uint16_t in);

/**
 * @brief Transfer Accumulator to X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_tax(cpu_t *cpu, uint16_t in);

/**
 * @brief Transfer Accumulator to Y
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_tay(cpu_t *cpu, uint16_t in);

/**
 * @brief Transfer Stack Pointer to X
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_tsx(cpu_t *cpu, uint16_t in);

/**
 * @brief Transfer X to Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_txa(cpu_t *cpu, uint16_t in);

/**
 * @brief Transfer X to Stack Pointer
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_txs(cpu_t *cpu, uint16_t in);

/**
 * @brief Transfer Y to Accumulator
 * @param cpu The executing cpu
 * @param opcode The source instruction
 */
void cpu_op_tya(cpu_t *cpu, uint16_t in);

#endif /* __6502_vm_cpu_h__ */