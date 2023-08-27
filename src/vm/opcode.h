#ifndef __6502_vm_opcode_h__

#define __6502_vm_opcode_h__

#include <stdint.h>

#define ADDR_ACC        0x00 /* accumulator */
#define ADDR_IMM        0x01 /* immediate */
#define ADDR_ABS        0x02 /* absolute */
#define ADDR_ABX        0x03 /* absolute, x-indexed */
#define ADDR_ABY        0x04 /* absolute, y-indexed */
#define ADDR_ZP         0x05 /* zero page */
#define ADDR_ZPX        0x06 /* zero page, x-indexed */
#define ADDR_ZPY        0x07 /* zero page, y-indexed */
#define ADDR_IND        0x08 /* indirect */
#define ADDR_INX        0x09 /* indirect, x-indexed */
#define ADDR_INY        0x0A /* indirect, y-indexed */
#define ADDR_REL        0x0B /* relative */
#define ADDR_IMP        0x0C /* implied */

#define OPCODE_ADC      0x00 /* add with carry */
#define OPCODE_AND      0x01 /* and (with accumulator) */
#define OPCODE_ASL      0x02 /* arithmetic shift left */
#define OPCODE_BCC      0x03 /* branch on carry clear */
#define OPCODE_BCS      0x04 /* branch on carry set */
#define OPCODE_BEQ      0x05 /* branch on equal (zero set) */
#define OPCODE_BIT      0x06 /* bit test */
#define OPCODE_BMI      0x07 /* branch on minus (negative set) */
#define OPCODE_BNE      0x08 /* branch on not equal (zero clear) */
#define OPCODE_BPL      0x09 /* branch on plus (negative clear) */
#define OPCODE_BRK      0x0A /* interrupt */
#define OPCODE_BVC      0x0B /* branch on overflow clear */
#define OPCODE_BVS      0x0C /* branch on overflow set */
#define OPCODE_CLC      0x0D /* clear carry */
#define OPCODE_CLD      0x0E /* clear decimal */
#define OPCODE_CLI      0x0F /* clear interrupt disable */
#define OPCODE_CLV      0x10 /* clear overflow */
#define OPCODE_CMP      0x11 /* compare (with accumulator) */
#define OPCODE_CPX      0x12 /* compare with X */
#define OPCODE_CPY      0x13 /* compare with Y */
#define OPCODE_DEC      0x14 /* decrement */
#define OPCODE_DEX      0x15 /* decrement X */
#define OPCODE_DEY      0x16 /* decrement Y */
#define OPCODE_EOR      0x17 /* exclusive or (with accumulator) */
#define OPCODE_INC      0x18 /* increment */
#define OPCODE_INX      0x19 /* increment X */
#define OPCODE_INY      0x1A /* increment Y */
#define OPCODE_JMP      0x1B /* jump */
#define OPCODE_JSR      0x1C /* jump subroutine */
#define OPCODE_LDA      0x1D /* load accumulator */
#define OPCODE_LDX      0x1E /* load X */
#define OPCODE_LDY      0x1F /* load Y */
#define OPCODE_LSR      0x20 /* logical shift right */
#define OPCODE_NOP      0x21 /* no operation */
#define OPCODE_ORA      0x22 /* or with accumulator */
#define OPCODE_PHA      0x23 /* push accumulator */
#define OPCODE_PHP      0x24 /* push processor status (SR) */
#define OPCODE_PLA      0x25 /* pull accumulator */
#define OPCODE_PLP      0x26 /* pull processor status (SR) */
#define OPCODE_ROL      0x27 /* rotate left */
#define OPCODE_ROR      0x28 /* rotate right */
#define OPCODE_RTI      0x29 /* return from interrupt */
#define OPCODE_RTS      0x2A /* return from subroutine */
#define OPCODE_SBC      0x2B /* subtract with carry */
#define OPCODE_SEC      0x2C /* set carry */
#define OPCODE_SED      0x2D /* set decimal */
#define OPCODE_SEI      0x2E /* set interrupt disable */
#define OPCODE_STA      0x2F /* store accumulator */
#define OPCODE_STX      0x30 /* store X */
#define OPCODE_STY      0x31 /* store Y */
#define OPCODE_TAX      0x32 /* transfer accumulator to X */
#define OPCODE_TAY      0x33 /* transfer accumulator to Y */
#define OPCODE_TSX      0x34 /* transfer stack pointer to X */
#define OPCODE_TXA      0x35 /* transfer X to accumulator */
#define OPCODE_TXS      0x36 /* transfer X to stack pointer */
#define OPCODE_TYA      0x37 /* transfer Y to accumulator */

#define OPCODE_ILL      0x38 /* illegal opcode */

typedef struct {
    uint8_t opcode;
    uint8_t address_mode;
    uint8_t cycles;
} opcode_t;


#endif /* __6502_vm_opcode_h__ */