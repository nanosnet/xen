/*
 * xen/arch/arm/arm64/debug-pl011.S
 *
 * PL011 specific debug code
 *
 * Copyright (c) 2013 Citrix Systems.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/asm_defns.h>

/* PL011 UART initialization
 * xb: register which containts the UART base address
 * c: scratch register number */
.macro early_uart_init xb, c
        ldr   x\c, =0x00000000       /* RXE | TXE | UARTEN */
        str   w\c, [\xb, #0x20]      /* -> UARTCR (Control Register) */
        mov   x\c, #((250000000 / (EARLY_PRINTK_BAUD * 8)) - 1)
        strh  w\c, [\xb, #0x28]      /* -> UARTIBRD (Baud divisor integer) */
        mov   x\c, #0x3              /* 8n1 */
        str   w\c, [\xb, #0x0C]      /* -> UARTLCR_H (Line control) */
        ldr   x\c, =0x00000007       /* RXE | TXE | UARTEN */
        str   w\c, [\xb, #0x20]      /* -> UARTCR (Control Register) */
        ldr   x\c, =0x00000003       /* ENABLE */
        str   w\c, [\xb, #0x04]      /* -> IER */
.endm

/* PL011 UART wait UART to be ready to transmit
 * xb: register which contains the UART base address
 * c: scratch register number */
.macro early_uart_ready xb, c
1:
        ldrh  w\c, [\xb, #0x14]      /* <- UARTFR (Flag register) */
        tst   w\c, #0x1              /* Check BUSY bit */
        b.ne  1b                     /* Wait for the UART to be ready */
2:
        ldrh  w\c, [\xb, #0x14]      /* <- UARTFR (Flag register) */
        tst   w\c, #0x20             /* Check BUSY bit */
        b.eq  2b                     /* Wait for the UART to be ready */
.endm

/* PL011 UART transmit character
 * xb: register which contains the UART base address
 * wt: register which contains the character to transmit */
.macro early_uart_transmit xb, wt
        strb  \wt, [\xb]             /* -> UARTDR (Data Register) */
.endm

/*
 * Local variables:
 * mode: ASM
 * indent-tabs-mode: nil
 * End:
 */
