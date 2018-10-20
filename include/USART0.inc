; ===============================================================================
;
; USARTnoInterrpt
;
; Copyright (C) 2009 Creative Sphere Limited
; All rights reserved. This program and the accompanying materials
; are made available under the terms of the Apache License v2.0
; which accompanies this distribution, and is available at
; https://www.apache.org/licenses/LICENSE-2.0
;
;  Contributors:
;    Creative Sphere - initial API and implementation
;
; @author Daniel Sendula
;
;
; This module reads and writes to/from serial interface not utilising interrupts
;


.equ        UART_Baud = (ProcessorSpeed / (UART_Baud_Rate*16))-1


;------------------------------------------------------------------
; Initialises USART hardware
;
; Uses:
;    A
;
USART_Init:
        ldi        A, high(UART_Baud)
        sts        UBRR0H, A
        ldi        A, low(UART_Baud)
        sts        UBRR0L, A

        ldi        A, (1 << RXEN0) | (1 << TXEN0);  | (1 << RXCIE0)
        sts        UCSR0B, A

        ldi        A, (1 << UCSZ01) | (1 << UCSZ00)
        sts        UCSR0C, A

        ret

;------------------------------------------------------------------
; Interrupt service routine that echoes received byte
;

;USART_Echo_int:
;        push        A
;
;        lds            A, UDR0
;USART_Echo_int_wait:
;        sbis        UCSR0A, UDRE0
;        rjmp        USART_Echo_int_wait
;        sts            UDR0, A
;
;        pop        A
;        reti

;------------------------------------------------------------------
; Sends one byte over USART
;
; In:
;    A - byte
;
USART_Send_Byte:
            push            A
_USART_Send_Byte_Wait:
            lds             A, UCSR0A
            sbrs            A, UDRE0
            rjmp            _USART_Send_Byte_Wait
            pop             A
            sts             UDR0, A
            ret

.macro      printA
            call            USART_Send_Byte
.endmacro

;------------------------------------------------------------------
; Sends bytes from program memory over USART
;
; In:
;    A - number of bytes
;    Z - address
; Uses:
;    B

USART_Send_Memory:
            lds             B, UCSR0A
            sbrs            B, UDRE0
            rjmp            USART_Send_Memory

            lpm             B, Z+
            sts             UDR0, B
            dec             A
            brne            USART_Send_Memory
            ret


;------------------------------------------------------------------
; Sends bytes from RAM(data memory) over USART
;
; In:
;    A - number of bytes
;    Z - address
; Uses:
;    B

USART_Send_Data:
            lds             B, UCSR0A
            sbrs            B, UDRE0
            rjmp            USART_Send_Data

            ld              B, Z+
            sts             UDR0, B
            dec             A
            brne            USART_Send_Data
            ret


;------------------------------------------------------------------
; Sends zero terminated string from program memory over USART
;
; In:
;    Z - address
; Uses:
;    A

USART_Send_Memory_String:
            lds             A, UCSR0A
            sbrs            A, UDRE0
            rjmp            USART_Send_Memory_String

            lpm             A, Z+
            tst             A
            breq            _USART_Send_Memory_String_end
            sts             UDR0, A
            rjmp            USART_Send_Memory_String
_USART_Send_Memory_String_end:
            ret

;------------------------------------------------------------------
; Sends zero terminated string from RAM (data memory) over USART
;
; In:
;    Z - address
; Uses:
;    A

USART_Send_Data_String:
            lds             A, UCSR0A
            sbrs            A, UDRE0
            rjmp            USART_Send_Data_String

            lpm             A, Z+
            tst             A
            breq            _USART_Send_Data_String_end
            sts             UDR0, A
            rjmp            USART_Send_Data_String
_USART_Send_Data_String_end:
            ret

.macro      prints
            ldi             ZL, low(@0 << 1)
            ldi             ZH, high(@0 << 1)
            call            USART_Send_Memory_String
.endmacro
