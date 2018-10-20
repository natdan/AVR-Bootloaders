; ===============================================================================
;
; Copyright (C) 2017 Creative Sphere Limited
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

.macro      printReg
            prints          S_@0
            ldi             A, @0
            rcall           RF24_Read_Reg
            hexnumber       A, B, C
            mov             A, B
            printA
            mov             A, C
            printA
            prints          S_CRLF
.endmacro

.macro      printMemoryHex
            push            ZH
            push            ZL
            ldi             ZH, high(@1)
            ldi             ZL, low(@1)
            ldi             D, @0
_print_pipe_addr_loop:
            ld              A, Z+
            hexnumber       A, B, C
            mov             A, B
            printA
            mov             A, C
            printA
            dec             D
            brne            _print_pipe_addr_loop

            prints          S_CRLF
            pop             ZL
            pop             ZH
.endmacro

.macro      printPipeAddr
            prints          S_@0
            ldi             ZH, high(@1)
            ldi             ZL, low(@1)
            ldi             A, @0
            rcall           RF24_Read_Pipe_Addr
            printMemoryHex  5, @1
.endmacro

;.macro      delay_10_us
;            push            A               ; 1 byte, 2 cycles
;            ldi             A, 52           ; 1 byte, 1 cycle
;_loop:      dec             A               ; 1 byte, 1 cycle
;            brne            _loop           ; 1 byte, 2/1 cycle(s)
;            pop             A               ; 1 byte, 2 cycles
;.endmacro                                   ; 5 bytes, 162 cycles
;
;.macro      delay_100_ms
;            push            A
;            ldi             A, 100
;_loop_100ms:
;            push            A
;            ldi             A, 100
;_loop_ms:
;            push            A               ; 1 byte, 2 cycles
;            ldi             A, 52           ; 1 byte, 1 cycle
;_loop_us:   dec             A               ; 1 byte, 1 cycle
;            brne            _loop_us        ; 1 byte, 2/1 cycle(s)
;            pop             A               ; 1 byte, 2 cycles
;                                            ; 5 bytes, 162 cycles = 10us
;            dec             A
;            brne            _loop_ms
;            pop             A
;
;            dec             A
;            brne            _loop_100ms
;            pop             A
;
;.endmacro

.macro      delay10us
            call            Delay_10us
.endmacro                                   ; 5 bytes, 162 cycles

.macro      delay100ms
            call            Delay_100ms
.endmacro                                   ; 5 bytes, 162 cycles

.macro      delay20ms
            call            Delay_20ms
.endmacro                                   ; 5 bytes, 162 cycles

.macro      delay2ms
            call            Delay_2ms
.endmacro                                   ; 5 bytes, 162 cycles

Delay_10us:                                 ; Call  4 bytes 4 cycles
            push            A               ;       1 byte, 2 cycles
            ldi             A, 22           ;       1 byte, 1 cycle
_loop:      dec             A               ;       1 byte, 1 cycle        3 cycles * 22 = 66
            brne            _loop           ;       1 byte, 2/1 cycle(s)
            pop             A               ;       1 byte, 2 cycles
            ret                             ;       1 byte, 4 cycles
                                            ;       total - 3*25 + 4 + 2 + 1 + 1 + 2 + 4 = 80
                                            ;       80/ 8Mhz ~ 10us


Delay_2ms:
            push            A
            ldi             A, 2
            rjmp            _loop_1ms
Delay_20ms:
            push            A
            ldi             A, 20
            rjmp            _loop_1ms
Delay_100ms:
            push            A
            ldi             A, 100
_loop_1ms:
            push            A
            ldi             A, 100
_loop_ms:
            push            A               ; 1 byte, 2 cycles
            ldi             A, 22           ; 1 byte, 1 cycle
_loop_us:   dec             A               ; 1 byte, 1 cycle
            brne            _loop_us        ; 1 byte, 2/1 cycle(s)
            pop             A               ; 1 byte, 2 cycles
                                            ; 5 bytes, 162 cycles = 10.125us ~ 10us
            dec             A
            brne            _loop_ms
            pop             A

            dec             A
            brne            _loop_1ms
            pop             A
            ret                             ; total 10.125 x 100 x100 = 101.25ms
