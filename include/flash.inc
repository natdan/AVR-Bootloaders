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

;-the routine writes one page of data from RAM to Flash

; In:
;       Y - the first data location in RAM is pointed to by the Y pointer
;       Z - the first data location in Flash is pointed to by the Z-pointer

; the first data location in RAM is pointed to by the Y pointer
; the first data location in Flash is pointed to by the Z-pointer

;-error handling is not included
;-the routine must be placed inside the Boot space

; (at least the Do_spm sub routine). Only code inside NRWW section can
; be read during Self-Programming (Page Erase and Page Write).
;-registers used: r0, r1, temp1 (r16), temp2 (r17), looplo (r24),
; loophi (r25), spmcrval A (r20)
; storing and restoring of registers is not included in the routine
; register usage can be optimized at the expense of code size
;-It is assumed that either the interrupt table is moved to the Boot
; loader section or that the interrupts are disabled.

.def        looplo = r15

.equ PAGESIZEB = PAGESIZE*2   ;PAGESIZEB is page size in BYTES, not words
; .org SMALLBOOTSTART

Write_page:

;            push            ZL
;            push            ZH

; Page Erase
            ldi             A, (1<<PGERS) | (1<<SELFPRGEN)
            call            Do_spm

; re-enable the RWW section
            ldi             A, (1<<RWWSRE) | (1<<SELFPRGEN)
            call            Do_spm

; transfer data from RAM to Flash page buffer
            ldi             A, PAGESIZE              ;init loop variable
            mov             looplo, A

Wrloop:
            ld              r0, Y+
            ld              r1, Y+
            ldi             A, (1<<SELFPRGEN)
            call            Do_spm

            ldi             A, 2                        ;            adiw            ZH:ZL, 2
            add             ZL, A
            clr             A
            adc             ZH, A

            dec             looplo
            brne            Wrloop

; execute Page Write
            subi            ZL, low(PAGESIZEB)
            sbci            ZH, high(PAGESIZEB)
;            pop             ZH
;            pop             ZL

            ldi             A, (1<<PGWRT) | (1<<SELFPRGEN)
            call Do_spm

; re-enable the RWW section
            ldi             A, (1<<RWWSRE) | (1<<SELFPRGEN)
            call            Do_spm

; read back and check, optional

;            ldi             A, PAGESIZEB                    ;init loop variable
;            mov             looplo, A
;
;            subi            YL, low(PAGESIZEB)
;            sbci            YH, high(PAGESIZEB)
;Rdloop:
;            lpm             r0, Z+
;            ld              r1, Y+
;            cpse            r0, r1
;            jmp             Error
;
;            dec             looplo
;            brne            Rdloop


; return to RWW section
;restore pointer
;use subi for PAGESIZEB<=256
; verify that RWW section is safe to read Return:
            in              B, SPMCSR
            sbrs            B, RWWSB                        ; If RWWSB is set, the RWW section is not ready yet ret
; re-enable the RWW section
            ldi             A, (1<<RWWSRE) | (1<<SELFPRGEN)
            call            Do_spm
            ret             ;            rjmp            Return
Error:
            ret



Do_spm:

Wait_spm:   ; check for previous SPM complete
            in              B, SPMCSR
            sbrc            B, SELFPRGEN
            rjmp            Wait_spm

; input: A determines SPM action
; disable interrupts if enabled, store status in temp2, SREG
; check that no EEPROM write access is present

Wait_ee:
            sbic            EECR, EEPE
            rjmp            Wait_ee

; SPM timed sequence
            out             SPMCSR, A
            spm             ; restore SREG (to enable interrupts if originally enabled) out SREG, temp2
            ret
