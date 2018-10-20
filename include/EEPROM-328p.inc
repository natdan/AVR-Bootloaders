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
;
; This module provides routines 4 digits 7 seg displays
;
; @author Daniel Sendula
;

.macro      readEEPROM      ; eeprom_address, memory_address, count ; uses ZH:ZL, YH:YL, A
            ldi             ZL, low(@0)
            ldi             ZH, high(@0)
            ldi             YL, low(@1)
            ldi             YH, high(@1)
            ldi             A, @2
            rcall           EEPROM_read_bytes
.endmacro

.macro      writeEEPROM
            ldi             ZL, low(@0)
            ldi             ZH, high(@0)
            ldi             YL, low(@1)
            ldi             YH, high(@1)
            ldi             A, @2
            rcall           EEPROM_write_bytes
.endmacro


;--------------------------------
; Writes byte to EEPROM
; In:
;   A - Byte
;   ZH, ZL - Address in 256

EEPROM_write:
            sbic            EECR, EEPE
            rjmp            EEPROM_write

            out             EEARH, ZH
            out             EEARL, ZL

            out             EEDR, A
            sbi             EECR, EEMPE
            sbi             EECR, EEPE
            ret

EEPROM_read:
            sbic            EECR, EEPE
            rjmp            EEPROM_read

            out             EEARH, ZH
            out             EEARL, ZL
            sbi             EECR, EERE

            in              A, EEDR
            ret


;--------------------------------
; Writes byte to EEPROM
; In:
;   A - Count
;   ZH, ZL - Address in EEPROM
;   YH, YL - Address in Memory

EEPROM_write_bytes:
            push            A
            ld              A, Y+
            rcall           EEPROM_write
            ldi             A, 1
            add             ZL, A
            clr             A
            adc             ZH, A
            pop             A
            dec             A
            brne            EEPROM_write_bytes
            ret

EEPROM_read_bytes:
            push            A
            rcall           EEPROM_read
            st              Y+, A
            ldi             A, 1
            add             ZL, A
            clr             A
            adc             ZH, A
            pop             A
            dec             A
            brne            EEPROM_read_bytes
            ret
