; ===============================================================================
;
; Bootloader for nRF24L01
;
; Copyright (C) 2017 Creative Sphere Limited
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
; Bootloader implemeting subset of stk500 protocol
; over nRF24L01
;
; Hardware:
;    ATmeta328p (or compatibile)
;       8bit timer for regular interrupts
;
;    Pins: (logical)
;       PB5 CLK
;       PB4 MISO (DI)
;       PB3 MOSI (DO)
;       PB2 CS (SS)
;       PB1 CE
;
; Fuses:
;       BOOTRST = 0 (programmed)
;            BootLoader Flash Section 0x800:
;       BOOTSZ1 = 0 (programmed)
;       BOOTSZ0 = 0 (programmed)
;
;       BLB02 = 1 unprogrammed
;       BLB01 = 1 unprogrammed
;       BLB12 = 1 unprogrammed
;       BLB11 = 1 unprogrammed
;       SELFPRGEN = 1 unprogrammed
;
;
; Algorithm:
;

.include "m328pdef.inc"

.equ        EEPROM_PAGE_SIZE = 4
.equ        FLASH_PAGE_SIZE = 128
.equ        ProcessorSpeed = 8000000
.equ        UART_Baud_Rate = 19200 ; 57600


;.def        DEBUG 1


.def        A = r16
.def        B = r17
.def        C = r18
.def        D = r19

.def        recLen = r20
.def        sendLen = r21
.def        destination = r22

.def        lenH = r25
.def        lenL = r24

.def        addrL = r4
.def        addrH = r5
.def        tempL = r6
.def        tempH = r7


; .def        recBufPtr = X
; .def        sendBufPtr = Y

.equ        SPI_DI = PB4    // Port B bit 6 (pin7): data in (data from MMC)
.equ        SPI_DO = PB3    // Port B bit 5 (pin6): data out (data to MMC)
.equ        SPI_CLK = PB5    // Port B bit 7 (pin8): clock
.equ        SPI_CS = PB2    // Port B bit 4 (pin5: chip select for MMC
.equ        SPI_CE = PB1    // Port B bit 4 (pin5: chip select for MMC

.equ        INPUT_PIN = PB0

.equ        BOOT_LOADER_ADDRESS = 0x3800

.equ        DATA_SIZE = 16
.equ        PACKET_SIZE = DATA_SIZE + 1
.equ        PAGE_SIZE = 128
.equ        RETRY_COUNT = 100

.equ        CHANNEL = 1

.equ        HW_VER = 0x0f
.equ        SW_MAJOR = 0x04
.equ        SW_MINOR = 0x0c

.dseg

SendBuffer:     .byte   PACKET_SIZE
ReceiveBuffer:  .byte   PACKET_SIZE
MemoryBuffer:   .byte   PAGE_SIZE


Buffer:          .byte    20








.cseg
.org        BOOT_LOADER_ADDRESS
            jmp             RESET
            jmp             RESET_NO_SKIP

;.org        BOOT_LOADER_ADDRESS + INT_VECTORS_SIZE

.include    "macros.inc"
.include    "ExtensionMacros.inc"
.include    "nRF24L01.inc"
.include    "USART0.inc"

.macro      resetSendBuf
            ldiw            Y, SendBuffer + 1
            ldi             sendLen, 0
.endmacro

.macro      readByteA
            ld              A, X+
            dec             recLen
.endmacro

.macro      readByteR
            ld              @0, X+
            dec             recLen
.endmacro

.macro      writeByteR
            st              Y+, @0
            inc             sendLen
.endmacro

.macro      writeByteI
            ldi             A, @0
            st              Y+, A
            inc             sendLen
.endmacro

.macro      moreBytes
            tst             recLen
.endmacro

.macro      moreSendSpace
            cpi            sendLen, 16
.endmacro


RESET_NO_SKIP:
            cli
            ldi             A, high(RAMEND)    ; Main program start
            out             SPH, A             ; Set Stack Pointer to top of RAM
            ldi             A, low(RAMEND)
            out             SPL, A
            rjmp            Continue_with_bootloader
RESET:
            cli
            ldi             A, high(RAMEND)    ; Main program start
            out             SPH, A             ; Set Stack Pointer to top of RAM
            ldi             A, low(RAMEND)
            out             SPL, A

            cbi             DDRB, INPUT_PIN
            sbi             PORTB, INPUT_PIN
            nop
            nop

            sbic            PINB, INPUT_PIN
            rjmp            DoReset

Continue_with_bootloader:
            nop
            call            USART_Init
            call            SPI_Init
            call            RF24_Init

            delay10us

            resetSendBuf
Loop:
            switchToRX
            startListening
            poolData        ; TODO add timeout!!!

            rcall            ReceiveData
            stopListening

            readByteA

            cpi             A, 0x75
            brne            _not_signature
            rjmp            Signature

_not_signature:
            cpi             A, 0x31
            brne            _not_read_id
            rjmp            ReadId

 _not_read_id:
            cpi             A, 0x41
            brne            _not_read_version
            rjmp            Version

 _not_read_version:
            cpi             A, 0x51
            brne            _not_reset
            rjmp            DoResetCommand             ; not coming back!

 _not_reset:
            cpi             A, 0x55
            brne            _not_address
            rjmp            ReceiveAddress

 _not_address:
            cpi             A, 0x64
            brne            _not_write
            rjmp            WriteMemory

 _not_write:
            cpi             A, 0x74
            brne            _not_read
            rjmp            ReadMemory

 _not_read:
            rcall           SendACK
            rjmp            Loop

ReadId:
            switchToTX

            ldi             D, RETRY_COUNT
_ReadId_Loop:
            writeFlushTX
            clearInterrupts

            ldi             A, PACKET_SIZE
            ldipw           Z, ID_PACKET
            call            RF24_Write_Payload_P

            andi            A, (1<<TX_DS)
            brne            _ReadId_OK

#ifdef DEBUG
            prints          Err
#endif
            mov             A, D

#ifdef DEBUG
            printA
            prints          CRLF
#endif
            dec             D
            brne            _ReadId_Loop

#ifdef DEBUG
            prints          Failed
#endif
            rjmp            Loop

_ReadId_OK:
#ifdef DEBUG
            prints          SentReadId
#endif
            rjmp            Loop

Signature:
            writeByteI      0x14
            clr             ZH
            clr             ZL

            ldi             A, 0x21; (1<<SIGRD) | (1<<SPMEM)
            out             SPMCSR, A
            lpm             A, Z
            writeByteR      A

            subi            ZL, -2
            ldi             A, 0x21; (1<<SIGRD) | (1<<SPMEM)
            out             SPMCSR, A
            lpm             A, Z
            writeByteR      A

            subi            ZL, -2
            ldi             A, 0x21; (1<<SIGRD) | (1<<SPMEM)
            out             SPMCSR, A
            lpm             A, Z
            writeByteR      A
            writeByteI      0x10
            rcall           SendData
            rjmp            Loop

Version:
            writeByteI      0x14
            readByteA
            cpi             A, 0x80
            brne            _not_hw_ver
            writeByteI      HW_VER
            rjmp            _version_end
_not_hw_ver:
            cpi             A, 0x81
            brne            _not_sw_maj
            writeByteI      SW_MAJOR
            rjmp            _version_end
_not_sw_maj:
            cpi             A, 0x82
            brne            _not_sw_min
            writeByteI      SW_MINOR
            rjmp            _version_end
_not_sw_min:
            cpi             A, 0xC7
            brne            _not_flash_page_size
            writeByteI      low(FLASH_PAGE_SIZE)
            writeByteI      high(FLASH_PAGE_SIZE)
            rjmp            _version_end
_not_flash_page_size:
            cpi             A, 0x52
            brne            _not_eeprom_page_size
            writeByteI      EEPROM_PAGE_SIZE
            rjmp            _version_end
_not_eeprom_page_size:
            cpi             A, 0x98
            brne            _not_something_else
            writeByteI      0x03
            rjmp            _version_end
_not_something_else:
            writeByteI      0x0
_version_end:
            writeByteI      0x10
            rcall           SendData
            rjmp            Loop

DoResetCommand:

            rcall           SendACK

;            ldi             A, (1<<WDE)
;            sts             WDTCSR, A
DoReset:
            clr             A
            out             DDRB, A
            jmp             0

ReceiveAddress:
            readByteR       addrL
            readByteR       addrH

            rcall           SendACK
            rjmp            Loop

ReadMemory:
            readByteR       lenH
            readByteR       lenL

            readByteR       destination   ; Destination 'E' or 'F'

            resetSendBuf
            writeByteI      0x14

            mov             ZL, addrL
            mov             ZH, addrH

_ReadMemory_loop1:
            tst             lenH
            brne            _ReadMemory_loop1_ok
            tst             lenL
            breq            _ReadMemory_loop1_end

_ReadMemory_loop1_ok:

_ReadMemory_loop2:
            moreSendSpace
            breq            _ReadMemory_loop2_end

            tst             lenH
            brne            _ReadMemory_loop2_ok
            tst             lenL
            breq            _ReadMemory_loop2_end

_ReadMemory_loop2_ok:

            cpi             destination, 'E'
            breq            _ReadEEPROM


; TODO - read byte first!
            lpm             A, Z+
_ReadMemory_cont:
            writeByteR      A

            ldi             A, 1
            sub             lenL, A
            clr             A
            sbc             lenH, A
            rjmp            _ReadMemory_loop2
_ReadEEPROM:
            sbic            EECR, EEPE
            rjmp            _ReadEEPROM
            out             EEARH, ZH
            out             EEARL, ZL
            sbi             EECR, EERE
            in              A, EEDR

            push            A
            ldi             A, 1
            add             ZL, A
            clr             A
            adc             ZH, A
            pop             A

            rjmp            _ReadMemory_cont


_ReadMemory_loop2_end:
            mov             addrL, Zl
            mov             addrH, ZH

            rcall           SendData

            switchToRX
            startListening
            poolData

            rcall       ReceiveData
            stopListening

            resetSendBuf

            mov             ZL, addrL
            mov             ZH, addrH

            rjmp            _ReadMemory_loop1



_ReadMemory_loop1_end:
            writeByteI      0x10
            rcall           SendData
            rjmp            Loop

WriteMemory:
            readByteR       lenH
            readByteR       lenL

            readByteR       destination     ; Destination 'E' or 'F'

            ldiw            Y, MemoryBuffer

_WriteMemory_loop1:
            tst             lenH
            brne            _WriteMemory_loop1_ok
            tst             lenL
            breq            _WriteMemory_loop1_end

_WriteMemory_loop1_ok:

_WriteMemory_loop2:
            moreBytes
            breq            _WriteMemory_loop2_end

            tst             lenH
            brne            _WriteMemory_loop2_ok
            tst             lenL
            breq            _WriteMemory_loop2_end

_WriteMemory_loop2_ok:
            readByteA

            cpi             destination, 'E'
            breq            _WriteEEPROM

            st              Y+, A
            dec             lenL         ; memory batch will always be less than 256

            rjmp            _WriteMemory_loop2

_WriteEEPROM:
            sbic            EECR, EEPE
            rjmp            _WriteEEPROM

            out             EEARH, addrH
            out             EEARL, addrL

            out             EEDR, A
            sbi             EECR, EEMPE
            sbi             EECR, EEPE

            ldi             A, 1
            add             addrL, A
            clr             A
            adc             addrH, A

;            dec             lenL
;            sbc             lenH, A

            ldi             A, 1
            sub             lenL, A
            clr             A
            sbc             lenH, A

            rjmp            _WriteMemory_loop2

_WriteMemory_loop2_end:

            tst             lenH
            brne            _WriteMemory_send_continuation
            tst             lenL
            breq            _WriteMemory_loop1
_WriteMemory_send_continuation:
            mov             tempL, YL
            mov             tempH, YH

            resetSendBuf
            rcall           SendData

            switchToRX
            startListening
            poolData

            rcall            ReceiveData
            stopListening

            mov             YL, tempL
            mov             YH, tempH
            rjmp            _WriteMemory_loop1

_WriteMemory_loop1_end:
            ; TODO add page size to addrL:H

            cpi             destination, 'E'
            breq            _WriteMemory_loop1_end_eeprom

            ldi             A, 128
            ldiw            Y, MemoryBuffer
            mov             ZL, addrL
            mov             ZH, addrH

            call            Write_page

            ldi             A, low(PAGE_SIZE)
            add             addrL, A

            ldi             A, high(PAGE_SIZE)
            adc             addrH, A

_WriteMemory_loop1_end_eeprom:
            rcall           SendACK
            rjmp            Loop


ReceiveData:
            ldi             A, PACKET_SIZE
            ldiw            Z, ReceiveBuffer
            rcall           RF24_Read_Payload
            ldiw            X, ReceiveBuffer
            ld              recLen, X+
            ret

SendData:
#ifdef DEBUG
            prints          SendingData
#endif
            ldi             D, RETRY_COUNT
_SendData_Loop:
            ldiw            Z, SendBuffer
            st              Z+, sendLen
            ldiw            Z, SendBuffer

            switchToTX
            writeFlushTX
            clearInterrupts

            ldi             A, PACKET_SIZE
            call            RF24_Write_Payload_M

            andi            A, (1<<TX_DS)
            brne            _SendData_OK

#ifdef DEBUG
            prints          Err
#endif
            mov             A, D
#ifdef DEBUG
            printA
            prints          CRLF
#endif
            dec             D
            brne            _SendData_Loop

#ifdef DEBUG
            prints          Failed
#endif
            writeRegI       STATUS, (1<<TX_DS) | (1<<MAX_RT)
            resetSendBuf
            ret

_SendData_OK:
#ifdef DEBUG
            prints          SentData
#endif
            writeRegI       STATUS, (1<<TX_DS) | (1<<MAX_RT)
            resetSendBuf

            ret

SendACK:
#ifdef DEBUG
            prints          SendingACK
#endif
            ldi             D, RETRY_COUNT
_SendACK_Loop:
            switchToTX
            writeFlushTX
            clearInterrupts

            ldi             A, PACKET_SIZE
            ldipw           Z, ACK_PACKET
            call            RF24_Write_Payload_P

            andi            A, (1<<TX_DS)
            brne            _SendACK_OK

#ifdef DEBUG
            prints          Err
#endif
            mov             A, D
#ifdef DEBUG
            printA
            prints          CRLF
#endif
            dec             D
            brne            _SendACK_Loop
#ifdef DEBUG
            prints          Failed
#endif
            ret

_SendACK_OK:
#ifdef DEBUG
            prints          SentAck
#endif
            ret

.include        "flash.inc"

ACK_PACKET:     .db         2, 0x14, 0x10
ID_PACKET:      .db         9, 0x14, "nRF ISP", 0x10

Pipe_Address:   .db         "BOOTL"

SendingACK:     .db         "SendingACK", 13, 10, 0
SentACK:        .db         "SentACK", 13, 10, 0
SendingData:    .db         "SendingData", 13, 10, 0
SentData:       .db         "SentData", 13, 10, 0
SentReadId:     .db         "SentReadId", 13, 10, 0
Err:            .db         "Err:", 0
Failed:         .db         "Failed", 13, 10, 0
CRLF:           .db         13, 10, 0