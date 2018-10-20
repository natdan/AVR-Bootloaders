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


; ==========================================================================
; SPINoInterrupt
;
; Copyright (C) 2009 Creative Sphere Limited
;
; @author Daniel Sendula
;
;
; This module reads and writes to memory card (SPI interface MMC/SD cards)
; using SPI interface of AVR not utilising interrupts
;
;

.equ        CONFIG = 0x00
.equ        EN_AA = 0x01
.equ        EN_RXADDR = 0x02
.equ        SETUP_AW = 0x03
.equ        SETUP_RETR = 0x04
.equ        RF_CH = 0x05
.equ        RF_SETUP = 0x06
.equ        STATUS = 0x07
.equ        OBSERVE_TX = 0x08
.equ        CD = 0x09
.equ        RX_ADDR_P0 = 0x0A
.equ        RX_ADDR_P1 = 0x0B
.equ        RX_ADDR_P2 = 0x0C
.equ        RX_ADDR_P3 = 0x0D
.equ        RX_ADDR_P4 = 0x0E
.equ        RX_ADDR_P5 = 0x0F
.equ        TX_ADDR = 0x10
.equ        RX_PW_P0 = 0x11
.equ        RX_PW_P1 = 0x12
.equ        RX_PW_P2 = 0x13
.equ        RX_PW_P3 = 0x14
.equ        RX_PW_P4 = 0x15
.equ        RX_PW_P5 = 0x16
.equ        FIFO_STATUS = 0x17
.equ        DYNPD	 = 0x1C
.equ        FEATURE	 = 0x1D

; Bit Mnemonics
.equ        MASK_RX_DR = 6
.equ        MASK_TX_DS = 5
.equ        MASK_MAX_RT = 4
.equ        EN_CRC = 3
.equ        CRCO = 2
.equ        PWR_UP = 1
.equ        PRIM_RX = 0
.equ        ENAA_P5 = 5
.equ        ENAA_P4 = 4
.equ        ENAA_P3 = 3
.equ        ENAA_P2 = 2
.equ        ENAA_P1 = 1
.equ        ENAA_P0 = 0
.equ        ERX_P5 = 5
.equ        ERX_P4 = 4
.equ        ERX_P3 = 3
.equ        ERX_P2 = 2
.equ        ERX_P1 = 1
.equ        ERX_P0 = 0
.equ        AW = 0
.equ        ARD = 4
.equ        ARC = 0
.equ        PLL_LOCK = 4
.equ        RF_DR = 3
.equ        RF_PWR = 6
.equ        RX_DR = 6
.equ        TX_DS = 5
.equ        MAX_RT = 4
.equ        RX_P_NO = 1
.equ        TX_FULL = 0
.equ        PLOS_CNT = 4
.equ        ARC_CNT = 0
.equ        TX_REUSE = 6
.equ        FIFO_FULL = 5
.equ        TX_EMPTY = 4
.equ        RX_FULL = 1
.equ        RX_EMPTY = 0
.equ        DPL_P5	 = 5
.equ        DPL_P4	 = 4
.equ        DPL_P3	 = 3
.equ        DPL_P2	 = 2
.equ        DPL_P1	 = 1
.equ        DPL_P0	 = 0
.equ        EN_DPL	 = 2
.equ        EN_ACK_PAY = 1
.equ        EN_DYN_ACK = 0

; Instruction Mnemonics
.equ        R_REGISTER = 0x00
.equ        W_REGISTER = 0x20
.equ        REGISTER_MASK = 0x1F
.equ        ACTIVATE = 0x50
.equ        R_RX_PL_WID = 0x60
.equ        R_RX_PAYLOAD = 0x61
.equ        W_TX_PAYLOAD = 0xA0
.equ        W_ACK_PAYLOAD = 0xA8
.equ        FLUSH_TX = 0xE1
.equ        FLUSH_RX = 0xE2
.equ        REUSE_TX_PL = 0xE3
.equ        RF24_NOP = 0xFF

; Non-P omissions
.equ        LNA_HCURR = 0

; P model memory Map
.equ        RPD = 0x09
.equ        W_TX_PAYLOAD_NO_ACK = 0xB0

; P model bit Mnemonics
.equ        RF_DR_LOW = 5
.equ        RF_DR_HIGH = 3
.equ        RF_PWR_LOW = 1
.equ        RF_PWR_HIGH = 2



.dseg

Pipe_Address_Mem:	.byte	5

.cseg



.macro      writeRegI
            ldi             B, @1
            ldi             A, @0
            rcall           RF24_Write_Reg
.endmacro

.macro      writeRegR
            mov             B, @1
            ldi             A, @0
            rcall           RF24_Write_Reg
.endmacro

.macro      readReg
            ldi             A, @0
            rcall           RF24_Read_Reg
.endmacro

.macro      writePipeAddr
            ldi             A, @0
            ldi             ZH, high(@1 << 1)
            ldi             ZL, low(@1 << 1)
            rcall           RF24_Write_Data
.endmacro

.macro      setCE
            sbi             PORTB, SPI_CE
.endmacro

.macro      clearCE
            cbi             PORTB, SPI_CE
.endmacro

.macro      writeFlushTX
            ldi             A, FLUSH_TX
            rcall            RF24_Write_Byte
.endmacro

.macro      writeFlushRX
            ldi             A, FLUSH_RX
            rcall           RF24_Write_Byte
.endmacro

.macro      switchToRX
            clearCE
            writeRegI       CONFIG, (1<<PWR_UP) | (1 <<CRCO) | (1<<EN_CRC) | (1<<MASK_MAX_RT) | (1<<PRIM_RX)    ; PWR_UP | CRCO | EN_CRC | MASK_MAX_RT | PRIM_RX
.endmacro

.macro      switchToTX
            clearCE
            writeRegI       CONFIG, (1<<PWR_UP) | (1 <<CRCO) | (1<<EN_CRC) | (1<<MASK_MAX_RT)                   ; PWR_UP | CRCO | EN_CRC | MASK_MAX_RT | PRIM_TX
            delay20ms
.endmacro

.macro      clearInterrupts
            writeRegI       STATUS, 0x71
.endmacro

.macro      startListening
            rcall           RF24_StartListening
.endmacro

.macro      stopListening
            rcall           RF24_StopListening
.endmacro

.macro      poolData
            rcall           RF24_PoolData
.endmacro

.macro      powerDown
            clearCE
            delay20ms
            readReg         CONFIG
            andi            A, 0xfd
            writeRegR       CONFIG, A
            delay20ms
.endmacro

.macro      powerUp
            readReg         CONFIG
            ori             A, 2
            writeRegR       CONFIG, A
            delay20ms
.endmacro


;------------------------------------------------------------------
; Initialises SPI hardware
;
; Uses:
;    A
;
SPI_Init:
            in              A, DDRB
            andi            A, ~(1 <<SPI_DI)
            ori             A, (1 << SPI_CLK) | (1 << SPI_DO) | (1 << SPI_CS) | (1 << SPI_CE)
            out             DDRB, A

            ; f(osc) / 128 (SPIX2 = 0), (SPR1 & SPR0 = 1)
            ldi             A, (1 << SPE) | (1 << MSTR) ; | (1 << SPR1) | (1 << SPR0)
            out             SPCR, A

            sbi             PORTB, SPI_CS
            cbi             PORTB, SPI_CE
            ret

RF24_Init:
            clearCE
            powerDown
            writeRegI       STATUS, (1<<RX_DR)
            writeRegI       EN_AA, 0x3f                     ; EN_AA_P0
            writeRegI       EN_RXADDR, 0x01                 ; ERX_P0
            writeRegI       SETUP_AW, 0x03                  ; 5 bytes
            writeRegI       RF_CH, CHANNEL                  ; channel 1 (MiLight 9, 40, 71 ?)
            writeRegI       RF_SETUP, 0x01                  ; LNA_Gain | 0 dBm | 1 Mbps
            writePipeAddr   RX_ADDR_P0, Pipe_Address
            writePipeAddr   TX_ADDR, Pipe_Address
            writeRegI       RX_PW_P0, PACKET_SIZE           ;
            writeRegI       SETUP_RETR, 0x2f                ;    # 15 retransmits | Wait 750 + 86us
            writeRegI       STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT)
            clearInterrupts
            delay20ms
            writeFlushRX
            writeFlushTX
            delay20ms
            switchToRX                                      ;         writeRegI        CONFIG, 0x1f        # PWR_UP | CRC0 | EN_CRC | MASK_MAX_RT | PTX

            delay100ms
            ret


RF24_StartListening:
            delay10us
            readReg         CONFIG
            ori             A, (1<<PWR_UP) | (1<<PRIM_RX)
            writeRegR       CONFIG, A
            writeRegI       STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT)
            writeFlushRX
            writeFlushTX
            setCE
            delay20ms
            ret


RF24_StopListening:
            clearCE
            ret


RF24_PoolData:
            readReg         STATUS
            andi            A, (1<<RX_DR)
            breq            RF24_PoolData
            writeRegI       STATUS, (1<<RX_DR)
            ret
;            ldi             A, FIFO_STATUS
;            rcall           RF24_Read_Reg
;            andi            A, 1
;            brne            Loop

;------------------------------------------------------------------
; Sends one byte over SPI and receives one byte back
;
; In:
;    A - byte
; Out:
;    A - byte
;
SPI_Send_Byte:
            out             SPDR, A

_SPI_Send_Byte:
            in              A, SPSR
            sbrs            A, SPIF
            rjmp            _SPI_Send_Byte
            in              A, SPDR
            ret

;------------------------------------------------------------------
; Reads one register over SPI
;
; In:
;    A - register address
; Out:
;    A - register value
;
RF24_Read_Reg:
            delay10us
            cbi             PORTB, SPI_CS
            delay10us
            rcall           SPI_Send_Byte
            delay10us
            rcall           SPI_Send_Byte
            delay10us
            sbi             PORTB, SPI_CS
            ret

;------------------------------------------------------------------
; Reads pipe address register over SPI
;
; In:
;    A - pipe register address
;    Z - address
; Out:
;    A - register value
;
RF24_Read_Pipe_Addr:
            delay10us
            cbi             PORTB, SPI_CS
            delay10us
            rcall           SPI_Send_Byte
            delay10us
            rcall           SPI_Send_Byte
            st              Z+, A
            rcall           SPI_Send_Byte
            st              Z+, A
            rcall           SPI_Send_Byte
            st              Z+, A
            rcall           SPI_Send_Byte
            st              Z+, A
            rcall           SPI_Send_Byte
            st              Z+, A
            delay10us
            sbi             PORTB, SPI_CS
            ret


;------------------------------------------------------------------
; Read payload to memory register over SPI
;
; In:
;    A - amount of bytes
;    Z - address
; Out:
;    A - register value
;
RF24_Read_Payload:
            delay10us
            cbi             PORTB, SPI_CS
            delay10us
            push            A
            ldi             A, R_RX_PAYLOAD
            rcall           SPI_Send_Byte
            delay10us
            pop             A
_RF24_Read_Loop:
            push            A
            delay10us
            rcall           SPI_Send_Byte
            st              Z+, A
            pop             A
            dec             A
            brne            _RF24_Read_Loop

            delay10us
            sbi             PORTB, SPI_CS
            ret

;------------------------------------------------------------------
; Writes one byte over SPI
;
; In:
;    A - write value
; Out:
;    A - read value
;
RF24_Write_Byte:
            delay10us
            cbi             PORTB, SPI_CS
            delay10us
            rcall           SPI_Send_Byte
            delay10us
            sbi             PORTB, SPI_CS
            ret

;------------------------------------------------------------------
; Writes one register over SPI
;
; In:
;    A - register address
;    B - register value
; Out:
;    A - register value
;
RF24_Write_Reg:
            delay10us
            cbi             PORTB, SPI_CS
            delay10us
            ori             A, 0x20
            rcall           SPI_Send_Byte
            delay10us
            mov             A, B
            rcall           SPI_Send_Byte
            delay10us
            sbi             PORTB, SPI_CS
            ret

;------------------------------------------------------------------
; Write data to one register over SPI
;
; In:
;    A - register address
;    Z - address ('0' terminated)
; Out:
;    A - register value
;
RF24_Write_Data:
            delay10us
            cbi             PORTB, SPI_CS
            delay10us
            ori             A, 0x20
            rcall           SPI_Send_Byte
            delay10us

_RF24_Write_Data_loop:
            lpm             A, Z+
            tst             A
            breq            _RF24_Write_Data_End
            rcall           SPI_Send_Byte
            rjmp            _RF24_Write_Data_loop

_RF24_Write_Data_end:
            delay10us
            sbi             PORTB, SPI_CS
            ret

;------------------------------------------------------------------
; Read payload to memory register over SPI
;
; In:
;    A - amount of bytes
;    Z - address
; Out:
;    A - register value
;
RF24_Write_Payload_M:
            delay10us
            cbi             PORTB, SPI_CS
            delay10us
            push            A
            ldi             A, W_TX_PAYLOAD
            rcall           SPI_Send_Byte
            delay10us
            pop             A

_RF24_Write_Payload_M_Loop:
            push            A
            delay10us
            ld              A, Z+
            rcall           SPI_Send_Byte
            pop             A
            dec             A
            brne            _RF24_Write_Payload_M_Loop

_RF24_Finish_Writing:

            delay10us
            sbi             PORTB, SPI_CS

            delay10us
            setCE
            delay10us
            clearCE

_RF24_Wait_Finished_Writing:
            readReg         STATUS
            andi            A, (1<<TX_DS) | (1<<MAX_RT)
            breq            _RF24_Wait_Finished_Writing

            ret

;------------------------------------------------------------------
; Read payload to memory register over SPI
;
; In:
;    A - amount of bytes
;    Z - address
; Out:
;    A - register value
;
RF24_Write_Payload_P:
            delay10us
            cbi             PORTB, SPI_CS
            delay10us
            push            A
            ldi             A, W_TX_PAYLOAD
            rcall           SPI_Send_Byte
            delay10us
            pop             A
_RF24_Read_Payload_P_Loop:
            push            A
            delay10us
            lpm             A, Z+
            rcall           SPI_Send_Byte
            pop             A
            dec             A
            brne            _RF24_Read_Payload_P_Loop
            rjmp            _RF24_Finish_Writing
