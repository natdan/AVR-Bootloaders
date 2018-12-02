; =======================================
; Bootloader for ATmega328p over i2c
;
; Copyright (C) 2018 Creative Sphere Limited
;
; @author Daniel Sendula
;
;
; Bootloader implemeting subset of stk500 protocol
; over i2c
;
; Hardware:
;    ATmeta328p (or compatibile)
;       8bit timer for regular interrupts
;
;    Pins: (logical)
;
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

.include    "m328pdef.inc"

.equ        INPUT_PIN = PB4

.equ        BOOT_LOADER_ADDRESS = 0x3800
;.equ        BOOT_LOADER_ADDRESS = 0

.equ        I2C_ADDRESS = 0x04

.equ        EEPROM_PAGE_SIZE = 4
.equ        FLASH_PAGE_SIZE = PAGESIZE * 2

.equ        MEMORY_BUFFER_SIZE = 32
.equ        PAGE_BUFFER_SIZE = FLASH_PAGE_SIZE

.equ        HW_VER = 0x0f

.equ        SW_MAJOR = 0x01
.equ        SW_MINOR = 0x01

.equ        MULTIPLICATION_FACTOR = 10

.def        R_LEN = r16
.def        R_NEXT_L = r17
.def        R_NEXT_H = r18
.def        R_DESTINATION = r21
.def        R_COMMAND = r22
.def        A = r23
.def        B = r24

.def        R_ACK = r2
.def        R_NACK = r3
.def        R_STOP = r4
.def        R_VERSION_TEMP = r5
.def        R_STORED_STATUS_REG = r6
.def        R_STORED_STATUS_REG2 = r7
.def        R_ADDR_L = r8
.def        R_ADDR_H = r9
.def        R_LEN_L = r10
.def        R_LEN_H = r11
.def        R_TEMP_L = r12
.def        R_TEMP_H = r13
.def        R_Loop = r14
.def        R_TEMP = r15

.equ        NONE_COMMAND = 0
.equ        READ_MEMORY_COMMAND = 1
.equ        READ_MEMORY_CONT_COMMAND = 2
.equ        WRITE_MEMORY_COMMAND = 3
.equ        RESET_COMMAND = 4
.equ        OK_COMMAND = 5
.equ        READID_COMMAND = 6
.equ        VERSION_COMMAND = 7
.equ        SIGNATURE_COMMAND = 8

.macro      ldiw
            ldi        @0L, low(@1)
            ldi        @0H, high(@1)
.endmacro

.macro      ldipw
            ldi        @0L, low(@1 << 1)
            ldi        @0H, high(@1 << 1)
.endmacro

.macro      ack
            sts             TWCR, R_ACK
.endmacro

.macro      ret_ack
            sts             TWCR, R_ACK
            out			    SREG, R_STORED_STATUS_REG
            reti
.endmacro

.macro      nack
            sts             TWCR, R_NACK
.endmacro

.macro      stop
            sts             TWCR, R_STOP
.endmacro

.macro      next_byte       ; address
            ldi		        R_NEXT_L, low(@0)
            ldi	        	R_NEXT_H, high(@0)
.endmacro

.macro      set_read
            ldiw            Y, MemoryBuffer
            st              Y, R_LEN
            ldiw            X, MemoryBuffer
.endmacro

.macro      prepare_read
            ldiw            X, MemoryBuffer
            ldiw            Y, MemoryBuffer
            clr             R_LEN
            st              Y+, R_LEN
.endmacro


.macro      writeByteR
            st              Y+, @0
            inc             R_LEN
.endmacro

.macro      writeByteI
            ldi             B, @0
            st              Y+, B
            inc             R_LEN
.endmacro


.equ        PAGE_SIZE = 128

.dseg

MemoryBuffer:       .byte   MEMORY_BUFFER_SIZE
PostMemoryBuffer:   .byte   2
PageBuffer:         .byte   PAGE_BUFFER_SIZE

.cseg

.org        BOOT_LOADER_ADDRESS
            jmp             Start
            jmp             Start_No_Skip

.org        BOOT_LOADER_ADDRESS + TWIaddr
            jmp             TWIInterrupt

.org        BOOT_LOADER_ADDRESS + INT_VECTORS_SIZE
Start_No_Skip:
            cli
            ldi             A, high(RAMEND)    ; Main program start
            out             SPH, A             ; Set Stack Pointer to top of RAM
            ldi             A, low(RAMEND)
            out             SPL, A
            rjmp            Continue_with_bootloader
Start:
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
            clr             R_LEN
            ldiw            X, PostMemoryBuffer
            st              X+, R_LEN
            st              X+, R_LEN

            next_byte       Read_First
            ldiw            X, MemoryBuffer

            prepare_read
            writeByteI      'R'
            writeByteI      'E'
            writeByteI      'A'
            writeByteI      'D'
            writeByteI      'Y'
            set_read

            ldi             A, (1<<TWINT) | (1<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (1<<TWEN) | (1<<TWIE)
            mov             R_ACK, A

            ldi             A, (1<<TWINT) | (0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (1<<TWEN) | (1<<TWIE)
            mov             R_NACK, A

            ldi             A, (1<<TWINT) | (1<<TWEA) | (0<<TWSTA) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE)
            mov             R_STOP, A

            ldi             A, I2C_ADDRESS << 1
            sts             TWAR, A

            ldi             A, (0<<TWINT) | (1<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (1<<TWEN) | (1<<TWIE)
            sts             TWCR, A

            ldi             A, (1<<IVCE)
            out             MCUCR, A
            ldi             A, (1<<IVSEL)
            out             MCUCR, A

            ldi             R_COMMAND, NONE_COMMAND

            sei

Loop:
            cpi             R_COMMAND, RESET_COMMAND
            breq            Got_Reset_Command

            cpi             R_COMMAND, READ_MEMORY_COMMAND
            breq            Got_Read_Command

            cpi             R_COMMAND, READ_MEMORY_CONT_COMMAND
            breq            Got_Read_Cont_Command

            cpi             R_COMMAND, WRITE_MEMORY_COMMAND
            breq            Got_Write_Command

            cpi             R_COMMAND, READID_COMMAND
            breq            Got_Read_Id_Command

            cpi             R_COMMAND, VERSION_COMMAND
            breq            Got_Version_Command

            cpi             R_COMMAND, SIGNATURE_COMMAND
            breq            Got_Signature_Command

            cpi             R_COMMAND, OK_COMMAND
            breq            Got_OK_Command

            rjmp            Loop


Got_Reset_Command:
            ldi             R_COMMAND, NONE_COMMAND
            jmp             SetupReset

Got_Read_Command:
            ldi             R_COMMAND, NONE_COMMAND
            jmp             Read_Memory

Got_Read_Cont_Command:
            ldi             R_COMMAND, NONE_COMMAND
            jmp             Read_Memory_Cont

Got_Write_Command:
            ldi             R_COMMAND, NONE_COMMAND
            jmp             Write_Memory

Got_Signature_Command:
            ldi             R_COMMAND, NONE_COMMAND
            jmp             Read_Signature

Got_Version_Command:
            ldi             R_COMMAND, NONE_COMMAND
            jmp             Read_Version

Got_OK_Command:
            ldi             R_COMMAND, NONE_COMMAND

            prepare_read
            writeByteI      0x14
            writeByteI      0x10
            set_read
            jmp             Loop


Got_Read_Id_Command:
            ldi             R_COMMAND, NONE_COMMAND

            prepare_read
            writeByteI      0x14
            writeByteI      'C'
            writeByteI      'S'
            writeByteI      ' '
            writeByteI      'I'
            writeByteI      '2'
            writeByteI      'C'
            writeByteI      ' '
            writeByteI      'S'
            writeByteI      'T'
            writeByteI      'K'
            writeByteI      0x10
            set_read

            jmp             Loop

Read_Signature:
            prepare_read

            writeByteI      0x14

            clr             ZH
            clr             ZL

            ldi             B, 0x21; (1<<SIGRD) | (1<<SPMEM)
            out             SPMCSR, B
            lpm             B, Z
            writeByteR      B

            subi            ZL, -2
            ldi             B, 0x21; (1<<SIGRD) | (1<<SPMEM)
            out             SPMCSR, B
            lpm             B, Z
            writeByteR      B

            subi            ZL, -2
            ldi             B, 0x21; (1<<SIGRD) | (1<<SPMEM)
            out             SPMCSR, B
            lpm             B, Z
            writeByteR      B
            writeByteI      0x10
            set_read
            jmp             Loop


Read_Version:
            prepare_read
            writeByteI      0x14

            mov             B, R_VERSION_TEMP
            cpi             B, 0x80
            brne            _not_hw_ver
            writeByteI      HW_VER
            rjmp            _version_end
_not_hw_ver:
            cpi             B, 0x81
            brne            _not_sw_maj
            writeByteI      SW_MAJOR
            rjmp            _version_end
_not_sw_maj:
            cpi             B, 0x82
            brne            _not_sw_min
            writeByteI      SW_MINOR
            rjmp            _version_end
_not_sw_min:
            cpi             B, 0xC7
            brne            _not_flash_page_size
            writeByteI      low(FLASH_PAGE_SIZE)
            writeByteI      high(FLASH_PAGE_SIZE)
            rjmp            _version_end
_not_flash_page_size:
            cpi             B, 0x52
            brne            _not_eeprom_page_size
            writeByteI      EEPROM_PAGE_SIZE
            rjmp            _version_end
_not_eeprom_page_size:
            cpi             B, 0x98
            brne            _not_something_else
            writeByteI      0x03
            rjmp            _version_end
_not_something_else:
            writeByteI      0x0
_version_end:
            writeByteI      0x10

            set_read
            jmp             Loop



SetupReset:
            prepare_read
            writeByteI      0x14
            writeByteI      0x10
            set_read

            ldi             B, 100
            mov             R_TEMP, B
Reset_Pause_2:
            ldi             B, 100
Reset_Pause_1:
            dec             B
            brne            Reset_Pause_1
            dec             R_TEMP
            brne            Reset_Pause_2

DoReset:
            cli
            ldi             B, (1<<IVCE)
            out             MCUCR, B
            ldi             B, (0<<IVSEL)
            out             MCUCR, B

            jmp             0


; --- Read

Read_Memory_Cont:
            prepare_read
            rjmp            _Read_Memory_Begin

Read_Memory:
            prepare_read
            writeByteI       0x14

_Read_Memory_Begin:
            mov             ZL, R_ADDR_L
            mov             ZH, R_ADDR_H

_ReadMemory_loop1:
            tst             R_LEN_H
            brne            _ReadMemory_loop1_ok
            tst             R_LEN_L
            breq            _ReadMemory_finished_all

            cpi             R_LEN, MEMORY_BUFFER_SIZE
            brge            _ReadMemory_filled_in_buffer

_ReadMemory_loop1_ok:

            cpi             R_DESTINATION, 'E'
            breq            _ReadEEPROM


            lpm             B, Z+
            rjmp            _ReadMemory_cont

_ReadEEPROM:
            sbic            EECR, EEPE
            rjmp            _ReadEEPROM
            out             EEARH, ZH
            out             EEARL, ZL
            sbi             EECR, EERE
            in              B, EEDR

            push            B
            ldi             B, 1
            add             ZL, B
            clr             B
            adc             ZH, B
            pop             B

_ReadMemory_cont:
            writeByteR      B

            ldi             B, 1
            sub             R_LEN_L, B
            clr             B
            sbc             R_LEN_H, B
            rjmp            _ReadMemory_loop1


_ReadMemory_filled_in_buffer:
            mov             R_ADDR_L, ZL
            mov             R_ADDR_H, ZH

            set_read
            jmp             Loop


_ReadMemory_finished_all:
            mov             R_ADDR_L, ZL
            mov             R_ADDR_H, ZH

            writeByteI      0x10
            set_read
            jmp             Loop

; --- Write

Write_Memory:
            prepare_read
            writeByteI       0x14


            ldiw            Z, PageBuffer

            cpi             R_DESTINATION, 'E'
            breq            _Write_EEPROM

_Write_Flash:

            push            YL
            push            YH

            ldiw            Y, PageBuffer
            mov             ZL, R_ADDR_L
            mov             ZH, R_ADDR_H

            call            Write_page

            ldi             B, low(PAGE_SIZE)
            add             R_ADDR_L, B
            ldi             B, high(PAGE_SIZE)
            adc             R_ADDR_H, B

            pop             YH
            pop             YL

            rjmp            _Write_Memory_Finished

_Write_EEPROM:
            tst             R_LEN_L
            breq            _Write_Memory_Finished

_Write_EEPROM_ok:
            ld              B, Z+

_Write_EEPROM_Wait:
            sbic            EECR, EEPE
            rjmp            _Write_EEPROM_Wait

            out             EEARH, R_ADDR_H
            out             EEARL, R_ADDR_L

            out             EEDR, B
            sbi             EECR, EEMPE
            sbi             EECR, EEPE

            ldi             B, 1
            add             R_ADDR_L, B
            clr             B
            adc             R_ADDR_H, B

            dec             R_LEN_L

            rjmp            _Write_EEPROM


_Write_Memory_Finished:
            writeByteI      0x10
            set_read

            rjmp            Loop


; --- TWI Interrupt --------------------------------------------------------------------------------------------
TWIInterrupt:
            in			    R_STORED_STATUS_REG, SREG
            lds             A, TWSR
            andi            A, 0xF8

Process_Status:
            cpi             A, 0xA8
            breq            Transmit_Data
            cpi             A, 0xB0
            breq            Transmit_Data
            cpi             A, 0xB8
            breq            Transmit_Data

            cpi             A, 0x08
            breq            Start_Repeated_Start
            cpi             A, 0x010
            breq            Start_Repeated_Start
            cpi             A, 0x60
            breq            Address_Received
            cpi             A, 0x68
            breq            Address_Received
            cpi             A, 0x70
            breq            Address_Received
            cpi             A, 0x78
            breq            Address_Received
            cpi             A, 0x80
            breq            Data_Received
            cpi             A, 0x90
            breq            Data_Received
            cpi             A, 0xA0
            breq            Slave_Stop
            cpi             A, 0x88
            breq            Received_Data_Nack
            cpi             A, 0x98
            breq            Received_Data_Nack
            cpi             A, 0xC0
            breq            Transmit_Data_Nack
            cpi             A, 0xC8
            breq            Transmit_Data_Nack

            rjmp            Continue_Master_Checks

Transmit_Data_Nack:
Master_Received_Ack:
Master_Arbitration_Lost:
Address_Received:
Start_Repeated_Start:
Slave_Stop:
Master_Data:
;            ldiw            X, MemoryBuffer
            ret_ack

Received_Data_Nack:
            nack
            out			    SREG, R_STORED_STATUS_REG
            reti


Data_Received:
;            ijmp            ; Z
            push            R_NEXT_L
            push            R_NEXT_H
            ret


Transmit_Data:
            ld              A, X+
            sts             TWDR, A
            ack

            cpi             XH, high(MemoryBuffer + MEMORY_BUFFER_SIZE + PAGE_BUFFER_SIZE + 1)
            breq            Transmit_Test_Low
            brlo            Transmit_OK
            rjmp            Transmit_Fix

Transmit_Test_Low:
            cpi             XL, low(MemoryBuffer + MEMORY_BUFFER_SIZE + PAGE_BUFFER_SIZE + 1)
            breq            Transmit_OK
            brlo            Transmit_OK

Transmit_Fix:
            ld              A, -X
Transmit_OK:
            out			    SREG, R_STORED_STATUS_REG
            reti



Continue_Master_Checks:
            cpi             A, 0x18
            breq            Master_Received_Ack
            cpi             A, 0x28
            breq            Master_Received_Ack

            cpi             A, 0x20
            breq            Error
            cpi             A, 0x30
            breq            Error

            cpi             A, 0x38
            breq            Master_Arbitration_Lost
            cpi             A, 0x40
            breq            Master_Data
            cpi             A, 0x50
            breq            Master_Data

            cpi             A, 0x48
            breq            Error
            cpi             A, 0x58
            breq            Error

            cpi             A, 0xF8
            brne            Error
            out			    SREG, R_STORED_STATUS_REG
            reti

Error:
            ldiw            X, MemoryBuffer
            stop

Error_Wait:
            lds             A, TWCR
            sbrc            A, TWSTO
            ; TODO - is this correct to wait stop bit inside of the interrupt?
            rjmp            Error_Wait

            out			    SREG, R_STORED_STATUS_REG
            reti


Read_First:
            lds             A, TWDR
            cpi             A, '0'
            breq            Read_First_OK_Request
            cpi             A, '1'
            breq            Read_First_Programmer_Id
            cpi             A, 'u'
            breq            Read_First_Signature
            cpi             A, 'A'
            breq            Read_First_Version
            cpi             A, 'Q'
            breq            Read_First_Reset
            cpi             A, 'U'
            breq            Read_First_Receive_Address
            cpi             A, 't'
            breq            Read_First_Read_Memory
            cpi             A, 1
            breq            Read_First_Read_Memory_Cont
            cpi             A, 'd'
            breq            Read_First_Write_Memory
            cpi             A, 2
            breq            Read_First_Write_Memory_Cont
            cpi             A, ' '
            cpi             A, 0

Reset_And_Ret:
            ldiw            X, MemoryBuffer
            ret_ack


Read_First_OK_Request:
            next_byte       OK_Request
            rjmp            Reset_And_Ret

Read_First_Programmer_Id:
            next_byte       Programmer_Id
            rjmp            Reset_And_Ret

Read_First_Signature:
            next_byte       Signature
            rjmp            Reset_And_Ret

Read_First_Version:
            next_byte       Version
            rjmp            Reset_And_Ret

Read_First_Reset:
            next_byte       Prepare_Reset
            rjmp            Reset_And_Ret

Read_First_Receive_Address:
            next_byte       Receive_Address_Low
            rjmp            Reset_And_Ret

Read_First_Read_Memory:
            next_byte       Read_Memory_High
            rjmp            Reset_And_Ret

Read_First_Read_Memory_Cont:
            ldi             R_COMMAND, READ_MEMORY_CONT_COMMAND
            next_byte       Read_First
            rjmp            Reset_And_Ret

Read_First_Write_Memory:
            next_byte       Write_Memory_High
            rjmp            Reset_And_Ret

Read_First_Write_Memory_Cont:
            next_byte       Read_First
            rjmp            Reset_And_Ret


Protocol_Error:
Read_First_Setup:
            ldiw            X, MemoryBuffer
            next_byte       Read_First
            ret_ack

; --- OK Request

OK_Request:
            lds             A, TWDR
            cpi             A, ' '
            brne            Protocol_Error
            ldi             R_COMMAND, OK_COMMAND
            rjmp            Read_First_Setup


; --- Programmer Id

Programmer_Id:
            lds             A, TWDR
            cpi             A, ' '
            brne            Protocol_Error
            ldi             R_COMMAND, READID_COMMAND
            rjmp            Read_First_Setup


; --- Programmer Id

Signature:
            lds             A, TWDR
            cpi             A, ' '
            brne            Protocol_Error
            ldi             R_COMMAND, SIGNATURE_COMMAND
            rjmp            Read_First_Setup

; --- Version

Version:
            lds             A, TWDR
            mov             R_VERSION_TEMP, A
            next_byte       Version_Last
            ret_ack

Version_Last:
            lds             A, TWDR
            cpi             A, ' '
            brne            Protocol_Error
            ldi             R_COMMAND, VERSION_COMMAND
            rjmp            Read_First_Setup


; --- Reset

Prepare_Reset:
            lds             A, TWDR
            cpi             A, ' '
            brne            Protocol_Error
            ldi             R_COMMAND, RESET_COMMAND
            rjmp            Read_First_Setup


; --- Receive Address

Receive_Address_Low:
            lds             R_ADDR_L, TWDR
            next_byte       Receive_Address_High
            ret_ack

Receive_Address_High:
            lds             R_ADDR_H, TWDR
            next_byte       Receive_Address_Last
            ret_ack

Receive_Address_Last:
            lds             A, TWDR
            ldi             R_COMMAND, OK_COMMAND
            rjmp            Read_First_Setup


; --- Read Memory
Read_Memory_High:
            lds             R_LEN_H, TWDR
            next_byte       Read_Memory_Low
            ret_ack

Read_Memory_Low:
            lds             R_LEN_L, TWDR
            next_byte       Read_Memory_Destination
            ret_ack

Read_Memory_Destination:
            lds             R_DESTINATION, TWDR
            ldi             R_COMMAND, READ_MEMORY_COMMAND
            rjmp            Read_First_Setup


; --- Read Memory
Write_Memory_High:
            prepare_read
            lds             R_LEN_H, TWDR
            next_byte       Write_Memory_Low
            ret_ack

Write_Memory_Low:
            lds             R_LEN_L, TWDR
            next_byte       Write_Memory_Destination
            ret_ack

Write_Memory_Destination:
            lds             R_DESTINATION, TWDR
            mov             R_LEN_H, R_LEN_L                ; save for later
            ldiw            X, PageBuffer
            next_byte       Write_Memory_Data
            ret_ack

Write_Memory_Data:
            lds             A, TWDR
            st              X+, A
            dec             R_LEN_L         ; it will never be over 128 bytes anyway
            breq            Write_Memory_End
            ret_ack

Write_Memory_End:
            mov             R_LEN_L, R_LEN_H
            clr             R_LEN_H
            next_byte       Write_Memory_Setup
            ret_ack

Write_Memory_Setup:
            lds             A, TWDR
            cpi             A, ' '
            brne            Protocol_Error_2
            ldi             R_COMMAND, WRITE_MEMORY_COMMAND
Protocol_Error_2:
            ldiw            X, MemoryBuffer
            clr             A
            st              X, A
            rjmp            Read_First_Setup




; --- Write Flash ---------------------------------------------

.equ PAGESIZEB = PAGESIZE*2   ;PAGESIZEB is page size in BYTES, not words

Write_page:

            ldi             B, (1<<PGERS) | (1<<SELFPRGEN)
            call            Do_spm

; re-enable the RWW section
            ldi             B, (1<<RWWSRE) | (1<<SELFPRGEN)
            call            Do_spm

; transfer data from RAM to Flash page buffer
            ldi             B, FLASH_PAGE_SIZE / 2              ;init loop variable
            mov             R_Loop, B

Wrloop:
            ld              r0, Y+
            ld              r1, Y+
            ldi             B, (1<<SELFPRGEN)
            call            Do_spm

            ldi             B, 2                        ;            adiw            ZH:ZL, 2
            add             ZL, B
            clr             B
            adc             ZH, B

            dec             R_Loop
            brne            Wrloop

; execute Page Write
            subi            ZL, low(PAGESIZEB)
            sbci            ZH, high(PAGESIZEB)

            ldi             B, (1<<PGWRT) | (1<<SELFPRGEN)
            call Do_spm

; re-enable the RWW section
            ldi             B, (1<<RWWSRE) | (1<<SELFPRGEN)
            call            Do_spm

; read back and check, optional


; return to RWW section
;restore pointer
;use subi for PAGESIZEB<=256
; verify that RWW section is safe to read Return:
            in              R_TEMP, SPMCSR
            sbrs            R_TEMP, RWWSB                        ; If RWWSB is set, the RWW section is not ready yet
            ret
; re-enable the RWW section
            ldi             B, (1<<RWWSRE) | (1<<SELFPRGEN)
            call            Do_spm
            ret


Do_spm:
; input: B determines SPM action
; disable interrupts if enabled, store status in temp2, SREG
; check that no EEPROM write access is present

Wait_spm:   ; check for previous SPM complete
            in              R_TEMP, SPMCSR
            sbrc            R_TEMP, SELFPRGEN
            rjmp            Wait_spm


            in              R_STORED_STATUS_REG2, SREG
            cli
Wait_ee:
            sbic            EECR, EEPE
            rjmp            Wait_ee

; SPM timed sequence
            out             SPMCSR, B
            spm
            ; restore SREG (to enable interrupts if originally enabled)
            out             SREG, R_STORED_STATUS_REG2
            sei
            ret
