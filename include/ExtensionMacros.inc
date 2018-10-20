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

.def        A = r16
.def        B = r17
.def        C = r18
.def        D = r19
.equ        ADDR = 20
.def        ADDR1 = r20
.def        ADDR2 = r21
.def        ADDR3 = r22
.def        ADDR4 = r23
.def        WL = r24
.def        WH = r25

.macro      ldilow
            ldi             A, @1
            mov             @0, A
.endmacro

.macro      movw
            mov        @0L, @1L
            mov        @0H, @1H
.endmacro

.macro      ldipw
            ldi        @0L, low(@1 << 1)
            ldi        @0H, high(@1 << 1)
.endmacro

.macro      clrw
            clr        @0L
            clr        @0H
.endmacro

.macro      ldiw
            ldi        @0L, low(@1)
            ldi        @0H, high(@1)
.endmacro

.macro      adiw16
            ldi        A, low(@1)
            add        @0L, A
            ldi        A, high(@1)
            adc        @0H, A
.endmacro

.macro      ldl
            lds        @0L, @2
            lds        @0H, @2 + 1
            lds        @1L, @2 + 2
            lds        @1H, @2 + 3
.endmacro

.macro      stsl
            sts        @0, @1L
            sts        @0 + 1, @1H
            sts        @0 + 2, @2L
            sts        @0 + 3, @2H
.endmacro

.macro      exchangeA
            mov        A, @0
            mov        @0, @1
            mov        @1, A
.endmacro


.macro      hexnumber               ; r0, -> r1, r2
            mov         @2, @0
            swap        @0
            andi        @0, 0x0F
            subi        @0, -48
            cpi         @0, 58
            brlt        _hex1
            subi        @0, -7
_hex1:
            mov         @1, @0
            mov         @0, @2

            push        @0
            andi        @0, 0x0F
            subi        @0, -48
            cpi         @0, 58
            brlt        _hex2
            subi        @0, -7
_hex2:
            mov         @2, @0
            pop         @0
.endmacro

.macro      setflag         ; flag_register, flag_name
            sbr             @0, 1 << @0_@1
.endmacro

.macro      clrflag         ; flag_register, flag_name
            cbr             @0, 1 << @0_@1
.endmacro

.macro      skipflag        ; flag_register, flag_name
            sbrs            @0, @0_@1
.endmacro

.macro      skipnotflag     ; flag_register, flag_name
            sbrc            @0, @0_@1
.endmacro



;            push            @0
;            mov             @1, @0
;            andi            @0, 0x0f
;            subi            @0, -48
;            cpi             @0, 58
;            brlt            _hex1
;            subi            @0, -7
;_hex1:
;            mov             @2, @0
;            mov             @0, @1
;            swap            @0
;            andi            @0, 0x0f
;            subi            @0, -48
;            brlt            _hex2
;            subi            @0, -7
;_hex2:
;
;            mov             @1, @0
;            pop             @0
;.endmacro

