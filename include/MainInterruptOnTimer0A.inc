; ===============================================================================
;
; MainInterruptOnTimer0 (8bit)
;
; Copyright (C) 2008 Creative Sphere Limited
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
; This module defines main interrupt that happens 100 times
; in a second.
;
; @author Daniel Sendula
;

.dseg

MainInterruptOnTimer0_Timer:	.byte	1

.cseg

;--------------------------------
; This routine invokes creation of a servo pulse
;
; In:
;    A - Pulse width 0 - 88
; Uses:
;    B, Timer2
;
MainInterruptOnTimer0_Init:

		clr		A
		out		TCNT0, A

		ldi		A, 78; (ProcessorSpeed / (1024 * 100))	; 78!
;		ldi		A, 156; (ProcessorSpeed / (1024 * 100))	; 78!

		out		OCR0A, A

;		ldi		A, (1<<WGM01) + (1<<CS02) + (1<<CS00); 0b00001101
				; FOC0 - 0
				; WGM01:0 - 10 - CTC - OCR0 Top
				; COM01:0 - 00 - Normal operation OC0 disabled
				; CS02:0 - 101 - 1024 prescaler

		ldi		A, (1<<WGM01)
		out		TCCR0A, A

		ldi		A, (1<<CS02) + (1<<CS00); 0b0000101
		out		TCCR0B, A

		ldi		A, 1
		sts		MainInterruptOnTimer0_Timer, A

		lds		A, TIMSK0
		sbr		A, (1<<OCIE0A)
		sts		TIMSK0, A

		ret



MainInterruptOnTimer0_int:

		push		A
		in			A, SREG
		push 		A
		push		B

;		in			A, PORTD
;		ldi			D, (1 << OK_LED)
;		eor			A, D
;		out			PORTD, A

		lds			A, MainInterruptOnTimer0_Timer
		andi		A, 0b1111
		brne		MainInterruptOnTimer0_tenths_dec
; ---- Tenths of second - do whatever each 10th of second is being done


		call		Interrupt_10x1sec

		lds			A, MainInterruptOnTimer0_Timer
		swap		A
		tst			A
		brne		MainInterruptOnTimer0_sec_dec

		ldi			A, 0x99		; two nibbles set as 9s

		sts			MainInterruptOnTimer0_Timer, A

; ---- Seconds - tick - do whatever each second is being done

		call		Interrupt_1x1sec

		rjmp		MainInterruptOnTimer0_notSeconds
MainInterruptOnTimer0_sec_dec:
		dec			A
		swap		A
		ori			A, 0x09
		sts			MainInterruptOnTimer0_Timer, A
		rjmp		MainInterruptOnTimer0_notSeconds

MainInterruptOnTimer0_tenths_dec:

		lds			A, MainInterruptOnTimer0_Timer
		dec			A
		sts			MainInterruptOnTimer0_Timer, A

MainInterruptOnTimer0_notSeconds:


		lds			A, MainInterruptOnTimer0_Timer
		andi		A, 1
		brne		MainInterruptOnTimer0_not100ths
; ---- 50th of second - do whatever needs to be done each 20ms

		call		Interrupt_50x1sec


MainInterruptOnTimer0_not100ths:
; ---- 100ths of second - do whatever is needed to be done too many times

		pop		B
		pop		A
		out		SREG, A
		pop		A
		reti

