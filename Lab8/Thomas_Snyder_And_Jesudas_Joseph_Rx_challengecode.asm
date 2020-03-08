;***********************************************************
;*
;*	Lab 8 Receiver Challenge
;*
;*	A program that runs a robot routine to receive commands
;*  from a transmitting remote via IR using the USART data
;*  bus. Also, when the halt or freeze commands are received
;*  the robot will display its speed on the LEDs with PWM
;*  indicators and a binary representation of the speed level.
;*
;*	This is the RECEIVE skeleton file for Lab 8 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Thomas Snyder and Jesudas Joseph
;*	   Date: 03/07/2020
;*
;***********************************************************
.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def    zero = r7
.def    SpeedInc = r8
.def	mpr = r16				; Multi-Purpose Register
.def    temp = r15
.def    MovementState = r17     ; Movement State register
.def    FreezeCount = r18       ; Freeze Count
.def	olcnt = r19				; Outer Loop Counter
.def	waitcnt = r20			; Wait Loop Counter
.def	ilcnt = r21				; Inner Loop Counter
.def    LastMovement = r22      ; For tracking the past movement state
.def    LastAddress = r23
.def    PWMVal      = r24
.def    SpdLvl      = r25



.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit

.equ	BotAddress = $7a;(Enter your robot's address here (8 bits))
.equ    FrzCmd  = $f8
.equ    FrzSig  = $55

;/////////////////////////////////////////////////////////////
;These macros are the values to make the TekBot Move.
;/////////////////////////////////////////////////////////////
.equ	MovFwd =  (1<<EngDirR|1<<EngDirL)	;0b01100000 Move Forward Action Code
.equ	MovBck =  $00						;0b00000000 Move Backward Action Code
.equ	TurnR =   (1<<EngDirL)				;0b01000000 Turn Right Action Code
.equ	TurnL =   (1<<EngDirR)				;0b00100000 Turn Left Action Code
.equ	Halt =    (1<<EngEnR|1<<EngEnL)		;0b10010000 Halt Action Code

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

;Should have Interrupt vectors for:
;- Left whisker
;- Right whisker
;- USART receive
.org    $0002
        rcall   RWhiskerTrig ; INT0
        reti

.org    $0004
        rcall   LWhiskerTrig ; INT1
        reti

.org    $003C
        rcall   USART_RX ; USART RX EN 1
        reti

.org	$0046					; End of Interrupt Vectors

;***********************************************************
;*	Program Initialization
;***********************************************************
INIT:
	;Stack Pointer (VERY IMPORTANT!!!!)
    ldi     mpr, high(RAMEND)
    out     SPH, mpr
    ldi     mpr, low(RAMEND)
    out     SPL, mpr


    ;I/O Ports
    ldi     mpr, $ff
    out     DDRB, mpr ; Init LEDs to outputs

    ldi     mpr, 0b00001000
    out     DDRD, mpr ; Init DDRD as inputs except
                        ; TX
    
    ldi     mpr, 0b11110111
    out     PORTD, mpr  ; Set pullups

    ;USART1
        ;Set baudrate at 2400bps
    ldi     mpr, high(832)
    sts     UBRR1H, mpr
    ldi     mpr, low(832)
    sts     UBRR1L, mpr

        ;Enable receiver and enable receive interrupts
        ;Set frame format: 8 data bits, 2 stop bits
    ldi     mpr, (1 << U2X1)
    sts     UCSR1A, mpr

    ldi     mpr, (1 << RXCIE1 | 1 << RXEN1 | 1 << TXEN1)
    sts     UCSR1B, mpr

    ldi     mpr, (7 << UCSZ10)
    sts     UCSR1C, mpr
    
    ;External Interrupts
    ldi     mpr, $03
    out     EIMSK, mpr

        ;Set the External Interrupt Mask
    ldi     mpr, $0a
    sts     EICRA, mpr
		;Set the Interrupt Sense Control to falling edge detection
    
    clr     zero ; Set zero val
    ldi     LastMovement, MovFwd ; Initialize to move forward
    ldi     MovementState, MovFwd
    clr     FreezeCount

    ; Set the speed levels
    ldi     mpr, $11
    mov     SpeedInc, mpr
    ldi     SpdLvl, $00
    ldi     PWMVal, $00

    ; Timer/counter PWM
    ; COM 1:0 = 11
    ; CS 2:0 = 001 (no prescale)
    ; 
    ldi     mpr, 0b01111001 ; No Prescale
    out     TCCR0, mpr 
    
    out     OCR0, PWMval ; Init zero


    ldi     mpr, 0b01111001 ; No Prescale
    out     TCCR2, mpr 

    out     OCR2, PWMval ; Init zero
        

    sei
    rjmp    MAIN
	;Other

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:
    andi    mpr, $00
    or      mpr, MovementState
    or      mpr, SpdLvl

    out     PORTB, mpr 
    rjmp    MAIN

;***********************************************************
;*	Functions and Subroutines
;***********************************************************



;-----------------------------------------------------------
;   Func:   USART_RX
;   Desc:   A function that runs the USART receive routine
;-----------------------------------------------------------
USART_RX:
    push    mpr


    lds     mpr, UDR1

    ; out     PORTB, mpr
    ; ldi     waitcnt, 50
    ; rcall   Wait

    mov     temp, mpr
    lsl     mpr
    mov     mpr, temp
    brcc    FreezeOrAddress ; If msb is zero, address or freeze

    ; MSB 1: command
    cpi     LastAddress, BotAddress ; Check if we received bot address
    brne    ENDRX

    ; We have the correct bot address
    ; check if frzcmd
    cpi     mpr, FrzCmd
    brne    MoveCommand
    ; Make sure it doesn't go down below 00
    cpi     SpdLvl, $0f
    breq    ENDRX
    ; Decrement Speed
    inc     SpdLvl
    add     PWMVal, SpeedInc
    ; Output to OCR0/2
    out     OCR0, PWMVal
    out     OCR2, PWMVal

    ; Output speed to PORTB
    in      mpr, PORTB
    andi    mpr, $f0
    andi    SpdLvl, $0f
    or      mpr, SpdLvl
    out     PORTB, mpr



    ; if it is a freeze cmd, send freeze
    ; disable the rx so that we do not have
    ; to worry about freezing ourself

    ; ldi     mpr, (1 << TXEN1)
    ; sts     UCSR1B, mpr

    ; ; Send freeze
    ; rcall   SENDFrz

    ; ; Load waitcnt
    ; ldi     waitcnt, 10
    ; ; Wait
    ; rcall   Wait

    ; ; Reenable RX, TX, and RXinterrupt
    ; ldi     mpr, (1 << RXEN1 | 1 << RXCIE1 | 1 << TXEN1)
    ; sts     UCSR1B, mpr


    clr     LastAddress
    rjmp    ENDRX

    MoveCommand:
    ; It's a move command
    lsl     mpr
CHECKSpeedDown:
    cpi     mpr, Halt
    brne    MOVEMENTCmd
    ; Make sure it doesn't go down below 00
    cpi     SpdLvl, $00
    breq    ENDRX
    ; Decrement Speed
    dec     SpdLvl


    sub     PWMVal, SpeedInc
    ; Output to OCR0/2
    out     OCR0, PWMVal
    out     OCR2, PWMVal

    ; Output speed to PORTB
    in      mpr, PORTB
    andi    mpr, $f0
    andi    SpdLvl, $0f
    or      mpr, SpdLvl
    out     PORTB, mpr

    clr     LastAddress
    rjmp    ENDRX

MOVEMENTCmd:

    out     PORTB, mpr
    mov     LastMovement,MovementState
    mov     MovementState, mpr ; NOTE: cmds are movements >> 1 | $80
    clr     LastAddress
    rjmp    ENDRX

    ; MSB is 0: freeze or address
    FreezeOrAddress:
    
    cpi     mpr, FrzSig
    breq    FREEZESignalHandle

    ; If its not a freeze signal, store to last address
    mov     LastAddress, mpr
    rjmp    ENDRX

    FREEZESignalHandle:
    ldi     mpr, Halt
    out     PORTB, mpr

    ; Freeze counter for counting how many times frozen
    ; to see if it dies
    inc     FreezeCount
    cpi     FreezeCount, 3 
    brne    NOTDead 
    
    FREEZEDEAD: rjmp    FREEZEDEAD

    NOTDead:
    
    ldi     mpr, 5
    ldi     waitcnt, 100
    WAITFrozen:
    rcall   Wait
    dec     mpr
    cpi     mpr, 0
    brne    WAITFrozen

ENDRX:
    rcall   FlushRxBuffer
    pop     mpr
    ret



;-----------------------------------------------------------
;   Func:   FlushRxBuffer
;   Desc:   loop and read until the buffer is flushed
;-----------------------------------------------------------
FlushRxBuffer:
    push    mpr 
FlushLoop:
    lds     mpr, UCSR1A
    ; Check if rx complete
    andi    mpr, (1 << RXC1)
    cpi     mpr, (1 << RXC1)
    brne    ENDFLUSH

    ; If rx complete, load it and call func again
    lds     mpr, UDR1
    ldi     mpr, $ff
    out     PORTB, mpr
    rjmp    FlushLoop

ENDFLUSH:
    pop mpr
    ret
    

;-----------------------------------------------------------
;   Func:   SENDFrz
;   Desc:   Waits until UDRE1 is set then loads the freeze
;           command into UDR1
;-----------------------------------------------------------
SENDFrz:
    push    mpr

TXWaitFrz:
    lds     mpr, UCSR1A
    ; sbrs    mpr, 5 ; poll UDRE1
    andi    mpr, (1 << 5)
    cpi     mpr, (1 << 5)
    brne    TXWaitFrz          ; If UDRE1 is not set, loop

    ; ldi     mpr, $ff
    ; out     PORTB, mpr

    ; Once UDRE1 is set
    ldi     mpr, FrzSig ; Load the freeze command
    sts     UDR1, mpr ; Put the freeze command in the TX buffer

    ; rcall   FlushRxBuffer

    pop     mpr
    ret

;-----------------------------------------------------------
;   Func: LWhiskerTrig
;   Desc: Implements the behavior: back up, turn right 
;-----------------------------------------------------------
LWhiskerTrig:
    push    mpr
    
    ; ldi     MovementState, BumpLeft

    ldi     mpr, MovBck ; Move backwards
    out     PORTB, mpr

    ldi     waitcnt, 100
    rcall   Wait   ; wait

    ldi     mpr, TurnR
    out     PORTB, mpr

    ldi     waitcnt, 100
    rcall   Wait ; Same configuration

    ldi     mpr, MovFwd
    out     PORTB, mpr ; Set back to forward

    ldi     mpr, $03
    out     EIFR, mpr ; Clear flag register
    
    pop     mpr
    ret

;-----------------------------------------------------------
;   Func: RWhiskerTrig
;   Desc: Implements the behavior: back up, turn left 
;-----------------------------------------------------------
RWhiskerTrig:
    push    mpr


    ldi     mpr, MovBck ; Move backwards
    out     PORTB, mpr

    ldi     waitcnt, 100
    rcall   Wait   ; wait

    ldi     mpr, TurnL
    out     PORTB, mpr

    ldi     waitcnt, 100
    rcall   Wait ; Same configuration

    ldi     mpr, MovFwd
    out     PORTB, mpr

    pop     mpr
    ldi     mpr, $03
    out     EIFR, mpr
    ret


;----------------------------------------------------------------
; Sub:	Wait
; Desc:	A wait loop that is 16 + 159975*waitcnt cycles or roughly 
;		waitcnt*10ms.  Just initialize wait for the specific amount 
;		of time in 10ms intervals. Here is the general eqaution
;		for the number of clock cycles in the wait loop:
;			((3 * ilcnt + 3) * olcnt + 3) * waitcnt + 13 + call
;----------------------------------------------------------------
Wait:
        push	waitcnt			; Save wait register
        push	ilcnt			; Save ilcnt register
        push	olcnt			; Save olcnt register

Loop:	ldi		olcnt, 224		; load olcnt register
OLoop:	ldi		ilcnt, 237		; load ilcnt register
ILoop:	dec		ilcnt			; decrement ilcnt
		brne	ILoop			; Continue Inner Loop
		dec		olcnt		; decrement olcnt
		brne	OLoop			; Continue Outer Loop
		dec		waitcnt		; Decrement wait 
		brne	Loop			; Continue Wait loop	

		pop		olcnt		; Restore olcnt register
		pop		ilcnt		; Restore ilcnt register
		pop		waitcnt		; Restore wait register
		ret				; Return from subroutine


;***********************************************************
;*	Stored Program Data
;***********************************************************

;***********************************************************
;*	Additional Program Includes
;***********************************************************
