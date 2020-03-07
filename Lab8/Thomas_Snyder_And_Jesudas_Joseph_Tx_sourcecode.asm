;***********************************************************
;*
;*	remote
;*
;*	This program is a remote for a simple teckbot
;*
;*
;***********************************************************
;*
;*	 Author: Jesudas Joseph, Thomas Snyder
;*	   Date: March 5th, 2020
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multi-Purpose Register
.def	sent = r20				; Indicates if all data is sent.

.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit
; Use these action codes between the remote and robot
; MSB = 1 thus:
; control signals are shifted right by one and ORed with 0b10000000 = $80
.equ	MovFwd =  ($80|1<<(EngDirR-1)|1<<(EngDirL-1))	;0b10110000 Move Forward Action Code
.equ	MovBck =  ($80|$00)								;0b10000000 Move Backward Action Code
.equ	TurnR =   ($80|1<<(EngDirL-1))					;0b10100000 Turn Right Action Code
.equ	TurnL =   ($80|1<<(EngDirR-1))					;0b10010000 Turn Left Action Code
.equ	Halt_cmd =    ($80|1<<(EngEnR-1)|1<<(EngEnL-1))	;0b11001000 Halt Action Code

.equ	Freeze_cmd =  0b11111000						;0b11111000 Freeze Action

;Bot address
.equ	botAddr = $7A

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

.org	$0046					; End of Interrupt Vectors

;***********************************************************
;*	Program Initialization
;***********************************************************
INIT:
	;Stack Pointer
	ldi mpr, low(RAMEND)
	out	SPL, mpr
	ldi mpr, high(RAMEND)
	out	SPH, mpr

	;I/O Ports
	ldi mpr, $FF
	out DDRB, mpr
	ldi mpr, $00
	out PORTB, mpr

	ldi mpr, 0b00001000
	out DDRD, mpr
	ldi mpr, $FF
	out PORTD, mpr

	;USART1 setup
		
	ldi mpr, low(832)		;Set baudrate at 2400bps
	sts UBRR1L, mpr
	ldi mpr, high(832)
	sts UBRR1H, mpr

	ldi mpr, 0b00000010
	sts UCSR1A, mpr			;Set Double transfer rate
	
	ldi mpr, 0b00001000		
	sts UCSR1B, mpr			;Set enable Tx

	ldi mpr, 0b00001110
	sts UCSR1C, mpr			;Set USART1 to Asynch. mode, Parity off, 2 stop bits and 8bit frame size


;***********************************************************
;*	Main Program
;***********************************************************
MAIN:

	;get PIND register
	in mpr, PIND
	;Mask PIND data to ignore USART data bits 2, 3
	ori mpr, 0b00001100

	;Poll for button presses
	;branch if equal to specified button press

	cpi mpr, 0b11111110
	breq Send_Fwd_J

	cpi mpr, 0b11111101
	breq Send_Bck_J

	cpi mpr, 0b11101111
	breq Send_Right_J

	cpi mpr, 0b11011111
	breq Send_Left_J

	cpi mpr, 0b10111111
	breq Send_Halt_J

	cpi mpr, 0b01111111
	breq Send_Frz_J

	rjmp	MAIN

;Just had enough of the branch call being out of range errors.
;rcall associated routine and then rjmp to main.
Send_Fwd_J:
rcall Send_Fwd
rjmp MAIN
Send_Bck_J:
rcall Send_Bck
rjmp MAIN
Send_Right_J:
rcall Send_Right
rjmp MAIN
Send_Left_J:
rcall Send_Left
rjmp MAIN
Send_Halt_J:
rcall Send_Halt
rjmp MAIN
Send_Frz_J:
rcall Send_Frz
rjmp MAIN

;***********************************************************
;*	Functions and Subroutines
;***********************************************************
;-----------------------------------------------------------
; Func:	Send_Fwd
; Desc: This routine sends the forward command to any bot 
;		with address botAddr
;-----------------------------------------------------------
Send_Fwd:
	push mpr

	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, botAddr	
	sts UDR1, mpr		;Send Bot addr
	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, MovFwd
	sts UDR1, mpr		;Send Forward command

	;Set LEDs to represent forward motion
	ldi mpr, 0b01100000
	out PORTB, mpr

	pop mpr
	ret

;-----------------------------------------------------------
; Func:	Send_Bck
; Desc: This routine sends the backward command to any bot 
;		with address botAddr
;-----------------------------------------------------------
Send_Bck:
	push mpr

	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, botAddr
	sts UDR1, mpr		;Send Bot addr
	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, MovBck
	sts UDR1, mpr		;Send Back command

	;Set LEDs to represent backward motion
	ldi mpr, 0b00000000
	out PORTB, mpr

	pop mpr
	ret

;-----------------------------------------------------------
; Func:	Send_Right
; Desc: This routine sends the right command to any bot 
;		with address botAddr
;-----------------------------------------------------------
Send_Right:
	push mpr

	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, botAddr
	sts UDR1, mpr		;Send Bot addr
	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, TurnR
	sts UDR1, mpr		;Send Right command

	;Set LEDs to represent right motion
	ldi mpr, 0b01000000
	out PORTB, mpr

	pop mpr
	ret

;-----------------------------------------------------------
; Func:	Send_Left
; Desc: This routine sends the left command to any bot 
;		with address botAddr
;-----------------------------------------------------------
Send_Left:
	push mpr

	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, botAddr
	sts UDR1, mpr		;Send Bot addr
	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, TurnL
	sts UDR1, mpr		;Send Left command

	;Set LEDs to represent left motion
	ldi mpr, 0b00100000
	out PORTB, mpr

	pop mpr
	ret

;-----------------------------------------------------------
; Func:	Send_Halt
; Desc: This routine sends the halt command to any bot 
;		with address botAddr
;-----------------------------------------------------------
Send_Halt:
	push mpr

	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, botAddr
	sts UDR1, mpr		;Send Bot addr
	rcall WAIT_SEND		;Wait till Data register is empty
	ldi mpr, Halt_cmd
	sts UDR1, mpr		;Send Halt command

	;Set LEDs to represent halt
	ldi mpr, 0b10010000
	out PORTB, mpr

	pop mpr
	ret

;-----------------------------------------------------------
; Func:	Send_Frz
; Desc: This routine sends the freeze command to any bot 
;		with address botAddr	
;-----------------------------------------------------------
Send_Frz:
	push mpr

	;rcall WAIT_SEND	;Wait till Data register is empty
	ldi mpr, botAddr
	sts UDR1, mpr		;Send Bot addr
	rcall WAIT_SEND	;Wait till Data register is empty
	ldi mpr, Freeze_cmd
	sts UDR1, mpr		;Send Freeze command

	; rcall WAIT_SEND
	; ldi mpr, $55
	; sts UDR1, mpr

	;Set LEDs to represent freeze
	ldi mpr, 0b11111111
	out PORTB, mpr

	pop mpr
	ret

;-----------------------------------------------------------
; Func:	WAIT_SEND
; Desc: This routine Checks for the USART data register to empty
;		Exits loop when data register is empty
;-----------------------------------------------------------
WAIT_SEND:
	lds sent, UCSR1A
	sbrs sent, 5
	rjmp WAIT_SEND
	ret

;***********************************************************
;*	Stored Program Data
;***********************************************************

;***********************************************************
;*	Additional Program Includes
;***********************************************************

