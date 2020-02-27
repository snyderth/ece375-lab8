;***********************************************************
;*
;*	Enter Name of file here
;*
;*	Enter the description of the program here
;*
;*	This is the RECEIVE skeleton file for Lab 8 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Enter your name
;*	   Date: Enter Date
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multi-Purpose Register
.def    WaitStatus = r17         ; holds the status of wait

.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit


.equ    FwdCmd  = $b0
.equ    BckCmd  = $80
.equ    RCmd    = $a0
.equ    LCmd    = $90
.equ    HltCmd  = $c8
.equ    FrzCmd  = $f8

.equ    Wait5Sec = 39062
.equ    Wait1Sec = 7812

.equ	BotAddress = $7a ;(Enter your robot's address here (8 bits))

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
;- USART UDR empty
.org    $0002  
        rcall   RWhiskerTrig    ; INT0
        reti

.org    $0004   
        rcall   LWhiskerTrig    ; INT0
        reti

.org    $0018
        rcall   WaitInterrupt     ; wait interrupt
        reti

.org    $003C
        ; rcall   USART_RX_Complete ; Rx data ready
        reti

.org    $003E
        ; rcall   USART_DR_Empty ; UDRI
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

    cli ;Disable interrupts just in case

	;I/O Ports
    ldi     mpr, $00
    out     DDRD, mpr ; Set PORTD as INPUTS
    ldi     mpr, $ff
    out     PORTD, mpr ; Set pullups

    ldi     mpr, $ff
    out     DDRB, mpr ; Set PORTB as OUTPUTS
    ldi     mpr, MovFwd
    out     PORTB, mpr ; Set first move to forward

	;USART1
		;Set baudrate at 2400bps
		;Enable receiver and enable receive interrupts
		;Set frame format: 8 data bits, 2 stop bits

    ldi     mpr, (1 << U2X1) ; Only setting the double data
    sts     UCSR1A, mpr

    ldi     mpr, (1 << RXCIE1 | 7 << TXEN1) ; Enable TX, RX, DRInt, RXInt
    sts     UCSR1B, mpr

    ldi     mpr, (7 << UCSZ10) ; Set 8-bit data with 2 stop bits
    sts     UCSR1C, mpr

    ldi     mpr, high(832) ; Load high byte into UBRRH
    sts     UBRR1H, mpr

    ldi     mpr, low(832) ; Load low byte into UBRRL
    sts     UBRR1L, mpr

    ;External Interrupts
        ;Set the External Interrupt Mask
        ;Set the Interrupt Sense Control to falling edge detection
    
    ; EICRA in extended I/O
    ldi     mpr, $0a    ; Using EICRA INT0-1 => 0000 1010
                        ;Falling edge detection
    sts     EICRA, mpr  ; write to EICRA
    ldi     mpr, $03    ; enable only INT1-0 => 0000 0011
    out     EIMSK, mpr

    ; Timer configuration

    ldi     mpr, high(Wait1Sec)
    out     OCR1AH, mpr ; Initialize to wait 1 second
    ldi     mpr, low(Wait1Sec)
    out     OCR1AL, mpr ; Initialize to wait 1 second

    ldi     mpr, $00
    out     TCCR1A, mpr ; No OCn Pins

    ldi     mpr, (1 << WGM12) 
    out     TCCR1B, mpr ; Set the CTC mode. Don't set prescale
                        ; Yet. We don't want to start counting

    ; Don't touch TCCR1C

    ; Leave Timer interrupt disabled until we want to set


    sei ; Enable global interrupts

	;Other

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:
		rjmp	MAIN

;***********************************************************
;*	Functions and Subroutines
;***********************************************************
;-----------------------------------------------------------
;   Func: WaitInterrupt
;   Desc: Inidcate to the wait function that the timer/counter is
;       finished
;-----------------------------------------------------------
WaitInterrupt:
    push    mpr
    ser     WaitStatus ; set status reg

    ldi     mpr, $3C
    out     TIFR, mpr ; Clear timer interrupts

    pop     mpr
    ret
 
;-----------------------------------------------------------
;   Func:   Wait
;   Desc:   Wait function that waits a set number of time
;           Parameters should be written to OCR1A for time count
;-----------------------------------------------------------
Wait:
    push    mpr
    clr     WaitStatus

    ; Disable All interrupts exept timer interrupt
    ldi     mpr, $00
    out     EIMSK, mpr ; Disable external interrupts

    ldi     mpr, (3 << TXEN1)
    sts     UCSR1B, mpr ; Disable USART interrupts

    ldi     mpr, (1 << OCIE1A)
    out     TIMSK, mpr ; Enable the timer/counter interrupt

    ldi     mpr, $00
    out     TCNT1H, mpr
    out     TCNT1L, mpr ; Reset the counter

    ldi     mpr, (1 << WGM12 | 7 << CS10)
    out     TCCR1B, mpr; Set prescaler to start counter


WAITLoop:
    cpi     WaitStatus, $ff ; Set when interrupt triggers
    brne    WAITLoop

    ldi     mpr, (1 << WGM12)
    out     TCCR1B, mpr ; Clear prescaler

    ldi     mpr, $00
    out     TIMSK, mpr ; Disable timer/counter interrupt


    ; Reenable interrupts
    ldi     mpr, $03
    out     EIMSK, mpr ; External interrupts

    ldi     mpr, (1 << RXCIE1 | 7 << TXEN1)
    sts     UCSR1B, mpr ; reenable USART
    pop     mpr

    ret


;-----------------------------------------------------------
;   Func: LWhiskerTrig
;   Desc: Implements the behavior: back up, turn right 
;-----------------------------------------------------------
LWhiskerTrig:
    push    mpr
    
    ldi     mpr, MovBck ; Move backwards
    out     PORTB, mpr


    ; Wait: configure
    ldi     r19, high(Wait1Sec)
    ldi     r18, low(Wait1Sec)

    out     OCR1AH, r19
    out     OCR1AL, r18 ; Make sure that we write high byte first

    rcall   Wait   ; wait

    ldi     mpr, TurnR
    out     PORTB, mpr

    rcall   Wait ; Same configuration

    ldi     mpr, MovFwd
    out     PORTB, mpr

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


    ; Wait: configure
    ldi     r19, high(Wait1Sec)
    ldi     r18, low(Wait1Sec)

    out     OCR1AH, r19
    out     OCR1AL, r18 ; Make sure that we write high byte first

    rcall   Wait   ; wait

    ldi     mpr, TurnL
    out     PORTB, mpr

    rcall   Wait ; Same configuration

    ldi     mpr, MovFwd
    out     PORTB, mpr

    pop     mpr
    ret

;***********************************************************
;*	Stored Program Data
;***********************************************************

;***********************************************************
;*	Additional Program Includes
;***********************************************************
