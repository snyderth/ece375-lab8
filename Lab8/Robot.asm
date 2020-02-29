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
.def    OCRCountH = r19         ; For loading OCR1AH
.def    OCRCountL = r18         ; For loading OCR1AL
.def    MovementState = r24     ; Holds the movement state
.def	FreezeCount = r20        ; Holds the freeze state

.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit
.equ    FrzTxBtn = PD4


.equ    Forward = $00
.equ    BumpLeft = $01
.equ    Stop = $02
.equ    BumpRight = $03



; Commands from remote
.equ    FwdCmd  = $b0
.equ    BckCmd  = $80
.equ    RCmd    = $a0
.equ    LCmd    = $90
.equ    HltCmd  = $c8
.equ    FrzCmd  = $f8

; Signals from other bots
.equ    FrzSig  = $55

.equ    Wait5Sec = 39062
.equ    Wait1Sec = 15624 

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
;- Timer/Counter1 OCP1A
.org    $0002  
        rcall   RWhiskerTrig    ; INT0
        reti

.org    $0004   
        rcall   LWhiskerTrig    ; INT1
        reti

.org    $0018
        rcall   WaitInterrupt     ; wait interrupt
        reti

.org    $003C
        rcall   USART_RX_Complete ; Rx data ready
        reti

; .org    $003E
        ; rcall   USART_DR_Empty ; UDRI
        ; reti

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
    ldi     MovementState, FwdCmd
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

    ; Set BAUD rate
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

    sts     TCCR1C, mpr

    ; ldi     mpr, (1 << WGM12) 
    ldi     mpr, 0b00001101
    out     TCCR1B, mpr ; Set the CTC mode. Don't set prescale
                        ; Yet. We don't want to start counting

    ; Leave Timer interrupt disabled until we want to set

    ; Clear Freeze count
    clr     FreezeCount
    sei ; Enable global interrupts




;***********************************************************
;*	Main Program
;***********************************************************
MAIN:

;\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
;////////////////////////////////////////////////////////
CHECKForwardCmd:
    cpi     MovementState, FwdCmd ; Check if moving forward
    brne    CHECKReverseCmd ; If not, skip to check reverse
    ldi     mpr, MovFwd
    out     PORTB, mpr

    
;\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
;////////////////////////////////////////////////////////
CHECKReverseCmd:
    cpi     MovementState, BckCmd ; Check if moving back
    brne    CHECKLeftCmd
    ldi     mpr, MovBck
    out     PORTB, mpr

    
;\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
;////////////////////////////////////////////////////////
CHECKLeftCmd:
    cpi     MovementState, LCmd ; Check if moving left
    brne    CHECKRightCmd
    ldi     mpr, TurnL
    out     PORTB, mpr

    
;\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
;////////////////////////////////////////////////////////
CHECKRightCmd:
    cpi     MovementState, RCmd ; Check if moving right
    brne    CHECKRightCmd
    ldi     mpr, TurnR
    out     PORTB, mpr


;\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
;////////////////////////////////////////////////////////
CHECKHaltCmd:
    cpi     MovementState, HltCmd ; Check if stopped
    brne    CHECKFreezeSig
    ldi     mpr, Halt
    out     PORTB, mpr

;\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
;////////////////////////////////////////////////////////
CHECKFreezeSig:
	cpi		MovementState, FrzSig ; Check if freeze signal
                                ;( From other bots )
	brne	POLLFrzTx
	ldi		mpr, Halt   ; If so, halt
	out		PORTB, mpr
    inc     FreezeCount ; Increment number of times frozen
    cpi     FreezeCount, 3 ; See how many timer we are frozen
    in      mpr, SREG
    andi    mpr, (1 << SREG_Z)
    cpi     mpr, (1 << SREG_Z) ;If zero is set (3 times frozen) \/
    brne    ENDFRZ
FROZEN: rjmp FROZEN     ; If we're frozen three times, remain frozen
    
    ; If we are not frozen three times, wait 5 seconds
    ldi     OCRCountH, high(Wait5Sec)
    ldi     OCRCountL, low(Wait5Sec)    ; Load 5 second wait

    out     OCR1AH, OCRCountH
    out     OCR1AL, OCRCountL

    rcall   Wait ; Wait 5 seconds
    sei     ; Must be called after wait

ENDFRZ:
    ldi     MovementState, FwdCmd ; Get out of freeze
;\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
;////////////////////////////////////////////////////////
POLLFrzTx:
    ; Check for sending freeze
    cpi     MovementState, FrzCmd
    brne    ENDMAIN
    ; If the freeze command is not rx, skip to endmain
    ; If freeze command is rx, send freeze
    rcall   SENDFrz

ENDMAIN:
    rjmp	MAIN
    
;/***********************************************************\
;/***********************************************************\
; ****************** END MAIN *******************************
;/***********************************************************/
;/***********************************************************/



;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
;   Func:   SENDFrz
;   Desc:   Waits until UDRE1 is set then loads the freeze
;           command into UDR1
;-----------------------------------------------------------
SENDFrz:
    push    mpr
    cli     ; Clear interrupts. We don't want to be interrupted

TXWaitFrz:
    lds      mpr, UCSR1A
    andi    mpr, (1 << UDRE1) ; Mask all but UDRE1
    cpi     mpr, (1 << UDRE1) ; Check if UDRE1 is set
    brne    TXWaitFrz          ; If UDRE1 is not set, loop

    ; Once UDRE1 is set
    ldi     mpr, FrzSig ; Load the freeze command
    sts     UDR1, mpr ; Put the freeze command in the TX buffer

    sei     ; Reenable interrupts
    pop     mpr
    ret


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
    sei
    ret
 
;-----------------------------------------------------------
;   Func:   Wait
;   Desc:   Wait function that waits a set number of time
;           Parameters should be written to OCR1A for time count.
;           NOTE: Interrups must be reenabled (sei or reti) after
;           this routine!!!
;-----------------------------------------------------------
Wait:
    push    mpr
    clr     WaitStatus

    ; Disable All interrupts exept timer interrupt
    ldi     mpr, $00
    out     EIMSK, mpr ; Disable external interrupts

    ldi     mpr, (3 << TXEN1)
    sts     UCSR1B, mpr ; Disable USART interrupts

    sei ; Set interrupts enabled 

    ldi     mpr, $00
    out     TCNT1H, mpr
    out     TCNT1L, mpr ; Reset the counter

    ldi     mpr, (1 << 4) 
    out     TIMSK, mpr ; Enable the timer/counter interrup

    ; ldi     mpr, (1 << WGM12 | 5 << CS10)
    ; out     TCCR1B, mpr; Set prescaler to start counter


WAITLoop:
    cpi     WaitStatus, $ff ; Set when interrupt triggers
    brne    WAITLoop

    ; ldi     mpr, (1 << WGM12)
    ; out     TCCR1B, mpr ; Clear prescaler
    cli     ; Disable gloabal interrupts

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
    
    ; ldi     MovementState, BumpLeft

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
    ldi     mpr, $03
    out     EIFR, mpr
    ret


;----------------------------------------------------------
;   Func:   USART_RX_Complete
;   Desc:   Runs upon completion of the USART Receiving data
;----------------------------------------------------------
USART_RX_Complete:
    push    mpr
    lds     mpr, UDR1
    cpi     mpr, BotAddress ; See if the byte received is addr.
    brne    CHECKCmdFrz ; If not our address, check if it's frz
WAITCmd:
    lds     mpr, UCSR1A ; Load in UCSR1A to check RXC1
    andi    mpr, (1 << RXC1) ; And to rid of all but RXC1
    cpi     mpr, (1 << RXC1) ; If RXC1 was set, they will be equal
    breq    LOADCmd     ; If RXC1 is set, load the command
    rjmp    WAITCmd     ; Wait until the next byte is received

LOADCmd:
    lds     mpr, UDR1   ; load command
    mov     MovementState, mpr ; Put command in Movement state
    rjmp    ENDRX

CHECKCmdFrz:
    cpi     mpr, FrzSig
    brne    ENDRX ; If its not a freeze command, skip
    ; If it is a freeze, set state to freeze
    mov     MovementState, mpr

ENDRX:
    pop     mpr
    ret

;***********************************************************
;*	Stored Program Data
;***********************************************************

;***********************************************************
;*	Additional Program Includes
;***********************************************************
