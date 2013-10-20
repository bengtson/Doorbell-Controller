
; Program - dmxpack.s
;
; Written by Michael Bengtson  20-Oct-2013
;
; This is the controller for the 4 channel DMX pack.  This controls 4 light
; circuits when connected to the DMX bus.
;
; Processor is PIC16F1508.  This receives the DMX stream, watches the zero-
; crossing detector and feeds the Pulse With Modulation engines.
;
; Pin out for the chip is as follow:
;
;       Pin 01 : Vdd : +3.3V
;       Pin 02 : RA5 : Clock In  20Mhz Resonator
;       Pin 03 : RA4 : Clock Out 20Mhz Resonator
;       Pin 04 : RA3 : ICSP - Program
;       Pin 05 : RC5 : PWM 1 : Driver for channel 0
;       Pin 06 : RC4 : Debug Pin - DMX Receive UART
;       Pin 07 : RC3 : PWM 2 : Driver for channel 1
;       Pin 08 : RC6 : Debug Pin - Zero Cross
;       Pin 09 : RC7 : Debug Pin - Timer 1
;       Pin 10 : RB7
;       Pin 11 : RB6
;       Pin 12 : RB5 : DMX Receive
;       Pin 13 : RB4
;       Pin 14 : RC2 : C12IN2 - Zero Crossing Input
;       Pin 15 : RC1 : PWM 4 : Driver for channel 3
;       Pin 16 : RC0
;       Pin 17 : RA2 : PWM 3 : Driver for channel 2
;       Pin 18 : RA1 : ICSP - Clock
;       Pin 19 : RA0 : ICSP - Data
;       Pin 20 : Vss : 0.0V

; 11-Jul-2013 : Turned on the watch dog timer through config bits.  Using the
;               default 2 second timeout value.  Timer is cleared each time a
;               DMX packet is received.  If not received, the timer will reset
;               the board.  Was seeing deaf boards that were still holding a
;               level.  Processor was triggering channels at the last level
;               it received but it was not getting new levels on the DMX
;               channel.  Watch dog is a weak fix.  Need to find out why the
;               boards are going deaf.

; 16-Jul-2013 : This is a version of the code that will allow all the boards
;               to be programmed with the same code.  Once the boards have had
;               DMX channels assigned, new software can be downloaded without
;               setting the channel map again.

; 16-Jul-2013 : If a channel is set to 0, it previous was triggering the triac
;               for a very small period of time.  Because of zero cross issues
;               it would sometimes trigger a full cycle, flashing like a
;               firefly.  Now, if the level is 0, the triac is never turned on.

; 16-Jul-2013 : If a channel is set to 255 (full on), it would only get a single
;               strobe at the start of the cycle.  Now, a full on level will
;               get strobes for the entire cycle.  Note that the trigger is still
;               turned off in each level position, before being turned back on.

; This is a prototype program to manage an illuminated encoder to be used in
; the Tack Sa Mycket home we are building.  This is only a prototype since
; the chip used here will likely be different than the chip in the final
; keypad controller.
;
; A timer is set to trigger every 1ms.  This will read the encoder bits and
; check the led pwm status.  LEDs can be set from 0-15 for their intensity.
;
; Need a little state machine to determine the direction of the encoder.

	list	p=16f1508
	include <p16F1508.inc>

; Configuration bits are listed here.  There are two configuration words.
;
; CONFIG1 : 0x00e2  : 0x0fa with watch dog timer.
;
;       Bit 13    :   0 : Disable Fail Safe Clock Monitor
;       Bit 12    :   0 : Disable Internal/External Switch Over
;
;       Bit 11    :   0 : Enable CLKOUT on the CLKOUT Pin
;       Bit 10-09 :  00 : Disable Brown-Out Detection
;       Bit 08    :   0 : Unimplemented  Reads as "1"

;       Bit 07    :   1 : Disable Code Protection
;       Bit 06    :   1 : MCLR is Reset Signal
;       Bit 05    :   1 : Disable Power Up Timer
;       Bit 04-03 :  00 : Disable Watch Dog Timer
;       Bit 04-03 :  11 : Enable Watch Dog Timer.
;       Bit 02-00 : 010 : HS Oscillator On OSC1 and OSC2
;
; CONFIG2 : 0x0803
;
;       Bit 13    :   0 : High Voltage Programming on MCLR
;       Bit 12    :   0 : In-Circuit Debugger Enabled

;       Bit 11    :   1 : Disable Low Power Brown Out Reset
;       Bit 10    :   0 : Brown Out High Trip Point Selected (Not Used)
;       Bit 09    :   0 : Disable Stack Over/Underflow Reset
;       Bit 08-02 :   0 : Unimplemented  Reads as "1"
;       Bit 01-00 :  11 : Write Protection Off

	__CONFIG	_CONFIG1, 0x00fa
	__CONFIG	_CONFIG2, 0x0803

;  Define parameters about this device.
DEVICE_TYPE_DMX_DEVICE	equ		002H
DEVICE_VERSION_MINOR	equ		000H
DEVICE_VERSION_MAJOR	equ		001H

;  About the channel assignments ...
;
;  The block of code at 0x20 (note that it is on a 32 byte boundary) holds the
;  DMX channel assignments for the board channels 0-3.  It's a small map table.
;  The table can be commented out if the board already has had the map set.
;  If the board has not had the map set, then the channel addresses should be
;  set (CHANNEL_ADDRESS_0 through 3), the code uncommented and the board loaded.
;
;  When the board starts up, the addresses are read from the program flash at
;  0x20 and placed into high ram for easy accessibility.

;  Define the DMX addresses for each channel.  These do not need to be
;  sequential and may be the same as in another DMX device.

CHANNEL_ADDRESS_0           equ     .24
CHANNEL_ADDRESS_1           equ     .25
CHANNEL_ADDRESS_2           equ     .26
CHANNEL_ADDRESS_3           equ     .27

;  Uncomment this table if the board needs to have it's channel map set.

    org     0x20
channel_address_table
CHANNEL_ADD_0_LOW   dw  CHANNEL_ADDRESS_0&0xff
CHANNEL_ADD_0_HIGH  dw  CHANNEL_ADDRESS_0>>8
CHANNEL_ADD_1_LOW   dw  CHANNEL_ADDRESS_1&0xff
CHANNEL_ADD_1_HIGH  dw  CHANNEL_ADDRESS_1>>8
CHANNEL_ADD_2_LOW   dw  CHANNEL_ADDRESS_2&0xff
CHANNEL_ADD_2_HIGH  dw  CHANNEL_ADDRESS_2>>8
CHANNEL_ADD_3_LOW   dw  CHANNEL_ADDRESS_3&0xff
CHANNEL_ADD_3_HIGH  dw  CHANNEL_ADDRESS_3>>8

;  Define the locations in RAM necessary.  Valid RAM locations are 20-5F.
;  Only 64 bytes to use them wisely.

ram		equ	0x20		; First address of available RAM

;  Define the locations in RAM for holding the channel addresses.
    cblock  0x70
        dmx_0_low
        dmx_0_high
        dmx_1_low
        dmx_1_high
        dmx_2_low
        dmx_2_high
        dmx_3_low
        dmx_3_high
    endc

;  Define all the variables used for this code.  These should fit between
;  0x20 and 0x7f.
	cblock 0x20
	endc

; ------------------------------------------------------------------------------
;
;  Vector Table
;
;  Setup the reset and interrupt vectors for the chip.

	org		0x00		; Set program memory base at reset vector 0x00.
	goto	boot		; Go to boot code.

	org		0x04
	goto	system_isr			; Handle interrupts.

;
; ------------------------------------------------------------------------------

; ------------------------------------------------------------------------------
;
;  System Module
;
;  Here is where we initialize all the functions on the chip.

	org		0x40

;  Routine - system_init : Initializes the processor.
system_init

	banksel INTCON
	bcf		INTCON,GIE		; Disable all interrupts.
	bcf		INTCON,INTE

    ; Set the clock to external 20Mhz.
    banksel OSCCON
    movlw   0x00            ; Clock is external
    movwf   OSCCON

    ; Set weak pull up resisters to be controlled individually.
    banksel OPTION_REG
    bcf     OPTION_REG,NOT_WPUEN

    ; Set all ports to not be analog.  Individual drivers can change the bits.
    banksel ANSELC
    clrf ANSELC

    banksel ANSELB
    clrf    ANSELB

    banksel ANSELA
    clrf    ANSELA

    ;  We are done setting everything up, go to main.
	return		; All done so return.

;  Routine - system_start : This final system startup items such as
;  enabling interrupts.
system_start

	;  Enable interrupts.
    banksel INTCON
    bsf     INTCON,PEIE     ; Enable peripheral interrupts.
    bsf     INTCON,GIE      ; Enable global interrupts.

    ;  Return to the caller.
    return


;  Routine - system_isr : This routine handles the interrupts for the
;  processor.  Simply save state then call each potential interrupt
;  source.

system_isr

	call	timer_isr
    call    comparator_isr
;    call    uart_isr

system_isr_return
	retfie

;  Routine - system_halt : If there are any errors, then the code jumps here
;  to hopefully report the error.

system_halt
    goto    system_halt

;  System Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  DMX Module
;
;  DMX module provides support for the DMX channels.

dmx_init
    BANKSEL PMADRL          ; Select Bank for PMCON registers
    MOVLW   0    ;
    MOVWF   PMADRH          ; Store MSB of address
    BCF     PMCON1,CFGS     ; Do not select Configuration Space

    MOVLW   0x20
    MOVWF   PMADRL          ; Store LSB of address
    BSF     PMCON1,RD       ; Initiate read
    NOP                     ; Ignored (Figure 10-2)
    NOP                     ; Ignored (Figure 10-2)
    MOVF    PMDATL,W        ; Get LSB of word
    movwf   dmx_0_low
;    MOVF    PMDATH,W        ; Get MSB of word
;    MOVWF   PROG_DATA_HI    ; Store in user location
;    BCF     PMCON1,RD

    MOVLW   0x21
    MOVWF   PMADRL          ; Store LSB of address
    BSF     PMCON1,RD       ; Initiate read
    NOP                     ; Ignored (Figure 10-2)
    NOP                     ; Ignored (Figure 10-2)
    MOVF    PMDATL,W        ; Get LSB of word
    movwf   dmx_0_high
;    MOVF    PMDATH,W        ; Get MSB of word
;    BCF     PMCON1,RD

    MOVLW   0x22
    MOVWF   PMADRL          ; Store LSB of address
    BSF     PMCON1,RD       ; Initiate read
    NOP                     ; Ignored (Figure 10-2)
    NOP                     ; Ignored (Figure 10-2)
    MOVF    PMDATL,W        ; Get LSB of word
    movwf   dmx_1_low

    MOVLW   0x23
    MOVWF   PMADRL          ; Store LSB of address
    BSF     PMCON1,RD       ; Initiate read
    NOP                     ; Ignored (Figure 10-2)
    NOP                     ; Ignored (Figure 10-2)
    MOVF    PMDATL,W        ; Get LSB of word
    movwf   dmx_1_high

    MOVLW   0x24
    MOVWF   PMADRL          ; Store LSB of address
    BSF     PMCON1,RD       ; Initiate read
    NOP                     ; Ignored (Figure 10-2)
    NOP                     ; Ignored (Figure 10-2)
    MOVF    PMDATL,W        ; Get LSB of word
    movwf   dmx_2_low

    MOVLW   0x25
    MOVWF   PMADRL          ; Store LSB of address
    BSF     PMCON1,RD       ; Initiate read
    NOP                     ; Ignored (Figure 10-2)
    NOP                     ; Ignored (Figure 10-2)
    MOVF    PMDATL,W        ; Get LSB of word
    movwf   dmx_2_high

    MOVLW   0x26
    MOVWF   PMADRL          ; Store LSB of address
    BSF     PMCON1,RD       ; Initiate read
    NOP                     ; Ignored (Figure 10-2)
    NOP                     ; Ignored (Figure 10-2)
    MOVF    PMDATL,W        ; Get LSB of word
    movwf   dmx_3_low

    MOVLW   0x27
    MOVWF   PMADRL          ; Store LSB of address
    BSF     PMCON1,RD       ; Initiate read
    NOP                     ; Ignored (Figure 10-2)
    NOP                     ; Ignored (Figure 10-2)
    MOVF    PMDATL,W        ; Get LSB of word
    movwf   dmx_3_high

;    movlw   LOW(CHANNEL_ADDRESS_0)
;    movwf   dmx_0_low
;    movlw   HIGH(CHANNEL_ADDRESS_0);
;    movwf   dmx_0_high
;    movlw   LOW(CHANNEL_ADDRESS_1)
;    movwf   dmx_1_low
;    movlw   HIGH(CHANNEL_ADDRESS_1);
;    movwf   dmx_1_high
;    movlw   LOW(CHANNEL_ADDRESS_2)
;    movwf   dmx_2_low
;    movlw   HIGH(CHANNEL_ADDRESS_2);
;    movwf   dmx_2_high
;    movlw   LOW(CHANNEL_ADDRESS_3)
;    movwf   dmx_3_low
;    movlw   HIGH(CHANNEL_ADDRESS_3);
;    movwf   dmx_3_high

    ;  Return to the caller.
    return

;  DMX Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  Comparator Module
;
;  Comparator module provides zero crossing detection and handling.

	cblock
        delay
	endc

;  Routine - comparator_init : Initializes all the settings necessary to
;  generate an interrupt for the zero crossing circuitry.

comparator_init

    ;  Start by setting up the Fixed Voltage Reference Control Register.  This
    ;  should set a reference voltage of 2.048V used for the comparator.
    banksel FVRCON
    bsf     FVRCON,FVREN        ;  Enable the reference generator.
    bsf     FVRCON,CDAFVR1       ;  Set reference to 2.048V
    bcf     FVRCON,CDAFVR0
    bcf     FVRCON,ADFVR1       ;  Turn off the ADC reference.
    bcf     FVRCON,ADFVR0

    ;  Setup the two comparator control registers.
    banksel CM1CON0
    bcf     CM1CON0,C1ON        ;  Disable the comparator for a moment.
    bcf     CM1CON0,C1OE        ;  Output is only used internally.
    bcf     CM1CON0,C1POL       ;  Invert the output.  Interrupt on leading edge.
    bsf     CM1CON0,C1SP        ;  Use high-speed comparisons.
    bcf     CM1CON0,C1HYS       ;  No hysterisis needed.
    bcf     CM1CON0,C1SYNC      ;  Output is asynchronous.

    bsf     CM1CON1,C1INTP      ;  Interrupt on negative going edge.
    bcf     CM1CON1,C1INTN
    bsf     CM1CON1,C1PCH1      ;  Positive comparator input from Fixed Ref.
    bcf     CM1CON1,C1PCH0
    bcf     CM1CON1,C1NCH2      ;  Negative comparator input from ac signal.
    bsf     CM1CON1,C1NCH1
    bcf     CM2CON1,C1NCH0

    ;  Setup the input signal for analog input.
    banksel ANSELC
    bsf     ANSELC,ANSC2

    ;  Setup the debug port, PORTC, pin 6.
    banksel TRISC
    bcf     TRISC,TRISC6
    banksel PORTC
    bcf     PORTC,RC6

    ;  Turn on the comparator.
    banksel CM1CON0
    bsf     CM1CON0,C1ON        ;  Enable the comparator.

    ;  Enable the interupts.
    banksel PIE2
    bsf     PIE2,C1IE

    ;  Return to the caller.
    return

;  Routine - comparator_isr : This handles the zero crossing interrupt.

comparator_isr

    banksel PIR2
    btfss   PIR2,C1IF
    return

    bcf     PIR2,C1IF

    ;  Synchronize Timer 1 to AC line.

;    banksel delay
;    movlw   0x0F
;    movwf   delay
;comp_loop
;    decfsz  delay,F
;    goto    comp_loop

    banksel PORTC
    bsf     PORTC,RC6
    bcf     PORTC,RC6

    call    pwm_sync

    ;  Return to the caller.
    return

;  Comparator Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  UART Module
;
;  This module handles the reception of the DMX data.  It is state machine
;  driven.  Data for the specified channels is saved in the RAM.

;  Define variables for the timer 1 module.
	cblock
        uart_state      ;  State of the uart state machine.
        uart_count_low  ;  Tracks number of bytes read.
        uart_count_high
        uart_data       ;  Stores current channel data.
        dmx0
        dmx1
        dmx2
        dmx3
	endc

;  Define uart states.
UART_STATE_BREAK    equ     000H        ;  Looking for the gap between packets.
UART_STATE_START    equ     001H        ;  Looking for the start character.
UART_STATE_DATA     equ     002H        ;  Reading the DMX channel data.

;  Routine - uart_init : This sets up the UART to receive data from the DMX
;  controller.

uart_init

    ;  Setup the baud rate generator to 250K baud, the standard for DMX.
    banksel BAUDCON
    bcf     BAUDCON,WUE     ; Wake up disabled.
    bcf     BAUDCON,ABDEN   ; Disable autobaud.
    bcf     BAUDCON,BRG16   ;  See formula in documentation.
    bsf     TXSTA,BRGH
    clrf    SPBRGH
    movlw   0x04
    movwf   SPBRGL

    ;  Set receiver.
    bcf     RCSTA,RX9
    bsf     RCSTA,CREN      ; Enable reception.

    ;  Turn on the serial port.
    bsf     RCSTA,SPEN      ; Enables the serial ports.

    ;  Set the state to look for the start of a packet.
    banksel uart_state
    movlw   UART_STATE_BREAK
    movwf   uart_state

    ;  Setup a debug pin for watching the state machine.
    banksel TRISC
    bcf     TRISC,TRISC4
    banksel PORTC
    bcf     PORTC,RC4

    ;  Enable UART interrupts.
    banksel PIE1
;    bsf     PIE1,RCIE

    ;  Return to the caller.
    return;

;  Routine - uart_isr : Handles an interrupt from the uart.

uart_isr

    ;  See if the uart is interrupting.
    banksel PIR1
    btfss   PIR1,RCIF
    goto    uart_isr_return

    ;  Check for break state and handle.
    banksel uart_state
    movfw   uart_state
    xorlw   UART_STATE_BREAK
    btfss   STATUS,Z
    goto    uart_state_check_start

    ;  STATE_BREAK : This state is looking for a first framing error.  This
    ;  is a possible indication of the break signal at the start of each
    ;  packet.

uart_state_break

    ;  Get the status register and see if there is a framing error.
    banksel RCSTA
    btfss   RCSTA,FERR
    goto    uart_state_break_discard

    ;  We have a framing error so next state is start where we look for
    ;  a start code.
    banksel uart_state
    movlw   UART_STATE_START
    movwf   uart_state

    ;  Indicate that we have a framing error.
    banksel PORTC
    bsf     PORTC,4
    bcf     PORTC,4
    goto    uart_isr_return

    ;  If we received a good data byte, we need to discard it.  We do this
    ;  until we get a framing error.
uart_state_break_discard

    ;  Read the data byte and exit.
    movfw   RCREG
    goto    uart_isr_return

    ;  Check for the start state.
uart_state_check_start
    banksel uart_state
    movfw   uart_state
    xorlw   UART_STATE_START
    btfss   STATUS,Z
    goto    uart_state_check_data

    ;  STATE_START : This is the state that looks for the start byte.
    ;  Discard any framing errors.  When a byte is received without an error,
    ;  it must be a "00H" and if not, we go back to looking for framing errors.

uart_state_start

    ;  Check the status of the receiver.  If we have framing error, return.
    banksel RCREG
    btfss   RCSTA,FERR
    goto    uart_state_start_data

    ;  We have a framing error.  Read the data and return from the isr.
    movfw   RCREG
    goto    uart_isr_return

    ;  We must have a good data byte so see if it is the 00H start byte.
uart_state_start_data
    movfw   RCREG
    btfsc   STATUS,Z
    goto    uart_state_start_detected

    ;  We did not find the start byte so reset to the break state.
    banksel uart_state
    movlw   UART_STATE_BREAK
    movwf   uart_state
    goto    uart_isr_return

    ;  We found the start byte.  Now it's time to read all 512 bytes of data.
uart_state_start_detected

    banksel uart_state
    clrf    uart_count_low
    clrf    uart_count_high

    movlw   UART_STATE_DATA
    movwf   uart_state

    ;  Indicate that we have found the start code.
    banksel PORTC
    bsf     PORTC,4
    bcf     PORTC,4
    bsf     PORTC,4
    bcf     PORTC,4

    goto    uart_isr_return

    ;  Looking for a start character, skipping framing errors.  If first
    ;  data received without error is not 00H, then go back to state break.
uart_state_check_data
    banksel uart_state
    movfw   uart_state
    xorlw   UART_STATE_DATA
    btfss   STATUS,Z
    goto    uart_state_error

    ;  STATE_DATA : This state reads 512 bytes of data and then starts looking
    ;  for the next DMX packet.
uart_state_data

    ;  Check for a framing error.  If we have one, reset to the break state.
    banksel RCSTA
    btfsc   RCSTA,FERR
    goto    uart_state_data_frame

    ;  We have a byte of data so read it and see if there is any more to read.
    movfw   RCREG
    banksel uart_data
;    bcf     STATUS,C
    movwf   uart_data
;    rrf     uart_data
;    rrf     uart_data

    ;  See if we have a channel of data.
    banksel uart_data
    movfw   uart_count_low
    xorwf   dmx_0_low,W
    btfss   STATUS,Z
    goto    uart_state_data_dmx1

    movfw   uart_count_high
    xorwf   dmx_0_high,W
    btfss   STATUS,Z
    goto    uart_state_data_dmx1

    ;  Move the data to the dmx0 slot.
    movfw   uart_data
;    iorlw   0C0H
    movwf   dmx0
    comf    dmx0

uart_state_data_dmx1

    ;  See if we have channel 1 data.
    movfw   uart_count_low
    xorwf   dmx_1_low,W
    btfss   STATUS,Z
    goto    uart_state_data_dmx2

    movfw   uart_count_high
    xorwf   dmx_1_high,W
    btfss   STATUS,Z
    goto    uart_state_data_dmx2

    ;  Move the data to the dmx0 slot.
    movfw   uart_data
;    iorlw   0C0H
    movwf   dmx1
    comf    dmx1

uart_state_data_dmx2

    ;  See if we have channel2 data.
    movfw   uart_count_low
    xorwf   dmx_2_low,W
    btfss   STATUS,Z
    goto    uart_state_data_dmx3

    movfw   uart_count_high
    xorwf   dmx_2_high,W
    btfss   STATUS,Z
    goto    uart_state_data_dmx3

    ;  Move the data to the dmx0 slot.
    movfw   uart_data
;    iorlw   0C0H
    movwf   dmx2
    comf    dmx2

uart_state_data_dmx3

    ;  See if we have channel 3 data.
    movfw   uart_count_low
    xorwf   dmx_3_low,W
    btfss   STATUS,Z
    goto    uart_state_data_inc

    movfw   uart_count_high
    xorwf   dmx_3_high,W
    btfss   STATUS,Z
    goto    uart_state_data_inc

    ;  Move the data to the dmx0 slot.
    movfw   uart_data
;    iorlw   0C0H
    movwf   dmx3
    comf    dmx3


uart_state_data_inc

    banksel uart_state
    incfsz  uart_count_low
    goto    uart_state_data_noinc
    incf    uart_count_high

uart_state_data_noinc

    movfw   uart_count_high
    xorlw   002H
    btfss   STATUS,Z
    goto    uart_isr_return

    ;  We have read 512 bytes.
    movlw   UART_STATE_BREAK
    movwf   uart_state

    ;  Indicate that we have found all 512 channels of data.
    banksel PORTC
    bsf     PORTC,4
    bcf     PORTC,4
    bsf     PORTC,4
    bcf     PORTC,4
    bsf     PORTC,4
    bcf     PORTC,4

    ;  Clear the watchdog timer.  If we don't get a packet, we reset.
    clrwdt

    goto    uart_isr_return

    ;  A framing error was detected.  Reset the state machine.
uart_state_data_frame
    banksel uart_state
    movlw   UART_STATE_BREAK
    movwf   uart_state
    goto    uart_isr_return

    ;  A state error was detected.  Not good.
uart_state_error
    banksel PORTC
    bsf     PORTC,4
    bcf     PORTC,4
    goto    uart_state_error

uart_isr_return
    ;  Return to the caller.
    return

;  Routine - uart_isr_set_data_state : This is a routine that determines the
;  next data state based on whether we should be grabbing channel data or not.
;  It just


;  UART Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  Timer 1 Module
;
;  This module handles the timing for the pwm of the channels.  It's resolution
;  is set for 64 levels based on a 1/120 second period.  The timer will count
;  until reset by the zero crossing detection.

;  Define variables for the timer 1 module.
	cblock
        level               ;  Holds the current level 0 to 63.
;		mstimerh	;  High byte of millisceond timer.
;        mstimertick ;  Bit 0 set if ms tick code should run.
	endc

;  The reload value is 65536-2604 = 0F5D4H.  This will cause an interrupt on
;  overflow every 1/120/64 seconds ... 130.2uS.  Based on 20Mhz oscillator.
timer_reload_low	equ		0efH        ; 0F4 for 64 levels
timer_reload_high	equ		0fDH        ; 0F5 for 64 levels

;  Routine - timer_init : Set up the timer to give us an interrupt every 1ms.
;  The isr code for this must be VERY lightweight.

timer_init

    banksel T1CON
    bsf     T1CON,TMR1CS0    ; Select internal clock.  System Clock.
    bcf     T1CON,TMR1CS1
    bsf     T1CON,NOT_T1SYNC  ; Not required for internal clock.
    bcf     T1CON,T1CKPS0   ; Prescaler 1:1
    bcf     T1CON,T1CKPS1

    banksel T1GCON
    bcf     T1GCON,TMR1GE

 	clrf	level
;	clrf	mstimerh

    movlw   timer_reload_high
    movwf   TMR1H
    movlw   timer_reload_low
    movwf   TMR1L

	bcf		PIR1,TMR1IF				;  Clear any pending interrupt.
	banksel PIE1
	bsf		PIE1,TMR1IE				;  Enable the interrupts
	banksel T1CON
	bsf		T1CON,TMR1ON			;  Start the timer.

    ;  Setup the debug port, PORTC, pin 7.
    banksel TRISC
    bcf     TRISC,TRISC7
    banksel PORTC
    bcf     PORTC,RC7

	return


;  Routine - timer_isr : This is called from the interrupt routine if a timer1
;  interrupt is detected.

timer_isr

	;  Check for a timer interrupt.
	banksel PIR1
	btfss	PIR1,TMR1IF				;  Check for timer 1 interrupt.
	goto	timer_isr_return		;  If not, return.

    banksel T1CON
	bcf		T1CON,TMR1ON			;  Turn off the timer.

    ; Reload the timer for the next cycle.
    banksel TMR1L
	movlw	timer_reload_low		;  Get low timer value.
	movwf	TMR1L					;  Set low byte.
	movlw	timer_reload_high		;  Get high timer value.
	movwf	TMR1H					;  Set high byte.

    banksel PIR1
	bcf		PIR1,TMR1IF				;  Clear any pending interrupt.
    banksel T1CON
	bsf		T1CON,TMR1ON			;  Turn timer back on.

    ; pulse the timer debug pin.
    banksel PORTC
    bsf     PORTC,RC7
    bcf     PORTC,RC7

    ; Turn off all the triac drivers.
    call    pwm_off

    ; Increment the level in the cycle.  If we hit 255, then hold the
    ; counter there and disable setting any drivers.
    banksel level
    incf    level
    btfss   STATUS,Z
    goto    timer_isr_setpwm

    decf    level
    goto    timer_isr_return

    ; Set the triac drivers for the new level.
timer_isr_setpwm
    call    pwm_set_channels                ;  Set the channel outputs

timer_isr_return
	return

;  Routine - timer_zero : This resets the timer so that it's synchronized
;  with the AC line.
timer_zero
    banksel T1CON
    bcf		T1CON,TMR1ON			;  Turn off the timer.

	movlw	timer_reload_low		;  Get low timer value.
	movwf	TMR1L					;  Set low byte.
	movlw	timer_reload_high		;  Get high timer value.
	movwf	TMR1H					;  Set high byte.

    banksel PIR1
	bcf		PIR1,TMR1IF				;  Clear any pending interrupt.
    banksel T1CON
	bsf		T1CON,TMR1ON			;  Turn timer back on.

    banksel level                   ;  Reset the level back to 0.
    clrf    level

    return

;  Timer Module : END
;
; ------------------------------------------------------------------------------


;  Routine - pwm_init : This initializes the pulse width modulation module.

pwm_init

    ;  Set the PWM bits to be outputs.
    banksel TRISA
    bcf     TRISC,TRISC5
    bcf     TRISC,TRISC3
    bcf     TRISA,TRISA2
    bcf     TRISC,TRISC1

    ;  Set all the outputs to zero.
    call    pwm_off

    ;  Return to the caller.
    return

;  Routine - pwm_clear : Turns off all 4 channels.
pwm_off

    ;  Set all the outputs to zero.
    banksel LATC
    bcf     LATC,RC5
    bcf     LATC,RC3
    bcf     LATA,RA2
    bcf     LATC,RC1

    ;  return to the caller.
    return

;  Routine - pwm_ac_sync : This should be called on each zero crossing.
pwm_sync

    call    timer_zero      ;  Make sure the timer gets zeroed.
    call    pwm_off         ;  Turn off all channels.
    call    pwm_set_channels    ;  Set channels since level has changed.
    return                  ;  Return to the caller.

;  Routine - pwm_set_channels : This sets the channel bits based on the
;  current level.

pwm_set_channels

    banksel dmx0

;    movfw   level
;    sublw   0x30
;    btfsc   STATUS,C
;    return
;
;    movfw   level
;    sublw   0x0f
;    btfss   STATUS,C
;    return

;    bcf     PORTC,RC5
;    movfw   level
;    xorlw   0x20
;    btfss   STATUS,Z
;    return

;  If the level is 255, don't bother turning on the triac.  This will allow
;  for an always off condition

    incfsz  level,W
    goto    pwm_set_channels_ok
    return

pwm_set_channels_ok
    movfw   dmx0        ; Get current dmx set level.
    btfss   STATUS,Z    ; If it's 0, then we are full on so always set the trigger.
    xorwf   level,W     ; If not 0, check to see if we are at the expected level.
    btfsc   STATUS,Z    ; If at level or at 0, then...
    bsf     PORTC,RC5   ;    set the trigger.

    movfw   dmx1
    btfss   STATUS,Z
    xorwf   level,W
    btfsc   STATUS,Z
    bsf     PORTC,RC3

    movfw   dmx2
    btfss   STATUS,Z
    xorwf   level,W
    btfsc   STATUS,Z
    bsf     PORTA,RA2

    movfw   dmx3
    btfss   STATUS,Z
    xorwf   level,W
    btfsc   STATUS,Z
    bsf     PORTC,RC1

    return



; ------------------------------------------------------------------------------
;
;  Start up the main program.  Call the initialization routines for each of the
;  modules.

boot

    ;  Initialize the system components.
    call    system_init
    call    dmx_init
    call    timer_init
    call    uart_init
    call    comparator_init
    call    pwm_init

    ;  Start the system.
    call    system_start

    ;  For testing set up dmx channels.
    movlw   0xc0
    movwf   dmx0
    movlw   0x80
    movwf   dmx1
    movlw   0x40
    movwf   dmx2
    movlw   0x00
    movwf   dmx3


    ;  Start of user code.
main

    call    uart_isr
    goto    main
main2
    goto    main2
    banksel TRISC
 ;   bcf     TRISC,4
    bcf     TRISC,5



mainlp
    banksel PORTC
    bsf     PORTC,4
    nop
    nop
    bcf     PORTC,4
;    bcf     PORTC,5

;    ;  Setup the serial port.
;    banksel BAUDCON
;    bcf     BAUDCON,SCKP    ;  Non inverting tx for async.
;    bcf     BAUDCON,WUE     ; Wake up disabled.
;    bcf     BAUDCON,ABDEN   ; Disable autobaud.

    ;  Setup for transmission.
    banksel BAUDCON
    bcf     TXSTA,CSRC      ; Clock Source Select : Don't care for async.
    bcf     TXSTA,TX9       ; Selects 8-bit transmissions.
    bcf     TXSTA,SYNC      ; Selects Asynchronous mode.


    ;  Setup the Baud Rate Generator for 250K baud.
    banksel BAUDCON
;    bcf     BAUDCON,BRG16
;    bsf     TXSTA,BRGH
;    clrf    SPBRGH
;    movlw   0x04
;    movwf   SPBRGL

;    bsf     RCSTA,SPEN      ; Enables the serial ports.

    ;  Get ready to send data.
    bsf     TXSTA,TXEN      ;  Enable transmission.

;    ;  Set receiver.
;    bcf     RCSTA,RX9
;    bsf     RCSTA,CREN      ; Enable reception.

txloop

    ;  See if we are ready to send data.
    btfss   TXSTA,TRMT
    goto    txloop

    ;  Write data to the tx reg.
    movlw   0x55
    movwf   TXREG

    goto    txloop

; End of main program.
;
; ------------------------------------------------------------------------------

	end










