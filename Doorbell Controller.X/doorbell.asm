
; Program - dmxpack.asm
;
; Written by Michael Bengtson  21-Oct-2013
;
; This is the code for the controller that handles the doorbells, the Seeburg
; jukebox remote and any alerts.  The system consists of an Ethernet to RS-232
; converter, power supply for the system, power supply for the jukebox remote,
; two MP3 player boards and the processor with associated electronics.
; See the schematic for details.
;
; Operation - The doorbells are connected into the system via RJ-45 connectors.
; Each doorbell is supplied with +5.0v for the led ring in the button.  The
; buttons are each tied to an input on the processor.  The button press can be
; detected and then an RS-232 command sent to the Mac Mini via the Ethernet to
; RS-232 converter.  The Mac Mini then determines what MP3 to play and sends
; a command back to the controller that will initiate an MP3 on the 'alert'
; MP3 player. The Seeburg has a single input line that provides a serial
; signal indicating the selection.  Each transition on this line is timestamped
; and sent to the Mac Mini via the RS-232/Ethernet connection.  The serial
; signal is decoded in the Mac.  The Mac then determines what has been selected
; and sends a command back to the controller which gets routed to the 'Seeburg'
; MP3 player. The Mac can also use other information and send alerts to the
; 'alert' MP3 player.  These could be weather information, clock chimes, etc.

; Here is the protocol that is used between the Mac Mini and the Controller.
;
;   Doorbell Alerts In:
;
;       01:00:TSL:TSH   :   Front Doorbell Pressed
;       01:01:TSL:TSH   :   Front Doorbell Released
;       01:02:TSL:TSH   :   Side Doorbell Pressed
;       01:03:TSL:TSH   :   Side Doorbell Released
;       01:04:TSL:TSH   :   Guest Doorbell Pressed
;       01:05:TSL:TSH   :   Guest Doorbell Released
;
;   Seeburg Selection In:
;
;       01:06:TSL:TSH   :   Seeburg Contact Closed
;       01:07:TSL:TSH   :   Seeburg Contact Open
;
;   MP3 Commands:
;
;       02:01:nn        ;   Plays Alert Track nn (0-255)
;       02:02:nn        :   Plays Seeburg Track nn (0-255)
;
;   Seeburg tracks are letter * 10 + number.
;       A0 = Track 0
;       A1 = Track 1
;       B0 = Track 10
;       V0 = Track 190
;       V9 = Track 199
;
;   Doorbell tracks are as follows:
;       Front = Track 0
;       Side = Track 1
;       Guest = Track 2

;
; Processor is PIC16F1508.  
; Pin out for the chip is as follow:
;
;       Pin 01 : Vdd : +5.0V
;       Pin 02 : RA5 : Clock In  20Mhz Oscillator
;       Pin 03 : RA4 : NC
;       Pin 04 : RA3 : ICSP - Program
;       Pin 05 : RC5 : NC
;       Pin 06 : RC4 : DEBUG - UART
;       Pin 07 : RC3 : Seeburg Select
;       Pin 08 : RC6 : DEBUG - BUTTONS
;       Pin 09 : RC7 : DEBUG - TIMER
;       Pin 10 : RB7 : TX - To Ethernet Converter
;       Pin 11 : RB6 : Alert MP3 TX
;       Pin 12 : RB5 : RX - From Ethernet Converter
;       Pin 13 : RB4 : Seeburg MP3 TX
;       Pin 14 : RC2 : Doorbell Guest
;       Pin 15 : RC1 : Doorbell Side
;       Pin 16 : RC0 : Doorbell Front
;       Pin 17 : RA2 : DEBUG - BAT
;       Pin 18 : RA1 : ICSP - Clock
;       Pin 19 : RA0 : ICSP - Data
;       Pin 20 : Vss : 0.0V - GND

; 21-Oct-2013 : Starting to write the code using the DMX controller code as
;               base.

; NEXT STEPS :
;
;   * Check configuration bits.
;   * Get programming working and check for timer pulses with oscilloscope.
;   * Design Mac / Controller communication.
;   * Get PIC UART Sending At 1200 Baud
;   * Decoding button presses.
;   * Write UART Send Routine With Circular Queue 16 Bytes Is Good
;   * Write Java code to receive from Ethernet/RS-232 converter.
;   * Get doorbell buttons to send commands to Mac.
;   * Write Java state machines to decode in bound data.
;   * Get PIC UART Receiving At 2400 Baud
;   * Write 2400 Bit Banged UART Code ... TX Only
;   Write state machine to take commands and send to MP3 channels.
;   Clean up RAM storage assignments.
;   ...

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
;    cblock  0x70
;        dmx_0_low
;        dmx_0_high
;        dmx_1_low
;        dmx_1_high
;        dmx_2_low
;        dmx_2_high
;        dmx_3_low
;        dmx_3_high
;    endc

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
;  UART Module
;
;  This module handles the reception of the DMX data.  It is state machine
;  driven.  Data for the specified channels is saved in the RAM.

UART_TX_QUEUE_SIZE  equ     010H        ;  TX queue size is 16 bytes.
UART_RX_QUEUE_SIZE  equ     004H        ;  TX queue size is 4 bytes.

;  Define variables for the timer 1 module.
	cblock
        uart_tx_head
        uart_tx_tail
        uart_tx_temp
        uart_rx_head
        uart_rx_tail
        uart_rx_temp
	endc

    cblock  0x60
        uart_rx_queue: UART_RX_QUEUE_SIZE
    endc
;  Define variables for the timer 1 module.
	cblock 0x70
;        uart_state      ;  State of the uart state machine.
;        uart_count_low  ;  Tracks number of bytes read.
;        uart_count_high
;        uart_data       ;  Stores current channel data.
;        dmx0
;        dmx1
;        dmx2
;        dmx3
        uart_tx_queue: UART_TX_QUEUE_SIZE
;        uart_tx_head
;        uart_tx_tail
;        uart_tx_temp
	endc
    cblock 0x26
    endc

;  Define uart states.
UART_STATE_BREAK    equ     000H        ;  Looking for the gap between packets.
UART_STATE_START    equ     001H        ;  Looking for the start character.
UART_STATE_DATA     equ     002H        ;  Reading the DMX channel data.

;  Routine - uart_init : This sets up the UART to receive data from the DMX
;  controller.

uart_init

    ;  Setup the baud rate generator to 9600 baud.
    banksel BAUDCON
    bcf     BAUDCON,WUE     ; Wake up disabled.
    bcf     BAUDCON,ABDEN   ; Disable autobaud.
    bsf     BAUDCON,BRG16   ;  See formula in documentation.
    bsf     TXSTA,BRGH

    banksel uart_tx_head
    clrf    uart_tx_head
    clrf    uart_tx_tail
    clrf    uart_rx_head
    clrf    uart_rx_tail

    ;  520 is count for baud rate generator to run at 9600 baud.  This is
    ;  0x0208 in hex.
    banksel BAUDCON
    movlw   0x02
    movwf   SPBRGH
    movlw   0x08
    movwf   SPBRGL

    ;  Turn on the transmitter.
    banksel TXSTA
    bcf     TXSTA,SYNC      ; Set asynchronous mode.

    banksel RCSTA
    bsf     RCSTA,SPEN      ; Enable the UART.

    banksel TXSTA
    bsf     TXSTA,TXEN      ; Enable the transmitter.

    ;  Set receiver.
    bcf     RCSTA,RX9
    bsf     RCSTA,CREN      ; Enable reception.

    ;  Setup the debug port, PORTC, pin 4.
    banksel TRISC
    bcf     TRISC,TRISC4
    banksel PORTC
    bcf     PORTC,RC4

    ;  Return to the caller.
    return;

;  Routine - uart_rx_tick : Looks for a character received and places it
;  in the queue.

uart_rx_tick

    banksel PIR1
    btfss   PIR1,RCIF
    goto    uart_rx_tick_return

    ; We have a character.  Put it in the queue if there is room.
    banksel uart_rx_head

    incf    uart_rx_tail,W      ; See if there is room.
    andlw   UART_RX_QUEUE_SIZE -1
    xorwf   uart_rx_head,W      ; If tail+1 == head, then no room.
    btfsc   STATUS,Z
    goto    uart_rx_error       ; No room so don't add it.

    ; Move character into the queue.
    banksel RCREG
    movfw   RCREG
    banksel uart_rx_temp
    movwf   uart_rx_temp

    movlw   uart_rx_queue
    movwf   FSR0L
    clrf    FSR0H
    movfw   uart_rx_tail
    addwf   FSR0L
    movfw   uart_rx_temp
    movwf   INDF0
    incf    uart_rx_tail
    movlw   UART_RX_QUEUE_SIZE - 1
    andwf   uart_rx_tail
;    goto    uart_rx_tick_return

    ; Pulse the UART debug pin.
    banksel PORTC
    bsf     PORTC,RC4
    bcf     PORTC,RC4
    goto    uart_rx_tick_return

uart_rx_error
    banksel RCREG
    movfw   RCREG
    banksel PORTC
    bsf     PORTC,RC4
    bcf     PORTC,RC4
    bsf     PORTC,RC4
    bcf     PORTC,RC4

uart_rx_tick_return
    return

;  Routine - uart_tx_tick : Transmits characters from the queue.

uart_tx_tick

    ; If no characters to send, then return.
    banksel uart_tx_head
    movfw   uart_tx_head
    xorwf   uart_tx_tail,W
    btfsc   STATUS,Z
    goto    uart_tx_tick_return

    ; If transmitter is not ready, then return.
    banksel PIR1
    btfss   PIR1,TXIF
    goto    uart_tx_tick_return

    ; Send next character then remove it from the queue.
    banksel uart_tx_queue
    movlw   uart_tx_queue
    movwf   FSR0L
    clrf    FSR0H
    movfw   uart_tx_head
    addwf   FSR0L
    movfw   INDF0

    banksel TXREG
;    movlw   0x55
    movwf   TXREG

    banksel uart_tx_head
    incf    uart_tx_head
    movlw   0x0f
    andwf   uart_tx_head

uart_tx_tick_return
    return


;  Routine - uart_tx_send_byte : This takes the contents of the W register
;  and appends it to the tail of the queue.  Character is dropped if there
;  is not any room.

uart_tx_send_byte

    banksel uart_tx_temp
    movwf   uart_tx_temp        ; Save the character to put into the queue.

    incf    uart_tx_tail,W      ; See if there is room.
    andlw   0x0f
    xorwf   uart_tx_head,W      ; If tail+1 == head, then no room.
    btfsc   STATUS,Z
    goto    uart_tx_send_error                      ; No room so don't add it.

    movlw   uart_tx_queue
    movwf   FSR0L
    clrf    FSR0H
    movfw   uart_tx_tail
    addwf   FSR0L
    movfw   uart_tx_temp
    movwf   INDF0
    incf    uart_tx_tail
    movlw   0x0f
    andwf   uart_tx_tail
    return

uart_tx_send_error
    clrwdt
    goto    uart_tx_send_error


;  UART Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  BAT Module
;
;  The BAT (Banged Asynchronous Transmitter) only transmits characters using
;  io on the PIC.  There are two tx lines that can be selected from, RB6 is
;  transmit to the Alert MP3 and RB7 is transmit to the Seeburg MP3.
;
;  Transmission is done at 2400 baud for MP3 boards.  This is interrupt driven
;  by the timer tick which is running at the frequency for 2400 baud.

;  Define variables for the bat module.

BAT_TX_QUEUE_SIZE  equ     004H        ;  TX queue size is 4 bytes.

	cblock
        bat_bit_count
        bat_data
        bat_port
        bat_tx_head
        bat_tx_tail
        bat_tx_temp
        bat_tx_queue: BAT_TX_QUEUE_SIZE
	endc

;  Routine - bat_init : Sets up the ports for transmitting.  Initializes
;  variables used to transmit.

bat_init

    ;  Turn the tx lines on as outputs and set them.
    banksel PORTB
    bsf     PORTB,RB4
    bsf     PORTB,RB6
    banksel TRISB
    bcf     TRISB,TRISB4
    bcf     TRISB,TRISB6

    ;  Reset variables.
    banksel bat_bit_count
    clrf    bat_bit_count

    ;  Setup the debug port, PORTA, pin 2.
    banksel TRISA
    bcf     TRISA,TRISA2
    banksel PORTA
    bcf     PORTA,RC2

    return

;  Routine - bat_set_port : The W register should contain 0x40 to set the
;  port for the Alert MP3 player and 0x80 to set the port for the Seeburg MP3
;  player.

bat_set_port

    banksel bat_port
    movwf   bat_port
    return

;  Routine - bat_ready : Returns Z set if the bat interface is ready to send
;  data.

bat_ready

    banksel bat_bit_count
    movfw   bat_bit_count
    return

;  Routine - bat_tick : Handles the bit sending on the specified port.

bat_tick

    ; If the bit count is 0, see if there is another byte to send.
    banksel bat_bit_count
    movfw   bat_bit_count
    btfss   STATUS,Z
    goto    bat_tick_send_start

    ; If no characters in the queue, return.
    banksel bat_tx_head
    movfw   bat_tx_head
    xorwf   bat_tx_tail,W
    btfsc   STATUS,Z
    return

    ; If there is another byte, then start it up.
    ; Send next character then remove it from the queue.
    banksel bat_tx_queue
    movlw   bat_tx_queue
    movwf   FSR0L
    clrf    FSR0H
    movfw   bat_tx_head
    addwf   FSR0L
    movfw   INDF0

    call    bat_send

    banksel bat_tx_queue
    incf    bat_tx_head
    movlw   BAT_TX_QUEUE_SIZE - 1
    andwf   bat_tx_head
    return

;    ; Turn off the lines.
;    banksel PORTB
;    bsf     PORTB,RC4
;    bsf     PORTB,RC6
;    return

;bat_tick_send_startx
;
;    movlw   1
;    xorwf   bat_bit_count,W
;    btfss   STATUS,Z
;    goto    bat_tick_send_start
;
;    ; Set the stop bit.
;    banksel PORTB
;    bsf     STATUS,C
;    goto    bat_tick_set

bat_tick_send_start

    ; If the bit count is 10, then do the start bit.
    movlw   0x0a
    xorwf   bat_bit_count,W
    btfss   STATUS,Z
    goto    bat_tick_send_stop

    ; Set the start bit.
    banksel PORTA
    bsf     PORTA,RA2
    bcf     PORTA,RA2

    bcf     STATUS,C
    goto    bat_tick_set

bat_tick_send_stop

    ; If the bit count is 1, then do the stop bit.
    movlw   1
    xorwf   bat_bit_count,W
    btfss   STATUS,Z
    goto    bat_tick_send_data

    ; Set the stop bit.
    banksel PORTB
    bsf     STATUS,C
    goto    bat_tick_set

bat_tick_send_data

    ; Get the next bit to send.
    banksel bat_data
    rrf     bat_data
    goto    bat_tick_set

bat_tick_set

    ; Based on the value of the carry, either clear or set the proper port.
    banksel PORTA
    bsf     PORTA,RA2
    bcf     PORTA,RA2

    banksel bat_bit_count
    decf    bat_bit_count       ; Always decrement the counter.

    btfss   STATUS,C
    goto    bat_send_1
    goto    bat_send_0

bat_send_0
    banksel bat_port
    btfss   bat_port,4
    goto    bat_send_0_alert
    goto    bat_send_0_seeburg

bat_send_0_alert
    banksel PORTB
    bsf     PORTB,6
    return

bat_send_0_seeburg
    banksel PORTB
    bsf     PORTB,4
    return

bat_send_1
    banksel bat_port
    btfss   bat_port,4
    goto    bat_send_1_alert
    goto    bat_send_1_seeburg

bat_send_1_alert
    banksel PORTB
    bcf     PORTB,6
    return

bat_send_1_seeburg
    banksel PORTB
    bcf     PORTB,4
    return

;  Routine - bat_send : Sends the byte in the W register.  Actually, it sets
;  the data into the driver and sets the bit counter to 10.  1 stop, 1 start
;  and 8 data bits.  All the sending is done with the tick routine.

bat_send

    banksel bat_data
    movwf   bat_data
    movlw   0x0a
    movwf   bat_bit_count
    return

;  Routine - bat_tx_send_byte : This takes the contents of the W register
;  and appends it to the tail of the queue.  Character is dropped if there
;  is not any room.

bat_tx_send_byte

    banksel bat_tx_temp
    movwf   bat_tx_temp        ; Save the character to put into the queue.

    incf    bat_tx_tail,W      ; See if there is room.
    andlw   BAT_TX_QUEUE_SIZE - 1
    xorwf   bat_tx_head,W      ; If tail+1 == head, then no room.
    btfsc   STATUS,Z
    goto    bat_tx_send_error                      ; No room so don't add it.

    movlw   bat_tx_queue
    movwf   FSR0L
    clrf    FSR0H
    movfw   bat_tx_tail
    addwf   FSR0L
    movfw   bat_tx_temp
    movwf   INDF0
    incf    bat_tx_tail
    movlw   BAT_TX_QUEUE_SIZE - 1
    andwf   bat_tx_tail
    return

bat_tx_send_error
    clrwdt
    goto    bat_tx_send_error


;  BAT Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  Timer 1 Module
;
;  The timer 1 module provides the main timing for the controller.  All RS-232
;  communication is setup for 12400 baud.  This is slow but the TX lines to the
;  MP3 player are 'bit banged' so the lower baud rate is easier to manage.
;  The messages are short and delay for a three byte command would be 12.5 ms,
;  which is plenty fast for this application.

;  Define variables for the timer 1 module.
	cblock
        timerlo     ;  Low byte of the timer counter.
		timerhi     ;  High byte of the timer counter.
        timerflags  ;  Flags for use in timer module.
	endc

;  Timer value calculated as 65536-(20,000,000/2400) = 57202 DF72 hex.

timer_reload_low	equ		072H
timer_reload_high	equ		0DFH

timer_flag_tick     equ     0           ; Bit 0 is set for timer tick code.

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

 	clrf	timerlo         ; Clear the timer counter.
	clrf	timerhi

    movlw   timer_reload_high       ;  Reload the timer for next value.
    movwf   TMR1H
    movlw   timer_reload_low
    movwf   TMR1L

	bcf		PIR1,TMR1IF				;  Clear any pending interrupt.
	banksel PIE1
	bsf		PIE1,TMR1IE				;  Enable the interrupts
	banksel T1CON
	bsf		T1CON,TMR1ON			;  Start the timer.

    ;  Clear timer tick code flag waiting for first full tick.
    banksel timerflags
    bcf     timerflags,timer_flag_tick

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

    ; Increment the timer counter.
    incf    timerlo
    btfsc   STATUS,Z
    incf    timerhi

    ; Set the timer tick flag.
    banksel timerflags
    bsf     timerflags,timer_flag_tick

    ; Call routines that need to run ON THE TICK.
    call    bat_tick

timer_isr_return
	return

;  Timer Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  Button Module
;
;  The button module watches for any changes on the RC0-3 lines and sends
;  change information to the UART and back to the Mac.

DOORBELL_FRONT_MASK  equ 1<<RC0
DOORBELL_SIDE_MASK   equ 1<<RC1
DOORBELL_GUEST_MASK  equ 1<<RC2
SEEBURG_SELECT_MASK  equ 1<<RC3

;  Define variables for the button module.
	cblock
        button_state    ;  Last state of the buttons.
        button_current  ;  Current button state read.
        button_counter  ;  Loop counter for getting presses.
        button_shift    ;  Place to shift the changed flags.
        button_temp     ;  Temp storage.
	endc

;  Routine - button_init : Sets up hardware and software to read the inputs
;  from the doorbell buttons and the seeburg select signal.

button_init

    ; Set up the lines for inputs.
    banksel TRISC
    movlw   DOORBELL_FRONT_MASK | DOORBELL_SIDE_MASK | DOORBELL_GUEST_MASK | SEEBURG_SELECT_MASK
    movwf   TRISC

    ; Set the initial state for the buttons.
    banksel button_state
    movlw   0x0F            ;  Assume no buttons pressed on power up.
    movwf   button_state

    ;  Setup the debug port, PORTC, pin 6.
    banksel TRISC
    bcf     TRISC,TRISC6
    banksel PORTC
    bcf     PORTC,RC6

    ; That's it.  Return
    return

;  Routine - button_tick : Checks to see if there are any changes on the
;  input lines.  If so, a message is generate to send to the Mac.  This should
;  only be called on the 1ms timer tick.

button_tick

    ; Read the inputs then compare to the previous state.
    banksel PORTC
    movfw   PORTC
    andlw   0x0F
    movwf   button_current
    xorwf   button_state,W
    btfsc   STATUS,Z
    goto    button_tick_return
    movwf   button_shift

    ; pulse the button debug pin.
    banksel PORTC
    bsf     PORTC,RC6
    bcf     PORTC,RC6
    bsf     PORTC,RC6
    bcf     PORTC,RC6

    banksel button_current
    movfw   button_current
    movwf   button_state

    ; W has lower 4 bits set that have a change.  Get the bit numbers that
    ; are set.
    banksel button_counter
    clrf    button_counter
button_tick_loop
    rrf     button_shift
    btfss   STATUS,C
    goto    button_tick_loop_inc

    ; This bit indicates a change.  Create the second byte of message by
    ; multiplying the count by two and setting last bit to pressed or released.
    rrf     button_current
    rlf     button_counter

    ; Here we create the packet to send to the host controller.
    movlw   0x01
    call    uart_tx_send_byte
    movfw   button_counter
    call    uart_tx_send_byte
    movfw   timerlo
    call    uart_tx_send_byte
    movfw   timerhi
    call    uart_tx_send_byte

    ; pulse the button debug pin.
    banksel button_counter
    movfw   button_counter
    xorlw   0x06
    banksel PORTC
    btfss   STATUS,Z
    goto    debugset

    bcf     PORTC,RC6
    goto    debug

debugset
    bsf     PORTC,RC6

debug

    ; Undo button_state and counter.
    banksel button_counter
    rrf     button_counter
    rlf     button_current

button_tick_loop_inc
    rrf     button_current
    movlw   0x04
    xorwf   button_counter,W
    btfsc   STATUS,Z
    goto    button_tick_return

    incf    button_counter
    goto    button_tick_loop

button_tick_return
;    ; Save current button state.
;    movfw   button_current
;    movwf   button_state
    return

;  Button Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  Command Module
;
;  This module handles commands from the host processor.
;  These are commands that will be sent to the MP3 boards.

;  Routine - command_init : This initializes the state machine that is used
;  to process the commands.

command_init
    return

;  Routine - command_tick : This runs the state machine that routes commands
;  to the MP3 boards.

command_tick

    ; If there are not any command bytes, then return.
    banksel uart_rx_head
    movfw   uart_rx_head
    xorwf   uart_rx_tail,W
    btfsc   STATUS,Z
    goto    command_tick_return

    ; If there are not three bytes, then we need to wait.
;!! What to do if we never hit three characters!  Need to reset after a time
;out period.
    movfw   uart_rx_head
    addlw   3
    andlw   UART_RX_QUEUE_SIZE-1
    xorwf   uart_rx_tail,W
    btfss   STATUS,Z
    goto    command_tick_return

    ; If we get here, then we need to process the command.
    clrf    uart_rx_head
    clrf    uart_rx_tail

    ; See what mp3 board to send this to.
    movfw   uart_rx_queue+1
    xorlw   1
    btfss   STATUS,Z
    goto    command_seeburg_mp3

command_alert_mp3
    movlw   0x40
    call    bat_set_port
    movlw   0x74            ; 't'
    call    bat_tx_send_byte
    movfw   uart_rx_queue+2
    call    bat_tx_send_byte
    goto    command_tick_return

command_seeburg_mp3
    movlw   0x10
    call    bat_set_port
    movlw   0x74            ; 't'
    call    bat_tx_send_byte
    movfw   uart_rx_queue+2
    call    bat_tx_send_byte
    goto    command_tick_return

command_tick_return
    return

;  Command Module : END
;
; ------------------------------------------------------------------------------


; ------------------------------------------------------------------------------
;
;  Start up the main program.  Call the initialization routines for each of the
;  modules.

boot

    ;  Initialize the system components.
    call    system_init
    call    timer_init
    call    uart_init
    call    button_init
    call    bat_init
    call    command_init

    ;  Start the system.
    call    system_start

    ;  Start of user code.
main

    ; If we are running here, we should hit the watch dog timer.  Everything
    ; is probably fine.
    clrwdt

    ;  Run timer tick code if tick flag is set.
    banksel timerflags
    btfss   timerflags,timer_flag_tick
    goto    skip_tick_code

    ;  Run tick code here.

    ;  Only run button tick every 4 ticks.  This is for button bounce.
    movfw   timerlo
    andlw   0x03
    btfsc   STATUS,Z
    call    button_tick

    call    uart_tx_tick
    call    uart_rx_tick
    call    command_tick

    banksel timerflags
    bcf     timerflags,timer_flag_tick

    goto    main

skip_tick_code
    goto    main
;    goto    main
    movlw   0x10
    call    bat_set_port

loop
    call    bat_ready
    btfss   STATUS,Z
    goto    main

    movlw   0x55
    call    bat_send
    ;  See if the transmitter data register is empty.
;    banksel PIR1
;    btfss   PIR1,TXIF       ;  Skip if ready for a character.
;    goto    main
;
;    banksel TXREG           ;  Send a test character.
;    movlw   0x95
;    movwf   TXREG
    goto    main


; End of main program.
;
; ------------------------------------------------------------------------------

	end










