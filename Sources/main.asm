;**********************************************
;* COE538 Final Project: Robot Guidance             *
;* Ashwin Sundaresan, Sayeed Ahmed, Aronno Das*
;* (501159998)    (500985882)      (501170036)
;* USE EEBOT: 102965*
;**********************************************

; export symbols
                    XDEF Entry, _Startup ;
              ABSENTRY Entry ; for absolute assembly: mark
              INCLUDE "derivative.inc"



; Liquid Crystal Display Equates
;-------------------------------
CLEAR_HOME    EQU   $01                   ; Clear the display and home the cursor
INTERFACE     EQU   $38                   ; 8 bit interface, two line display
CURSOR_OFF    EQU   $0C                   ; Display on, cursor off
SHIFT_OFF     EQU   $06                   ; Address increments, no character shift
LCD_SEC_LINE  EQU   64                    ; Starting addr. of 2nd line of LCD (note decimal value!)

; LCD Addresses
; -------------
LCD_CNTR      EQU   PTJ                   ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT       EQU   PORTB                 ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E         EQU   $80                   ; LCD E-signal pin
LCD_RS        EQU   $40                   ; LCD RS-signal pin


NULL          EQU   00                    ; Null Character
CR            EQU   $0D                   ; Carry Return character
SPACE         EQU   ' '                   ; Space Character

;Timers
;---------------
T_LEFT        EQU   7
T_RIGHT       EQU   7

; State Machine for Robot
;-----------------
START         EQU   0
FWD           EQU   1
ALL_STOP      EQU   2
LEFT_TRN      EQU   3
RIGHT_TRN     EQU   4
REV_TRN       EQU   5                     
LEFT_ALIGN    EQU   6                     
RIGHT_ALIGN   EQU   7                     

; variable/data section
; ---------------------
              ORG   $3800

; Initial values based on the initial readings & variance
; -----------------------------------------------------------
BASE_LINE     FCB   $9D
BASE_BOW      FCB   $CA
BASE_MID      FCB   $CA
BASE_PORT     FCB   $CC
BASE_STBD     FCB   $CC


LINE_VARIANCE           FCB   $18           ; Adding variance based on testing to 
BOW_VARIANCE            FCB   $30           ; Establish baseline for sensors
PORT_VARIANCE           FCB   $20                     
MID_VARIANCE            FCB   $20
STARBOARD_VARIANCE      FCB   $15

TOP_LINE      RMB   20                      ; Top line of display
              FCB   NULL                    ; terminated by null constant string
              
BOT_LINE      RMB   20                      ; Bottom line of display
              FCB   NULL                    ; terminated by null constant string

CLEAR_LINE    FCC   '                  '    ; Clear the line of display
              FCB   NULL                    ; terminated by null constant string

TEMP          RMB   1                       ; Temporary location

; Store the Guider Sensor Values 
; ------------------------------------------------------
SENSOR_LINE   FCB   $01                    
SENSOR_BOW    FCB   $23                    
SENSOR_PORT   FCB   $45
SENSOR_MID    FCB   $67
SENSOR_STBD   FCB   $89
SENSOR_NUM    RMB   1 

; Variable Section
;***************************************************************************************************
              ORG   $3850                   ; Start address for the variables in memory
TOF_COUNTER   dc.b  0                       ; Counts timer overflows, which is used for timing events
CRNT_STATE    dc.b  2                       ; Keeps track of the robot's current state
T_TURN        ds.b  1                       ; Stores 1 byte for how long the robot should turn
TEN_THOUS     ds.b  1                       ; Store 1 byte the ten-thousands digit for displaying numbers
THOUSANDS     ds.b  1                       ; Store 1 byte the thousands digit for displaying numbers
HUNDREDS      ds.b  1                       ; Store 1 byte the hundreds digit for displaying numbers
TENS          ds.b  1                       ; Store 1 byte the tens digit for displaying numbers
UNITS         ds.b  1                       ; Store 1 byte the units (ones) digit for displaying numbers
NO_BLANK      ds.b  1                       ; Store 1 byte a flag to avoid showing extra zeros in numbers
HEX_TABLE       FCC   '0123456789ABCDEF'    ; Store 1 byte a table to convert numbers to hex digits
BCD_SPARE     RMB   2                       ; Temporary space 


; Code Section
;***************************************************************************************************
              ORG   $4000                  ; Start address for the program
Entry:                                                                       
_Startup: 

              LDS   #$4000                 ; Set the stack pointer to $4000
              CLI                          ; Enable global interrupts
              JSR   INIT                   ; Jump to subroutine to initialize the input/output
              JSR   openADC                ; Jump to subroutine to initialize the Analog-to-Digital Converter (ADC)
              JSR   initLCD                ; Jump to subroutine to initialize  the LCD display
              JSR   CLR_LCD_BUF            ; Jump to subroutine to initialize the LCD buffer with spaces
              BSET  DDRA,%00000011         ; Set the motor direction pins as outputs
              BSET  DDRT,%00110000         ; Set the motor speed pins as outputs
              JSR   initAD                 ; Jump to subroutine to initialize  the ADC for reading sensors
              JSR   initLCD                ;Jump to subroutine to initialize to re-initialize the LCD
              JSR   clrLCD                 ; Jump to subroutine to initialize to clear the LCD and reset the cursor position
              LDX   #msg1                  ; Load  first message 
              JSR   putsLCD                ; Jump to subroutine to initialize display the first message on the LCD
              LDAA  #$C0                   ; Set the LCD cursor to the second line
              JSR   cmd2LCD                ; Jump to subroutine to send the command to move the cursor
              LDX   #msg2                  ; Load  second message 
              JSR   putsLCD                ;Jump to subroutine to display the second message on the LCD
              JSR   ENABLE_TOF             ;Jump to subroutine to enable the timer overflow for system timing

MAIN        
              JSR   G_LEDS_ON              ; Turn on guider LEDs for sensing
              JSR   READ_SENSORS           ; Read data from the 5 sensors
              JSR   G_LEDS_OFF             ; Turn off guider LEDs
              JSR   UPDT_DISPL             ; Update the LCD with current state and battery info
              LDAA  CRNT_STATE             ; Load the current robot state
              JSR   DISPATCHER             ; Run the state handler
              BRA   MAIN                   ; Repeat the main loop
           

; data section
;***************************************************************************************************
msg1          dc.b  "Battery volt ",0   ; Message 1 with null terminator
msg2          dc.b  "State",0            ; Message 2 with null terminator
tab           dc.b  "start  ",0          ; State names with null terminators
              dc.b  "fwd    ",0
              dc.b  "all_stp",0
              dc.b  "LeftTurn  ",0
              dc.b  "RightTurn  ",0
              dc.b  "RevTrn ",0
              dc.b  "LeftTimed ",0     
              dc.b  "RTimed ",0
  

; subroutine section
;***************************************************************************************************
DISPATCHER        JSR   VERIFY_START                        ; Start the state checking process
                  RTS                                       ; Return to subroutine after checking  is complete

VERIFY_START      CMPA  #START                              ; Check if robot is in START state
                  BNE   VERIFY_FORWARD                      ; If not, move to checking FORWARD state
                  JSR   START_ST                            ; Handle START state logic
                  RTS                                       ; Return after processing START state

VERIFY_FORWARD    CMPA  #FWD                                ; Check if robot is in FORWARD state
                  BNE   VERIFY_STOP                         ; If not, move to checking ALL_STOP state
                  JSR   FWD_ST                              ; Handle FORWARD state logic
                  RTS                                       ; Return after processing FORWARD state
                  
VERIFY_REV_TRN    CMPA  #REV_TRN                            ; Check if robot is in REV_TURN state
                  BNE   VERIFY_LEFT_ALIGN                   ; If not, move to checking LEFT_ALIGN state
                  JSR   REV_TRN_ST                          ; Handle REV_TURN state logic
                  RTS                                       ; Return after processing REV_TURN state

VERIFY_STOP       CMPA  #ALL_STOP                           ; Check if robot is in state is ALL_STOP state
                  BNE   VERIFY_LEFT_TRN                     ; If not, move to checking LEFT_TURN state
                  JSR   ALL_STOP_ST                         ; Handle ALL_STOP state logic
                  RTS                                       ; Return after processing ALL_STOP state

VERIFY_LEFT_TRN   CMPA  #LEFT_TRN                           ; Check if  robot is in LEFT_TURN state
                  BNE   VERIFY_RIGHT_TRN                    ; If not, move to checking RIGHT_TURN state
                  JSR   LEFT                                ; Handle LEFT_TURN state logic
                  RTS                                       ; Return after processing LEFT_TURN state

VERIFY_LEFT_ALIGN CMPA  #LEFT_ALIGN                         ; Check if  robot is in LEFT_ALIGN state
                  BNE   VERIFY_RIGHT_ALIGN                  ; If not, move to checking RIGHT_ALIGN state
                  JSR   LEFT_ALIGN_DONE                     ; Handle LEFT_ALIGN state logic
                  RTS                                       ; Return after processing LEFT_ALIGN state

VERIFY_RIGHT_TRN  CMPA  #RIGHT_TRN                          ; Check if robot is in RIGHT_TURN state
                  BNE   VERIFY_REV_TRN                      ; If not, move to checking REV_TURN state
                  JSR   RIGHT                               ; Handle RIGHT_TURN state logic

VERIFY_RIGHT_ALIGN CMPA  #RIGHT_ALIGN                       ; Check if robot is in RIGHT_ALIGN state
                  JSR   RIGHT_ALIGN_DONE                    ; Handle RIGHT_ALIGN state logic
                  RTS                                       ; Return after processing RIGHT_ALIGN state


;Movement
;***************************************************************************************************
START_ST          BRCLR   PORTAD0, %00000100, RELEASE       ; Check if the start signal is active
                  JSR     INIT_FWD                          ; Initialize the robot to move forward
                  MOVB    #FWD, CRNT_STATE                  ; Set the current state to FORWARD

RELEASE           RTS                                       ; Return to subroutine
                                                                                                                              

;***************************************************************************************************

FWD_ST            BRSET   PORTAD0, $04, NO_FWD_BUMP           ; Check if the bow bumper is triggered (hit an obstacle)
                  MOVB    #REV_TRN, CRNT_STATE                ; If bow bumper is triggered, change state to REV_TURN (reverse)
                  JSR     UPDT_DISPL                          ; Update the display with the current status
                  JSR     INIT_REV                            ; Initialize the reverse movement
                  LDY     #12000                              ; Set a delay time for how long the robot should reverse
                  JSR     del_50us                            
                  JSR     INIT_RIGHT                          ; Initialize the right turn after reverse
                  LDY     #12000                              ; Set delay time for how long the robot should right turn
                  JSR     del_50us                           
                  LBRA    EXIT                                 ; Exit the subroutine
                  
;************************************************
;MAY NOT WORK REMOVE BEFORE DEMO*
;************************************************
NO_FWD_BUMP       BRSET   PORTAD0, $04, NO_FWD_REAR_BUMP      ; Check if the stern bumper is triggered (hit a rear obstacle)
                  MOVB    #ALL_STOP, CRNT_STATE               ; If true, change state to ALL_STOP (stop the robot)
                  JSR     INIT_STOP                           ; Initialize stop routine
                  LBRA    EXIT                                 ; Exit the subroutine


NO_FWD_REAR_BUMP 
                   LDAA    SENSOR_BOW                           ; Read the bow sensor value
                  ADDA    BOW_VARIANCE                        ; Add variance to the sensor value to account for small fluctuations
                  CMPA    BASE_BOW                             ; Compare with baseline value
                  BPL     NOT_ALIGNED                         ; If aligned, continue checking other sensors
                  
                  LDAA    SENSOR_MID                           ; Read the middle sensor value
                  ADDA    MID_VARIANCE                        ; Add variance to the sensor value to account for small fluctuations
                  CMPA    BASE_MID                             ; Compare with baseline 
                  BPL     NOT_ALIGNED                         ; If aligned, continue checking
                  
                  LDAA    SENSOR_LINE                          ; Read the line sensor value
                  ADDA    LINE_VARIANCE                       ; Add variance to the sensor value to account for small fluctuations
                  CMPA    BASE_LINE                             ; Compare with baseline value
                  BPL     CHECK_RIGHT_ALIGN                   ; If aligned, check the right side first
                  
                  LDAA    SENSOR_LINE                          ; Re-read the line sensor
                  SUBA    LINE_VARIANCE                       ; Subtract variance to check if the bot is not centered on line
                  CMPA    BASE_LINE                             ; Compare with baseline value
                  BMI     CHECK_LEFT_ALIGN                    ; If misaligned, check the left side


;***************************************************************************************************                                                                  

NOT_ALIGNED       
                  LDAA    SENSOR_PORT                       ; Read the value from the port sensor (left sensor)
                  ADDA    PORT_VARIANCE                     ; Add variance to the sensor value to account for small fluctuations
                  CMPA    BASE_PORT                         ; Compare the adjusted value with the baseline
                  BPL     PARTIAL_LEFT_TRN                 ; If the value is within range make a left allignment
                  BMI     NO_PORT                          ; If the value is below baseline, check the port sensor again

NO_PORT           
                  LDAA    SENSOR_BOW                        ; Read the value from the bow sensor (front sensor)
                  ADDA    BOW_VARIANCE                      ; Add variance to the sensor value to account for small fluctuations
                  CMPA    BASE_BOW                          ; Compare the adjusted value with the baseline
                  BPL     EXIT                             ; If value is within range, exit this check, no misalignment detected
                  BMI     NO_BOW                           ; If misaligned, check the starboard sensor next

NO_BOW           
                   LDAA    SENSOR_STBD                       ; Read the value from the starboard sensor (right side)
                  ADDA    STARBOARD_VARIANCE               ; Add variance to the sensor value to account for small fluctuations
                  CMPA    BASE_STBD                        ; Compare the adjusted value with the baseline 
                  BPL     PARTIAL_RIGHT_TRN                ; If the value is within range make a right allignment
                  BMI     EXIT                             ; If misaligned, exit this check


;***************************************************************************************************

PARTIAL_LEFT_TRN  
                  LDY     #6000                              ; Delay time for the partial left turn
                  jsr     del_50us                           ;
                  JSR     INIT_LEFT                          ; Initialize the left turn (set motor directions, etc.)
                  MOVB    #LEFT_TRN, CRNT_STATE              ; Set the current state to LEFT_TRN (partial left turn)
                  LDY     #6000                              ; Delay time  for stabilization after the turn is made
                  JSR     del_50us                          
                  BRA     EXIT                                ; Exit this subroutine after the turn is completed

CHECK_LEFT_ALIGN  
                  JSR     INIT_LEFT                          ; Initialize the left turn (prepare motors, etc.)
                  MOVB    #LEFT_ALIGN, CRNT_STATE            ; Set the current state to LEFT_ALIGN (indicating alignment correction)
                  BRA     EXIT                                ; Exit the subroutine after aligning to the left

;*************************************************************************************************** 

PARTIAL_RIGHT_TRN 
                    LDY     #8000                              ; Delay time for the partial right turn
                   jsr     del_50us                           
                   JSR     INIT_RIGHT                         ; Initialize the right turn (set motor directions, etc.)
                   MOVB    #RIGHT_TRN, CRNT_STATE             ; Set the current state to RIGHT_TRN (partial right turn)
                   LDY     #8000                              ; Delay time for stabilization after the turn
                   JSR     del_50us                           
                   BRA     EXIT                                ; Exit this subroutine after the turn is completed

CHECK_RIGHT_ALIGN  
                   JSR     INIT_RIGHT                         ; Initialize the right turn (prepare motors, etc.)
                   MOVB    #RIGHT_ALIGN, CRNT_STATE           ; Set the current state to RIGHT_ALIGN (indicating alignment correction)
                   BRA     EXIT                                ; Exit the subroutine after aligning to the right

EXIT               RTS                                       ; Return from the subroutine
                                                                                                                                                
;***************************************************************************************************                                                                            

LEFT              LDAA    SENSOR_BOW                           ; Read the bow sensor value (front sensor)
                  ADDA    BOW_VARIANCE                        ; Add variance to the sensor value to account for small fluctuations
                  CMPA    BASE_BOW                            ; Compare the adjusted bow sensor value with the baseline (expected value)
                  BPL     LEFT_ALIGN_DONE                     ; If the sensor value is within acceptable range (aligned), move to LEFT_ALIGN_DONE
                  BMI     EXIT                                ; If misaligned (below baseline), exit this subroutine

LEFT_ALIGN_DONE   MOVB    #FWD, CRNT_STATE                    ; Set the current state to FORWARD (robot will continue moving forward)
                  JSR     INIT_FWD                            ; Initialize forward movement (start the motors)
                  BRA     EXIT                                ; Exit the subroutine after initialization

RIGHT             LDAA    SENSOR_BOW                           ; Read the bow sensor value (front sensor)
                  ADDA    BOW_VARIANCE                        ; Add variance to the sensor value to account for small fluctuations
                  CMPA    BASE_BOW                            ; Compare the adjusted bow sensor value with the baseline (expected value)
                  BPL     RIGHT_ALIGN_DONE                    ; If the sensor value is within acceptable range (aligned), move to RIGHT_ALIGN_DONE
                  BMI     EXIT                                ; If misaligned (below baseline), exit this subroutine

RIGHT_ALIGN_DONE  MOVB    #FWD, CRNT_STATE                    ; Set the current state to FORWARD (robot will continue moving forward)
                  JSR     INIT_FWD                            ; Initialize forward movement (start the motors)
                  BRA     EXIT                                ; Exit the subroutine after initialization

;***************************************************************************************************

REV_TRN_ST        LDAA    SENSOR_BOW                           ; Read the bow sensor value (front sensor)
                  ADDA    BOW_VARIANCE                        ; Add variance to the bow sensor reading (to allow for minor inaccuracies)
                  CMPA    BASE_BOW                            ; Compare the adjusted bow sensor value with the baseline (expected value)
                  BMI     EXIT                                ; If misaligned (below baseline), exit this subroutine
                  JSR     INIT_LEFT                           ; If aligned, initialize the left turn (set motor directions, etc.)
                  MOVB    #FWD, CRNT_STATE                   ; Set the current state to FORWARD (robot will move forward after turn)
                  JSR     INIT_FWD                            ; Initialize forward movement (start the motors)
                  BRA     EXIT                                ; Exit the subroutine after initialization

ALL_STOP_ST       BRSET   PORTAD0, %00000100, NO_START_BUMP    ; Check if the start bumper is triggered (check bit in PORTAD0)
                  MOVB    #START, CRNT_STATE                  ; If the bumper is triggered, set the current state to START (reset state)
                  
NO_START_BUMP     RTS                                       ; Return from subroutine, as the start bumper was not triggered
                                                                           

; Initialization Subroutines
;***************************************************************************************************

;RIGHT MOTOR IS ON, LEFT MOTOR IS OFF
INIT_RIGHT        BSET    PORTA, %00000010            ; Set bit 1 of PORTA to turn on the right motor (right direction)
                  BCLR    PORTA, %00000001            ; Clear bit 0 of PORTA to turn off the left motor (left direction)
                  LDAA    TOF_COUNTER                ; Load the current value of the timer overflow counter
                  ADDA    #T_RIGHT                   ; Add the right turn time constant (T_RIGHT) to the timer counter
                  STAA    T_TURN                     ; Store the result in T_TURN to mark the time for the right turn
                  RTS                                ; Return from the subroutine

; RIGHT MOTOR IS OFF, LEFT MOTOR IS ON
INIT_LEFT         BSET    PORTA, %00000001            ; Set bit 0 of PORTA to turn on the left motor (left direction)
                  BCLR    PORTA, %00000010            ; Clear bit 1 of PORTA to turn off the right motor (right direction)
                  LDAA    TOF_COUNTER                ; Load the current value of the timer overflow counter
                  ADDA    #T_LEFT                    ; Add the left turn time constant (T_LEFT) to the timer counter
                  STAA    T_TURN                     ; Store the result in T_TURN to mark the time for the left turn
                  RTS                                ; Return from the subroutine

;BOTH MOTORS ON FORWARD
INIT_FWD          BCLR    PORTA, %00000011            ; Clear both bits of PORTA to set forward direction for both motors (both motors go forward)
                  BSET    PTT, %00110000             ; Set bits of PTT to turn on the drive motors (motor power ON)
                  RTS                                ; Return from the subroutine

;BOTH MOTORS ON REVERSE
INIT_REV          BSET    PORTA, %00000011            ; Set both bits of PORTA to reverse direction for both motors (both motors go in reverse)
                  BSET    PTT, %00110000             ; Set bits of PTT to turn on the drive motors (motor power ON)
                  RTS                                ; Return from the subroutine
                  
;BOTH MOTORS OFF TO STOP
INIT_STOP         BCLR    PTT, %00110000             ; Clear the bits of PTT to turn off the drive motors (stop the motors)
                  RTS                                ; Return from the subroutine



;***************************************************************************************************
;       Initialize Sensors
INIT              BCLR   DDRAD,$FF ; Make PORTAD an input (DDRAD @ $0272)
                  BSET   DDRA,$FF  ; Make PORTA an output (DDRA @ $0002)
                  BSET   DDRB,$FF  ; Make PORTB an output (DDRB @ $0003)
                  BSET   DDRJ,$C0  ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                  RTS


;***************************************************************************************************
;        Initialize ADC              
openADC           MOVB   #$80,ATDCTL2 ; Turn on ADC (ATDCTL2 @ $0082)
                  LDY    #1           ; Wait for 50 us for ADC to be ready
                  JSR    del_50us     ; - " -
                  MOVB   #$20,ATDCTL3 ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                  MOVB   #$97,ATDCTL4 ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                  RTS

;---------------------------------------------------------------------------
;                           Clear LCD Buffer
CLR_LCD_BUF       LDX   #CLEAR_LINE                     ; Load the address of the string "CLEAR_LINE" into register X (used for clearing the LCD display)
                  LDY   #TOP_LINE                       ; Load the address of the top line of the LCD display into register Y
                  JSR   STRCPY                          ; Call the STRCPY subroutine to copy "CLEAR_LINE" to the LCD's top line

CLB_SECOND        LDX   #CLEAR_LINE                     ; Load the address of the string "CLEAR_LINE" into register X (used for clearing the LCD display)
                  LDY   #BOT_LINE                       ; Load the address of the bottom line of the LCD display into register Y
                  JSR   STRCPY                          ; Call the STRCPY subroutine to copy "CLEAR_LINE" to the LCD's bottom line

CLB_EXIT          RTS                                  ; Return from the subroutine after clearing both lines on the LCD


; -------------------------------------------------------------------------------------------------      
; String Copy Subroutine

STRCPY            PSHX            ; Save the X register (source pointer) on the stack
                  PSHY            ; Save the Y register (destination pointer) on the stack
                  PSHA            ; Save the A register (accumulator) on the stack

STRCPY_LOOP       LDAA 0,X        ; Load the character from the source string (pointed by X) into the A register
                  STAA 0,Y        ; Store the character from A into the destination string (pointed by Y)
                  BEQ STRCPY_EXIT ; If the character is null (end of string, 0), exit the loop
                  INX             ; Increment the source pointer (X) to move to the next character
                  INY             ; Increment the destination pointer (Y) to move to the next position
                  BRA STRCPY_LOOP ; Repeat the loop to copy the next character

STRCPY_EXIT       PULA            ; Restore the A register from the stack (after the copy operation)
                  PULY            ; Restore the Y register (destination pointer) from the stack
                  PULX            ; Restore the X register (source pointer) from the stack
                  RTS             ; Return from the subroutine after the string copy is complete


; -------------------------------------------------------------------------------------------------      
;                                   Guider LEDs ON                                                 
                                                                
G_LEDS_ON         BSET PORTA,%00100000 ; Set bit 5 to turn on the LEDs for Sensors to read                                                 
                  RTS                                                                         

; -------------------------------------------------------------------------------------------------      
;                                   Guider LEDs OFF                                                |

G_LEDS_OFF        BCLR PORTA,%00100000 ; Clear bit 5 to turn off the LEDs for the Sensors to not read                                              
                  RTS                                                                               

; -------------------------------------------------------------------------------------------------      
;                               Read Sensors

READ_SENSORS      CLR   SENSOR_NUM     ; Select sensor number 0
                  LDX   #SENSOR_LINE   ; Point at the start of the sensor array

RS_MAIN_LOOP      LDAA  SENSOR_NUM     ; Select the correct sensor input
                  JSR   SELECT_SENSOR  ; on the hardware
                  LDY   #600           ;  delay to allow the sensor to stabilize
                  JSR   del_50us      
                  LDAA  #%10000001     ; Start A/D conversion on AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
                  LDAA  ATDDR0L        ; A/D conversion is complete in ATDDR0L
                  STAA  0,X            ; so copy it to the sensor register
                  CPX   #SENSOR_STBD   ; If this is the last reading
                  BEQ   RS_EXIT        ; Then exit
                  INC   SENSOR_NUM     ; Else, increment the sensor number
                  INX                  ; and the pointer into the sensor array
                  BRA   RS_MAIN_LOOP   ; and do it again

RS_EXIT           RTS


; -------------------------------------------------------------------------------------------------      
;                               Select Sensor
; -------------------------------------------------------------------------------------------------      
SELECT_SENSOR     PSHA                ; Save the sensor number for the moment
                  LDAA PORTA          ; Clear the sensor selection bits to zeros
                  ANDA #%11100011
                  STAA TEMP           ; and save it into TEMP
                  PULA                ; Get the sensor number
                  ASLA                ; Shift the selection number left, twice
                  ASLA 
                  ANDA #%00011100     ; Clear irrelevant bit positions
                  ORAA TEMP           ; OR it into the sensor bit positions
                  STAA PORTA          ; Update the hardware
                  RTS


; -------------------------------------------------------------------------------------------------      
;                               Display Sensors
; -------------------------------------------------------------------------------------------------
DP_FRONT_SENSOR   EQU TOP_LINE+3
DP_PORT_SENSOR    EQU BOT_LINE+0
DP_MID_SENSOR     EQU BOT_LINE+3
DP_STBD_SENSOR    EQU BOT_LINE+6
DP_LINE_SENSOR    EQU BOT_LINE+9

DISPLAY_SENSORS   LDAA  SENSOR_BOW        ; Get the FRONT sensor value
                  JSR   BIN2ASC           ; Convert to ascii string in D
                  LDX   #DP_FRONT_SENSOR  ; Point to the LCD buffer position
                  STD   0,X               ; and write the 2 ascii digits there
                  LDAA  SENSOR_PORT       ; Repeat for the PORT value
                  JSR   BIN2ASC
                  LDX   #DP_PORT_SENSOR
                  STD   0,X
                  LDAA  SENSOR_MID        ; Repeat for the MID value
                  JSR   BIN2ASC
                  LDX   #DP_MID_SENSOR
                  STD   0,X
                  LDAA  SENSOR_STBD       ; Repeat for the STARBOARD value
                  JSR   BIN2ASC
                  LDX   #DP_STBD_SENSOR
                  STD   0,X
                  LDAA  SENSOR_LINE       ; Repeat for the LINE value
                  JSR   BIN2ASC
                  LDX   #DP_LINE_SENSOR
                  STD   0,X
                  LDAA  #CLEAR_HOME       ; Clear the display and home the cursor
                  JSR   cmd2LCD           ; "
                  LDY   #40               ; Wait 2 ms until "clear display" command is complete
                  JSR   del_50us
                  LDX   #TOP_LINE         ; Now copy the buffer top line to the LCD
                  JSR   putsLCD
                  LDAA  #LCD_SEC_LINE     ; Position the LCD cursor on the second line
                  JSR   LCD_POS_CRSR
                  LDX   #BOT_LINE         ; Copy the buffer bottom line to the LCD
                  JSR   putsLCD
                  RTS

;***************************************************************************************************
;*                      Update Display (Battery Voltage + Current State)                           *
;***************************************************************************************************
UPDT_DISPL        MOVB    #$90,ATDCTL5    ; R-just., uns., sing. conv., mult., ch=0, start
                  BRCLR   ATDSTAT0,$80,*  ; Wait until the conver. seq. is complete
                  LDAA    ATDDR0L         ; Load the ch0 result - battery volt - into A
                  LDAB    #39             ;AccB = 39
                  MUL                     ;AccD = 1st result x 39
                  ADDD    #600            ;AccD = 1st result x 39 + 600
                  JSR     int2BCD
                  JSR     BCD2ASC
                  LDAA    #$8D            ;move LCD cursor to the 1st row, end of msg1
                  JSR     cmd2LCD
                  LDAA    TEN_THOUS       ;output the TEN_THOUS ASCII character
                  JSR     putcLCD 
                  LDAA    THOUSANDS       ;output the THOUSANDS character
                  JSR     putcLCD
                  LDAA    #'.'            ; add the decimal place
                  JSR     putcLCD         ; put the dot into LCD
                  LDAA    HUNDREDS        ;output the HUNDREDS ASCII character
                  JSR     putcLCD         ;same for THOUSANDS, ?.? and HUNDREDS
                  LDAA    #$C7            ; Move LCD cursor to the 2nd row, end of msg2
                  JSR     cmd2LCD         ;
                  LDAB    CRNT_STATE      ; Display current state
                  LSLB                    ; "
                  LSLB                    ; "
                  LSLB
                  LDX     #tab            ; "
                  ABX                     ; "
                  JSR     putsLCD         ; "
                  RTS

;***************************************************************************************************
ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1           ; Enable TCNT
                  STAA    TFLG2           ; Clear TOF
                  LDAA    #%10000100      ; Enable TOI and select prescale factor equal to 16
                  STAA    TSCR2
                  RTS

TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000      ; Clear
                  STAA    TFLG2           ; TOF
                  RTI


; utility subroutines
;***************************************************************************************************
initLCD:          BSET    DDRB,%11111111  ; configure pins PS7,PS6,PS5,PS4 for output
                  BSET    DDRJ,%11000000  ; configure pins PE7,PE4 for output
                  LDY     #2000
                  JSR     del_50us
                  LDAA    #$28
                  JSR     cmd2LCD
                  LDAA    #$0C
                  JSR     cmd2LCD
                  LDAA    #$06
                  JSR     cmd2LCD
                  RTS

;***************************************************************************************************
clrLCD:           LDAA  #$01
                  JSR   cmd2LCD
                  LDY   #40
                  JSR   del_50us
                  RTS

;***************************************************************************************************
del_50us          PSHX                   ; (2 E-clk) Protect the X register
eloop             LDX   #300             ; (2 E-clk) Initialize the inner loop counter
iloop             NOP                    ; (1 E-clk) No operation
                  DBNE X,iloop           ; (3 E-clk) If the inner cntr not 0, loop again
                  DBNE Y,eloop           ; (3 E-clk) If the outer cntr not 0, loop again
                  PULX                   ; (3 E-clk) Restore the X register
                  RTS                    ; (5 E-clk) Else return

;***************************************************************************************************
cmd2LCD:          BCLR  LCD_CNTR, LCD_RS ; select the LCD instruction
                  JSR   dataMov          ; send data to IR
                  RTS

;***************************************************************************************************
putsLCD:          LDAA  1,X+             ; get one character from  string
                  BEQ   donePS           ; get NULL character
                  JSR   putcLCD
                  BRA   putsLCD

donePS            RTS

;***************************************************************************************************
putcLCD:          BSET  LCD_CNTR, LCD_RS  ; select the LCD data register (DR)c
                  JSR   dataMov           ; send data to DR
                  RTS

;***************************************************************************************************
dataMov:          BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the upper 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LSLA                    ; match the lower 4 bits with LCD data pins
                  LSLA                    ; ""
                  LSLA                    ; ""
                  LSLA                    ; ""
                  BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the lower 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LDY   #1                ; adding this delay allows
                  JSR   del_50us          ; completion of most instructions
                  RTS

;***************************************************************************************************
initAD            MOVB  #$C0,ATDCTL2      ;power up AD, select fast flag clear
                  JSR   del_50us          ;wait for 50 us
                  MOVB  #$00,ATDCTL3      ;8 conversions in a sequence
                  MOVB  #$85,ATDCTL4      ;res=8, conv-clks=2, prescal=12
                  BSET  ATDDIEN,$0C       ;configure pins AN03,AN02 as digital inputs
                  RTS

;***************************************************************************************************
int2BCD           XGDX                    ;Save the binary number into .X
                  LDAA #0                 ;Clear the BCD_BUFFER
                  STAA TEN_THOUS
                  STAA THOUSANDS
                  STAA HUNDREDS
                  STAA TENS
                  STAA UNITS
                  STAA BCD_SPARE
                  STAA BCD_SPARE+1
                  CPX #0                  ; Check for a zero input
                  BEQ CON_EXIT            ; and if so, exit
                  XGDX                    ; Not zero, get the binary number back to .D as dividend
                  LDX #10                 ; Setup 10 (Decimal!) as the divisor
                  IDIV                    ; Divide Quotient is now in .X, remainder in .D
                  STAB UNITS              ; Store remainder
                  CPX #0                  ; If quotient is zero,
                  BEQ CON_EXIT            ; then exit
                  XGDX                    ; else swap first quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TENS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB HUNDREDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB THOUSANDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TEN_THOUS

CON_EXIT          RTS                     ; Were done the conversion

LCD_POS_CRSR      ORAA #%10000000         ; Set the high bit of the control word
                  JSR cmd2LCD             ; and set the cursor address
                  RTS

;***************************************************************************************************
BIN2ASC               PSHA               ; Save a copy of the input number
                      TAB            
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the LSnibble
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                
                      LDAA 0,X            ; Get the LSnibble character
                      PULB                ; Retrieve the input number into ACCB
                      PSHA                ; and push the LSnibble character in its place
                      RORB                ; Move the upper nibble of the input number
                      RORB                ;  into the lower nibble position.
                      RORB
                      RORB 
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the MSnibble 
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                                                               
                      LDAA 0,X            ; Get the MSnibble character into ACCA
                      PULB                ; Retrieve the LSnibble character into ACCB
                      RTS

;***************************************************************************************************
;* BCD to ASCII Conversion Routine
;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag ?NO_BLANK? starts cleared and is set once a non-zero
;* digit has been detected.
;* The ?units? digit is never blanked, even if it and all the
;* preceding digits are zero.
BCD2ASC           LDAA    #0            ; Initialize the blanking flag
                  STAA    NO_BLANK

C_TTHOU           LDAA    TEN_THOUS     ; Check... (6 KB left)
                  ORAA    NO_BLANK
                  BNE     NOT_BLANK1

ISBLANK1          LDAA    #' '          ; It?s blank
                  STAA    TEN_THOUS     ; so store a space
                  BRA     C_THOU        ; and check the ?thousands? digit

NOT_BLANK1        LDAA    TEN_THOUS     ; Get the ?ten_thousands? digit
                  ORAA    #$30          ; Convert to ascii
                  STAA    TEN_THOUS
                  LDAA    #$1           ; Signal that we have seen a ?non-blank? digit
                  STAA    NO_BLANK

C_THOU            LDAA    THOUSANDS     ; Check the thousands digit for blankness
                  ORAA    NO_BLANK      ; If it?s blank and ?no-blank? is still zero
                  BNE     NOT_BLANK2

ISBLANK2          LDAA    #' '          ; Thousands digit is blank
                  STAA    THOUSANDS     ; so store a space
                  BRA     C_HUNS        ; and check the hundreds digit

NOT_BLANK2        LDAA    THOUSANDS     ; (similar to ?ten_thousands? case)
                  ORAA    #$30
                  STAA    THOUSANDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_HUNS            LDAA    HUNDREDS      ; Check the hundreds digit for blankness
                  ORAA    NO_BLANK      ; If it?s blank and ?no-blank? is still zero
                  BNE     NOT_BLANK3

ISBLANK3          LDAA    #' '          ; Hundreds digit is blank
                  STAA    HUNDREDS       ; so store a space
                  BRA     C_TENS          ; and check the tens digit

NOT_BLANK3        LDAA    HUNDREDS          ; (similar to ?ten_thousands? case)
                  ORAA    #$30
                  STAA    HUNDREDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_TENS            LDAA    TENS          ; Check the tens digit for blankness
                  ORAA    NO_BLANK      ; If it?s blank and ?no-blank? is still zero
                  BNE     NOT_BLANK4

ISBLANK4          LDAA    #' '          ; Tens digit is blank
                  STAA    TENS          ; so store a space
                  BRA     C_UNITS       ; and check the units digit

NOT_BLANK4        LDAA    TENS          ; (similar to ?ten_thousands? case)
                  ORAA    #$30
                  STAA    TENS

C_UNITS           LDAA    UNITS         ; No blank check necessary, convert to ascii.
                  ORAA    #$30
                  STAA    UNITS
                  RTS                 ; We?re done

;***************************************************************************************************

; Display the battery voltage
;----------------------------
                  LDAA    #$C7            ; Move LCD cursor to the 2nd row, end of msg2
                  JSR     cmd2LCD         ;
                  LDAB    CRNT_STATE      ; Display current state
                  LSLB                    ; "
                  LSLB                    ; "
                  LSLB
                  LDX     #tab            ; "
                  ABX                     ; "
                  JSR     putsLCD         ; "
                  RTS

;***************************************************************************************************
;*                                Interrupt Vectors                                                *
;***************************************************************************************************
                  ORG     $FFFE
                  DC.W    Entry ; Reset Vector
                  ORG     $FFDE
                  DC.W    TOF_ISR ; Timer Overflow Interrupt Vector