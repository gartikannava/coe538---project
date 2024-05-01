;********************************************************************************************
;* eebot Guider Program                                                                     *
;* Alex Samaru, 501043498, Section 1                                                        *
;* Gartikan Navaratnarajah, 501030903, Section 1                                            *
;********************************************************************************************

; export symbols
            XDEF Entry, _Startup         ; export 'Entry' symbol
            ABSENTRY Entry               ; for absolute assembly: mark this as application entry point

; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

; Liquid Crystal Display Equates
;-------------------------------
CLEAR_HOME EQU $01 ; Clear the display and home the cursor
INTERFACE  EQU $28 ; 4 bit interface, two line display
CURSOR_OFF EQU $0C ; Display on, cursor off
SHIFT_OFF  EQU $06 ; Address increments, no character shift
LCD_SEC_LINE EQU 64 ; Starting addr. of 2nd line of LCD (note decimal value!)
; LCD Addresses
LCD_DAT         EQU   PORTB             ; LCD data port, bits - PB7,...,PB0
LCD_CNTR        EQU   PTJ               ; LCD control port, bits - PJ6(RS),PJ7(E)
LCD_E           EQU   $80               ; LCD E-signal pin
LCD_RS          EQU   $40               ; LCD RS-signal pin
; Other codes
NULL            EQU   00                ; The string 'null terminator'
CR              EQU   $0D               ; 'Carriage Return' character
SPACE           EQU   ' '               ; The 'space' character

;--------------------------------------------------------------------------------------------
; Pathing and states
START           EQU   0                 ; State values
FWD             EQU   1                 ; 
REV             EQU   2                 ; 
R_TURN          EQU   3                 ; 
L_TURN          EQU   4                 ; 
TURN_BCK        EQU   5                 ; 
ALL_STOP        EQU   6                 ;
PRIMARY_PATHING EQU   0                 ; Primary path value (0 for right)
SECONDARY_PATHING EQU   1               ; Secondary path value (1 for left)

; Guider Sensor Thresholds
;--------------------------------------------------------------------------------------------
SENSOR_A        EQU   $C0               ; Path detection threshold
SENSOR_B        EQU   $CB               ; 
SENSOR_C        EQU   $DB               ; 
SENSOR_D        EQU   $CB               ; 
SENSOR_E        EQU   $60               ; If the line sensor is less than SENSOR_E robot steers right
SENSOR_F        EQU   $B4               ; If the line sensor is greater than SENSOR_F robot must shift left

; variable/data section

                ORG   $3850
;------------------------------------------------------------------------------                
CRNT_STATE      dc.b  6                 ; Current state register
A_DETECT        dc.b  0                 ; Sensor A detection (can be either high or low signal based on threshold)
B_DETECT        dc.b  0                 ; Sensor B detection 
C_DETECT        dc.b  0                 ; Sensor C detection 
D_DETECT        dc.b  0                 ; Sensor D detection 
E_DETECT        dc.b  0                 ; Sensor E detection 
F_DETECT        dc.b  0                 ; Sensor F detection 
RETURN          dc.b  0                 ; RETURN (TRUE = 1, FALSE = 0)
NEXT_DIR        dc.b  1                 ; Next direction instruction
TEN_THOUS       ds.b  1                 ; 10,000 digit
THOUSANDS       ds.b  1                 ; 1,000 digit
HUNDREDS        ds.b  1                 ; 100 digit
TENS            ds.b  1                 ; 10 digit
UNITS           ds.b  1                 ; 1 digit
NO_BLANK        ds.b  1                 ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE       ds.b  10                ; Extra space for decimal point and string termina
L_TRN_COUNT     dc.w  0                 ; 2-byte counter defaulted to 0 to track when the bot should make a left turn
R_TRN_COUNT     dc.w  0                 ; 2-byte counter defaulted to 0 to track when the bot should make a right turn
;------------------------------------------------------------------------------
; Storage Registers (9S12C32 RAM space: $3800 ... $3FFF)

SENSOR_LINE     dc.b  $0                ; (LINE ) Storage for guider sensor readings
SENSOR_BOW      dc.b  $0                ; (FRONT) 
SENSOR_PORT     dc.b  $0                ; (LEFT )
SENSOR_MID      dc.b  $0                ; (MIDDLE)
SENSOR_STBD     dc.b  $0                ; (RIGHT)
SENSOR_NUM      dc.b  1                 ; The currently selected sensor
TEMP            dc.b  1                 ; Temporary location

; code section
                ORG   $4000             ; Program start location
Entry:                                                                                     
_Startup:                                                                          
                LDS   #$4000            ; Initialize the stack pointer              
                CLI                     ; Enable interrupts                                                                              
                JSR   INIT              ; Initialize the ports                                                                                     
                JSR   initAD            ; Initialize ATD converter                                                             
                JSR   initLCD           ; Initialize the LCD                        
                JSR   clrLCD            ; Clear LCD & home cursor                                                              
                JSR   initTCNT          ; Initialize the TCNT                                                                                                                          
                LDX   #msg1             ; Display msg1                              
                JSR   putsLCD           ; ""                                                                                   
                LDAA  #$8A              ; Move LCD cursor to the end of msg1        
                JSR   cmd2LCD                                                   
                LDX   #msg2             ; Display msg2                              
                JSR   putsLCD           ; ""                                                                                 
                LDAA  #$C0              ; Move LCD cursor to the 2nd row            
                JSR   cmd2LCD           ; ""                                        
                LDX   #msg3             ; Display msg3                              
                JSR   putsLCD           ; ""                                                                                                             
                                                             
;--------------------------------------------------------------------------------------------
; main section                                                
          MAIN: JSR   SENS_DETECT              
                JSR   UPDT_DISPL                
                LDAA  CRNT_STATE                
                JSR   DISPATCHER                
                BRA   MAIN                      
                                                
;--------------------------------------------------------------------------------------------
; data section
          msg1: dc.b  "S:",0            ; Current state 
          msg2: dc.b  "R:",0            ; Sensor reading
          msg3: dc.b  "V:",0            ; Battery voltage
          
           tab: dc.b  "START  ",0       ; States
                dc.b  "FWD    ",0       
                dc.b  "REV    ",0       
                dc.b  "R_TURN ",0       
                dc.b  "L_TURN ",0       
                dc.b  "RETURN ",0       
                dc.b  "STOP   ",0       

;--------------------------------------------------------------------------------------------
; motor control subroutines
STARON      ;LDAB PTT                   ; Load port T into accumulator B
            ;ORAB #%00100000            ; Logical OR pin 5 with a 1 (Set)
            ;STAB PTT                   ; Store to port T
            BSET  PTT,%00100000
            RTS                        ; Return
            
STAROFF     ;LDAB PTT                   ; Load port T into accumulator B
            ;ANDB #%11011111            ; Logical AND pin 5 with a 0 (Clear)
            ;STAB PTT                   ; Store to port T
            BCLR  PTT,%00100000
            RTS                        ; Return
             
PORTON      ;LDAB PTT                   ; Load port T into accumulator B
            ;ORAB #%00010000            ; Logical OR pin 4 with a 1 (Set)
            ;STAB PTT                   ; Store to port T
            BSET  PTT,%00010000
            RTS                        ; Return
     
PORTOFF     ;LDAB PTT                   ; Load port T into accumulator B
            ;ANDB #%11101111            ; Logical AND pin 4 with a 0 (Clear)
            ;STAB PTT                   ; Store to port T
            BCLR  PTT,%00010000
            RTS                        ; Return
            
     
STARFWD     ;LDAB PORTA                 ; Load port A into accumulator B
            ;ANDB #%11111101            ; Logical AND pin 1 with a 0 (Clear)
            ;STAB PORTA                 ; Store to port A
            BCLR  PORTA,%00000010
            RTS                        ; Return
     
STARREV     ;LDAB PORTA                 ; Load port A into accumulator B
            ;ORAB #%00000010            ; Logical OR pin 1 with a 1 (Set)
            ;STAB PORTA                 ; Store to port A
            BSET  PORTA,%00000010
            RTS                        ; Return
             
PORTFWD     ;LDAB PORTA                 ; Load port A into accumulator B
            ;ANDB #%11111110            ; Logical AND pin 0 with a 0 (Clear)
            ;STAB PORTA                 ; Store to port A
            BCLR  PORTA,%00000001
            RTS                        ; Return
             
PORTREV     ;LDAB PORTA                 ; Load port A into accumulator B
            ;ORAB #%00000001            ; Logical OR pin 0 with a 1 (Set)
            ;STAB PORTA                 ; Store to port A
            BSET  PORTA,%00000001
            RTS                        ; Return
;--------------------------------------------------------------------------------------------
; initialization subroutine section
INIT            BCLR  DDRAD,$FF           ; Set PORTAD as input
                BSET  DDRA, $FF           ; Set PORTA as output
                BSET  DDRB,$FF            ; Make PORTB an output (DDRB @ $0003)
                BSET  DDRJ,$C0            ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                BSET  DDRT, $30           ; Set channels 4 & 5 of PORTT as output
                RTS
;--------------------------------------------------------------------------------------------
; Initialization of the LCD: 4-bit data width, 2-line display, 
; turn on display, cursor and blinking off. Shift cursor right.
initLCD         BSET  DDRB,%11111111      ; configure pins PB7,...,PB0 for output
                BSET  DDRJ,%11000000      ; configure pins PJ7(E), PJ6(RS) for output
                LDY   #2000               ; wait for LCD to be ready
                JSR   del_50us            ; -"-
                LDAA  #INTERFACE          ; set 4-bit data, 2-line display
                JSR   cmd2LCD             ; -"-
                LDAA  #CURSOR_OFF         ; display on, cursor off, blinking off
                JSR   cmd2LCD             ; -"-
                LDAA  #SHIFT_OFF          ; move cursor right after entering a character
                JSR   cmd2LCD             ; -"-
                RTS
;--------------------------------------------------------------------------------------------
clrLCD          LDAA  #$01                ; clear cursor and return to home position
                JSR   cmd2LCD             ; -"-
                LDY   #40                 ; wait until "clear cursor" command is complete
                JSR   del_50us            ; -"-
                RTS
;--------------------------------------------------------------------------------------------
initTCNT        MOVB  #$80,TSCR1          ; enable TCNT
                MOVB  #$00,TSCR2          ; disable TCNT OVF interrupt, set prescaler to 1
                MOVB  #$FC,TIOS           ; channels PT1/IC1,PT0/IC0 are input captures
                MOVB  #$05,TCTL4          ; capture on rising edges of IC1,IC0 signals
                MOVB  #$03,TFLG1          ; clear the C1F,C0F input capture flags
                MOVB  #$03,TIE            ; enable interrupts for channels IC1,IC0
                RTS
;--------------------------------------------------------------------------------------------
; dispatcher control
DISPATCHER      CMPA  #START                    ; If the current state is the start state 
                BNE   NOT_START                                                            
                JSR   START_ST                  ; call START_ST routine                
                RTS                             ; Exit                                  
                                           
NOT_START       CMPA  #FWD                      ; If the current state is the forward state            
                BNE   NOT_FWD                                                          
                JMP   FWD_ST                    ; then call the FWD_ST routine              
                                         
NOT_FWD         CMPA  #R_TURN                   ; If the current state is the right turn state         
                BNE   NOT_R_TURN                                                           
                JSR   R_TURN_ST                 ; call the R_TURN_ST routine          
                RTS                             ; Exit 
                
NOT_REV         CMPA  #TURN_BCK                 ; If the current state is the turn back state          
                BNE   NOT_TURN_BCK                                                           
                JMP   BK_TRK_ST                 ; call the TURN_BCK_ST routine                                     
                                         
NOT_R_TURN      CMPA  #L_TURN                   ; If the current state is the left turn state          
                BNE   NOT_L_TURN                                                           
                JSR   L_TURN_ST                 ; call LT_TRN_ST routine               
                RTS                             ; Exit                                  
                                     
NOT_L_TURN      CMPA  #REV                      ; If the current state is the reverse state            
                BNE   NOT_REV                                                          
                JSR   REV_ST                    ; call the REV_ST routine              
                RTS                             ; and exit       
                                     
NOT_TURN_BCK    CMPA  #ALL_STOP                 ; If the current state is the stop state            
                BNE   NOT_STOP                                                              
                JSR   ALL_STOP_ST               ; call the ALL_STOP_ST routine              
                RTS                             ; Exit                                  
                                       
NOT_STOP        NOP                             ; Otherwise, the current state is not known, so    
DISP_EXIT       RTS                             ; Return to Main 
;--------------------------------------------------------------------------------------------
START_ST        BRCLR PORTAD0,$04,NO_FWD        ; If "NOT" FWD_BUMP
                JSR   INIT_FWD                  ; Initialize FWD state 
                MOVB  #FWD,CRNT_STATE           ; change current state
                BRA   START_EXIT                ; Exit
                                                
NO_FWD          NOP                             ; otherwise return to main
START_EXIT      RTS                             ; 

FWD_ST          PULD                            
                BRSET PORTAD0,$04,NO_FWD_BUMP   ; If FWD_BUMP then
                LDAA  SECONDARY_PATHING         ; store NEXT_DIR value
                STAA  NEXT_DIR                    
                JSR   INIT_REV                  ; Initialize REVERSE routine
                MOVB  #REV,CRNT_STATE           ; change current state
                JMP   FWD_EXIT                  ; and return
              
NO_FWD_BUMP     BRSET PORTAD0,$08,NO_REV_BUMP   ; If REV_BUMP is pressed 
                JMP   INIT_ALL_STOP             ; Initialize the ALL_STOP state
                MOVB  #ALL_STOP,CRNT_STATE      ; change state
                JMP   FWD_EXIT                  ; return to main

NO_REV_BUMP     LDAA  D_DETECT                  ; For D equals 1 then
                BEQ   NO_R_CROSS                ; robot makes a right turn
                LDAA  NEXT_DIR                  ; Push direction for the previous 
                PSHA                            ; Intersection to the stack
                LDAA  PRIMARY_PATHING           ; store direction taken to NEXT_DIR
                STAA  NEXT_DIR                  ; ""
                JSR   INIT_R_TURN               ; Initialize the R_TURN state
                MOVB  #R_TURN,CRNT_STATE        ; change state
                JMP   FWD_EXIT                  ; Then exit
                
NO_R_CROSS      LDAA  B_DETECT                  ; For B equals 1
                BEQ   NO_L_CROSS                ; Check if A equals 1
                LDAA  A_DETECT                  ; If so. a straight path exists
                BEQ   LT_TURN                   ; eebot continues forward
                LDAA  NEXT_DIR                  ; add direction for previous intersection to stack
                PSHA                             
                LDAA  PRIMARY_PATHING           ; Store direction taken
                STAA  NEXT_DIR                    ; 
                BRA   NO_STEER_L                ; For A equals 0
                                
LT_TURN         LDAA  NEXT_DIR                  ; Push direction from last intersection to stack
                PSHA                            
                LDAA  SECONDARY_PATHING         ; store direction taken to NEXT_DIR
                STAA  NEXT_DIR                  
                JSR   INIT_L_TURN               ; The robot should make a left turn
                MOVB  #L_TURN,CRNT_STATE        ; Initialize the left turn state
                JMP   FWD_EXIT                  ; Set current state to left turn and exit

NO_L_CROSS      LDAA  F_DETECT                  ; For F equals 1
                BEQ   NO_STEER_R                ; robot should steer right
                JSR   PORTON                    ; so, turn on the portside motor     
                
RT_FWD_DIS      LDD   R_TRN_COUNT               ; Turn bot right a small increment before stopping to resume forward
                CPD   #500                      ; 
                BLO   RT_FWD_DIS                ; If right counter > defined increment then
                JSR   INIT_FWD                  ; Turn motors off
                JMP   FWD_EXIT                  ; Exit
                
NO_STEER_R      LDAA  E_DETECT                  ; For E equals 1
                BEQ   NO_STEER_L                ; The robot should steer left
                JSR   STARON                    ; and turn on the starside motor           
                
LT_FWD_DIS      LDD   L_TRN_COUNT               ; Turn bot left a small increment before stopping to resume forward
                CPD   #500                      ; 
                BLO   LT_FWD_DIS                ; If left counter > increment
                JSR   INIT_FWD                  ; Turn motors off
                JMP   FWD_EXIT                  ; Exit
                
NO_STEER_L      JSR   STARON                    ; Turn motors on
                JSR   PORTON                    
                
FWD_STR_DIS     LDD   L_TRN_COUNT               ; Move bot forward by an increment before stopping to check for intersection
                CPD   #1000                     ;
                BLO   FWD_STR_DIS               ; If left counter (used for forward in this instance) > predefined increment then
                JSR   INIT_FWD                  ; Turn motors off
                
FWD_EXIT        JMP   MAIN                      ; return to main

REV_ST          LDD   L_TRN_COUNT               ; If left turn timer (used to time reverse after a front bumper press) is greater than reverse time
                CPD   #1000                     ; u-turn
                BLO   REV_ST                     
                JSR   STARFWD                   ; Set starboard motor direction
                LDD   #0                        ; Reset left turn timer
                STD   L_TRN_COUNT               
                
REV_U_TRN       LDD   L_TRN_COUNT               ; If left counter > turn increment
                CPD   #11500                    ; robot should stop
                BLO   REV_U_TRN                 ; 
                JSR   INIT_FWD                  ; Initialize the FWD state
                LDAA  RETURN                    ; If RETURN equals 1 
                BNE   TURN_BCK_REV               
                MOVB  #FWD,CRNT_STATE           ; change state to FWD
                BRA   REV_EXIT                  ; exit
                
TURN_BCK_REV    JSR   INIT_FWD                  ;
                MOVB  #TURN_BCK,CRNT_STATE      ; set state to TURN_BCK
               
REV_EXIT        RTS                             ; return to main

R_TURN_ST       LDD   R_TRN_COUNT               ; If right counter > increment
                CPD   #1000                     ; The robot should turn
                BLO   R_TURN_ST                 ; 
                JSR   STAROFF                   ; Set starside motor off
                LDD   #0                        ; Reset right timer
                STD   R_TRN_COUNT               ; 
                
R_TURN_CHECK    LDD   R_TRN_COUNT               ; If right counter > turn increment
                CPD   #9000                   ; The robot should stop
                BLO   R_TURN_CHECK              ; 
                JSR   INIT_FWD                  ; Initialize the FWD state
                LDAA  RETURN                    ; If RETURN equals 1 
                BNE   TRN_BCK_R_TRN             ;
                MOVB  #FWD,CRNT_STATE           ; change to FWD
                BRA   R_TURN_EXIT               ; exit
                
TRN_BCK_R_TRN   MOVB  #TURN_BCK,CRNT_STATE      ; set state to TURN_BCK
            
R_TURN_EXIT     RTS                             ; return to main

L_TURN_ST       LDD   L_TRN_COUNT               ; If left counter > increment
                CPD   #1000                     ; robot should make a turn
                BLO   L_TURN_ST                 ; 
                JSR   PORTOFF                   ; Turn off port motor
                LDD   #0                        ; Reset left timer
                STD   L_TRN_COUNT               ; 
                
L_TURN_CHECK    LDD   L_TRN_COUNT               ; If left counter > turn increment
                CPD   #9000                    ; robot should stop
                BLO   L_TURN_CHECK              ; 
                JSR   INIT_FWD                  ; Initialize the FWD state
                LDAA  RETURN                    ; If RETURN equals 1 
                BNE   TRN_BCK_L_TRN             ;
                MOVB  #FWD,CRNT_STATE           ; change to FWD
                BRA   L_TURN_EXIT               ; exit
                
TRN_BCK_L_TRN   MOVB  #TURN_BCK,CRNT_STATE      ; set state to TURN_BACK

L_TURN_EXIT     RTS                             ; return to the MAIN routine

BK_TRK_ST       PULD                            ;
                BRSET PORTAD0,$08,NO_BK_BUMP    ; If REV_BUMP, stop
                JSR   INIT_ALL_STOP             ; Initialize the Stop state
                MOVB  #ALL_STOP,CRNT_STATE      ; set the state to stop
                JMP   TURN_BACK_EXIT            ; Exit

NO_BK_BUMP      LDAA  NEXT_DIR                  ; If NEXT_DIR equals 0
                BEQ   R_T_PATHING               ; Use pathing mode assuming a right turn will be made
                BNE   L_T_PATHING               ; use pathing mode assuming a left turn will be made

R_T_PATHING     LDAA  D_DETECT                  ; For D equals 1
                BEQ   NO_RT_TRN                 ; robot should make a right turn
                PULA                            ; Pull the next direction value from the stack, store it in NEXT_DIR
                PULA                             
                STAA  NEXT_DIR                   
                JSR   INIT_R_TURN               ; Initialize the R_TURN state
                MOVB  #R_TURN,CRNT_STATE        ; change current state
                JMP   TURN_BACK_EXIT            ; exit
                
NO_RT_TRN       LDAA  B_DETECT                  ; For B equals 1
                BEQ   R_LINE_FOLLOW             ; Check if A equals 1
                LDAA  A_DETECT                  ; if so, a forward path exists
                BEQ   LEFT_TURN                 ; The robot should continue forward
                PULA                            ; Pull the next direction value from the stack, store it in NEXT_DIR
                PULA                            
                STAA  NEXT_DIR                    
                BRA   NO_LINE                   ; For A equals 0
                
LEFT_TURN       PULA                            ; robot should make a left turn
                PULA                            ; Pull the next direction from the stack, store it in NEXT_DIR
                STAA  NEXT_DIR                  
                JSR   INIT_L_TURN               ; Initialize L_TURN state
                MOVB  #L_TURN,CRNT_STATE        ; change current state
                JMP   TURN_BACK_EXIT            ; Exit                
                
L_T_PATHING     LDAA  B_DETECT                  ; For B equals 1
                BEQ   NO_LT_TRN                 ; robot should make a left turn
                PULA                            ; Pull the next direction value from the stack, store it in NEXT_DIR
                STAA  NEXT_DIR                   
                JSR   INIT_L_TURN               ; Initialize the L_TURN state
                MOVB  #L_TURN,CRNT_STATE        ; change current state
                JMP   TURN_BACK_EXIT            ; Exit

NO_LT_TRN       LDAA  D_DETECT                  ; For D equals 1
                BEQ   R_LINE_FOLLOW             ; Check if A equals 1
                LDAA  A_DETECT                  ; If so a forward path exists
                BEQ   RIGHT_TURN                ; robot should continue forward
                PULA                            ; Pull the next direction value from the stack, and store it in NEXT_DIR
                STAA  NEXT_DIR                  
                BRA   NO_LINE                   ; For A equals 0
                
RIGHT_TURN      PULA                            ; robot should make a right turn
                STAA  NEXT_DIR                  ; Pull the next direction value from the stack
                JSR   INIT_R_TURN               ; Initialize the R_TURN state
                MOVB  #R_TURN,CRNT_STATE        ; change current state
                JMP   TURN_BACK_EXIT            ; exit
               

R_LINE_FOLLOW   LDAA  F_DETECT                  ; For F equals 1
                BEQ   L_LINE_FOLLOW             ; robot should steer left
                JSR   PORTON                    ; turn on the portside motor
                
FWD_R           LDD   R_TRN_COUNT               ; Go to forward state after a right turn has been made
                CPD   #500                  
                BLO   FWD_R                     ; If right counter > increment
                JSR   INIT_FWD                  ; Turn motors off
                JMP   TURN_BACK_EXIT            ; and exit

L_LINE_FOLLOW   LDAA  E_DETECT                  ; For E equals 1
                BEQ   NO_LINE                   ; robot should steer right
                JSR   STARON                    ; turn on the starside motor
                
FWD_L           LDD   L_TRN_COUNT               ; go to forward state after a left turn has been made
                CPD   #500                  
                BLO   FWD_L                     ; If left counter > increment
                JSR   INIT_FWD                  ; Turn motors off
                JMP   TURN_BACK_EXIT            ; Exit

NO_LINE         JSR   STARON                    ; Turn motors on
                JSR   PORTON                    ; Go forward with no turn logic essentially
                
FWD_STR         LDD   L_TRN_COUNT               ; go forward     
                CPD   #1000                  ;
                BLO   FWD_STR                   ; If left counter > increment
                JSR   INIT_FWD                  ; Turn motors off
                
TURN_BACK_EXIT  JMP   MAIN                      ; return to the MAIN routine

ALL_STOP_ST     BRSET PORTAD0,$04,NO_START      ; If FWD_BUMP
                JSR   INIT_ALL_STOP             ; Initialize the Stop state
                MOVB  #START,CRNT_STATE         ; Set current state to start
                BRA   ALL_STOP_EXIT             ; Then exit
                                                ;
NO_START        NOP                             ; Else
ALL_STOP_EXIT   RTS                             ; return to the MAIN routine

;-------------------------------------------------------------------------------------------
; state initialization
INIT_FWD        BCLR  PTT,%00110000             ; Clear port T pins 4 & 5 to turn off drive motors
                LDD   #0                        ; Reset timer count
                STD   R_TRN_COUNT                    
                STD   L_TRN_COUNT                    
                BCLR  PORTA,%00000011           ; Clear port A pins 0 & 1 to set fwd direction for motors
                RTS

INIT_REV        BSET  PORTA,%00000011           ; Set port A pins 0 & 1 to set rev direction for motors
                LDD   #0                        ; Reset timer count
                STD   L_TRN_COUNT                    
                BSET  PTT,%00110000             ; Set port T pins 4 & 5 to turn on drive motors
                RTS

INIT_R_TURN     BCLR  PORTA,%00000011           ; Clear port A pins 0 & 1 to set fwd direction for motors
                LDD   #0                        ; Reset timer count
                STD   R_TRN_COUNT                    
                BSET  PTT,%00110000             ; Set port T pins 4 & 5 to turn on drive motors
                RTS

INIT_L_TURN     BCLR  PORTA,%00000011           ; Clear port A pins 0 & 1 to set fwd direction for motors
                LDD   #0                        ; Reset timer count
                STD   L_TRN_COUNT                    
                BSET  PTT,%00110000             ; Set port T pins 4 & 5 to turn on drive motors
                RTS

INIT_TURN_BCK   INC   RETURN                    ; Change RETURN value to 1
                PULA                            ; Pull the next direction value from the stack, store it in NEXT_D
                STAA  NEXT_DIR                  
                JSR   INIT_REV                  ; Initialize the reverse routine
                JSR   REV_ST                    ; Jump to REV_ST routine
                JMP   MAIN

INIT_ALL_STOP   BCLR  PTT,%00110000             ; Clear port T pins 4 & 5 to turn off drive motors
                RTS                             ; Return
                
;---------------------------------------------------------------------------
; Guider LEDs ON
; This routine enables the guider LEDs so that readings of the sensor
; correspond to the ’illuminated’ situation.
; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed
G_LEDS_ON       BSET  PORTA,%00100000   ; Set bit 5
                RTS
;
; Guider LEDs OFF
; This routine disables the guider LEDs. Readings of the sensor
; correspond to the ’ambient lighting’ situation.
; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed
G_LEDS_OFF      BCLR  PORTA,%00100000   ; Clear bit 5
                RTS
;---------------------------------------------------------------------------
; Read Sensors
; This routine reads the eebot guider sensors and puts the results in RAM
; registers.
; Note: Do not confuse the analog multiplexer on the Guider board with the
; multiplexer in the HCS12. The guider board mux must be set to the
; appropriate channel using the SELECT_SENSOR routine. The HCS12 always
; reads the selected sensor on the HCS12 A/D channel AN1.
; The A/D conversion mode used in this routine is to read the A/D channel
; AN1 four times into HCS12 data registers ATDDR0,1,2,3. The only result
; used in this routine is the value from AN1, read from ATDDR0. However,
; other routines may wish to use the results in ATDDR1, 2 and 3.
; Consequently, Scan=0, Mult=0 and Channel=001 for the ATDCTL5 control word.
; Passed: None
; Returns: Sensor readings in:
; SENSOR_LINE (0) (Sensor E/F)
; SENSOR_BOW (1) (Sensor A)
; SENSOR_PORT (2) (Sensor B)
; SENSOR_MID (3) (Sensor C)
; SENSOR_STBD (4) (Sensor D)
; Note:
; The sensor number is shown in brackets
;
; Algorithm:
; Initialize the sensor number to 0

; Initialize a pointer into the RAM at the start of the Sensor Array storage
; Loop Store %10000001 to the ATDCTL5 (to select AN1 and start a conversion)
; Repeat
; Read ATDSTAT0
; Until Bit SCF of ATDSTAT0 == 1 (at which time the conversion is complete)
; Store the contents of ATDDR0L at the pointer
; If the pointer is at the last entry in Sensor Array, then
; Exit
; Else
; Increment the sensor number
; Increment the pointer
; Loop again.
READ_SENSORS    CLR   SENSOR_NUM           ; Select sensor number 0
                LDX   #SENSOR_LINE         ; Point at the start of the sensor array
RS_MAIN_LOOP:   LDAA  SENSOR_NUM           ; Select the correct sensor input
                JSR   SELECT_SENSOR        ; on the hardware
                LDY   #400                 ; 20 ms delay to allow the
                JSR   del_50us             ; sensor to stabilize
                LDAA  #%10000001           ; Start A/D conversion on AN1
                STAA  ATDCTL5
                BRCLR ATDSTAT0,$80,*       ; Repeat until A/D signals done
                LDAA  ATDDR0L              ; A/D conversion is complete in ATDDR0L
                STAA  0,X                  ; so copy it to the sensor register
                CPX   #SENSOR_STBD         ; If this is the last reading
                BEQ   RS_EXIT              ; Then exit
                INC   SENSOR_NUM           ; Else, increment the sensor number
                INX                        ; and the pointer into the sensor array
                BRA   RS_MAIN_LOOP         ; and do it again
       RS_EXIT: RTS
;---------------------------------------------------------------------------
; Select Sensor
; This routine selects the sensor number passed in ACCA. The motor direction
; bits 0, 1, the guider sensor select bit 5 and the unused bits 6,7 in the
; same machine register PORTA are not affected.
; Bits PA2,PA3,PA4 are connected to a 74HC4051 analog mux on the guider board,
; which selects the guider sensor to be connected to AN1.
; Passed: Sensor Number in ACCA
; Returns: Nothing
; Side Effects: ACCA is changed
; Algorithm:
; First, copy the contents of PORTA into a temporary location TEMP and clear
; the sensor bits 2,3,4 in the TEMP to zeros by ANDing it with the mask
; 11100011. The zeros in the mask clear the corresponding bits in the
; TEMP. The 1’s have no effect.
; Next, move the sensor selection number left two positions to align it
; with the correct bit positions for sensor selection.
; Clear all the bits around the (shifted) sensor number by ANDing it with
; the mask 00011100. The zeros in the mask clear everything except
; the sensor number.
; Now we can combine the sensor number with the TEMP using logical OR.
; The effect is that only bits 2,3,4 are changed in the TEMP, and these
; bits now correspond to the sensor number.
; Finally, save the TEMP to the hardware.
SELECT_SENSOR   PSHA                         ; Save the sensor number for the moment
                LDAA  PORTA                  ; Clear the sensor selection bits to zeros
                ANDA  #%11100011                
                STAA  TEMP                   ; and save it into TEMP
                PULA                         ; Get the sensor number
                ASLA                         ; Shift the selection number left, twice
                ASLA
                ANDA  #%00011100             ; Clear irrelevant bit positions
                ORAA  TEMP                   ; OR it into the sensor bit positions
                STAA  PORTA                  ; Update the hardware
                RTS
;--------------------------------------------------------------------------------------------
; sensor detection
SENS_DETECT     JSR   G_LEDS_ON                 ; Turn ON LEDS
                JSR   READ_SENSORS              ; Take readings from sensors
                JSR   G_LEDS_OFF                ; Turn OFF LEDS                
                LDAA  #0                        ; Clear detection values
                STAA  A_DETECT                  ; Sensor A
                STAA  B_DETECT                  ; Sensor B
                STAA  C_DETECT                  ; Sensor C
                STAA  D_DETECT                  ; Sensor D
                STAA  E_DETECT                  ; Sensor E
                STAA  F_DETECT                  ; Sensor F
                
CHECK_A         LDAA  SENSOR_BOW                ; Compare SENSOR_BOW is higher than SENSOR_A
                CMPA  #SENSOR_A                  
                BLO   CHECK_B                   ; Check next sensor if clear
                INC   A_DETECT                  ; Set A

CHECK_B         LDAA  SENSOR_PORT               ; Compare SENSOR_PORT is higher than SENSOR_B
                CMPA  #SENSOR_B                 
                BLO   CHECK_C                   ; Check next sensor if clear
                INC   B_DETECT                  ; Set B

CHECK_C         LDAA  SENSOR_MID                ; Compare SENSOR_MID is higher than SESNSOR_C
                CMPA  #SENSOR_C                  
                BLO   CHECK_D                   ; Check next sensor if clear
                INC   C_DETECT                  ; Set C
                
CHECK_D         LDAA  SENSOR_STBD               ; Compare SENSOR_STBD is higher than SENSOR_D
                CMPA  #SENSOR_D                  
                BLO   CHECK_E                   ; Check next sensor if clear
                INC   D_DETECT                  ; Set D

CHECK_E         LDAA  SENSOR_LINE               ; Compare SENSOR_LINE is less than SENSOR_E
                CMPA  #SENSOR_E                 
                BHI   CHECK_F                   ; Check next sensor if clear
                INC   E_DETECT                  ; Set E
                
CHECK_F         LDAA  SENSOR_LINE               ; Compare SENSOR_LINE is higher than SENSOR_F
                CMPA  #SENSOR_F                 ; 
                BLO   UPDATE_COMP               ; Return to main
                INC   F_DETECT                  ; set F
                
UPDATE_COMP     RTS
;--------------------------------------------------------------------------------------------
; subroutine section
;---------------------------------------------------------------------------
; Position the Cursor
; This routine positions the display cursor in preparation for the writing
; of a character or string.
; For a 20x2 display:
; The first line of the display runs from 0 .. 19.
; The second line runs from 64 .. 83.
; The control instruction to position the cursor has the format
; 1aaaaaaa
; where aaaaaaa is a 7 bit address.
; Passed: 7 bit cursor Address in ACCA
; Returns: Nothing
; Side Effects: None
LCD_POS_CRSR    ORAA #%10000000 ; Set the high bit of the control word
                JSR cmd2LCD ; and set the cursor address
                RTS
;--------------------------------------------------------------------------------------------                
del_50us:       PSHX                         ;2 E-clk Protect the X register
eloop:          LDX   #300                   ;2 E-clk Initialize the inner loop counter
iloop:          NOP                          ;1 E-clk No operation
                DBNE  X,iloop                ;3 E-clk If the inner cntr not 0, loop again
                DBNE  Y,eloop                ;3 E-clk If the outer cntr not 0, loop again
                PULX                         ;3 E-clk Restore the X register
                RTS                          ;5 E-clk Else return
;--------------------------------------------------------------------------------------------
cmd2LCD:        BCLR  LCD_CNTR,LCD_RS        ; select the LCD Instruction Register (IR)
                JSR   dataMov                ; send data to IR
      	        RTS
;--------------------------------------------------------------------------------------------
putsLCD         LDAA  1,X+                   ; get one character from the string
                BEQ   donePS                 ; reach NULL character?
                JSR   putcLCD
                BRA   putsLCD
donePS 	        RTS
;--------------------------------------------------------------------------------------------
putcLCD         BSET  LCD_CNTR,LCD_RS        ; select the LCD Data register (DR)
                JSR   dataMov                ; send data to DR
                RTS
;---------------------------------------------------------------------------
; Send data to the LCD IR or DR depending on the RS signal
dataMov         BSET  LCD_CNTR,LCD_E         ; pull the LCD E-sigal high
                STAA  LCD_DAT                ; send the upper 4 bits of data to LCD
                BCLR  LCD_CNTR,LCD_E         ; pull the LCD E-signal low to complete the write oper.
                LSLA                         ; match the lower 4 bits with the LCD data pins
                LSLA                         ; -"-
                LSLA                         ; -"-
                LSLA                         ; -"-
                BSET  LCD_CNTR,LCD_E         ; pull the LCD E signal high
                STAA  LCD_DAT                ; send the lower 4 bits of data to LCD
                BCLR  LCD_CNTR,LCD_E         ; pull the LCD E-signal low to complete the write oper.
                LDY   #1                     ; adding this delay will complete the internal
                JSR   del_50us               ; operation for most instructions
                RTS
                       
initAD          MOVB  #$C0,ATDCTL2           ; power up AD, select fast flag clear
                JSR   del_50us               ; wait for 50 us
                MOVB  #$00,ATDCTL3           ; 8 conversions in a sequence
                MOVB  #$85,ATDCTL4           ; res=8, conv-clks=2, prescal=12
                BSET  ATDDIEN,$0C            ; configure pins AN03,AN02 as digital inputs
                RTS   
;---------------------------------------------------------------------------                
;*****************************************************************
;* Integer to BCD Conversion Routine
;* This routine converts a 16 bit binary number in .D into
;* BCD digits in BCD_BUFFER.
;* Peter Hiscocks
;* Algorithm:
;* Because the IDIV (Integer Division) instruction is available on
;* the HCS12, we can determine the decimal digits by repeatedly
;* dividing the binary number by ten: the remainder each time is
;* a decimal digit. Conceptually, what we are doing is shifting
;* the decimal number one place to the right past the decimal
;* point with each divide operation. The remainder must be
;* a decimal digit between 0 and 9, because we divided by 10.
;* The algorithm terminates when the quotient has become zero.
;* Bug note: XGDX does not set any condition codes, so test for
;* quotient zero must be done explicitly with CPX.
;* Data structure:
;* BCD_BUFFER EQU * The following registers are the BCD buffer area
;* TEN_THOUS RMB 1 10,000 digit, max size for 16 bit binary
;* THOUSANDS RMB 1 1,000 digit
;* HUNDREDS RMB 1 100 digit
;* TENS RMB 1 10 digit
;* UNITS RMB 1 1 digit
;* BCD_SPARE RMB 2 Extra space for decimal point and string terminator

int2BCD         XGDX                         ; Save the binary number into .X
                LDAA  #0                     ; Clear the BCD_BUFFER
                STAA  TEN_THOUS
                STAA  THOUSANDS
                STAA  HUNDREDS
                STAA  TENS
                STAA  UNITS
                STAA  BCD_SPARE
                STAA  BCD_SPARE+1

                CPX   #0                     ; Check for a zero input
                BEQ   CON_EXIT               ; and if so, exit

                XGDX                         ; Not zero, get the binary number back to .D as dividend
                LDX   #10                    ; Setup 10 (Decimal!) as the divisor
                IDIV                         ; Divide: Quotient is now in .X, remainder in .D
                STAB  UNITS                  ; Store remainder
                CPX   #0                     ; If quotient is zero,
                BEQ   CON_EXIT               ; then exit

                XGDX                         ; else swap first quotient back into .D
                LDX   #10                    ; and setup for another divide by 10
                IDIV
                STAB  TENS
                CPX   #0
                BEQ   CON_EXIT

                XGDX                         ; Swap quotient back into .D
                LDX   #10                    ; and setup for another divide by 10
                IDIV
                STAB  HUNDREDS
                CPX   #0
                BEQ   CON_EXIT

                XGDX                         ; Swap quotient back into .D
                LDX   #10                    ; and setup for another divide by 10
                IDIV
                STAB  THOUSANDS
                CPX   #0
                BEQ   CON_EXIT

                XGDX                         ; Swap quotient back into .D
                LDX   #10                    ; and setup for another divide by 10
                IDIV
                STAB  TEN_THOUS

      CON_EXIT: RTS                          ; We’re done the conversion
;---------------------------------------------------------------------------------------------
;****************************************************************
;* BCD to ASCII Conversion Routine
;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag ’NO_BLANK’ starts cleared and is set once a non-zero
;* digit has been detected.
;* The ’units’ digit is never blanked, even if it and all the
;* preceding digits are zero.
;* Peter Hiscocks

BCD2ASC         LDAA  #$0                    ; Initialize the blanking flag
                STAA  NO_BLANK

       C_TTHOU: LDAA  TEN_THOUS              ; Check the ’ten_thousands’ digit
                ORAA  NO_BLANK
                BNE   NOT_BLANK1

      ISBLANK1: LDAA  #$20                   ; It’s blank
                STAA  TEN_THOUS              ; so store a space
                BRA   C_THOU                 ; and check the ’thousands’ digit

    NOT_BLANK1: LDAA  TEN_THOUS              ; Get the ’ten_thousands’ digit
                ORAA  #$30                   ; Convert to ascii
                STAA  TEN_THOUS
                LDAA  #$1                    ; Signal that we have seen a ’non-blank’ digit
                STAA  NO_BLANK

        C_THOU: LDAA  THOUSANDS              ; Check the thousands digit for blankness
                ORAA  NO_BLANK               ; If it’s blank and ’no-blank’ is still zero
                BNE   NOT_BLANK2
                     
      ISBLANK2: LDAA  #$30                   ; Thousands digit is blank
                STAA  THOUSANDS              ; so store a space
                BRA   C_HUNS                 ; and check the hundreds digit

    NOT_BLANK2: LDAA  THOUSANDS              ; (similar to ’ten_thousands’ case)
                ORAA  #$30
                STAA  THOUSANDS
                LDAA  #$1
                STAA  NO_BLANK

        C_HUNS: LDAA  HUNDREDS               ; Check the hundreds digit for blankness
                ORAA  NO_BLANK               ; If it’s blank and ’no-blank’ is still zero
                BNE   NOT_BLANK3

      ISBLANK3: LDAA  #$20                   ; Hundreds digit is blank
                STAA  HUNDREDS               ; so store a space
                BRA   C_TENS                 ; and check the tens digit
                     
    NOT_BLANK3: LDAA  HUNDREDS               ; (similar to ’ten_thousands’ case)
                ORAA  #$30
                STAA  HUNDREDS
                LDAA  #$1
                STAA  NO_BLANK

        C_TENS: LDAA  TENS                   ; Check the tens digit for blankness
                ORAA  NO_BLANK               ; If it’s blank and ’no-blank’ is still zero
                BNE   NOT_BLANK4
                     
      ISBLANK4: LDAA  #$20                   ; Tens digit is blank
                STAA  TENS                   ; so store a space
                BRA   C_UNITS                ; and check the units digit

    NOT_BLANK4: LDAA  TENS                   ; (similar to ’ten_thousands’ case)
                ORAA  #$30
                STAA  TENS

       C_UNITS: LDAA  UNITS                  ; No blank check necessary, convert to ascii.
                ORAA  #$30
                STAA  UNITS

                RTS                          ; We’re done
;---------------------------------------------------------------------------
; Binary to ASCII
; Converts an 8 bit binary value in ACCA to the equivalent ASCII character 2
; character string in accumulator D
; Uses a table-driven method rather than various tricks.
; Passed: Binary value in ACCA
; Returns: ASCII Character string in D
; Side Fx: ACCB is destroyed

HEX_TABLE       FCC '0123456789ABCDEF'       ; Table for converting values
BIN2ASC         PSHA                         ; Save a copy of the input number on the stack
                TAB                          ; and copy it into ACCB
                ANDB #%00001111              ; Strip off the upper nibble of ACCB
                CLRA                         ; D now contains 000n where n is the LSnibble
                ADDD #HEX_TABLE              ; Set up for indexed load
                XGDX                
                LDAA 0,X                     ; Get the LSnibble character
                PULB                         ; Retrieve the input number into ACCB
                PSHA                         ; and push the LSnibble character in its place
                RORB                         ; Move the upper nibble of the input number
                RORB                         ;  into the lower nibble position.
                RORB
                RORB 
                ANDB #%00001111              ; Strip off the upper nibble
                CLRA                         ; D now contains 000n where n is the MSnibble 
                ADDD #HEX_TABLE              ; Set up for indexed load
                XGDX                                                               
                LDAA 0,X                     ; Get the MSnibble character into ACCA
                PULB                         ; Retrieve the LSnibble character into ACCB
                RTS

;---------------------------------------------------------------------------------------------
;Update Display   
UPDT_DISPL      LDAA  #$82                      ; Move LCD cursor to the end of msg1
                JSR   cmd2LCD                   ;
                
                LDAB  CRNT_STATE                ; Display current state
                LSLB                            ; "
                LSLB                            ; "
                LSLB                            ; "
                LDX   #tab                      ; "
                ABX                             ; "
                JSR   putsLCD                   ; "               
                LDAA  #$8F                      ; Move LCD cursor to the end of msg2
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_BOW                ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""
                LDAA  #$92                      ; Move LCD cursor to Line position 
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_LINE               ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""
                LDAA  #$CC                      ; Move LCD cursor to Port position on 2nd row 
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_PORT               ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""
                LDAA  #$CF                      ; Move LCD cursor to Mid position on 2nd row 
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_MID                ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""
                LDAA  #$D2                      ; Move LCD cursor to Starboard position on 2nd row 
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_STBD               ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""           
                MOVB  #$90,ATDCTL5              ; R-just., uns., sing. conv., mult., ch=0, start
                BRCLR ATDSTAT0,$80,*            ; Wait until the conver. seq. is complete
                LDAA  ATDDR0L                   ; Load the ch0 result - battery volt - into A
                LDAB  #39                       ; AccB = 39
                MUL                             ; AccD = 1st result x 39
                ADDD  #600                      ; AccD = 1st result x 39 + 600
                JSR   int2BCD
                JSR   BCD2ASC
                LDAA  #$C2                      ; move LCD cursor to the end of msg3
                JSR   cmd2LCD                   ; "                
                LDAA  TEN_THOUS                 ; output the TEN_THOUS ASCII character
                JSR   putcLCD                   ; "
                LDAA  THOUSANDS                 ; output the THOUSANDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  #$2E                      ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  HUNDREDS                  ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "                
                RTS
                                
;-----------------------------------------------------------------------------------------
; Service routine for left turn counter
ISR1            MOVB  #$01,TFLG1             ; clear the C0F input capture flag
                INC   L_TRN_COUNT            ; increment left counter
                RTI
;---------------------------------------------------------------------------------------------
; Service routine for right turn counter
ISR2            MOVB  #$02,TFLG1             ; clear the C1F input capture flag
                INC   R_TRN_COUNT            ; increment right counter 
                RTI
;---------------------------------------------------------------------------------------------
; Interrupt Vectors                                                                        
                ORG   $FFFE
                DC.W  Entry                  ; Reset Vector
                ORG   $FFEE
                DC.W  ISR1                   ; initialize left turn counter
                ORG   $FFEC
                DC.W  ISR2                   ; initialize right turn counter