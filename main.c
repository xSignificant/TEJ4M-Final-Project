#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <cp0defs.h>

#define SYS_CLK_FREQUENCY           (8000000ul)
#define PB_CLK_FREQUENCY            SYS_CLK_FREQUENCY/1ul
#define TICKS_PER_MILLI_SECOND        ((SYS_CLK_FREQUENCY/2ul)/1000ul)

// Optosensor Trip Thresholds
// edited to 100, only need to use one of the values.

#define LEFT_THRESHOLD      100
#define MIDDLE_THRESHOLD    100
#define RIGHT_THRESHOLD     100

void Initialize(void); // Initialize peripheral SFRs and global variables
void Get_Inputs(void); // Read Peripheral, PIN and/or variable states
void Decide(void); // Make decisions on the inputs to manipulate global variables
void Do_Outputs(void); // Output data onto the pins or into the peripherals
void Timing(void); // Determines how fast the control loop executes
void CoreTimerStart(uint16_t DelayValueInMilliseconds); // Reset Core Timer for a 1-65536 mS delay
int CoreTimerIsFinished(void); // Poll Core Timer for Interrupt Flag
void msDelay(uint16_t DelayValueInMilliseconds); // Provide a blocking delay of up to 65535 mS
#pragma config FNOSC = FRC
#pragma config FPBDIV = DIV_1
#pragma config POSCMOD = OFF
#pragma config FSOSCEN = OFF
#pragma config FWDTEN = OFF
#pragma config ICESEL = ICS_PGx1
#pragma config JTAGEN = OFF
#pragma config IOL1WAY = OFF

// values
int switch_level; // used to save the state of the USER SWITCH
uint16_t pot_value; // used to save the value of the potentiometer
uint16_t left_opto; // RB12/AN12: Left Opto Sensor Signal
uint16_t middle_opto; // RB3/AN5: Middle Opto Sensor Signal
uint16_t right_opto; // RB2/AN4: Right Opto Sensor Signal

int main(void) {
    Initialize();
    // motor
    OC4RS = 100;
    OC2RS = 100;

    while (1) {
        Get_Inputs();
        Decide();
        // todo: do we even need do outputs?
        // Do_Outputs();
        Timing();
    }

}

// not edited,
void Get_Inputs(void) {
    // Read USER SWITCH (S1) level (0 = pressed, 1 = not pressed)
    switch_level = PORTAbits.RA1;

    // Read USER POT (R1) voltage
    AD1CHSbits.CH0SA = 0; // select analog input channel RA0/AN0
    AD1CON1bits.SAMP = 1; // start sampling
    while (!AD1CON1bits.DONE); // wait to complete conversion
    pot_value = ADC1BUF0; // read/save the conversion result

    // Sample Optosensor Inputs

    // LEFT (RB12/AN12))
    AD1CHSbits.CH0SA = 12; // select analog input channel RB12/AN12
    AD1CON1bits.SAMP = 1; // start sampling
    while (!AD1CON1bits.DONE); // wait to complete conversion
    left_opto = ADC1BUF0; // read/save the conversion result

    // MIDDLE (RB3/AN5))
    AD1CHSbits.CH0SA = 5; // select analog input channel RB3/AN5
    AD1CON1bits.SAMP = 1; // start sampling
    while (!AD1CON1bits.DONE); // wait to complete conversion
    middle_opto = ADC1BUF0; // read/save the conversion result

    // RIGHT (RB2/AN4))
    AD1CHSbits.CH0SA = 4; // select analog input channel RB2/AN4
    AD1CON1bits.SAMP = 1; // start sampling
    while (!AD1CON1bits.DONE); // wait to complete conversion
    right_opto = ADC1BUF0; // read/save the conversion result
}


// member var: holds the number of times the bot has detected bbb
int count = 0;
// did we reach the end within the last run
int reached = 0; // hood boolean

void Decide(void) {
    int t = LEFT_THRESHOLD; // easier than typing it all out

    // if we reached the end do a 180
    if (reached == 1) {
        forward();
        msDelay(500);
        turnRight();
        msDelay(3000);
        stop();

        // we have turned now.
        reached = 0;
    }

    // bbb
    if (left_opto < t && middle_opto < t && right_opto < t) {
        count++;

        // first time dont count
        if (count == 1) {
            forward();
        } else {
            reached = 1;
        }
        return;
    }

        //www
    else if (left_opto > t && middle_opto > t && right_opto > t) {
        forward();
    }
        //wbw
    else if (left_opto > t && middle_opto < t && right_opto > t) {
        forward();
    }
        //bww
    else if (left_opto < t && middle_opto > t && right_opto > t) {
        turnLeft();
    }
        //wwb
    else if (left_opto > t && middle_opto > t && right_opto < t) {
        turnRight();
    }

    // No, we didnt reach the end
    reached = 0;
}

// not edited, not used

void Do_Outputs(void) {
    // Add a switch statement to make the robot to move forward, backward or
    // stop or to turn left or right, or make a 180 degree turn.

    // RIGHT MOTOR (MOT1) - forward movement
    LATBbits.LATB15 = 0; // 1A
    LATBbits.LATB14 = 1; // 2A
    // LEFT MOTOR (MOT2) - forward movement
    LATBbits.LATB6 = 0; // 3A
    LATBbits.LATB7 = 1; // 4A


    // Modify Motor torque (PWM Duty Cycle) based on potentiometer setting
    OC4RS = 255; // Right motor: 0-511 scale for duty cycle
    OC2RS = 255; // Left motor: 0-511 scale for duty cycle
}

// movement
// swap numbers for taimoor&cole

void turnRight(void) {
    //TURN RIGHT//
    //RIGHT - reverse
    LATBbits.LATB15 = 1; // 1A
    LATBbits.LATB14 = 0; // 2A
    //LEFT - forward
    LATBbits.LATB6 = 0; // 3A
    LATBbits.LATB7 = 1; // 4A
}

void turnLeft(void) {
    //TURN LEFT/
    //RIGHT - forward
    LATBbits.LATB15 = 0; // 1A
    LATBbits.LATB14 = 1; // 2A
    //LEFT - reverse
    LATBbits.LATB6 = 1; // 3A
    LATBbits.LATB7 = 0; // 4A
}

void forward(void) {
    //GO FORWARD//
    //RIGHT - forward
    LATBbits.LATB15 = 1; // 1A
    LATBbits.LATB14 = 0; // 2A
    // LEFT - forward
    LATBbits.LATB6 = 1; // 3A
    LATBbits.LATB7 = 0; // 4A


}

void backward(void) {
    //GO BACKWARD//
    //RIGHT - reverse
    LATBbits.LATB15 = 0; // 1A
    LATBbits.LATB14 = 1; // 2A
    //LEFT - reverse
    LATBbits.LATB6 = 0; // 3A
    LATBbits.LATB7 = 1; // 4A


}

void stop(void) {
    //STOP//
    //RIGHT - brake
    LATBbits.LATB15 = 0; // 1A
    LATBbits.LATB14 = 0; // 2A
    //LEFT - brake
    LATBbits.LATB6 = 0; // 3A
    LATBbits.LATB7 = 0; // 4A
}

// built in methods
// dont edit these

void Initialize(void) {
    // Set up PPS for all I/O in this application
    // U2RX <-- RPB11   (DEBUG PORT PC-TX pin)
    // U2TX --> RPB10   (DEBUG PORT PC-RX pin)
    // OC2 --> RPB5     (LEFT MOTOR PWM signal)
    // OC4 --> RPB13    (RIGHT MOTOR PWM signal)

    // PPS unlock sequence
    SYSKEY = 0x0;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    CFGCONbits.IOLOCK = 0; // unlock PPS registers for writing

    // modify the PPS registers for the application (per table 11-2 in data sheet)
    U2RXR = 3; // Map RPB11 to U2RX
    RPB10R = 2; // Map U2TX to RPB10
    RPB5R = 5; // Map OC2 to RPB5
    RPB13R = 5; // Map OC4 to RPB13

    // PPS re-lock sequence
    CFGCONbits.IOLOCK = 1;
    SYSKEY = 0x0;

    // Initialize UART2 for use as the DEBUG PORT for printf() messages)
    U2BRG = 12; // Baud Rate generator set to 38400 baud

    // Initialize Analog & Digital I/O Pins

    // LED D6 (RA2) - "USER LED" - Digital Output
    LATAbits.LATA2 = 0; // Initial level to place on pin
    TRISAbits.TRISA2 = 0; // Make the pin a digital output

    // Switch S1 (RA1) - "USER SWITCH" - Digital Input
    ANSELAbits.ANSA1 = 0; // AN1/RA1 configured as "digital" input
    TRISAbits.TRISA1 = 1; // Digital I/O configured as digital input

    // Potentiometer R1 (RA0) - "USER POT" - Analog Input
    ANSELAbits.ANSA0 = 1; // AN0/RA0 configured as "analog" input
    // LEFT Optosensor U3 (RB12/AN12)
    ANSELBbits.ANSB12 = 1; // RB12/AN12 configured as "analog" input
    // MIDDLE Optosensor U2 (RB3/AN5)
    ANSELBbits.ANSB3 = 1; // RB3/AN5 configured as "analog" input
    // RIGHT Optosensor U1 (RB2/AN4)
    ANSELBbits.ANSB2 = 1; // RB2/AN4 configured as "analog" input

    // Initialize Analog-to-Digital Converter
    AD1CON1 = 0x00E0; // auto convert after end of sampling
    AD1CSSL = 0; // no scanning required
    AD1CON2 = 0; // use MUXA, AVss/AVdd used as Vref-/+
    AD1CON3 = 0x1F3F; // max sample time = 31Tad
    AD1CON1SET = 0x8000; // turn on the ADC

    // Read USER POT (R1) voltage
    AD1CHSbits.CH0SA = 0; // select analog input channel RA0/AN0
    AD1CON1bits.SAMP = 1; // start sampling
    while (!AD1CON1bits.DONE); // wait to complete conversion
    pot_value = ADC1BUF0; // read/save the conversion result

    // Initialize I/O and Peripherals to drive "3-4EN" PWM signal
    // Drives LEFT MOTOR

    //    L293DNE Truth Table
    //    3-4EN   3A  4A  Motor Action
    //        1   0   0   Brake (low side)
    //        1   0   1   Forward
    //        1   1   0   Reverse
    //        1   1   1   Brake (high side)
    //        0   x   x   Coast

    // Initialize "3-4EN" PWM control output (RB5/OC2 - see PPS mapping table)

    TRISBbits.TRISB5 = 0; // Make RB5 digital O/P
    LATBbits.LATB5 = 0; // Initialize the pin voltage level

    // Initialize "3A" control output (RB6)

    TRISBbits.TRISB6 = 0; // Make RB6 digital O/P
    LATBbits.LATB6 = 0; // Initialize the pin voltage level for "Forward"

    // Initialize "4A" control output (RB7)

    TRISBbits.TRISB7 = 0; // Make RB7 digital O/P
    LATBbits.LATB7 = 1; // Initialize the pin voltage level for "Forward"

    // Initialize Output Compare 2 (OC2)
    // Drive LEFT MOTOR via PWM signal on "3-4EN"
    // We want to create 61.04Hz PWM frequency @ 1024 bits resolution

    OC2CONbits.ON = 0; // Disable the OC module
    OC2R = pot_value; // Write the duty cycle for the 1st PWM pulse
    OC2CONbits.OCTSEL = 0; // Select Timer 2 as the OC time base
    OC2CONbits.OCM = 0b110; // Select the OC mode (Edge PWM)
    OC2CONbits.ON = 1; // Enable the OC module

    // Initialize I/O and Peripherals to drive "1-2EN" PWM signal
    // Drives RIGHT MOTOR

    //    L293DNE Truth Table
    //    1-2EN   1A  2A  Motor Action
    //        1   0   0   Brake (low side)
    //        1   0   1   Forward
    //        1   1   0   Reverse
    //        1   1   1   Brake (high side)
    //        0   x   x   Coast

    // Initialize "1-2EN" PWM control output (RB13/OC4 - see PPS mapping table)

    TRISBbits.TRISB13 = 0; // Make RB13 digital O/P
    LATBbits.LATB13 = 0; // Initialize the pin voltage level

    // Initialize "1A" control output (RB15/AN9)

    TRISBbits.TRISB15 = 0; // Make RB15 digital O/P
    LATBbits.LATB15 = 0; // Initialize the pin voltage level for "Forward"

    // Initialize "2A" control output (RB14/AN10)

    TRISBbits.TRISB14 = 0; // Make RB14 digital O/P
    LATBbits.LATB14 = 1; // Initialize the pin voltage level for "Forward"

    // Initialize Output Compare 4 (OC4)
    // Drive RIGHT MOTOR via PWM signal on "1-2EN"
    // We want to create 61.04Hz PWM frequency @ 1024 bits resolution

    OC4CONbits.ON = 0; // Disable the OC module
    OC4R = pot_value; // Write the duty cycle for the 1st PWM pulse
    OC4CONbits.OCTSEL = 0; // Select Timer 2 as the OC time base
    OC4CONbits.OCM = 0b110; // Select the OC mode (Edge PWM)
    OC4CONbits.ON = 1; // Enable the OC module

    // Initialize and enable Timer 2 to create a 61.04Hz PWM frequency for both PWM channels
    // Duty cycle resolution  = 512 steps (storing 0-511 into OCxRS correspond to 0-100%)

    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal peripheral bus clock (PBCLK)
    T2CONbits.TGATE = 0; // Disable Gated Timer Mode
    T2CONbits.TCKPS = 7; // Select 1:256 prescale (31.250kHz)
    TMR2 = 0; // Clear timer register
    PR2 = 512; // Load the period register

    IFS0bits.T2IF = 0; // Clear Timer 2 interrupt flag
    T2CONbits.TON = 1; // Start timer (starts both PWMs)


}

void Timing(void) {
    // 100mS Super Loop Execution rate
    msDelay(250);
}

void CoreTimerStart(uint16_t DelayValueInMilliseconds) {
    uint32_t CoreTimerDelayVal = DelayValueInMilliseconds * TICKS_PER_MILLI_SECOND;
    // clear the Core Timer COUNT register
    __builtin_mtc0(_CP0_COUNT, _CP0_COUNT_SELECT, 0x00000000u);
    // initialize Core Timer period register COMPARE
    __builtin_mtc0(_CP0_COMPARE, _CP0_COMPARE_SELECT, CoreTimerDelayVal);
    // clear the Core Timer Interrupt Flag
    IFS0bits.CTIF = 0;
}

int CoreTimerIsFinished(void) {
    if (IFS0bits.CTIF) {
        return 1;
    } else {
        return 0;
    }
}

void msDelay(uint16_t DelayValueInMilliseconds) {
    CoreTimerStart(DelayValueInMilliseconds);
    while (!CoreTimerIsFinished());
}