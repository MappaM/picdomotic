/**
 * @author Tom Barbette <tom.barbette@ulg.ac.be>
 * www.tombarbette.be
 * GPLv2
 */

#include <xc.h>
#include <plib/adc.h>
#include <plib/usart.h>
#include <stdlib.h>

//Configuration options

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 22        // Brown Out Reset Voltage bits (VBOR set to 3.0 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bit (WDT is always enabled. SWDTEN bit has no effect)
#pragma config WDTPS = 1024    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = OFF       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (001000-001FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (001000-001FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (001000-001FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)
#define _XTAL_FREQ 1000000


//Ports assignements
#define LED PORTAbits.RA2
#define BTN PORTBbits.RB3

//Relays. Just for info, theses defines are not used
#define RELAY_0 PORTCbits.RC0
#define RELAY_1 PORTCbits.RC1
#define RELAY_2 PORTCbits.RC2
#define RELAY_3 PORTCbits.RC3
#define RELAY_4 PORTBbits.RB0
#define RELAY_5 PORTBbits.RA7
#define RELAY_6 PORTBbits.RA6
#define RELAY_7 PORTBbits.RA5
#define RELAY_8 PORTBbits.RA4

void setMask(unsigned short data);

unsigned short globdata; //Actual bitmask for the relay. A bit set to one enables the relay
unsigned short globdata2; //A second bitmask to set if the user do a long press on the button (usually to disable all equipment, but it's usefull if you want to keep a security camera on for example)
unsigned short globumask; //Bitmask to invert some bits. For example some relay use PNP transistors, so setting the output to 1 will in fact disable the relay...

/**
 * Read default values from EEPROM (can be written using some commands)
 */
void setDefault() {
    globdata = (EEPROM_READ(0x00) << 8) | EEPROM_READ(0x01);
    globumask = (EEPROM_READ(0x02) << 8) | EEPROM_READ(0x03) ;
    globdata2 = (EEPROM_READ(0x04) << 8) | EEPROM_READ(0x05) ;
}


/* Do not use : block uart !
 * Old delay function using the watchdog timer, allowing to do a real SLEEP with less power. But it blocked UART.
 * As the PIC is throttled to 1MHZ, it didn't gained much power anyway...
 *
void __delay_s(int t) {
    WDTCONbits.SWDTEN = 1;
    SLEEP();
    WDTCONbits.SWDTEN = 0;
    CLRWDT();
}*/

 
void main() {
    
     // Set the OSCILLATOR Control Register to 1 MHz
    OSCCONbits.IRCF2 = 0;    
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    
    //Set the 2 temperature sensor, the button, the RX UART as input
    TRISC = 0b10000000;
    TRISB = 0b00001000;
    TRISA = 0b00001011;
    
    //Enable SBOR, used to reset if voltage drops too much
    SBOREN = 1;
    
    //Enable FVR, used to get the fixed value of the FVR (1.2Volts) with the FDC. Usefull as we cannot assume the exactiveness of VDC
    FVREN = 1;
    
    //Value for Low Voltage Detection
    HLVDL3 = 1;
    HLVDL2 = 0;
    HLVDL1 = 1;
    HLVDL0 = 0;
    
    //Enable voltage detectio to interrupt if voltage drops
    VDIRMAG = 0;
    HLVDEN = 1;
    HLVDIF = 0;
    HLVDIE = 1;

    //Blink if the button is pressed
    while (BTN == 1) {
             __delay_ms(50);
             LED = 1;
             __delay_ms(50);
             LED = 0;
             CLRWDT();
    }
    
    //Set default values read from eeprom
    setDefault();

    //Get last value saved in EEPROM
    unsigned short lastv = (EEPROM_READ(0x06) << 8) | EEPROM_READ(0x07) ;
    setMask(lastv);
    
    //Enable UART
    unsigned int spbrg = (unsigned int)( ((float)1 * 1.0e6) /(4.0*(float)9600) + 0.5 - 1.0);
    OpenUSART( USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT &
             USART_CONT_RX & USART_BRGH_HIGH, spbrg );
    BAUDCONbits.BRG16 = 1;
        
    //Clear the watchdog as we could have already taken some time
    CLRWDT();
    
    Delay10KTCYx(1); // small 4x0.0125 s delay to allow communication speed to stabilize

    //Delay?Watchdog. Do it too much is better than resetting...
    CLRWDT();
    
    //Enable ADC
    OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_0ANA);
    PIR1bits.ADIF=0;
  
    //Enable interrupt
    INTCONbits.PEIE=1;
    PIE1bits.ADIE=0;
  
    //Enable the interrupt systems
    ei();

    //Never exit this program...
    while (1) {
        int i = 0;
        
        //Count the number of milliseconds the user press the button
        while (BTN == 1) {
            i ++;
            __delay_ms(1);
            LED = (i % 4) == 0;
            if (i > 1000)
                break;
        }
        
        if (i > 800) { //Long pressed, set the second mode 
            globdata = globdata2;
            setMask(globdata);
            LED = 1;
            for (int i =0; i < 5; i++)
                __delay_ms(100);
            LED = 0;
        } else if (i > 5) { //Small pression, revert to first default mode
            setDefault();
            setMask(globdata);
            LED = 1;
            for (int i =0; i < 5; i++)
                __delay_ms(100);
            LED = 0;
        }
        
        //If users stay on button after action
         while (BTN == 1 && i > 5) { //Blink more slowly to let him know it should stop pressing the button.
             __delay_ms(100);
             LED = 1;
             __delay_ms(100);
             LED = 0;

        }
        CLRWDT();
    }
    
}


unsigned char ADCStringVal[4];

/*
 * Read a short value from UART
 */
unsigned short ReadSHORT() {
     unsigned short d = 0;
     d = ReadUSART() << 8;
     while (!DataRdyUSART());
     d |= ReadUSART();
     return d;
}

/*
 * Read temperature from an ADC channel connected to LM35
 * @param channel the channel index
 */
 float readtemp(int channel) {
     SetChanADC(ADC_CH15);
     ConvertADC();
     while(BusyADC());
     int vfor12 = ReadADC();

     SetChanADC(channel);
     ConvertADC();
     while(BusyADC());
     int vadc = ReadADC();

     float v = (((120/(float)vfor12) * (float)vadc));
     return v;
 }

/**
 *  The interrupt handler. Eaither Low voltage detect or data available from UART
 */
 void interrupt interruptHandler()
 {
     if (HLVDIF == 1) {
        //Just blink 5 times...
         for (int i =0 ; i <5; i++) {
             __delay_ms(100);
             LED = 1;
             __delay_ms(100);
             LED = 0;
             CLRWDT();
         }
         HLVDIF = 0;
     }
     else if (PIR1bits.RCIF) { //Set mask
        char command = ReadUSART();
        LED = 1; //Enable led to let know we received the command
        switch (command) {
            case 'S': //Set a bitmask directly received from USART
                while (!DataRdyUSART());
                unsigned short device_mask = ReadSHORT();
                setMask(device_mask);
                break;
            case 'G': //Get the actual bitmask and write it to the USART
                while (BusyUSART());
                char sg[65];
                utoa(&sg,globdata,2);
                putsUSART(sg);
                break;
            case 'H': //idem but give the inverting mask (H is after G on keyboard...)
                while (BusyUSART());
                char sg[65];
                utoa(&sg,globumask,2);
                putsUSART(sg);
                break;
            case 'I': //idem but give the default value (I is after H in alphabet. I was sleepy...)
                while (BusyUSART());
                unsigned short defaut = (EEPROM_READ(0x00) << 8) | EEPROM_READ(0x01);
                char sg[65];
                utoa(&sg,defaut,2);
                putsUSART(sg);
                break;
            case 'J': //Idem but gibe the alternative default value (well...)
                while (BusyUSART());
                unsigned short defaut = (EEPROM_READ(0x04) << 8) | EEPROM_READ(0x05);
                char sg[65];
                utoa(&sg,defaut,2);
                putsUSART(sg);
                break;
            case 'T': //Return Temperature
                while (BusyUSART());
                int s;
                putsUSART(ftoa(readtemp(ADC_CH0),&s)); //convert to string
                break;
            case 't': //Return Temperature of second sensor
                while (BusyUSART());
                int s;
                putsUSART(ftoa(readtemp(ADC_CH1) + 3.0f,&s)); //We add 3.0f because it's behind a 4m cable... SHould use an AOP but it's simpler to strehten the voltage with a capacitor and calculate the loss due to the resistance of the long cable
                break;
            case 'D': //Set the Default value
                while (!DataRdyUSART());
                char data = ReadUSART();
                EEPROM_WRITE(0x00,data);
                while (!DataRdyUSART());
                data = ReadUSART();
                EEPROM_WRITE(0x01,data);
                break;
            case 'K': //set the second default value (don't remember why "K")
                while (!DataRdyUSART());
                char data = ReadUSART();
                EEPROM_WRITE(0x04,data);
                while (!DataRdyUSART());
                data = ReadUSART();
                EEPROM_WRITE(0x05,data);
                break;
            case 'U': //set the Umask, used to invert the bits for low-enabled device
                while (!DataRdyUSART());
                char data = ReadUSART();
                EEPROM_WRITE(0x02,data);
                while (!DataRdyUSART());
                data = ReadUSART();
                EEPROM_WRITE(0x03,data);
                globumask = (EEPROM_READ(0x02) << 8) | EEPROM_READ(0x03) ;
                setMask(globdata);
                break;
            case 'A': //enable a device, or multiple at once using added power of two (A for Allumer in french)
                while (!DataRdyUSART());
                unsigned short dev = ReadSHORT();
                setMask(globdata | dev);
                break;
            case 'E': //disable a device (E pour Eteindre in french)
                 while (!DataRdyUSART());
                unsigned short dev = ReadSHORT();
                setMask(globdata & ~dev);
                break;
            default:
                while (BusyUSART());
                putsUSART("Unknow command");
        }
        LED = 0; //Disable LED
        PIR1bits.RCIF = 0; //Clean interrupt
    }
}


 void setMask(unsigned short data) {
    //Write the last setted value in EEPROM. If we have a power loss, it will be setted back in the same state. Let's hope it wasn't due to a short circuit...
    EEPROM_WRITE(0x06,data >> 8);
    while (WR);
    EEPROM_WRITE(0x07,data);

    globdata = data;
    unsigned short out = data ^ globumask;
    PORTC = (0x0f & out);
    PORTBbits.RB0 = (out >> 4) & 0x1;
    PORTAbits.RA7 = (out >> 5) & 0x1; //Stupidly inverted bits for RA4-7... But now I'm stuck with it
    PORTAbits.RA6 = (out >> 6) & 0x1;
    PORTAbits.RA5 = (out >> 7) & 0x1;
    PORTAbits.RA4 = (out >> 8) & 0x1;
 }


