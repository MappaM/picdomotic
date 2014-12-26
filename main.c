#include <xc.h>
#include <plib/adc.h>
#include <plib/usart.h>
#include <stdlib.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

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

#define LED PORTAbits.RA2
#define BTN PORTBbits.RB3

void setMask(unsigned short data);

unsigned short globdata;
unsigned short globdata2;
unsigned short globumask;


void setDefault() {
    globdata = (EEPROM_READ(0x00) << 8) | EEPROM_READ(0x01);
    globumask = (EEPROM_READ(0x02) << 8) | EEPROM_READ(0x03) ;
    globdata2 = (EEPROM_READ(0x04) << 8) | EEPROM_READ(0x05) ;
}


/* Do not use : block uart !
void __delay_s(int t) {
    WDTCONbits.SWDTEN = 1;
    SLEEP();
    WDTCONbits.SWDTEN = 0;
    CLRWDT();
}*/
 int chan = 0;
void main() {
    OSCCONbits.IRCF2 = 0;     // Set the OSCILLATOR Control Register to 1 MHz
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    TRISC = 0b10000000;
    TRISB = 0b00001000;
    TRISA = 0b00001011;
    SBOREN = 1;
    FVREN = 1;
    HLVDL3 = 1;
    HLVDL2 = 0;
    HLVDL1 = 1;
    HLVDL0 = 0;
    VDIRMAG = 0;
    HLVDEN = 1;
    HLVDIF = 0;
    HLVDIE = 1;

    while (BTN == 1) {
             __delay_ms(50);
             LED = 1;
             __delay_ms(50);
             LED = 0;
             CLRWDT();

    }
    setDefault();

    unsigned short lastv = (EEPROM_READ(0x06) << 8) | EEPROM_READ(0x07) ;
    setMask(lastv);
    
    unsigned int spbrg = (unsigned int)( ((float)1 * 1.0e6) /(4.0*(float)9600) + 0.5 - 1.0);
    OpenUSART( USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT &
             USART_CONT_RX & USART_BRGH_HIGH, spbrg );
    BAUDCONbits.BRG16 = 1;  // needed so we can use a 16-bit spbrg
        // Note that this is not discussed in the c18 Compiler Libraries guide
    CLRWDT();
  Delay10KTCYx(1); // small 4x0.0125 s delay to allow communication speed to stabilize
                    // part of the C18 delays.h library

CLRWDT();
  OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_0ANA);
  PIR1bits.ADIF=0;
  INTCONbits.PEIE=1;
  PIE1bits.ADIE=0;
  
  ei();


    while (1) {
        int i = 0;
        
        while (BTN == 1) {
            i ++;
            __delay_ms(1);
            LED = (i % 4) == 0;
            if (i > 1000)
                break;
        }
        if (i > 800) { //Button pressed
            globdata = globdata2;
            setMask(globdata);
            LED = 1;
            for (int i =0; i < 5; i++)
                __delay_ms(100);
            LED = 0;
        } else if (i > 5) {
            setDefault();
            setMask(globdata);
            LED = 1;
            for (int i =0; i < 5; i++)
                __delay_ms(100);
            LED = 0;
        }
        
        //If users stay on button after action
         while (BTN == 1 && i > 5) {
             __delay_ms(100);
             LED = 1;
             __delay_ms(100);
             LED = 0;

         }
        CLRWDT();
    }
}
  unsigned char ADCStringVal[4];

 unsigned short ReadSHORT() {
     unsigned short d = 0;
     d = ReadUSART() << 8;
     while (!DataRdyUSART());
     d |= ReadUSART();
     return d;
 }

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

 void interrupt interruptHandler()
{
     if (HLVDIF == 1) {

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
        LED = 1;
        switch (command) {
            case 'S':
                while (!DataRdyUSART());
                unsigned short device_mask = ReadSHORT();
                setMask(device_mask);
                break;
            case 'G':
                while (BusyUSART());
                char sg[65];
                utoa(&sg,globdata,2);
                putsUSART(sg);
                break;
            case 'H':
                while (BusyUSART());
                char sg[65];
                utoa(&sg,globumask,2);
                putsUSART(sg);
                break;
            case 'I':
                while (BusyUSART());
                unsigned short defaut = (EEPROM_READ(0x00) << 8) | EEPROM_READ(0x01);
                char sg[65];
                utoa(&sg,defaut,2);
                putsUSART(sg);
                break;
            case 'J':
                while (BusyUSART());
                unsigned short defaut = (EEPROM_READ(0x04) << 8) | EEPROM_READ(0x05);
                char sg[65];
                utoa(&sg,defaut,2);
                putsUSART(sg);
                break;
            case 'T':
                 while (BusyUSART());
                int s;
                putsUSART(ftoa(readtemp(ADC_CH0),&s)); //convert to string
                break;
            case 't':
                while (BusyUSART());
                int s;
                putsUSART(ftoa(readtemp(ADC_CH1) + 3.0f,&s)); //convert to string
                break;
            case 'D':
                while (!DataRdyUSART());
                char data = ReadUSART();
                EEPROM_WRITE(0x00,data);
                while (!DataRdyUSART());
                data = ReadUSART();
                EEPROM_WRITE(0x01,data);
                break;
            case 'K':
                while (!DataRdyUSART());
                char data = ReadUSART();
                EEPROM_WRITE(0x04,data);
                while (!DataRdyUSART());
                data = ReadUSART();
                EEPROM_WRITE(0x05,data);
                break;
            case 'U':
                while (!DataRdyUSART());
                char data = ReadUSART();
                EEPROM_WRITE(0x02,data);
                while (!DataRdyUSART());
                data = ReadUSART();
                EEPROM_WRITE(0x03,data);
                globumask = (EEPROM_READ(0x02) << 8) | EEPROM_READ(0x03) ;
                setMask(globdata);
                break;
            case 'A':
                while (!DataRdyUSART());
                unsigned short dev = ReadSHORT();
                setMask(globdata | dev);
                break;
           /* case 'P':
                putsUSART("Sleep...");
                __delay_ms(1);
                __delay_s(1);
                break;*/
            case 'E':
                 while (!DataRdyUSART());
                unsigned short dev = ReadSHORT();
                setMask(globdata & ~dev);
                break;
            default:
                while (BusyUSART());
                putsUSART("Unknow command");
        }
        LED = 0;
        PIR1bits.RCIF = 0;
    }
}


 void setMask(unsigned short data) {

    EEPROM_WRITE(0x06,data >> 8);
    while (WR);
    EEPROM_WRITE(0x07,data);

     globdata = data;
     unsigned short out = data ^ globumask;
     PORTC = (0x0f & out);
     PORTBbits.RB0 = (out >> 4) & 0x1;
     PORTAbits.RA7 = (out >> 5) & 0x1;
     PORTAbits.RA6 = (out >> 6) & 0x1;
     PORTAbits.RA5 = (out >> 7) & 0x1;
     PORTAbits.RA4 = (out >> 8) & 0x1;
 }


