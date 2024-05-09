#pragma config ALTI2C1 = ON

#define FCY 4000000UL
#include "xc.h"
#include <libpic30.h>

#define PIN_EN (1 << 2)
#define BACKLIGHT (1 << 3)
#define RET_STRING 0b10000000
#define LCD_CLEAR 0b00000001
const char *My_asci[1] = {"0"};

#define LCD_ADRESS 0x4E // или 0x7E

void initI2C(void)
{
 I2C1BRG = 0xC3; // частота синхронизации
 I2C1CONLbits.I2CEN = 1; //Включить I2C
 I2C1CONLbits.I2CSIDL=1; //
}
void I2CWrite(uint8_t adr,uint8_t data)
{
 while(I2C1CONLbits.PEN);
 I2C1CONLbits.SEN = 1;
 while (I2C1CONLbits.SEN != 0);
 I2C1TRN = adr;
 while (I2C1STATbits.TRSTAT == 1);
 I2C1TRN = data;
 while (I2C1STATbits.TRSTAT == 1);
 I2C1CONLbits.PEN = 1;
}

typedef enum
{
 COMMAND=0,
 DATA=1 
}
LDCType_t;


void I2CWriteBuff(uint8_t adr,uint8_t* data,uint16_t len)
{
 uint16_t i;
 while(I2C1CONLbits.PEN);
 I2C1CONLbits.SEN = 1;
 while (I2C1CONLbits.SEN != 0);
 I2C1TRN = adr;
 for(i=0;i<len;i++)
 {
 while (I2C1STATbits.TRSTAT == 1);
 I2C1TRN=data[i];
 } 
 while (I2C1STATbits.TRSTAT == 1);
 I2C1CONLbits.PEN = 1;
}
void LCDSend(uint8_t lcd_addr, uint8_t data, LDCType_t flags)
{
 uint8_t high = data & 0xF0;
 uint8_t low = (data << 4) & 0xF0;
 uint8_t data_arr[6];
 data_arr[0] = high|flags|BACKLIGHT;
 data_arr[1] = high|flags|BACKLIGHT|PIN_EN;
 data_arr[2] = high|flags|BACKLIGHT;
 data_arr[3] = low|flags|BACKLIGHT;
 data_arr[4] = low|flags|BACKLIGHT|PIN_EN;
 data_arr[5] = low|flags|BACKLIGHT;
 I2CWriteBuff(lcd_addr,&data_arr,6);
}
void LCDInit(void)
{
 LCDSend(LCD_ADRESS, 0b00110000,COMMAND);
 LCDSend(LCD_ADRESS, 0b00000010,COMMAND);
 LCDSend(LCD_ADRESS, 0b00001100,COMMAND);
 LCDSend(LCD_ADRESS, LCD_CLEAR,COMMAND); 
}
void LCDPritStr(uint8_t* str, uint16_t len)
{
 uint16_t i;
 for(i=0;i<len;i++)
 {
 LCDSend(LCD_ADRESS, str[i],DATA);
 } 
}


void Adc_init()
{
 ADCON1Lbits.ADSIDL = 0;
 ADCON1Hbits.FORM = 0;
 ADCON1Hbits.SHRRES0 = 1;
 ADCON1Hbits.SHRRES1 = 1;
 ADCON1Lbits.ADON = 0x1;
 ADCON5Lbits.SHRPWR = 1; 
 while(ADCON5Lbits.SHRRDY == 0);
 ADCON3Hbits.SHREN = 1; 
 ADTRIG3L = 0x100;
 ADTRIG0H = 0x100;//ADTRIG0H[12:8]
 ADTRIG1L = 0x001;//ADTRIG1L[4:0]
}

uint16_t read_Adc(uint8_t channel){
 uint16_t result;
 
 ADCON3Lbits.SWCTRG = 1;
 __delay_ms(5)
 switch(channel)
 {
 case 0:
 while(!ADSTATLbits.AN3RDY);
 __delay_ms(5)
 result = ADCBUF3;
 break;
 case 1:
 while(!ADSTATLbits.AN4RDY);
 __delay_ms(5)
 result = ADCBUF4;
 break;
 case 2:
 while(!ADSTATLbits.AN13RDY);
 __delay_ms(5)

 result = ADCBUF13;
 break;
 }
 return result;
}

int main(void)
{   
    TRISEbits.TRISE12=0;
    
    const char *My_numbers[10] = {"0", "1", "2","3","4","5","6","7","8","9"};
    initI2C();
    I2CWrite(LCD_ADRESS, 0x00);
    LCDInit();
    
    Adc_init();
    uint16_t d = 0;
    uint16_t s = 1;
    uint16_t r = 1;
    while(1)
    {
    s = 1;
    __delay_ms(50);
    r = read_Adc(2);
    d = r;
    LCDSend(LCD_ADRESS, LCD_CLEAR,COMMAND);
    while (r/s>=10){s*=10;}
    while (s>0){
        LCDPritStr(My_numbers[d/s], 1);
        //LCDPritStr(d/s + My_asci[0], 1);
        d=d%s;
        s=s/10;}
    }
    return 0;
}