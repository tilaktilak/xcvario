// This is intended to be included in the main .c, after static_i2c.h
#include <stdint.h>
#include "bmp280.h"
#define BMP_ADDR (0x76<<1)

#define REG_ID              0xD0
#define REG_RESET           0xE0
#define REG_CTRL_MEAS       0xF4
#define REG_CONFIG          0xF5

#define REG_CALIB_firstLSB  0x88

#define REG_TEMP_MSB        0xFA
#define REG_PRES_MSB        0xF7

#include "i2c_master.h"
#include <avr/io.h>

int InitBMP280(){
    uint8_t data;

    i2c_readReg(BMP_ADDR,REG_ID,&data,1);
    if(data != 0x58) return 1;
    
    // OSRS_T = x2      / 010
    // OSRS_P = x16     / 101
    // Mode NORMAL      / 11
    uint8_t ctrl_meas = 0b01010111;
    i2c_writeReg(BMP_ADDR,REG_CTRL_MEAS,&ctrl_meas,1);

    // tstdy =  0.5     /000
    // IRR = 16         /100
    // SPI ENABLE       /0
    uint8_t config = 0b0001000;
    i2c_writeReg(BMP_ADDR,REG_CONFIG,&config,1);

    uint8_t address = REG_CALIB_firstLSB;

    // Get Calibration Data
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_T1,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_T1+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&git_T2,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&git_T2+1,1);
    
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_T3,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_T3+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P1,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P1+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P2,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P2+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P3,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P3+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P4,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P4+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P5,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P5+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P6,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P6+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P7,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P7+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P8,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P8+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P9,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_P9+1,1);
}

int32_t ReadT_BMP280(void){
    int32_t temp;
    i2c_readReg(BMP_ADDR,REG_TEMP_MSB,  (uint8_t*)&temp+2,1);
    i2c_readReg(BMP_ADDR,REG_TEMP_MSB+1,(uint8_t*)&temp+1,1);
    i2c_readReg(BMP_ADDR,REG_TEMP_MSB+2,(uint8_t*)&temp  ,1);
    //temp = temp >> 4;
    return temp;
}

int32_t ReadP_BMP280(void){
    uint32_t pres;
    uint8_t buf[3];
    i2c_readReg(BMP_ADDR,REG_PRES_MSB,buf,3);
    pres = buf[2];
    //i2c_readReg(BMP_ADDR,REG_PRES_MSB+1,(uint8_t*)&pres+1,1);
    //i2c_readReg(BMP_ADDR,REG_PRES_MSB+2,(uint8_t*)&pres  ,1);
    //pres = pres >> 4;
    //printf(" BUF : 0x%02x %02x %02x\n",buf[0],buf[1],buf[2]);
    pres = buf[0]<<16 ;//+ buf[1]<<8 + buf[2];// + buf[1]<<1 + buf[2];
    printf(" PRES : %08x - %d\n",pres,pres);

    return pres;
}

#if 0
int InitBMP280()
{
    int r;

    SendStart();
    r = SendByte( 0 );
    if( !r ) { SendStop(); SendString(PSTR("I2C Fault")); return -2; }
    SendStop();

    SendStart();
    r = SendByte( BMP280_ADDY );
    if( r ) { SendStop(); SendString(PSTR("BMP Fault")); return -3; }
    SendByte( 0xE0 );
    SendByte( 0xB6 ); //Reboot
    SendStop();

    SendStart();
    r = SendByte( BMP280_ADDY );
    if( r ) { SendStop(); SendString(PSTR("BMP ID Fault")); return -4; }
    SendByte( 0xD0 );
    SendStop();

    SendStart();
    r = SendByte( BMP280_ADDY | 1 );
    if( r ) { SendStop(); SendString(PSTR("BMP Fault")); return -5; }
    if( (r=GetByte(1)) != 0x58 ) { SendStop(); PromptVal(PSTR("BMPID"),r); return -6; }
    SendStop();


    //Datasheet recommends setting up for: 
    // osrs_p = x16      osrs_p = 101
    // osrs_t = x2       osrs_t = 010
    // filter iir = 16              (though I question this)
    // ODR = 26.3        
    // Mode = Normal     mode = 11
    // Standby = 0       t_sb = 000
    

    SendStart();
    r = SendByte( BMP280_ADDY );
    if( r ) { SendStop(); SendString(PSTR("BMP ID Fault")); return -4; }
    SendByte( 0xF4 );
    SendByte( 0b01010111 ); //osrs_t osrs_p mode


    //XXX Filter may be incorrect
    //t_sb filter spi3w_en
    SendByte( 0b00010100 );

    SendStop();


    return 0;
}

uint8_t BMPGetStatus()
{
    uint8_t ret = 0;
    SendStart();
    if( SendByte( BMP280_ADDY ) ) { SendStop(); SendString(PSTR("BMP STAT")); return 0xff; }
    SendByte( 0xF3 );
    SendStop();

    SendStart();
    if( SendByte( BMP280_ADDY | 1 ) ) { SendStop(); SendString(PSTR("BMP STAT")); return 0xff; }
    ret = GetByte(1);
    SendStop();
    return ret;
}


//Which = 0 or 12.  Returns 12 values.
int GetBMPCalVals( int offset, uint8_t * vals )
{
    int i, r;
    i = 0;
    for( i = 0; i < 100; i++ )
    {
        if( !(BMPGetStatus() & 1 )) break;
        _delay_us(10);
    }
    if( i == 10 )
    {
        SendString(PSTR("BMP TO")); return -1;
    }
/*
retry:
    SendStart();
    if( SendByte( BMP280_ADDY ) ) { SendStop();  }
    SendByte( 0xF3 );
    SendStop();

    SendStart();
    r = SendByte( BMP280_ADDY | 1 );
    if( r ) { SendStop(); SendString(PSTR("BMP ST Fault")); return -5; }
    if( GetByte(1) & 1 ) {
        i++;
        SendStop();
        if( i > 100 )
        {
            SendString(PSTR("BMP ST TO"));
            return -9;
        }
        goto retry;
    }
    SendStop();

*/
    SendStart();
    r = SendByte( BMP280_ADDY );
    if( r ) { SendStop(); SendString(PSTR("BMP CC Fault")); return -4; }
    SendByte( 0x88 + offset );
    SendStop();

    SendStart();
    r = SendByte( BMP280_ADDY | 1 );
    if( r ) { SendStop(); SendString(PSTR("BMP CC Fault")); return -5; }
    for( i = 0; i < 11; i++ )
        vals[i] = GetByte(0);
    vals[11] = GetByte(1);
    SendStop();

    return 0;
}

//Returns 6 values.  MSB first for Pressure (3 bytes) then Temp (3 bytes)
int GetBMPTelem( uint8_t * vals )
{
    uint8_t i;

    SendStart();
    if( SendByte( BMP280_ADDY ) ) { SendStop(); SendString(PSTR("BMP BB Fault")); return -4; }
    SendByte( 0xF7 );
    SendStop();

    SendStart();
    if( SendByte( BMP280_ADDY | 1 ) ) { SendStop(); SendString(PSTR("BMP BB Fault")); return -5; }
    for( i = 0; i < 5; i++ )
        vals[i] = GetByte(0);
    vals[5] = GetByte(1);
    SendStop();

    return 0;
}


#endif

// CODE FROM DATASHEET

 
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t t_fine;
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)git_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t bmp280_compensate_P_int32(int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((uint32_t)var1);
    }
    else
    {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}

int32_t comp_pres_BMP280(void){
    ReadT_BMP280(); // Update t_fine variable
    int32_t pres = ReadP_BMP280();
//    printf("pres Ox%x pres\n",pres);
    pres =  bmp280_compensate_P_int32(pres);
    return pres;
}
