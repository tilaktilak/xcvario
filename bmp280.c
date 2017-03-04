// This is intended to be included in the main .c, after static_i2c.h
#include <stdint.h>
#include <math.h>
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
#include <stdio.h>

int InitBMP280(){
    uint8_t data;
    uint8_t retval = 0;

    i2c_readReg(BMP_ADDR,REG_ID,&data,1);
    if(data != 0x58) return 1;

    //uint8_t reset = 0xB6;
    //i2c_writeReg(BMP_ADDR,REG_RESET,&reset,1);

    // OSRS_T = x2      / 010
    // OSRS_P = x16     / 101
    // Mode NORMAL      / 11
    uint8_t ctrl_meas = 0b01010111;
    retval += i2c_writeReg(BMP_ADDR,REG_CTRL_MEAS,&ctrl_meas,1);

    // WARNING ! Will write on reserved bit
    // tstdy =  0.5     /000
    // IRR = 4         /000
    // SPI ENABLE       /0
    //uint8_t config = 0b01001000;
    //uint8_t config = 0x00;
    //retval += i2c_writeReg(BMP_ADDR,REG_CONFIG,&config,1);

    uint8_t address = REG_CALIB_firstLSB;
    // Get Calibration Data


    retval += i2c_readReg(BMP_ADDR,address,b.buff,24);
   
/*

b.dig_T1 = 27504;
b.dig_T2 =    26435;
b.dig_T3 =-1000;
b.dig_P1 =36477;
b.dig_P2 =-10685;
b.dig_P3 =3024;
b.dig_P4 =2855;
b.dig_P5 =140;
b.dig_P6 =-7;  
b.dig_P7 =15500;
b.dig_P8 =-14600;
b.dig_P9 =6000;*/
    /*printf("%d %d %d %d %d %d %d %d %d %d %d %d\n",b.dig_T1,b.dig_T2,b.dig_T3,
            b.dig_P1,b.dig_P2,b.dig_P3,b.dig_P4,b.dig_P5,b.dig_P6,
            b.dig_P7,b.dig_P8,b.dig_P9);
    */
    return retval;
}

uint32_t ReadT_BMP280(void){
    uint32_t temp;
    uint8_t buf[3];
    i2c_readReg(BMP_ADDR,REG_TEMP_MSB,  (uint8_t*)&buf,3);
    temp = buf[0];
    temp = temp << 8;
    temp |= buf[1];
    temp = temp<<4;
    temp |= buf[2]>>4;
    return temp;
}

uint32_t ReadP_BMP280(void){
    uint32_t pres;
    uint8_t buf[3];
    i2c_readReg(BMP_ADDR,REG_PRES_MSB,buf,3);
    pres = buf[0];
    pres = pres << 8;
    pres |= buf[1];
    pres = pres<<4;
    pres |= buf[2]>>4;
    return pres;
}



float TemperatureBMP280(void){
  int32_t var1, var2;

  uint32_t adc_T = ReadT_BMP280();

  var1  = ((((adc_T>>3) - ((int32_t)b.dig_T1 <<1))) *
	   ((int32_t)b.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)b.dig_T1)) *
	     ((adc_T>>4) - ((int32_t)b.dig_T1))) >> 12) *
	   ((int32_t)b.dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  return T/100;
}
float test1;

float PressureBMP280(void) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  TemperatureBMP280();

  uint32_t adc_P = ReadP_BMP280();

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)b.dig_P6;
  var2 = var2 + ((var1*(int64_t)b.dig_P5)<<17);
  var2 = var2 + (((int64_t)b.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)b.dig_P3)>>8) +
    ((var1 * (int64_t)b.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)b.dig_P1)>>33;


  if (var1 == 0) {
    return -1.;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)b.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)b.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)b.dig_P7)<<4);
  test1 = (float)p/256;
  return (float)p/256;
}


float AltitudeBMP280(void){
    float press = PressureBMP280();
    float alt = 0.f;;
    alt = (1.f - powf(((float)press)/101325.f,0.190284f))*145366.45f;
    alt = alt * 0.3048f;
    return alt;
}
