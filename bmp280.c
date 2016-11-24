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

int InitBMP280(){
    uint8_t data;

    i2c_readReg(BMP_ADDR,REG_ID,&data,1);
    if(data != 0x58) return 1;
    
    // OSRS_T = x2      / 010
    // OSRS_P = x16     / 101
    // Mode NORMAL      / 11
    uint8_t ctrl_meas = 0b01010111;
    //uint8_t ctrl_meas = 0x3F;
    i2c_writeReg(BMP_ADDR,REG_CTRL_MEAS,&ctrl_meas,1);

    // tstdy =  0.5     /000
    /////////////////////////// IRR = 16         /100
    // IRR = 4         /010
    // SPI ENABLE       /0
    uint8_t config = 0b0000100;
    i2c_writeReg(BMP_ADDR,REG_CONFIG,&config,1);

    uint8_t address = REG_CALIB_firstLSB;

    // Get Calibration Data
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_T1,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_T1+1,1);

    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_T2,1);
    i2c_readReg(BMP_ADDR,address++,(uint8_t*)&dig_T2+1,1);
    
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
    uint8_t buf[3];
    i2c_readReg(BMP_ADDR,REG_TEMP_MSB,  (uint8_t*)&buf,3);
    temp = buf[0];
    temp = temp << 8;
    temp += buf[1];
    temp = temp<<4;
    temp += buf[2]>>4;
    //printf("TEMP %ld\n",temp);

    return temp;
}

int32_t ReadP_BMP280(void){
    int32_t pres;
    uint8_t buf[3];
    i2c_readReg(BMP_ADDR,REG_PRES_MSB,buf,3);

    pres = buf[0];
    pres = pres << 8;
    pres += buf[1];
    pres = pres<<4;
    pres += buf[2]>>4;


    return pres;
}


// CODE FROM DATASHEET

 
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t t_fine;
/*int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
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
*/
float readTemperature(void){
  int32_t var1, var2;

  int32_t adc_T = ReadT_BMP280();
  //adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)dig_T1 <<1))) *
	   ((int32_t)dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) *
	     ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
	   ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  return T/100;
}

float readPressure(void) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = ReadP_BMP280();
  //adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
  var2 = var2 + (((int64_t)dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) +
    ((var1 * (int64_t)dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
  return (float)p/256;
}

float readAltitude(float seaLevelhPa) {
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}
int32_t comp_pres_BMP280(void){
    ReadT_BMP280(); // Update t_fine variable
    int32_t pres = ReadP_BMP280();
    //pres =  bmp280_compensate_P_int32(pres);
    //printf(" %lx %li\n",pres,pres);//-%02x %02x %02x\n",pres,buf[0],buf[1],buf[2]);
    return pres;
}

float alt_BMP280(void){
    int32_t pres = readPressure();
    float alt = (1.f - powf(((float)pres)/101325.,0.190284))*145366.45;
    alt = alt * 0.3048;

    printf(" %ld \n",(int32_t)(alt*100.));
}

// CONF ADAFRUIT : temp oversample x1
// pres oversample x16
// MODE normal
