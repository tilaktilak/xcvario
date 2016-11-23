// This is intended to be included in the main .c, after static_i2c.h

#define BMP280_ADDY 0xEC

int InitBMP280(void);

int32_t comp_pres_BMP280(void);

//uint8_t BMPGetStatus();

//Which = 0 or 12.  Returns 12 values.
//int GetBMPCalVals( int offset, uint8_t * vals );

//Returns 6 values.  MSB first for Pressure (3 bytes) then Temp (3 bytes)
//int GetBMPTelem( uint8_t * vals );
unsigned short dig_T1; 
short git_T2;
short dig_T3;
short dig_P6;
short dig_P5;
short dig_P4;
short dig_P3;
short dig_P2;
unsigned short dig_P1;
short dig_P9;
short dig_P8;
short dig_P7;




