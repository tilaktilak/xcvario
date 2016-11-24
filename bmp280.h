#define BMP280_ADDY 0xEC

int InitBMP280(void);

float AltitudeBMP280(void);

float PressureBMP280(void);

float alt;
int32_t t_fine;

unsigned short dig_T1; 
short dig_T2;
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




