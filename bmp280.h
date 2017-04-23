
#ifndef BMP280_H
#define BMP280_H
#define BMP280_ADDY 0xEC

int InitBMP280(void);

float AltitudeBMP280(void);

float PressureBMP280(void);

float press;

int32_t t_fine;
union {
    struct {
        uint16_t dig_T1; 
        int16_t dig_T2;
        int16_t dig_T3;
        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
    };
    uint8_t buff[24];
}b;



#endif
