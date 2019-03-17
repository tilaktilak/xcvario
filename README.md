##  Subject

Interface board to transform a KOBO in an XCSoar glide computer


[![DSC00004.jpg](https://i.postimg.cc/hvsq8fkh/DSC00004.jpg)](https://postimg.cc/5QXGbNNM)

## How it works : 

Based on an ATMEGA328p microcontroler, 
this project interface a barometer and a GNSS sensor to process vertical through a Kalman filter.    
Then a PWM sound signal is produce to generate a BIP sound through a simple transistor amplifier and a Buzzer onboard.   
Finally the board is able to communicate with a modified Kobo e-reader that run XCSoar software : Position, Altitude and VZ data are send through UART and then Glide computer is then fully operationnal.

[![DSC00008-1.jpg](https://i.postimg.cc/W1hPrpXb/DSC00008-1.jpg)](https://postimg.cc/yD45CHMw)
