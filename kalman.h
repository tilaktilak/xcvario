#ifndef KALMAN_H
#define KALMAN_H


extern _Accum X[2];

int kalman_init(_Accum altitude_init);

int kalman_update(_Accum alt);

int kalman_predict(_Accum dt);

#endif
