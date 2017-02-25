#ifndef KALMAN_H
#define KALMAN_H


extern float X[2];

int kalman_init(float altitude_init);

int kalman_update(float alt);

int kalman_predict(float dt);

#endif
