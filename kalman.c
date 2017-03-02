#include "kalman.h"
#include <stdio.h>
#include <avr/io.h>
#include <stdfix.h>

float measure[2] = {0.f,0.f};

float H[2][2]={{1.f,0.f},
               {0.f,0.f}};

float R[2][2]={{0.6f*0.6f,0.f},
               {0.f      ,0.f}};

float A[2][2]={{1.f    ,0.008f},
               {0.f    ,1.f }};

float Q[2][2]={{0.f   ,0.f},
               {0.f    ,1.f }};
//float Q[2][2]={{.3f   ,0.f},
//               {0.f    ,0.05f }};
// Réglages cool : Q:0.2, R:1;

float Pp[2][2]={{0.f},{0.f}};
float P[2][2]={{0.f},{0.f}};
float K[2][2]={{0.f},{0.f}};
float Xp[2]={0.f};

float X[2];
int kalman_init(float altitude_init){
    X[0] = altitude_init;
    X[1] = 0.f;
    return 0;

}

/*int kalman_update(float *Pp,
  float *H,
  float *R,
  float *Xp){
 */
int kalman_update(float alt){
/*
alt = 73.4156f;

Pp[0][0] =0.0150f;
Pp[0][1] =0.0238f;
Pp[1][0] =0.0238f;
Pp[1][1] =2.9998f;
Xp[0] = 73.5231f;
Xp[1] = 0.0006f;*/
/*                             [    pp1        ]
                               [ ----------  0 ]
                               [ sig1 + pp1    ]
(%o47)                         [               ]
                               [    pp3        ]
                               [ ----------  0 ]
                               [ sig1 + pp1    ]
*/

    K[0][0] = Pp[0][0]/(R[0][0]+Pp[0][0]);
    K[0][1] = 0.f;
    K[1][0] = Pp[1][0]/(R[0][0]+Pp[0][0]);
    K[1][1] = 0.f;
/*printf("K %f %f %f %f \r\n",(double)K[0][0],
                          (double)K[0][1],
                          (double)K[1][0],
                          (double)K[1][1]);
*/
/*(%i76) Pp - K.H.Pp;
                        [ pp1 - k1 pp1  pp2 - k1 pp2 ]
(%o76)                  [                            ]
                        [ pp3 - k3 pp1  pp4 - k3 pp2 ]

*/

    P[0][0] = Pp[0][0]*(1.f-K[0][0]);
    P[0][1] = Pp[0][1]*(1.f-K[0][0]);
    P[1][0] = Pp[1][0]-K[1][0]*Pp[0][0];
    P[1][1] = Pp[1][1]-K[1][0]*Pp[0][1];

/*printf("P %f %f %f %f \r\n",(double)P[0][0],
                          (double)P[0][1],
                          (double)P[1][0],
                          (double)P[1][1]);
*/

/*
(%i65) Xp+K.(transpose(mes)-H.Xp);
                      [ xp1 + k1 (mes1 - xp1) + k2 mes2 ]
(%o65)                [                                 ]
                      [ xp2 + k3 (mes1 - xp1) + k4 mes2 ]

*/
    X[0] = Xp[0] + K[0][0]*(alt-Xp[0]);
    X[1] = Xp[1] + K[1][0]*(alt-Xp[0]);
/*
printf("X %f %f \r\n",(double)X[0],
                          (double)X[1]);

while(1);*/
/*
printf("%f %f\r\n %f %f\r\n",K[0][0],
                             K[0][1],
                             K[1][0],
                             K[1][1]);
*/
    return 0;
}

/*int kalman_predit(float *P,
                  float *A,
                  float *Q,
                  float *X){
                  */
int kalman_predict(float dt){
//X[0] = 73.5231f;
//X[1] = 0.0006f;

    //X = A.X
    Xp[0] = X[0] + dt*X[1];
    Xp[1] = X[1]/2;
/*  P = A*P*A' + Q
       [ 0.008 (0.008 p4 + p3) + 0.008 p2 + p1 + 0.005  0.008 p4 + p2 ]
(%o56) [                                                              ]
       [                 0.008 p4 + p3                     p4 + 1     ]
*/

    Pp[0][0] = dt*(dt*P[1][1]+P[1][0]) + dt*P[0][1] * P[0][0] + Q[0][0];
    Pp[0][1] = dt*(P[1][1]) + P[0][1];
    Pp[1][0] = dt*P[1][1] + P[1][0];
    Pp[1][1] = P[1][1] + Q[1][1];
/*
printf("XP %f %f\r\n",(double)Xp[0],(double)Xp[1]);
printf("Pp %f %f %f %f \r\n",(double)Pp[0][0],
                          (double)Pp[0][1],
                          (double)Pp[1][0],
                          (double)Pp[1][1]);
*/
    return 0;
}
