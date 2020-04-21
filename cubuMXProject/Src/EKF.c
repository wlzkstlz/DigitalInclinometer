#include "EKF.h"
#include <math.h>

float gG;        //Earth Gravity
float gX_hat[2]; //State Variable:pitch and roll angel. unit: rad
float gX_bar[2];

float gP_hat[2][2] = {{DEG2RAD(1.0) * DEG2RAD(1.0), 0},
                      {0, DEG2RAD(1.0) * DEG2RAD(1.0)}}; //State CovMetrix;
float gP_bar[2][2];

//const float gF[2][2] = {{1, 0}, {0, 1}};
float gH[3][2];

const float gR[2][2] = {{DEG2RAD(0.5) * DEG2RAD(0.5), 0},
                        {0, DEG2RAD(0.5) * DEG2RAD(0.5)}};

#define _IMU_ACC_VAR_ ((7.9e-5 * 9.8) * (7.9e-5 * 9.8)) // at 10Hz
const float gQ[3][3] = {{_IMU_ACC_VAR_, 0, 0},
                        {0, _IMU_ACC_VAR_, 0},
                        {0, 0, _IMU_ACC_VAR_}}; //IMU parameter

float gK[2][3]; //kalman Gain

float gErr[3][1];

unsigned char gbEKFInited = 0;
void EKFInit(const float acc[3])
{
    gG = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    CalAngelFromAcc(acc, gX_hat);
    gbEKFInited = 1;
}

void EKFPredict(void)
{
    if (0 == gbEKFInited)
        return;
    AddMetrix(&gP_hat[0][0], &gR[0][0], &gP_bar[0][0], 2, 2);

    for (int i = 0; i < 2; i++)
        gX_bar[i] = gX_hat[i]; //角度不变假设
}

void EKFMeasure(const float ax, const float ay, const float az)
{
    if (0 == gbEKFInited)
        return;
    float acc_measure[3];
    acc_measure[0] = ax;
    acc_measure[1] = ay;
    acc_measure[2] = az;

    CalHMetrix();
    CalKMetrix();
    CalErr(acc_measure);

    float tmp1[2][1];
    MulMetrix(&gK[0][0], &gErr[0][0], &tmp1[0][0], 2, 3, 1);

    for (int i = 0; i < 2; i++)
    {
        gX_hat[i] = gX_bar[i] + tmp1[i][0];
    }

    CalPhatMetrix();
}

void CalHMetrix(void)
{
    gH[0][0] = -cos(gX_bar[0]) * gG;
    gH[0][1] = 0;
    gH[1][0] = -sin(gX_bar[1]) * sin(gX_bar[0]) * gG;
    gH[1][1] = cos(gX_bar[1]) * cos(gX_bar[0]) * gG;
    gH[2][0] = -cos(gX_bar[1]) * sin(gX_bar[0]) * gG;
    gH[2][1] = -sin(gX_bar[1]) * cos(gX_bar[0]) * gG;
}

void CalKMetrix(void)
{
    float h_t[2][3];
    TransposeMetrix(&gH[0][0], &h_t[0][0], 3, 2);

    float tmp1[2][3];
    MulMetrix(&gP_bar[0][0], &h_t[0][0], &tmp1[0][0], 2, 2, 3);

    float tmp2[3][2];
    MulMetrix(&gH[0][0], &gP_bar[0][0], &tmp2[0][0], 3, 2, 2);

    float tmp3[3][3];
    MulMetrix(&tmp2[0][0], &h_t[0][0], &tmp3[0][0], 3, 2, 3);

    AddMetrix(&tmp3[0][0], &gQ[0][0], &tmp3[0][0], 3, 3);

    float tmp4[3][3];
    CalMetrix33Inverse(tmp3, tmp4);

    MulMetrix(&tmp1[0][0], &tmp4[0][0], &gK[0][0], 2, 3, 3);
}

void CalErr(float measure[3])
{
    float acc_predict[3];
    CalAccFromAngel(gX_bar, acc_predict);

    for (int i = 0; i < 3; i++)
        gErr[i][0] = measure[i] - acc_predict[i];
}

void CalPhatMetrix(void)
{
    float tmp1[2][2];
    MulMetrix(&gK[0][0], &gH[0][0], &tmp1[0][0], 2, 3, 2);
    const float ones[2][2] = {{1, 0}, {0, 1}};
    float tmp2[2][2];
    AddMetrixFactor(&ones[0][0], &tmp1[0][0], &tmp2[0][0], 2, 2, -1.0);
    MulMetrix(&tmp2[0][0], &gP_bar[0][0], &gP_hat[0][0], 2, 2, 2);
}

void CalAngelFromAcc(const float acc[3], float angels[2])
{
    angels[0] = asin(-acc[0] / gG);    //pitch
    angels[1] = atan(acc[1] / acc[2]); //roll
}
void CalAccFromAngel(const float angels[2], float acc[3])
{
    acc[0] = -sin(angels[0]) * gG;
    acc[1] = cos(angels[0]) * gG * sin(angels[1]);
    acc[2] = cos(angels[0]) * gG * cos(angels[1]);
}

void CalMetrix33Inverse(const float data[][3], float out[][3])
{
    const float a1 = data[0][0];
    const float b1 = data[0][1];
    const float c1 = data[0][2];

    const float a2 = data[1][0];
    const float b2 = data[1][1];
    const float c2 = data[1][2];

    const float a3 = data[2][0];
    const float b3 = data[2][1];
    const float c3 = data[2][2];

    float factor = a1 * (b2 * c3 - c2 * b3) - a2 * (b1 * c3 - c1 * b3) + a3 * (b1 * c2 - c1 * b2);

    out[0][0] = b2 * c3 - c2 * b3;
    out[0][1] = c1 * b3 - b1 * c3;
    out[0][2] = b1 * c2 - c1 * b2;

    out[1][0] = c2 * a3 - a2 * c3;
    out[1][1] = a1 * c3 - c1 * a3;
    out[1][2] = a2 * c1 - a1 * c2;

    out[2][0] = a2 * b3 - b2 * a3;
    out[2][1] = b1 * a3 - a1 * b3;
    out[2][2] = a1 * b2 - a2 * b1;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            out[i][j] = out[i][j] / factor;
        }
    }
}

void MulMetrix(const float *data1, const float *data2, float *out, int rows1, int cols1, int cols2)
{
    for (int r = 0; r < rows1; r++)
    {
        for (int c = 0; c < cols2; c++)
        {
            out[r * cols2 + c] = 0;
        }
    }

    for (int r = 0; r < rows1; r++)
    {
        for (int c = 0; c < cols2; c++)
        {
            for (int k = 0; k < cols1; k++)
            {
                out[r * cols2 + c] += data1[r * cols1 + k] * data2[k * cols2 + c];
            }
        }
    }
}

void AddMetrix(const float *data1, const float *data2, float *out, int rows, int cols)
{
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            out[r * cols + c] = data1[r * cols + c] + data2[r * cols + c];
        }
    }
}

void AddMetrixFactor(const float *data1, const float *data2, float *out, int rows, int cols, float factor)
{
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            out[r * cols + c] = data1[r * cols + c] + factor * data2[r * cols + c];
        }
    }
}

void TransposeMetrix(const float *input, float *output, int rows_in, int cols_in)
{
    for (int r = 0; r < rows_in; r++)
    {
        for (int c = 0; c < cols_in; c++)
        {
            output[c * rows_in + r] = input[r * cols_in + c];
        }
    }
}
