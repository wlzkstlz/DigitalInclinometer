#ifndef _EKF_H_
#define _EKF_H_



void EKFInit(const float acc[3]);

void EKFPredict();

void EKFMeasure(const float ax,const float ay,const float az);

void CalHMetrix();
void CalKMetrix();
void CalPhatMetrix();

void CalAngelFromAcc(const float acc[3], float angels[2]);
void CalAccFromAngel(const float angels[2], float acc[3]);

void CalMetrix33Inverse(const float data[][3], float out[][3]);
void TransposeMetrix(const float *input, float *output, int rows_in, int cols_in);
void MulMetrix(const float *data1, const float *data2, float *out, int rows1, int cols1, int cols2);
void AddMetrix(const float *data1, const float *data2, float *out, int rows, int cols);
void AddMetrixFactor(const float *data1, const float *data2, float *out, int rows, int cols, float factor);
#endif