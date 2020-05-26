#ifndef _EKF_H_
#define _EKF_H_

#include <stdint.h>

#define PI 3.1415926535897932385
#define DEG2RAD(x) ((x) / 180.0 * PI)
#define RAD2DEG(x) ((x) / PI * 180.0)

extern float gX_hat[2];
extern float gP_hat[2][2];
extern float gErr[3][1];

// for calibration
#define CALIB_BUFFER_SIZE  30 // 对应约3s时间
extern int32_t acc_calibrate_buffer[3][CALIB_BUFFER_SIZE];
extern uint32_t calibrate_buffer_id;
extern float gG;
extern float g_acc_offsets[3];

void EKFInit(const float acc[3]);

void EKFPredict(void);

void EKFMeasure(const float ax,const float ay,const float az);

void CalHMetrix(void);
void CalKMetrix(void);
void CalErr(float measure[3]);
void CalPhatMetrix(void);

void UpdateG(float measure[3]);

void CalAngelFromAcc(const float acc[3], float angels[2]);
void CalAccFromAngel(const float angels[2], float acc[3]);

void CalMetrix33Inverse(const float data[][3], float out[][3]);
void TransposeMetrix(const float *input, float *output, int rows_in, int cols_in);
void MulMetrix(const float *data1, const float *data2, float *out, int rows1, int cols1, int cols2);
void AddMetrix(const float *data1, const float *data2, float *out, int rows, int cols);
void AddMetrixFactor(const float *data1, const float *data2, float *out, int rows, int cols, float factor);
#endif
