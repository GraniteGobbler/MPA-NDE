/*
 * QMC5883.c
 *
 *  Created on: 11 May 2021
 *      Author: Serdar
 */
#include"QMC5883.h"
#include "math.h"
#include "float.h"

//###############################################################################################################
// Initialize the Kalman filter
void Kalman_Init(KalmanFilter_t *kf, float process_noise, float measurement_noise, float initial_estimate) {
    kf->estimate = initial_estimate;
    kf->error_cov = 1.0f; // Initial error covariance
    kf->process_noise = process_noise;
    kf->measurement_noise = measurement_noise;
}

// Update the Kalman filter with a new measurement
float Kalman_Update(KalmanFilter_t *kf, float measurement) {
    // Validate measurement
    if (isnan(measurement) || isinf(measurement)) {
        return kf->estimate; // Return the last valid estimate
    }

    // Prediction step
    kf->error_cov += kf->process_noise;

    // Validate error covariance
    if (kf->error_cov <= 0.0f) {
        kf->error_cov = FLT_EPSILON; // Prevent division by zero
    }

    // Kalman gain
    float denominator = kf->error_cov + kf->measurement_noise;
    if (denominator <= 0.0f) {
        denominator = FLT_EPSILON; // Prevent division by zero
    }
    float kalman_gain = kf->error_cov / denominator;

    // Update estimate
    kf->estimate += kalman_gain * (measurement - kf->estimate);

    // Update error covariance
    kf->error_cov *= (1.0f - kalman_gain);

    return kf->estimate;
}


uint8_t QMC_init(QMC_t *qmc,I2C_HandleTypeDef *i2c,uint8_t Output_Data_Rate)
{
	/*Kalman Filter Init*/
	Kalman_Init(&qmc->kalman_filter, 0.05f, 1.2f, 0.0f);

	uint8_t array[2];
	qmc->i2c=i2c;
	qmc->Control_Register=0x11;
	array[0]=1;
	array[1]=qmc->Control_Register;

	if(Output_Data_Rate==200)qmc->Control_Register|=0b00001100;
	else if(Output_Data_Rate==100)qmc->Control_Register|=0b00001000;
	else if(Output_Data_Rate==50)qmc->Control_Register|=0b00000100;
	else if(Output_Data_Rate==10)qmc->Control_Register|=0b00000000;
	else qmc->Control_Register|=0b00001100;

	if(HAL_I2C_Mem_Write(qmc->i2c, 0x1A, 0x0B, 1, &array[0], 1, 100)!=HAL_OK)return 1;
	if(HAL_I2C_Mem_Write(qmc->i2c, 0x1A, 0x09, 1, &array[1], 1, 100)!=HAL_OK)return 1;



	return 0;
}

uint8_t QMC_read(QMC_t *qmc)
{
	  qmc->datas[0]=0;
	  HAL_I2C_Mem_Read(qmc->i2c, 0x1A, 0x06, 1, qmc->datas, 1, 100);

	  if((qmc->datas[0]&0x01)==1)
	  {
		  HAL_I2C_Mem_Read(qmc->i2c, 0x1A, 0x00, 1, qmc->datas, 6, 100);
		  qmc->Xaxis= (qmc->datas[1]<<8) | qmc->datas[0];
		  qmc->Yaxis= (qmc->datas[3]<<8) | qmc->datas[2];
		  qmc->Zaxis= (qmc->datas[5]<<8) | qmc->datas[4];

		  qmc->compas=atan2f(qmc->Xaxis,qmc->Yaxis)*180.00/M_PI;
		  qmc->filtered_compas = Kalman_Update(&qmc->kalman_filter, qmc->compas);

		  if(qmc->compas>0)
		  {
			  qmc->filtered_heading= qmc->filtered_compas;
			  qmc->heading = qmc->compas;
		  }
		  else
		  {
			  qmc->filtered_heading=360+qmc->filtered_compas;
			  qmc->heading = 360+qmc->compas;

		  }
	  }
	  else
	  {
		  return 1;
	  }
return 0;
}

float QMC_readHeading(QMC_t *qmc)
{
	QMC_read(qmc);
	return qmc->heading;
}

uint8_t QMC_Standby(QMC_t *qmc)
{
	uint8_t array[1]={0};
	if(HAL_I2C_Mem_Write(qmc->i2c, 0x1A, 0x09, 1, &array[0], 1, 100)!=HAL_OK)return 1;
	return 0;
}
uint8_t QMC_Reset(QMC_t *qmc)
{
	uint8_t array[1]={0x80};
	if(HAL_I2C_Mem_Write(qmc->i2c, 0x1A, 0x0A, 1, &array[0], 1, 100)!=HAL_OK)return 1;
	return 0;
}

