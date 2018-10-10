#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "windows.h"

class KalmanFilter
{
private:
	float Q;   /* Ковариация шума процесса. */
	float R;   /* Ковариация шума измерения. */
	float K;   /* Усиление Калмана. */
	float P;   /* Предсказание ошибки. */
	float f_time[2]; /* Используется  для получения интервала между поступаемыми значениями. */
	float u[2]; /* Управляющее воздействие(угловая скорость). */

public:
	float xk;  /* Отфильтрованное значение. */
	bool init_Kalman; /* Флаг для инициализации. */
	bool flag_rvus; /* Флаг используется при переключении на команду РВУС. */

	KalmanFilter() { init_Kalman = 0; flag_rvus = 0;};
	/*
	Фильтр Калмана
	fi - фильтруемый параметр(угол);
	vel_in - получаемая скорость с ГС;
	vel_out - указанная в управлении скорость;
	mode - режим работы ГС (1 - УАР; 2 - УТП; 3 - РУУ; 4 - РВУС).
	*/
	void ToFilter(float fi, float vel_in, float vel_out, int mode); 
};
#endif