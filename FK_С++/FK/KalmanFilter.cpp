#include "KalmanFilter.h"

void KalmanFilter::ToFilter(float fi, float vel_in, float vel_out, int mode)
{
	if (init_Kalman == 0)
	{
		/* Инициализация. */
		flag_rvus = 0;
		Q = 0.0025;
		R = 0.09;
		K = 0.0;
		P = 0.0;
		xk = fi;
		u[1] = 0;
		f_time[0] = (float)GetTickCount();
		init_Kalman = 1; /* Переустановка флага. */
		if (mode == 4)
		{
			u[0] = vel_out;
			flag_rvus = 1;

		}
	}
	else
	{

		/* Проверка режима, в котором находитс ГС. */
		if ((mode == 1) || (mode == 2)) /* Режим УАР или УТП: угловая скорость равна нулю. */
		{
			u[1] = 0.0;
			flag_rvus = 0;
		}
		else if (mode == 3) /* Режим РУУ: используем получаемую скорость. */
		{
			u[1] = vel_in;
			flag_rvus = 0;
		}
		else if (mode == 4) /* Режим РВУС: получаем скорсть от управления. */
		{
			/*Т.к. только отправили команду об изменении скорости, используется ее предыдущее значение.*/
			if (flag_rvus == 0)
			{
				u[0] = vel_out;
				flag_rvus = 1;
			}
			else 
			{
				u[1] = u[0];
				u[0] = vel_out;
			}
		}
		/* Фильтрация. */
		f_time[1] = (float)GetTickCount();
		xk = xk + u[1] * (f_time[1] - f_time[0]) / 1000.0; /* Предсказание состояния. */
		P = P + Q; /* Предсказание ошибки. */
		K = P / (P + R); /* Вычисление усиления Калмана. */
		P = (1 - K) * P; /* Обновление ошибки. */
		xk = xk + K * (fi - xk); /* Обновление оценки. */

		f_time[0] = f_time[1];
	}
}