#include "KalmanFilter.h"

void KalmanFilter::ToFilter(float fi, float vel_in, float vel_out, int mode)
{
	if (init_Kalman == 0)
	{
		/* �������������. */
		flag_rvus = 0;
		Q = 0.0025;
		R = 0.09;
		K = 0.0;
		P = 0.0;
		xk = fi;
		u[1] = 0;
		f_time[0] = (float)GetTickCount();
		init_Kalman = 1; /* ������������� �����. */
		if (mode == 4)
		{
			u[0] = vel_out;
			flag_rvus = 1;

		}
	}
	else
	{

		/* �������� ������, � ������� �������� ��. */
		if ((mode == 1) || (mode == 2)) /* ����� ��� ��� ���: ������� �������� ����� ����. */
		{
			u[1] = 0.0;
			flag_rvus = 0;
		}
		else if (mode == 3) /* ����� ���: ���������� ���������� ��������. */
		{
			u[1] = vel_in;
			flag_rvus = 0;
		}
		else if (mode == 4) /* ����� ����: �������� ������� �� ����������. */
		{
			/*�.�. ������ ��������� ������� �� ��������� ��������, ������������ �� ���������� ��������.*/
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
		/* ����������. */
		f_time[1] = (float)GetTickCount();
		xk = xk + u[1] * (f_time[1] - f_time[0]) / 1000.0; /* ������������ ���������. */
		P = P + Q; /* ������������ ������. */
		K = P / (P + R); /* ���������� �������� �������. */
		P = (1 - K) * P; /* ���������� ������. */
		xk = xk + K * (fi - xk); /* ���������� ������. */

		f_time[0] = f_time[1];
	}
}