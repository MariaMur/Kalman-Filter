#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "windows.h"

class KalmanFilter
{
private:
	float Q;   /* ���������� ���� ��������. */
	float R;   /* ���������� ���� ���������. */
	float K;   /* �������� �������. */
	float P;   /* ������������ ������. */
	float f_time[2]; /* ������������  ��� ��������� ��������� ����� ������������ ����������. */
	float u[2]; /* ����������� �����������(������� ��������). */

public:
	float xk;  /* ��������������� ��������. */
	bool init_Kalman; /* ���� ��� �������������. */
	bool flag_rvus; /* ���� ������������ ��� ������������ �� ������� ����. */

	KalmanFilter() { init_Kalman = 0; flag_rvus = 0;};
	/*
	������ �������
	fi - ����������� ��������(����);
	vel_in - ���������� �������� � ��;
	vel_out - ��������� � ���������� ��������;
	mode - ����� ������ �� (1 - ���; 2 - ���; 3 - ���; 4 - ����).
	*/
	void ToFilter(float fi, float vel_in, float vel_out, int mode); 
};
#endif