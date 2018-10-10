
#include <KalmanFilter.h>

#include <iostream>
#include <cstdlib>
#include <math.h>
using namespace std;


/* ������ ������� ================================= */
static float sigma_modeli_azimuta = 0.0025f;	/* ���������� ������ ������ �������. */
static float sigma_modeli_naklona = 0.0025f;	/* ���������� ������ ������ �������. */
static float sigma_sensora_azimuta = 0.09f;	/* ���������� ������ ������� �������. */
static float sigma_sensora_naklona = 0.09f;	/* ���������� ������ ������� �������. */
static float kalman_azimut = 0.0f;  /* ����������� ������� ��� �������� ���� ������� �� (�������) */
static float kalman_naklon = 0.0f;  /* ����������� ������� ��� �������� ���� ������� �� (�������) */
static float prev_azimut = 0.0f;	/* ���������� ����������������� �������� ���� ������� �� (�������) */
static float prev_naklon = 0.0f;	/* ���������� ����������������� �������� ���� ������� �� (�������) */
static float best_azimut = 0.0f;  /* ��������������� �������� ���� ������� �� (�������) */
static float best_naklon = 0.0f;  /* ��������������� �������� ���� ������� �� (�������) */
static float delta_azimut = 0.0f;  /* ������� ����� ����������� ������ ���������� ���� ������� � ����������� ������ ���������� */
static float delta_naklon = 0.0f;  /* ������� ����� ����������� ������ ���������� ���� ������� � ����������� ������ ���������� */
static float best_azimut_1 = 0.0f;  /* ��������������� �������� ���� ������� ��� ������ ��������� �� (�������) */
static float best_naklon_1 = 0.0f;  /* ��������������� �������� ���� ������� ��� ������ ��������� �� (�������) */
static float error_azimut = 0.0f;  /* ������ �������� ���� ������� �� (�������) */
static float error_naklon = 0.0f;  /* ������ �������� ���� ������� �� (�������) */


//.......
//	������������� ���������!!!
//
//	/* ������� �������� ��� ������ �����, ������� ����� �������������� �������� �������. */
//	error_azimut = sigma_sensora_azimuta;
//error_naklon = sigma_sensora_naklona;
//.......
//


	/* �������, �������������� ��������� ���� ������� �������� �������. */
	/* fi_A - ������ �������. */
	/* ������� ���������� ����������������� �������� ���� �������. */
	float correct_azimut(float fi_A)
{
	//������ ����������
	error_azimut = error_azimut + sigma_modeli_azimuta;  //������������ ������
	kalman_azimut = error_azimut / (error_azimut + sigma_sensora_azimuta);  //���������� �������� ������� 
	error_azimut = (1 - kalman_azimut) * error_azimut;  //���������� ������
	best_azimut_1 = (delta_azimut + best_azimut) + kalman_azimut * (fi_A - (delta_azimut + best_azimut));  //���������� ������ 
	//��������� ����������
	error_azimut = error_azimut + sigma_modeli_azimuta;  //������������ ������
	kalman_azimut = error_azimut / (error_azimut + sigma_sensora_azimuta);  //���������� �������� ������� 
	error_azimut = (1 - kalman_azimut) * error_azimut;  //���������� ������
	//���������� ������� ����� ����������� ������ ���������� � ����������� ������ ����������
	delta_azimut = ((2 * delta_azimut + best_azimut) + kalman_azimut * (best_azimut_1 - (2 * delta_azimut + best_azimut))) - best_azimut_1;  
	return (best_azimut_1 - delta_azimut);
}

/* �������, �������������� ��������� ���� ������� �������� �������. */
/* fi_N - ������ �������. */
/* ������� ���������� ����������������� �������� ���� �������. */
float correct_naklon(float fi_N)
{
	//������ ����������
	error_naklon = error_naklon + sigma_modeli_naklona;  //������������ ������
	kalman_naklon = error_naklon / (error_naklon + sigma_sensora_naklona);  //���������� �������� ������� 
	error_naklon = (1 - kalman_naklon) * error_naklon;  //���������� ������
	best_naklon_1 = (delta_naklon + best_naklon) + kalman_naklon * (fi_N - (delta_naklon + best_naklon));  //���������� ������ 
	//��������� ����������
	error_naklon = error_naklon + sigma_modeli_naklona;  //������������ ������
	kalman_naklon = error_naklon / (error_naklon + sigma_sensora_naklona);  //���������� �������� ������� 
	error_naklon = (1 - kalman_naklon) * error_naklon;  //���������� ������
	//���������� ������� ����� ����������� ������ ���������� � ����������� ������ ����������
	delta_naklon = ((2 * delta_naklon + best_naklon) + kalman_naklon * (best_naklon_1 - (2 * delta_naklon + best_naklon))) - best_naklon_1; 
	return (best_naklon_1 - delta_naklon);
}
void main(void)
{
	error_azimut = sigma_sensora_azimuta;
	error_naklon = sigma_sensora_naklona;
	float y[1001];
	float source[1001];
	float result[1001];
	float result1[1001];
	float delta1[1001];
	float delta2[1001];
	float x;
	KalmanFilter KF;
	
	for(int i = 0; i < 1000; i++)
	{
		x = ((rand() % 41) - 20);
		x = x  / 100;
		int u;
		//���������
		//y[i] = sin((float)i / 100.0) + x;
		//source[i] = sin((float)i / 100.0);
		//������ ����������� �����
		//if (i <= 250)
		//{
		//	y[i] = i/25.0 + x;
		//	source[i] = i/25.0;
		//	u = 1.0/25.0;
		//}
		//else if(i <= 500)
		//{
		//	y[i] = (500 - i)/25.0 + x;
		//	source[i] = (500 - i)/25.0;
		//	u = -1.0/25.0;
		//}
		//else if (i <= 750)
		//{
		//	y[i] = (i - 500)/25.0 + x;
		//	source[i] = (i - 500)/25.0;
		//	u = 1.0/25.0;
		//}
		//else 
		//{
		//	y[i] = (1000 - i)/25.0 + x;
		//	source[i] = (1000 - i)/25.0;
		//	u = -1.0/25.0;
		//}
		source[i] = sin((float)i/20);
		y[i] = source[i] + x;

		delta1[i] = y[i] - source[i];
		//result[i] = correct_azimut(y[i]);
		KF.ToFilter(y[i], 0, 0, 1);
		result[i] = KF.xk;
		best_azimut = result[i];
		delta2[i] = result[i] - source[i];
		//result1[i] = correct_naklon(y[i]);
		//best_naklon = result1[i];

	}
	int a = 0;

}