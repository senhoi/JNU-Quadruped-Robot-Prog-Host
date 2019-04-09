#include "ploy.h"

/**
  * @brief  �������ʽ��ֵ
  * @param  coeffi 	����ʽϵ������ָ�룬�����˺����Ķ���ʽϵ��Ӧ�Ǵӵʹ���ϵ����ߴ���ϵ������,������������[1 3 2],����Ϊ1+3*x+2*x^2
  *         num:   	����ʽϵ�������Ԫ�ظ���
  *         x:  		x����ȡֵ
  * @retval ������ά�ռ����ȵ�ĩ�˵�켣��ֵ�������
  */
float polyval(float *coeffi, unsigned num, float x)
{
	int i, j;
	float temp = 0;
	float sum = *(coeffi);
	for (i = 1; i < num; i++)
	{
		temp = x;
		for (j = 1; j < i; j++)
		{
			temp *= x;
		}
		sum += *(coeffi + i) * temp;
	}
	return sum;
}
