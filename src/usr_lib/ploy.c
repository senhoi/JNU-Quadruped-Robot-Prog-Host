#include "ploy.h"

/**
  * @brief  计算多项式的值
  * @param  coeffi 	多项式系数数组指针，传给此函数的多项式系数应是从低次项系数向高次项系数排列,如若传递数组[1 3 2],则表达为1+3*x+2*x^2
  *         num:   	多项式系数数组的元素个数
  *         x:  		x参数取值
  * @retval 返回三维空间内腿的末端点轨迹插值坐标矩阵
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
