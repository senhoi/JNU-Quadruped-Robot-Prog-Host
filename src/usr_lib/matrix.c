/**
  ******************************************************************************
  * @file    matrix.c
  * @author  Zhang Chunyu
  * @version V1.1
  * @date    04-October-2018
  * @brief   本文件为舵机四足机器人的运动学解算提供了所需的数学运算函数，包括：
  *           + 矩阵乘法
  *           + 矩阵连乘 
  *           + 产生齐次变换矩阵     
  *           + 矩阵显示
  * 		  + 算子生成（运动学算子/轨迹规划算子）
  * 		  + 求矩阵行列式（限3阶/4阶矩阵）
  * 		  + 求4阶矩阵的伴随矩阵
  * 		  + 求4阶矩阵的逆矩阵
  *
 ===================================================================      
                 ##### How to use this file #####
 =================================================================== 
 [..]
   (#) 
  *
  ******************************************************************************
  * @attention
  *		 + matrix变量使用前应先先划分内存区域
  * 	 + 产生旋转的齐次变换矩阵的输入量是角度值而非弧度制
  * 
  *
  ******************************************************************************  
  */

/* Includes ------------------------------------------------------------------*/
#include "matrix.h"

/* Display functions ----------------------------------------------------------*/
/**
  * @brief  将矩阵打印到控制台
  * @param  sMatrix: 要打印的矩阵
  * @retval None
  */
void MatrixDisplay(Matrix_t sMatrix)
{
	int i, j;
	printf("ROW:%d,COLUMN:%d\n", sMatrix.row, sMatrix.column);
	for (i = 0; i < sMatrix.row; i++)
	{
		for (j = 0; j < sMatrix.column; j++)
			printf("\t%f", *(sMatrix.pMatrix + i * sMatrix.column + j));
		printf("\n");
	}
}

/* Arithmetical operation ------------------------------------------------------*/
/**
  * @brief  计算绕x轴旋转的齐次变换矩阵
  * @param  degree: 绕x轴旋转的角度[DEG]
  * @retval 绕x轴旋转的齐次变换矩阵[4x4]
  */
Matrix_t SE3_Rx(float degree)
{
	float rad = (float)(degree / 180.0 * PI);
	int i, j;
	Matrix_t temp;
	float *se_x = (float *)malloc(sizeof(float) * 4 * 4);
	if (se_x == NULL)
		printf("ERROR: Malloc failed!-09");
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			*(se_x + i * 4 + j) = 0;
	*(se_x + 0 * 4 + 0) = 1;
	*(se_x + 1 * 4 + 1) = (float)cos(rad);
	*(se_x + 1 * 4 + 2) = (float)-sin(rad);
	*(se_x + 2 * 4 + 1) = -*(se_x + 1 * 4 + 2);
	*(se_x + 2 * 4 + 2) = *(se_x + 1 * 4 + 1);
	*(se_x + 3 * 4 + 3) = 1;

	temp.pMatrix = se_x;
	temp.row = 4;
	temp.column = 4;

	return temp;
}

/**
  * @brief  计算绕y轴旋转的齐次变换矩阵
  * @param  degree: 绕y轴旋转的角度[DEG]
  * @retval 绕y轴旋转的齐次变换矩阵[4x4]
  */
Matrix_t SE3_Ry(float degree)
{
	float rad = (float)(degree / 180.0 * PI);
	int i, j;
	Matrix_t temp;
	float *se_y = (float *)malloc(sizeof(float) * 4 * 4);
	if (se_y == NULL)
		printf("ERROR: Malloc failed!-08");
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			*(se_y + i * 4 + j) = 0;
	*(se_y + 0 * 4 + 0) = (float)cos(rad);
	*(se_y + 0 * 4 + 2) = (float)sin(rad);
	*(se_y + 1 * 4 + 1) = 1;
	*(se_y + 2 * 4 + 0) = -(float)sin(rad);
	*(se_y + 2 * 4 + 2) = (float)cos(rad);
	*(se_y + 3 * 4 + 3) = 1;

	temp.pMatrix = se_y;
	temp.row = 4;
	temp.column = 4;

	return temp;
}

/**
  * @brief  计算绕z轴旋转的齐次变换矩阵
  * @param  degree: 绕z轴旋转的角度[DEG]
  * @retval 绕z轴旋转的齐次变换矩阵[4x4]
  */
Matrix_t SE3_Rz(float degree)
{
	float rad = (float)(degree / 180.0 * PI);
	int i, j;
	Matrix_t temp;
	float *se_z = (float *)malloc(sizeof(float) * 4 * 4);
	if (se_z == NULL)
		printf("ERROR: Malloc failed!-07");
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			*(se_z + i * 4 + j) = 0;
	*(se_z + 0 * 4 + 0) = (float)cos(rad);
	*(se_z + 0 * 4 + 1) = (float)-sin(rad);
	*(se_z + 1 * 4 + 1) = *(se_z + 0 * 4 + 0);
	*(se_z + 1 * 4 + 0) = -*(se_z + 0 * 4 + 1);
	*(se_z + 2 * 4 + 2) = 1;
	*(se_z + 3 * 4 + 3) = 1;

	temp.pMatrix = se_z;
	temp.row = 4;
	temp.column = 4;

	return temp;
}

/**
  * @brief  计算平移变换的齐次变换矩阵
  * @param  dis_x:平移变换的x轴方向距离
  * 		dis_y:平移变换的y轴方向距离
  * 		dis_z:平移变换的z轴方向距离
  * @retval 平移变换的齐次变换矩阵[4x4]
  */
Matrix_t SE3_T(float dis_x, float dis_y, float dis_z)
{
	int i, j;
	Matrix_t temp;
	float *se_t = (float *)malloc(sizeof(float) * 4 * 4);
	if (se_t == NULL)
		printf("ERROR: Malloc failed!-06");
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			*(se_t + i * 4 + j) = 0;
	*(se_t + 0 * 4 + 3) = dis_x;
	*(se_t + 1 * 4 + 3) = dis_y;
	*(se_t + 2 * 4 + 3) = dis_z;
	*(se_t + 0 * 4 + 0) = 1;
	*(se_t + 1 * 4 + 1) = 1;
	*(se_t + 2 * 4 + 2) = 1;
	*(se_t + 3 * 4 + 3) = 1;

	temp.pMatrix = se_t;
	temp.row = 4;
	temp.column = 4;

	return temp;
}

/**
  * @brief  产生用于3次多项式轨迹规划的算子
  * @param  tf：相对当前轨迹段的时间[s]
  * @retval 用于3次多项式轨迹规划的算子[4x4]
  */
Matrix_t Traj3_Mat(float tf)
{
	int i, j;
	Matrix_t temp;
	float *trans_arr = (float *)malloc(sizeof(float) * 4 * 4);
	if (trans_arr == NULL)
		printf("ERROR: Malloc failed!-05");
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			*(trans_arr + i * 4 + j) = 0;
	*(trans_arr + 0 * 4 + 0) = 1;
	*(trans_arr + 1 * 4 + 1) = 1;
	*(trans_arr + 2 * 4 + 0) = 1;
	*(trans_arr + 2 * 4 + 1) = tf;
	*(trans_arr + 2 * 4 + 2) = tf * tf;
	*(trans_arr + 2 * 4 + 3) = tf * tf * tf;
	*(trans_arr + 3 * 4 + 1) = 1;
	*(trans_arr + 3 * 4 + 2) = 2 * tf;
	*(trans_arr + 3 * 4 + 3) = 3 * tf * tf;

	temp.pMatrix = trans_arr;
	temp.row = 4;
	temp.column = 4;

	return temp;
}

/**
  * @brief  产生用于3次多项式轨迹规划的多项式参数矩阵
  * @param  init_pos: 	初始点位置[mm]
  * 		init_vel：	初始点速度[mm/s]
  * 		final_pos:	终止点位置[mm]
  * 		final_vel:	终止点速度[mm/s]
  * @retval 用于3次多项式轨迹规划的多项式参数矩阵[4x1]
  */
Matrix_t Traj3_Para_Mat(float init_pos, float init_vel, float final_pos, float final_vel)
{
	int i;
	Matrix_t temp;
	float *coeffi_arr = (float *)malloc(sizeof(float) * 4);
	if (coeffi_arr == NULL)
		printf("ERROR: Malloc failed!-04");
	for (i = 0; i < 4; i++)
		*(coeffi_arr + i) = 0; //MD 原来写的是*(coeffi_arr + 4 * i ) = 0;  我是傻逼  一直内存报错  但是VS真的牛逼  在树莓派上跑竟然没问题
	*(coeffi_arr + 0 * 4 + 0) = init_pos;
	*(coeffi_arr + 0 * 4 + 1) = init_vel;
	*(coeffi_arr + 0 * 4 + 2) = final_pos;
	*(coeffi_arr + 0 * 4 + 3) = final_vel;

	temp.pMatrix = coeffi_arr;
	temp.row = 4;
	temp.column = 1;

	return temp;
}

/**
  * @brief  求4阶矩阵的行列式结果
  * @param  sMatrix: 	要求行列式结果的矩阵
  * @retval 4阶矩阵的行列式结果
  */
float Det4(Matrix_t sMatrix)
{
	float arr[3][3];
	float sum = 0;
	int i, x, y;

	if (sMatrix.row == 4 && sMatrix.column == 4)
	{
		for (i = 0; i < 4; i++)
		{
			for (x = 0; x < 4; x++)
			{
				if (x != i)
				{
					for (y = 1; y < 4; y++)
					{
						if (x < i)
							arr[y - 1][x] = *(sMatrix.pMatrix + y * sMatrix.column + x);
						else
							arr[y - 1][x - 1] = *(sMatrix.pMatrix + y * sMatrix.column + x);
					}
				}
			}
			if (i == 0 || i == 2)
				sum = sum + *(sMatrix.pMatrix + i) * (arr[0][0] * arr[1][1] * arr[2][2] + arr[0][1] * arr[1][2] * arr[2][0] + arr[0][2] * arr[1][0] * arr[2][1] - arr[0][0] * arr[1][2] * arr[2][1] - arr[0][1] * arr[1][0] * arr[2][2] - arr[0][2] * arr[1][1] * arr[2][0]);
			else
				sum = sum + *(sMatrix.pMatrix + i) * (-1) * (arr[0][0] * arr[1][1] * arr[2][2] + arr[0][1] * arr[1][2] * arr[2][0] + arr[0][2] * arr[1][0] * arr[2][1] - arr[0][0] * arr[1][2] * arr[2][1] - arr[0][1] * arr[1][0] * arr[2][2] - arr[0][2] * arr[1][1] * arr[2][0]);
		}
		return sum;
	}
	else
	{
		printf("ERROR: Cannot calculate determinant, matrix must be a 4x4 square matrix!\n");
		return 0;
	}
}

/**
  * @brief  求3阶矩阵的行列式结果
  * @param  sMatrix: 	要求行列式结果的矩阵
  * @retval 3阶矩阵的行列式结果
  */
float Det3(Matrix_t sMatrix)
{
	float *arr = sMatrix.pMatrix;
	float sum = 0;

	if (sMatrix.row == 3 && sMatrix.column == 3)
	{
		sum = *(arr + 0 * 3 + 0) * *(arr + 1 * 3 + 1) * *(arr + 2 * 3 + 2) + *(arr + 0 * 3 + 1) * *(arr + 1 * 3 + 2) * *(arr + 2 * 3 + 0) + *(arr + 0 * 3 + 3) * *(arr + 1 * 2 + 0) * *(arr + 2 * 3 + 1) - *(arr + 0 * 3 + 0) * *(arr + 1 * 3 + 2) * *(arr + 2 * 3 + 1) - *(arr + 0 * 3 + 1) * *(arr + 1 * 3 + 0) * *(arr + 2 * 3 + 2) - *(arr + 0 * 3 + 2) * *(arr + 1 * 3 + 1) * *(arr + 2 * 3 + 0);
		return sum;
	}
	else
	{
		printf("ERROR: Cannot calculate determinant, matrix must be a 4x4 square matrix!\n");
		return 0;
	}
}

/**
  * @brief  求4阶矩阵的伴随矩阵
  * @param  sMatrix: 	要求伴随矩阵的矩阵
  * @retval 伴随矩阵
  */
Matrix_t Companion_Mat4(Matrix_t sMatrix) //严格意义上讲矩阵是没有代数余子式的，代数余子式属于行列式|A|的元素aij，而非矩阵元素aij
{
	float arr[3][3];
	float *arr_res;
	int i, j, x, y;
	Matrix_t mat_err = {NULL, 0, 0};
	arr_res = (float *)malloc(sizeof(float) * 4 * 4);
	if (arr_res == NULL)
	{
		printf("ERROR: Malloc failed!-03");
		return mat_err;
	}
	if (sMatrix.row == 4 && sMatrix.column == 4) //伴随矩阵用于求矩阵的逆，矩阵有逆的必要条件是矩阵是个方阵
	{
		for (i = 0; i < 4; i++)
		{
			for (j = 0; j < 4; j++)
			{
				for (x = 0; x < 4; x++)
				{
					for (y = 0; y < 4; y++)
					{
						if (x != i)
						{
							if (y != j)
							{
								if (y < j) //选出矩阵的余子式
								{
									if (x < i)
										arr[y][x] = *(sMatrix.pMatrix + y * sMatrix.column + x);
									else
										arr[y][x - 1] = *(sMatrix.pMatrix + y * sMatrix.column + x);
								}
								else
								{
									if (x < i)
										arr[y - 1][x] = *(sMatrix.pMatrix + y * sMatrix.column + x);
									else
										arr[y - 1][x - 1] = *(sMatrix.pMatrix + y * sMatrix.column + x);
								}
							}
						}
					}
				}
				if ((i + j) % 2 == 0) //计算代数余子式的值并将值存放在结果伴随矩阵的相应位置
					*(arr_res + i * 4 + j) = Det3(Arr2Mat(arr[0], 3, 3));
				else
					*(arr_res + i * 4 + j) = (-1) * Det3(Arr2Mat(arr[0], 3, 3));
			}
		}
		return Arr2Mat(arr_res, 4, 4);
	}
	else
	{
		printf("ERROR: Cannot calculate determinant, matrix must be a 4x4 square matrix!\n");
		return mat_err;
	}
}

/**
  * @brief  求4阶矩阵的逆矩阵
  * @param  sMatrix: 	要求逆矩阵的矩阵
  * @retval 逆矩阵
  */
Matrix_t Inv4(Matrix_t sMatrix)
{
	Matrix_t cp_mat;
	Matrix_t mat_err = {NULL, 0, 0};
	float det;
	int x, y;
	cp_mat = Companion_Mat4(sMatrix);			 //求伴随矩阵
	det = Det4(sMatrix);						 //求矩阵行列式
	if (sMatrix.row == 4 && sMatrix.column == 4) //(A^-1) = 1/|A|*(A*)
	{
		for (x = 0; x < 4; x++)
		{
			for (y = 0; y < 4; y++)
			{
				*(cp_mat.pMatrix + x * 4 + y) /= det;
			}
		}
		return cp_mat;
	}
	else
	{
		printf("ERROR: Cannot calculate inverse matrix, matrix must be a 4x4 square matrix!\n");
		return mat_err;
	}
}

/**
  * @brief  将一个浮点型数组转换为矩阵类型
  * @param  pMatrix:	指向浮点型数组的指针
  * 		row:		浮点型数组的行数
  * 		column:		浮点型数组的列数
  * @retval 转换后的矩阵
  */
Matrix_t Arr2Mat(float *pMatrix, uint8_t row, uint8_t column)
{
	Matrix_t temp;
	temp.pMatrix = pMatrix;
	temp.row = row;
	temp.column = column;
	return temp;
}

/**
  * @brief  矩阵乘法
  * @param  num:		进行矩阵乘法的矩阵个数
  * 		...[matrix]:进行矩阵乘法的矩阵，数量可变，但必须与num的值相一致！
  * @retval 矩阵乘法运算结果
  */
Matrix_t MatrixMultiply(int num, ...)
{
	int i;
	float sum = 0;
	uint8_t c, r, k;
	Matrix_t mat_group[MATRIX_MAX_MULTIPLY];
	Matrix_t mat_res;
	Matrix_t mat_temp;
	Matrix_t mat_err = {NULL, 0, 0};

	va_list argp;
	va_start(argp, num);
	for (i = 0; i < num; i++)
	{
		mat_group[i] = va_arg(argp, Matrix_t);
	}
	va_end(argp);

	if (num < 2)
	{
		printf("ERROR: Martixs not enough!\n");
		return mat_err;
	}
	else
	{
		mat_res = mat_group[0];
		for (i = 1; i < num; i++)
		{
			if (mat_group[i - 1].column != mat_group[i].row)
			{
				printf("ERROR: Martixs sizes don't match!\n");
			}
			mat_temp.row = mat_res.row;
			mat_temp.column = mat_group[i].column;
			mat_temp.pMatrix = (float *)malloc(sizeof(float) * mat_temp.row * mat_temp.column); //注意有无内存泄漏
			if (mat_temp.pMatrix == NULL)
				printf("ERROR: Malloc failed!-00");
			for (r = 0; r < mat_res.row; r++)
			{
				for (c = 0; c < mat_group[i].column; c++)
				{
					for (k = 0; k < mat_res.column; k++)
					{
						sum = sum + *(mat_res.pMatrix + mat_res.column * r + k) * *(mat_group[i].pMatrix + k * mat_group[i].column + c);
					}
					*(mat_temp.pMatrix + r * mat_temp.column + c) = sum;
					sum = 0;
				}
			}
			if (i != 1)
				free(mat_res.pMatrix);
			mat_res = mat_temp;
		}
		//Martix_display(mat_temp);
	}
	return mat_res;
}
