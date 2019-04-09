#ifndef __MATRIX_H
#define __MATRIX_H

#include "stdarg.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#define PI (float)3.141592654f

/* Private define -------------------------------------------------------------*/
#define MATRIX_MAX_MULTIPLY 10 //最大连乘矩阵数量

/* 定义新类型：	@Matrix_t	矩阵类型 */
typedef struct
{
  float *pMatrix; //float类型二维数组指
  uint8_t row;    //矩阵行数
  uint8_t column; //矩阵列数
} Matrix_t;

/** @attention	由于matrix类型数据只能保存二位数组的指针，因此定义matrix数组前，需要用
  *					float* ptr = (float*)malloc(sizeof(float) * [r] * [c]);
  *				或
  *					float arr[r][c];
  *				来划分一片内存区域用于存储矩阵内容。
  *
  *				注意这两种定义方式各有优缺点：
  *				 + 使用malloc()函数划分区域时，应当注意变量失效（如在子函数中定义但
  *				不作为返回值返回、matrix变量相互赋值）前，应当释放malloc()划分的内存
  *				，否则将导致内存泄露
  *				 + 在子函数中使用二维数组划分区域时，若要将matrix变量返回给上层函数，
  *				由于自动类型变量在子函数结束后内存会被回收的特性，返回给上层函数的
  *				matrix变量实际上将只有尺寸而没有任何数据
  *				 + 因此在子函数中使用matrix变量时，若其不作为返回值，建议使用定义数组
  *				方式，若要作为返回值，应使用malloc()方式，但要注意内存回收。
  */

/* Display functions -----------------------------------------------------------*/
void MatrixDisplay(Matrix_t sMatrix); //打印矩阵

/* Arithmetic operators --------------------------------------------------------*/
Matrix_t SE3_Rx(float degree);                                                             //产生绕x轴旋转的齐次变换矩阵
Matrix_t SE3_Ry(float degree);                                                             //产生绕y轴旋转的齐次变换矩阵
Matrix_t SE3_Rz(float degree);                                                             //产生绕z轴旋转的齐次变换矩阵
Matrix_t SE3_T(float x, float y, float z);                                                 //产生平移的齐次变换矩阵
Matrix_t Traj3_Mat(float tf);                                                              //产生用三次多项式进行轨迹规划的方程系数矩阵
Matrix_t Traj3_Para_Mat(float init_pos, float init_vel, float final_pos, float final_vel); //产生用三次多项式进行轨迹规划的系数矩阵

/* Arithmetical operation ------------------------------------------------------*/
float Det3(Matrix_t sMatrix);              //计算3阶矩阵的行列式
float Det4(Matrix_t sMatrix);              //计算4阶矩阵的行列式
Matrix_t Companion_Mat4(Matrix_t sMatrix); //计算4阶矩阵的伴随矩阵
Matrix_t Inv4(Matrix_t sMatrix);           //计算4阶矩阵的逆矩阵
Matrix_t MatrixMultiply(int num, ...);     //矩阵乘法运算

/* Auxiliary operation ------------------------------------------------------*/
Matrix_t Arr2Mat(float *pArr, uint8_t row, uint8_t column); //将一个浮点型数组转换为矩阵类型

#endif
