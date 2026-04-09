#include "Struct.h"
/****************************************************************************
函数编号：    01
函数目的：    实现向量的加法计算
变量含义：
Dim1         第一个向量的维度
Dim2         第二个向量的维度
Vector1[]    第一个向量具体的值
Vector2[]    第二个向量具体的值
Vector3[]    第一二个向量相加的结果向量
返回值       0表示函数运行正常，-1表示函数运行错误
编写时间：2024.9.20
****************************************************************************/
int VectorAddtion(int Dim1, int Dim2, double Vector1[], double Vector2[], double Vector3[]) {
	if (Dim1 == Dim2) {
		for (int i = 0; i < Dim1; i++) {
			Vector3[i] = Vector1[i] + Vector2[i];
		}
		return 0;
	}
	else {
		cout << "Vector error!The dimensions of the additive vectors do not match!";
		return -1;
	}

}
/****************************************************************************
函数编号：    02
函数目的：    实现向量的减法计算
变量含义：
Dim1         第一个向量的维度
Dim2         第二个向量的维度
Vector1[]    第一个向量具体的值
Vector2[]    第二个向量具体的值
Vector3[]    第一二个向量相减的结果向量
返回值       0表示函数运行正常，-1表示函数运行错误
编写时间：2024.9.20
****************************************************************************/
int VectorSubtraction(int Dim1, int Dim2, double Vector1[], double Vector2[], double Vector3[]) {
	if (Dim1 == Dim2) {
		for (int i = 0; i < Dim1; i++) {
			Vector3[i] = Vector1[i] - Vector2[i];
		}
		return 0;
	}
	else {
		cout << "Vector error!The dimensions of the subtraction vectors do not match!";
		return -1;
	}
}
/****************************************************************************
函数编号：    03
函数目的：    实现向量的点积计算
变量含义：
Dim1         第一个向量的维度
Dim2         第二个向量的维度
Vector1[]    第一个向量具体的值
Vector2[]    第二个向量具体的值
Answer       第一二个向量点积的结果
返回值       0表示函数运行正常，-1表示函数运行错误
编写时间：2024.9.20
****************************************************************************/
int VectorDotProduct(int Dim1, int Dim2, double Vector1[], double Vector2[], double& Answer) {
	if (Dim1 == Dim2) {
		Answer = 0;
		for (int i = 0; i < Dim1; i++) {
			Answer += Vector1[i] * Vector2[i];
		}
		return 0;
	}
	else {
		cout << "Vector error!The dimensions of the vector dot product do not match!";
		return -1;
	}
}
/****************************************************************************
函数编号：    04
函数目的：    实现向量的叉积计算
变量含义：
Dim1         第一个向量的维度
Dim2         第二个向量的维度
Vector1[]    第一个向量具体的值
Vector2[]    第二个向量具体的值
Vector3[]    第一二个向量叉积的结果向量
返回值       0表示函数运行正常，-1表示函数运行错误
编写时间：2024.9.20
****************************************************************************/
int VectorCrossProduct(int Dim1, int Dim2, double Vector1[], double Vector2[], double Vector3[]) {
	if (Dim1 == Dim2 && Dim1 == 3) {
		Vector3[0] = Vector1[1] * Vector2[2] - Vector1[2] * Vector2[1];
		Vector3[1] = Vector1[0] * Vector2[2] - Vector1[2] * Vector2[0];
		Vector3[2] = Vector1[0] * Vector2[1] - Vector1[1] * Vector2[0];
		return 0;
	}

	else {
		cout << "Vector error!The dimensions of the vector cross product do not match!";
		return -1;
	}
}