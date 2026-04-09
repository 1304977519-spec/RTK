#include "Struct.h"
/****************************************************************************
函数编号：    01
函数目的：    实现矩阵的加法计算
变量含义：
Col1         第一个矩阵的列
Row1         第一个矩阵的行
Col2         第二个矩阵的列
Row2         第二个矩阵的行
Matrix1[]    第一个矩阵具体的值
Matrix2[]    第二个矩阵具体的值
Matrix3[]    第一二个矩阵相加的结果矩阵
返回值       0表示函数运行正常，-1表示函数运行错误
编写时间：2024.9.19
****************************************************************************/
int MatrixAddition(int Col1, int Row1, int Col2, int Row2, double Matrix1[], double Matrix2[], double Matrix3[]) {
	if (Col1 == Col2 && Row1 == Row2) {
		for (int i = 0; i < Row1; i++) {
			for (int j = 0; j < Col1; j++) {
				Matrix3[i * Col1 + j] = Matrix1[i * Col1 + j] + Matrix2[i * Col1 + j];
			}
		}
		return 0;
	}
	else {
		cout << "Matrix error!Additive matrix rows and columns do not match!";
		return -1;
	}
}

/****************************************************************************
函数编号：    02
函数目的：    实现矩阵的减法计算
变量含义：
Col1         第一个矩阵的列
Row1         第一个矩阵的行
Col2         第二个矩阵的列
Row2         第二个矩阵的行
Matrix1[]    第一个矩阵具体的值
Matrix2[]    第二个矩阵具体的值
Matrix3[]    第一二个矩阵相减的结果矩阵
返回值       0表示函数运行正常，-1表示函数运行错误
编写时间：2024.9.19
****************************************************************************/
int MatrixSubtraction(int Col1, int Row1, int Col2, int Row2, double Matrix1[], double Matrix2[], double Matrix3[]) {
	if (Col1 == Col2 && Row1 == Row2) {
		for (int i = 0; i < Row1; i++) {
			for (int j = 0; j < Col1; j++) {
				Matrix3[i * Col1 + j] = Matrix1[i * Col1 + j] - Matrix2[i * Col1 + j];
			}
		}
		return 0;
	}
	else {
		cout << "Matrix error!The subtracted matrix rows and columns do not match!";
		return -1;
	}
}

/****************************************************************************
函数编号：    03
函数目的：    实现矩阵的转置
变量含义：
Col          矩阵的列
Row          矩阵的行
Matrix1[]    矩阵具体的值
Matrix2[]    矩阵矩阵转置的结果矩阵
编写时间：2024.9.20
****************************************************************************/
void MatrixTranpose(int Col, int Row, double Matrix1[], double Matrix2[]) {
	for (int i = 0; i < Col; i++) {
		for (int j = 0; j < Row; j++) {
			Matrix2[i * Row + j] = Matrix1[j * Col + i];
		}
	}
}

/****************************************************************************
函数编号：    04
函数目的：    实现矩阵的乘法计算
变量含义：
Col1         第一个矩阵的列
Row1         第一个矩阵的行
Col2         第二个矩阵的列
Row2         第二个矩阵的行
Matrix1[]    第一个矩阵具体的值
Matrix2[]    第二个矩阵具体的值
Matrix3[]    第一二个矩阵相乘的结果矩阵
返回值       0表示函数运行正常，-1表示函数运行错误
编写时间：2024.9.20
****************************************************************************/
int MatrixMultiplies(int Col1, int Row1, int Col2, int Row2, double Matrix1[], double Matrix2[], double Matrix3[]) {
	if (Col1 != Row2) {
		cout << "Matrix error! The number of columns of the first matrix must equal the number of rows of the second matrix!";
		return -1;
	}

	// 遍历结果矩阵的每一行和每一列
	for (int i = 0; i < Row1; ++i) {
		for (int j = 0; j < Col2; ++j) {
			Matrix3[i * Col2 + j] = 0; // 初始化结果元素
			for (int k = 0; k < Col1; ++k) {
				Matrix3[i * Col2 + j] += Matrix1[i * Col1 + k] * Matrix2[k * Col2 + j];
			}
		}
	}
	return 0;
}

/****************************************************************************
函数编号：    05
目的：矩阵求逆,采用全选主元高斯-约当法
变量含义：
n      M1的行数和列数
a      输入矩阵
b      输出矩阵   b=inv(a)
返回值：1=正常，0=致命错误
编写时间：2024.9.20
****************************************************************************/
int MatrixInv(int n, int n1, double a[], double b[])
{
	int i, j, k, l, u, v, is[100], js[100];   /* matrix dimension <= 100 */
	double d, p;

	if (n <= 0)
	{
		printf("Error dimension in MatrixInv!\n");
		exit(EXIT_FAILURE);
	}

	/* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			b[i * n + j] = a[i * n + j];
		}
	}

	for (k = 0; k < n; k++)
	{
		d = 0.0;
		for (i = k; i < n; i++)   /* 查找右下角方阵中主元素的位置 */
		{
			for (j = k; j < n; j++)
			{
				l = n * i + j;
				p = fabs(b[l]);
				if (p > d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (d < 1E-15)   /* 主元素接近于0，矩阵不可逆 */
		{
			//printf("Divided by 0 in MatrixInv!\n");
			return 0;
			//exit(EXIT_FAILURE);
		}

		if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
		{
			for (j = 0; j < n; j++)
			{
				u = k * n + j;
				v = is[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
		{
			for (i = 0; i < n; i++)
			{
				u = i * n + k;
				v = i * n + js[k];
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		l = k * n + k;
		b[l] = 1.0 / b[l];  /* 初等行变换 */
		for (j = 0; j < n; j++)
		{
			if (j != k)
			{
				u = k * n + j;
				b[u] = b[u] * b[l];
			}
		}
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				for (j = 0; j < n; j++)
				{
					if (j != k)
					{
						u = i * n + j;
						b[u] = b[u] - b[i * n + k] * b[k * n + j];
					}
				}
			}
		}
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				u = i * n + k;
				b[u] = -b[u] * b[l];
			}
		}
	}

	for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
	{
		if (js[k] != k)
		{
			for (j = 0; j < n; j++)
			{
				u = k * n + j;
				v = js[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
		if (is[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				u = i * n + k;
				v = is[k] + i * n;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
	}

	return (1);
}
/****************************************************************************
函数编号：    06
函数目的：    实现矩阵重构，删除第n行，第n列的数
变量含义：
Co               矩阵的列
Row              矩阵的行
RowToDelete      删掉第几行
ColToDelete      删掉第几列
a[]              原矩阵
b[]              重构后的矩阵
编写时间：2024.11.22
****************************************************************************/
void DeleteRowAndCol(int Col, int Row, int ColToDelete, int RowToDelete, int deleteType, double p[], double q[]) {
	int newRow = (deleteType == 1) ? Row : Row - 1; // 如果只删列，行数不变；否则减一
	int newCol = (deleteType == 0) ? Col : Col - 1; // 如果只删行，列数不变；否则减一

	if ((deleteType == 2 && (ColToDelete <= 0 || RowToDelete <= 0 || ColToDelete > Col || RowToDelete > Row)) ||
		(deleteType == 0 && (RowToDelete <= 0 || RowToDelete > Row)) ||
		(deleteType == 1 && (ColToDelete <= 0 || ColToDelete > Col))) {
		// 检查参数是否合法
		return;
	}

	for (int i = 0, qi = 0; i < Row; ++i) {
		if (deleteType != 1 && i == RowToDelete - 1) continue; // 如果不是只删列，跳过被删除的行

		for (int j = 0, qj = 0; j < Col; ++j) {
			if (deleteType != 0 && j == ColToDelete - 1) continue; // 如果不是只删行，跳过被删除的列

			// 填充到新矩阵 q
			q[qi * newCol + qj] = p[i * Col + j];
			qj++;
		}
		qi++;
	}
}

/****************************************************************************
函数编号：    07
目的：  矩阵打印
变量含义：
Col       矩阵的列数
Row       矩阵的行数
matrix    输出矩阵 
编写时间：2024.11.28
****************************************************************************/
void PrintMatrix(int Col, int Row, double matrix[]) {
	for (int i = 0; i < Row; ++i) {
		for (int j = 0; j < Col; ++j) {
			cout << setiosflags(ios::fixed) << setprecision(1) << setw(5) << matrix[i * Col + j] << " ";
		}
		cout << endl;
	}
}