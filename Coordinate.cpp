#include "Struct.h"

//椭球参数
#define a 6378137
#define f 1 / 298.257223563
#define ec sqrt(2 * f - f * f)
/****************************************************************************
函数编号：    01
函数目的：    实现大地坐标到笛卡尔坐标的转换算法
变量含义：
Blh    大地坐标系
Xyz    笛卡尔坐标系
编写时间：2024.9.23
****************************************************************************/
void BLHToXYZ(BLh Blh, XYZ& Xyz) {
	double N;
	Blh.latitude = Blh.latitude * Pi / 180;
	Blh.longitude = Blh.longitude * Pi / 180;
	N = a / sqrt(1 - ec * ec * sin(Blh.latitude) * sin(Blh.latitude));
	Xyz.x = (N + Blh.height)  * cos(Blh.latitude) * cos(Blh.longitude);
	Xyz.y = (N + Blh.height)  * cos(Blh.latitude) * sin(Blh.longitude);
	Xyz.z = (N * (1 - ec * ec) + Blh.height) * sin(Blh.latitude);
}
/****************************************************************************
函数编号：    02
函数目的：    实现笛卡尔坐标到大地坐标的转换算法
变量含义：
 Blh   大地坐标系
Xyz    笛卡尔坐标系
编写时间：2024.9.23
****************************************************************************/
void XYZToBLH(XYZ Xyz, BLh& Blh) {
	// 初始化
	if (std::fabs(Xyz.x) < 1e-6 && std::fabs(Xyz.y) < 1e-6 && std::fabs(Xyz.z) < 1e-6) {
		Blh.latitude = 0;
		Blh.longitude = 0;
		Blh.height = 0;
		return;
	}

	// 计算平面距离
	double p = std::sqrt(Xyz.x * Xyz.x + Xyz.y * Xyz.y);
	Blh.longitude = std::atan2(Xyz.y, Xyz.x); // 经度（弧度）

	// 初始纬度估计
	double phi = std::atan2(Xyz.z, p * (1 - ec * ec));
	double phi_prev = 0.0; // 前一次纬度值
	double N = 0.0;        // 曲率半径
	double h = 0.0;        // 高程

	// 迭代计算纬度和高程
	while (std::fabs(phi - phi_prev) > 1e-12) { // 精度阈值
		phi_prev = phi;
		N = a / std::sqrt(1 - ec * ec * std::sin(phi) * std::sin(phi));
		h = p / std::cos(phi) - N;
		phi = std::atan2(Xyz.z + ec * ec * N * std::sin(phi), p);
	}


	Blh.height = h;
	// 将经度和纬度转换为度
	Blh.latitude = phi * 180.0 / Pi;
	Blh.longitude = Blh.longitude * 180.0 / Pi;
}

/****************************************************************************
函数编号：    03
函数目的：    实现测站地平坐标转换矩阵计算函数
变量含义：
Xs       卫星的笛卡尔坐标
Xr       测站的笛卡尔坐标
enu       卫星的站心坐标
编写时间：2024.9.24
****************************************************************************/
void XYZ2ENU(XYZ Xr, XYZ Xs, NEU& neu) {
	BLh BlhStations;
	Matrix<double, 3, 1> Dneu;
	Matrix<double, 3, 1> Dxyz;
	Matrix<double, 3, 3> r;
	XYZToBLH(Xr, BlhStations);
	double lat = BlhStations.latitude * Pi / 180.0; // 转弧度
	double lon = BlhStations.longitude * Pi / 180.0; // 转弧度
	r << -sin(lon), cos(lon), 0,
		-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat),
		cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat);
	Dxyz << Xs.x - Xr.x,
			Xs.y - Xr.y,
			Xs.z - Xr.z;

	Dneu = r * Dxyz;
	neu.dE = Dneu(0, 0);
	neu.dN = Dneu(1, 0);
	neu.dU = Dneu(2, 0);
}
/****************************************************************************
函数编号：    04
函数目的：    实现卫星高度角方位角计算函数和方位角计算
变量含义：
Xs       卫星的笛卡尔坐标
Xr       测站的笛卡尔坐标
Elev     高度角
Azim     方位角
编写时间：2024.9.24
****************************************************************************/
void CompSatElAz(XYZ Xr, XYZ Xs, double& Elev, double& Azim) {
	NEU neu;
	XYZ2ENU(Xr, Xs, neu);
	Elev = atan(neu.dU / sqrt(neu.dE * neu.dE + neu.dN * neu.dN)) / Pi * 180;
	Azim = atan2(neu.dE, neu.dN);
}
/****************************************************************************
函数编号：    05
函数目的：    实现测站地平系的定位误差
变量含义：
X0       测站的精确笛卡尔坐标 
Xr       测站的笛卡尔坐标
enu      测站的定位误差
编写时间：2024.9.24
****************************************************************************/
void CompEnudPos(XYZ Xr, XYZ X0, NEU& neu) {
	XYZ2ENU(Xr, X0, neu);
}