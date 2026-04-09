#include "Struct.h"
#include <cmath>

//对流层延迟的常量
#define H0 0
#define T0 15 + 273.16
#define P0 1013.25
#define RH0 0.5
#define HW 11000
#define DeltaGF 0.05
#define DeltaMW 3.0
#define R 6378000
#define h 375000
/****************************************************************************
函数编号：    01
函数目的：    对流层改正函数设计
变量含义：
H       测站高度
Elev    高度角
编写时间：2024.11.22
****************************************************************************/
double Hopfield(const double H, const double Elev) {
	if (H > 10e4 || H < -100) return 0;
	double dtrop, kd, kw, p, t, hd, rh, e0;
	p = P0 * pow(1 - 0.0000226 * (H - H0), 5.225);
	t = T0 - 0.0065 * (H - H0);
	rh = RH0 * exp(-0.0006396 * (H - H0));
	e0 = rh * exp(-37.2465 + 0.213166 * t - 0.000256908 * t * t);
	hd = 40136 + 148.72 * (T0 - 273.16);
	kw = 155.2e-7 * 4810 / (t * t) * e0 * (HW - H);
	kd = 155.2e-7 * p / t * (hd - H);
	dtrop = kd / sin(sqrt(Elev * Elev + 6.25) * Pi / 180.0) + kw / sin(sqrt(Elev * Elev + 2.25) * Pi / 180.0);
	return dtrop;
}

/****************************************************************************
函数编号：    02
函数目的：    粗差探测函数设计
变量含义：
Obs      观测数据
编写时间：2024.11.22
****************************************************************************/
void DetectOutlier(EPOCHOBSDATA* Obs) {
    double f1, f2, MW, GF, dMW, dGF;

    for (int i = 0; i < Obs->SatNum; i++) {
        Obs->SatObs[i].Valid = 1;

        // 检查双频伪距和相位数据是否完整
        for (int j = 0; j < 2; j++) {
            if (Obs->SatObs[i].P[j] == 0 || Obs->SatObs[i].L[j] == 0) {
                Obs->SatObs[i].Valid = 0;
                break;
            }
        }
        if (!Obs->SatObs[i].Valid) continue;

        Obs->ComObs[i].Prn = Obs->SatObs[i].Prn;
        Obs->ComObs[i].Sys = Obs->SatObs[i].System;
        // 设置频率值
        if (Obs->SatObs[i].System == GPS) {
            f1 = FG1_GPS;
            f2 = FG2_GPS;
        }
        else if (Obs->SatObs[i].System == BDS) {
            f1 = FG1_BDS;
            f2 = FG3_BDS;
        }
        else {
            continue;
        }

        // 计算GF和MW
        MW = (1 / (f1 - f2)) * (f1 * Obs->SatObs[i].L[0] - f2 * Obs->SatObs[i].L[1])
            - (1 / (f1 + f2)) * (f1 * Obs->SatObs[i].P[0] + f2 * Obs->SatObs[i].P[1]);
        GF = Obs->SatObs[i].L[0] - Obs->SatObs[i].L[1];


        // 计算当前历元与上一历元的差值
        dMW = fabs(MW - Obs->ComObs[i].MW);
        dGF = fabs(GF - Obs->ComObs[i].GF);

        if (fabs(Obs->ComObs[i].MW) < 10e-6 && fabs(Obs->ComObs[i].GF) < 10e-6) dMW = dGF = 0;

        // 检查是否超限
        if (dGF < DeltaGF && dMW < DeltaMW) {
            // 数据有效，更新平滑值
            Obs->SatObs[i].Valid = 1;
            Obs->ComObs[i].MW = (Obs->ComObs[i].n * Obs->ComObs[i].MW + MW) / (Obs->ComObs[i].n + 1);
            Obs->ComObs[i].GF = GF;
            Obs->ComObs[i].n++;
        }
        else {
            // 数据无效
            Obs->ComObs[i].n = 0;
            Obs->ComObs[i].MW = MW;
            Obs->ComObs[i].GF = GF;
            Obs->SatObs[i].Valid = 0;
        }

        // 计算伪距的F组合观测值
        if (Obs->SatObs[i].Valid) {
            Obs->ComObs[i].PIF = (1 / (f1 * f1 - f2 * f2)) *
                (f1 * f1 * Obs->SatObs[i].P[0] - f2 * f2 * Obs->SatObs[i].P[1]);
        }


    }


}