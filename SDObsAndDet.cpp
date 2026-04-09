#include "Struct.h"
#define DeltaGF 0.05
#define DeltaMW 3.0
/****************************************************************************
函数编号：    01
函数目的：    实现站间单差
变量含义：
EpkA        基站历元的数据
EpkB        流动站历元的数据
SDObs       每个历元的单差观测数据
编写时间：2025.3.3
****************************************************************************/
void FormSDEpochObs(EPOCHOBSDATA* EpkA, EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs) {
	SDObs->Time.Week = EpkB->Time.Week;
	SDObs->Time.SecOfWeek = EpkB->Time.SecOfWeek;
	int satnum = 0;
	for (int i = 0; i < EpkA->SatNum; i++)
	{
		for (int j = 0; j < EpkB->SatNum; j++)
		{
			if (EpkA->SatObs[i].System != EpkB->SatObs[j].System || EpkA->SatObs[i].Prn != EpkB->SatObs[j].Prn) continue;
			if (EpkA->SatObs[i].Valid == 1 && EpkA->SatPVT[i].Valid == 1 && 
				EpkB->SatObs[j].Valid == 1 && EpkB->SatPVT[j].Valid == 1 && 
				EpkA->SatObs[i].Half[0] != 0 && EpkA->SatObs[i].Half[1] != 0 && 
				EpkB->SatObs[i].Half[0] != 0 && EpkB->SatObs[i].Half[1] != 0)
			{
					for (int k = 0; k < 2; k++)
					{
						SDObs->SdSatObs[satnum].dL[k] =  EpkB->SatObs[j].L[k] - EpkA->SatObs[i].L[k];
						SDObs->SdSatObs[satnum].dP[k] =  EpkB->SatObs[j].P[k] - EpkA->SatObs[i].P[k];
					}
					SDObs->SdSatObs[satnum].System = EpkA->SatObs[i].System;
					SDObs->SdSatObs[satnum].Prn = EpkA->SatObs[i].Prn;
					SDObs->SdSatObs[satnum].nBas = i;
					SDObs->SdSatObs[satnum].nRov = j;
					satnum++;
			}
			else continue;
		}
	}
	SDObs->SatNum = satnum;
}

/****************************************************************************
函数编号：    2
函数目的：    实现周跳探测与半周标记
变量含义：
SDObs       每个历元的单差观测数据
编写时间：2025.3.3
****************************************************************************/
void DetectCycleSlip(SDEPOCHOBS* Obs) {
	double f1, f2, MW, GF, dMW, dGF;
	
	for (int i = 0; i < Obs->SatNum; i++) {
		// 检查双频伪距和相位数据是否完整
		for (int j = 0; j < 2; j++) {
			if (Obs->SdSatObs[i].dP[j] == 0 || Obs->SdSatObs[i].dL[j] == 0) {
				Obs->SdSatObs[i].Valid = 0;
				break;
			}
		}
		if (!Obs->SdSatObs[i].Valid) continue;

		Obs->SdCObs[i].Prn = Obs->SdSatObs[i].Prn;
		Obs->SdCObs[i].Sys = Obs->SdSatObs[i].System;
		// 设置频率值
		if (Obs->SdCObs[i].Sys == GPS) {
			f1 = FG1_GPS;
			f2 = FG2_GPS;
		}
		else if (Obs->SdCObs[i].Sys == BDS) {
			f1 = FG1_BDS;
			f2 = FG3_BDS;
		}
		else {
			continue;
		}

		// 计算GF和MW
		MW = (1 / (f1 - f2)) * (f1 * Obs->SdSatObs[i].dL[0] - f2 * Obs->SdSatObs[i].dL[1])
			- (1 / (f1 + f2)) * (f1 * Obs->SdSatObs[i].dP[0] + f2 * Obs->SdSatObs[i].dP[1]);
		GF = Obs->SdSatObs[i].dL[0] - Obs->SdSatObs[i].dL[1];


		// 计算当前历元与上一历元的差值
		dMW = fabs(MW - Obs->SdCObs[i].MW);
		dGF = fabs(GF - Obs->SdCObs[i].GF);

		if (fabs(Obs->SdCObs[i].MW) < 10e-6 && fabs(Obs->SdCObs[i].GF) < 10e-6) dMW = dGF = 0;

		// 检查是否超限
		if (dGF < DeltaGF && dMW < DeltaMW) {
			// 数据有效，更新平滑值
			Obs->SdSatObs[i].Valid = 1;
			Obs->SdCObs[i].MW = (Obs->SdCObs[i].n * Obs->SdCObs[i].MW + MW) / (Obs->SdCObs[i].n + 1);
			Obs->SdCObs[i].GF = GF;
			Obs->SdCObs[i].n++;
		}
		else {
			// 数据无效
			Obs->SdCObs[i].n = 0;
			Obs->SdCObs[i].MW = MW;
			Obs->SdCObs[i].GF = GF;
			Obs->SdSatObs[i].Valid = 0;
		}

		// 计算伪距的F组合观测值
		if (Obs->SdSatObs[i].Valid) {
			Obs->SdCObs[i].PIF = (1 / (f1 * f1 - f2 * f2)) *
				(f1 * f1 * Obs->SdSatObs[i].dP[0] - f2 * f2 * Obs->SdSatObs[i].dP[1]);
		}

	}
}