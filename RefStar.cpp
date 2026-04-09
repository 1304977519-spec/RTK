#include "Struct.h"
/****************************************************************************
函数编号：    01
函数目的：    实现参考星选取
变量含义：
EpkA        基站历元的数据
EpkB        流动站历元的数据
SDObs       每个历元的单差观测数据
DDObs       每个历元的双差观测数据
编写时间：2025.3.3
****************************************************************************/
void DetRefSat(EPOCHOBSDATA* EpkA, EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs) {
	int idSys = 0;
	double MaxEle[2] = { 0 };
	int idMaxEle[2] = { 0 }, SatNum[2] = { 0 }, idPrn[2] = { 0 };
	for (int i = 0; i < SDObs->SatNum; i++) {
		//伪距和载波相位通过周跳探测，没有粗差和周跳标记
		if (SDObs->SdSatObs[i].Valid == 0) continue;
		//每个卫星导航系统各选取一颗卫星作为参考星
		idSys = SDObs->SdSatObs[i].System == GPS ? 0 : 1;
		SatNum[idSys]++;
		//高度角大或CN0大
		if (EpkB->SatPVT[SDObs->SdSatObs[i].nRov].Elevation > MaxEle[idSys]
			&& EpkA->SatObs[SDObs->SdSatObs[i].nBas].Cn0[0] > 40
			&& EpkB->SatObs[SDObs->SdSatObs[i].nRov].Cn0[0] > 40) {
			MaxEle[idSys] = EpkB->SatPVT[SDObs->SdSatObs[i].nRov].Elevation;
			idMaxEle[idSys] = i;
		}
	}
	for (int i = 0; i < 2; i++) {
		if (MaxEle[i] > 0.0) {
			DDObs->RefPos[idSys] = idMaxEle[idSys];
			DDObs->RefPrn[idSys] = SDObs->SdSatObs[idMaxEle[idSys]].Prn;

		}
		else {
			DDObs->RefPrn[i] = 0;
			DDObs->RefPos[i] = -1;
		}        
		if (DDObs->RefPrn[i] != 0) DDObs->DDSatNum[i] = SatNum[i] - 1;
		else DDObs->DDSatNum[i] = 0;
	}
}