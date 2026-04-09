#include "Struct.h"
#pragma warning(disable: 4996)

int main() {
	int Status = 0;
	RAWDAT Raw;
	PPRESULT Base;
	PPRESULT Rov;
	POSRES ResBase;
	POSRES ResRov;
	FILE* FBas = fopen("oem719-202404110900-1.bin", "rb"); // 以二进制模式打开文件
	FILE* FRov = fopen("oem719-202404110900-2.bin", "rb"); // 以二进制模式打开文件
	
	if (!FBas) {
		cout << "open error!" << endl;
		return -1;
	}
	if (!FRov) {
		cout << "open error!" << endl;;
		return -1;
	}
	while (Status != -1) {
		Status = GetSynObsFile(FBas, FRov, &Raw, &Rov);
		if (Status == 0)
		{
			cout << "时间同步失败！" << endl;
			continue;
		}
		if (Status == 1)
		{
			cout << Raw.SdObs.Time.Week << "周" << Raw.SdObs.Time.SecOfWeek << "秒" << endl;
			if (SPP(&Raw.RovEpk, Raw.GpsEph, Raw.BdsEph, &ResRov, &Rov))
			{
				SPV(&Raw.RovEpk, &ResRov, &Rov);
				if (SPP(&Raw.BasEpk, Raw.GpsEph, Raw.BdsEph, &ResBase, &Base))
				{
					SPV(&Raw.BasEpk, &ResBase, &Base);
				}

				FormSDEpochObs(&Raw.BasEpk, &Raw.RovEpk, &Raw.SdObs);
				DetectCycleSlip(&Raw.SdObs);
				DetRefSat(&Raw.BasEpk, &Raw.RovEpk, &Raw.SdObs, &Raw.DDObs);
				if (RTKFloat(&Raw, &Base, &Rov) == false) continue;

				if (RTKlambda(&Raw) == 0) continue;
				

			}
			else
			{
				cout << "流动站单点定位失败" << endl;
			}
			cout << endl;
		}


	}
	fclose(FBas);
	fclose(FRov);
	return 0;
}