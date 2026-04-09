#include "Struct.h"

int main() {
	int Status = 0;

	RAWDAT Raw;

	PPRESULT Base;
	PPRESULT Rov;

	POSRES ResBase;
	POSRES ResRov;

	SOCKET  NetGps;
	SOCKET  NetGps1;

	XYZ xyz;
	XYZ xyz1;
	BLh blh;
	BLh blh1;
	NEU neu;

	if (OpenSocket(NetGps, "8.148.22.229", 7002) == false) {
		printf("This ip & port was not opened.\n");
		return 0;
	}

	if (OpenSocket(NetGps1, "8.148.22.229", 4002) == false) {
		printf("This ip & port was not opened.\n");
		return 0;
	}


	ofstream outFile("PosOutPut.pos");

	ofstream outFile2("OutPutError.txt");

	Status = 0;
	while (Status != -1) {
		Status = RealTimeSync(NetGps, NetGps1, &Raw);
		if (Status == 1)
		{
			Sleep(980);
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

				if (Raw.DDObs.bFixed == 0) continue;
				xyz.x = Raw.DDObs.Rov[0];
				xyz.y = Raw.DDObs.Rov[1];
				xyz.z = Raw.DDObs.Rov[2];

				xyz1.x = Raw.DDObs.bestpos[0];
				xyz1.y = Raw.DDObs.bestpos[1];
				xyz1.z = Raw.DDObs.bestpos[2];

				XYZToBLH(xyz, blh);
				XYZToBLH(xyz1, blh1);

				CompEnudPos(xyz1, xyz, neu);

				cout << neu.dN << " " << neu.dE << " " << neu.dU << endl;

				cout << Raw.DDObs.Rov[0] << " " << Raw.DDObs.Rov[1] << " " << Raw.DDObs.Rov[2] << " " << endl;

				cout << Raw.SdObs.Time.Week << " " << Raw.SdObs.Time.SecOfWeek << " " << setiosflags(ios::fixed) << setprecision(6) <<
					blh.longitude << " " << blh.latitude << " " << blh.height << " " << setiosflags(ios::fixed) << setprecision(0)
					<< Raw.DDObs.bFixed << " " << setiosflags(ios::fixed) << setprecision(6) <<
					blh1.longitude << " " << blh1.latitude << " " << blh1.height << " "
					<< Raw.DDObs.Sats << endl;

				outFile << Raw.SdObs.Time.Week << " " << Raw.SdObs.Time.SecOfWeek << " " << setiosflags(ios::fixed) << setprecision(6) <<
					blh.longitude << " " << blh.latitude << " " << blh.height << " "
					<< Raw.DDObs.Sats << std::endl;

				// 输出定位误差（以ENU坐标系表示）
				outFile2 << Raw.SdObs.Time.Week << " " << Raw.SdObs.Time.SecOfWeek << " " <<
					setiosflags(ios::fixed) << setprecision(8)
					<< neu.dE << " "
					<< neu.dN << " "
					<< neu.dU << " " << std::endl;
			}
			else cout << "流动站单点定位失败" << endl;
			cout << endl;
		}
	}

	outFile.close();
	outFile2.close();
	CloseSocket(NetGps);
	CloseSocket(NetGps1);

	return 0;
}