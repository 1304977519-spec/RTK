#include "Struct.h"
#pragma warning(disable: 4996)

int main() {

	//unsigned char buff[MAXRAWLEN];
	//GPSEPHREC geph[MAXGPSNUM];
	//GPSEPHREC beph[MAXBDSNUM];
	//SATMIDRES Mid[MAXCHANNUM];
	//EPOCHOBSDATA obs;
	//BLh blh;
	//XYZ xyz;
	//POSRES Pos;
	//POSRES Res;
	//int LenRead, NumWritten = 0, lenR = 0, lenD = 0, isObs = 0;
	//FILE* fp;
	//COMMONTIME t;
	SOCKET  NetGps;
	int count = 0;
	ofstream outFile("PosOutPut.txt");
	ofstream outFile1("ResOutPut.txt");
	ofstream outFile2("OutPutError.txt");
	if (OpenSocket(NetGps, "47.114.134.129", 7190) == false) {
		printf("This ip & port was not opened.\n");
		return 0;
	}

	while (count <= 3600 * 8 + 3000) {
		count++;
		Sleep(980);
		if ((lenR = recv(NetGps, (char*)buff, MAXRAWLEN, 0)) > 0) {
			memcpy(buff + lenD, buff, lenR);
			lenD += lenR;
			DecodeNovOem7Dat(buff, lenD, &obs, geph, beph, &Res);
			GPSTime2CommonTime(obs.Time, t);
			
			if (SPP(&obs, geph, beph, &Pos)) SPV(&obs, &Pos);
			
			if (t.Hour + 8 < 24) std::cout << t.Year << " " << t.Month << " " << t.Day << " " << t.Hour + 8 << " " << t.Minute << " " << (int)round(t.Second) << " " << Pos.SatNum << endl;
			else std::cout << t.Year << " " << t.Month << " " << t.Day + 1 << " " << t.Hour + 8 - 24 << " " << t.Minute << " " << (int)round(t.Second) << " " << Pos.SatNum << endl;
			cout << "XYZ  :" << setiosflags(ios::fixed) << Pos.Pos[0] << "  " << Pos.Pos[1] << "  " << Pos.Pos[2] << endl;
			cout << "PDOP  :" << Pos.PDOP << endl;
			cout << "SigmaPos  :" << Pos.SigmaPos << endl;
			cout << "Vel  :" << Pos.Vel[0] << "  " << Pos.Vel[1] << "  " << Pos.Vel[2] << endl;
			cout << "SigmaVel  :" << Pos.SigmaVel << endl;

			blh.latitude = Res.Pos[0];
			blh.longitude = Res.Pos[1];
			blh.height = Res.Pos[2];
			BLHToXYZ(blh, xyz);
			cout << "ResBLH  :" << Res.Pos[0] << " " << Res.Pos[1] << " " << Res.Pos[2] << endl;
			cout << "ResXYZ  :" << xyz.x << " " << xyz.y << " " << xyz.z << endl;
			cout << "Pos  :" << Pos.Pos[0] << " " << Pos.Pos[1] << " " << Pos.Pos[2] << endl;
			cout << endl;

			outFile << setiosflags(ios::fixed) << setprecision(4) <<
				Pos.Pos[0] << " " << Pos.Pos[1] << " " << Pos.Pos[2] << endl;

			outFile1 << setiosflags(ios::fixed) << setprecision(4) <<
				xyz.x << " " << xyz.y << " " << xyz.z << endl;

			NEU errorENU;  // 存储ENU方向的定位误差
			XYZ pos;
			pos.x = Pos.Pos[0];
			pos.y = Pos.Pos[1];
			pos.z = Pos.Pos[2];

			CompEnudPos(pos, xyz, errorENU);  // 计算ENU方向的误差

			// 输出定位误差（以ENU坐标系表示）
			outFile2 << setiosflags(ios::fixed) << setprecision(8)
				<< errorENU.dE << " "
				<< errorENU.dN << " "
				<< errorENU.dU << " " << std::endl;
		}
	}
	cout << count << endl;
	CloseSocket(NetGps);
	outFile.close();
	outFile1.close();
	outFile2.close();

	//if ((fp = fopen("oem719-202404021900-1.bin", "rb")) == NULL)
	//{
	//	printf("Cannot open GPS obs file. \n");
	//	return 0;
	//}
	//while (!feof(fp)) {
	//	LenRead = fread(buff + NumWritten, 1, MAXRAWLEN - NumWritten, fp);
	//	NumWritten += LenRead;
	//	DecodeNovOem7Dat(buff, NumWritten, &obs, geph, beph, &Res);
	//	GPSTime2CommonTime(obs.Time, t);

	//	if (SPP(&obs, geph, beph, &Pos)) SPV(&obs, &Pos);
	//	else continue;

	//	std::cout << t.Year << " " << t.Month << " " << t.Day << " " << t.Hour + 8 << " " << t.Minute << " " << (int)round(t.Second) << " " << Pos.SatNum << endl;
	//	cout << "XYZ  :" << setiosflags(ios::fixed) << Pos.Pos[0] << "  " << Pos.Pos[1] << "  " << Pos.Pos[2] << endl;
	//	cout << "PDOP  :" << Pos.PDOP << endl;
	//	cout << "SigmaPos  :" << Pos.SigmaPos << endl;
	//	cout << "Vel  :" << Pos.Vel[0] << "  " << Pos.Vel[1] << "  " << Pos.Vel[2] << endl;
	//	cout << "SigmaVel  :" << Pos.SigmaVel << endl;

	//	blh.latitude = Res.Pos[0];
	//	blh.longitude = Res.Pos[1];
	//	blh.height = Res.Pos[2];
	//	BLHToXYZ(blh, xyz);
	//	cout << "ResBLH  :" << Res.Pos[0] << " " << Res.Pos[1] << " " << Res.Pos[2] << endl;
	//	cout << "ResXYZ  :" << xyz.x << " " << xyz.y << " " << xyz.z << endl;
	//	cout << "Pos  :" << Pos.Pos[0] << " " << Pos.Pos[1] << " " << Pos.Pos[2] << endl;
	//	cout << endl;

	//	std::cout << std::endl;
	//}
	//fclose(fp);
	//
	return 0;
}