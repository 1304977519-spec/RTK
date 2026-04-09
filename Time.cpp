#include "Struct.h"
/****************************************************************************
函数编号：    01
函数目的：    实现通用时到简化儒略日的算法
变量含义：
COMMONTIME    通用时的结构体，包含年月日时分秒的成员
MJDTIME       简化儒略日的结构体，包含整数部分和小数部分的成员
编写时间：2024.9.23
****************************************************************************/
void CommonTimeToMjdTime(COMMONTIME Ctime, MJDTIME& MJD) {
	if (Ctime.Month <= 2) {
		Ctime.Month += 12;
		Ctime.Year -= 1;
	}
	double UT = (Ctime.Hour  + (double)Ctime.Minute / 60 + (double)Ctime.Second / 3600) / 24;
	double JD = int(365.25 * Ctime.Year) + int(30.6001 * (Ctime.Month + 1)) + Ctime.Day + UT + 1720981.5;
	MJD.Days = int(JD - 2400000.5);
	MJD.FracDay = UT - (int)UT;
}
/****************************************************************************
函数编号：    02
函数目的：    实现简化儒略日到通用时的算法
变量含义：
MJDTIME       简化儒略日的结构体，包含整数部分和小数部分的成员
COMMONTIME    通用时的结构体，包含年月日时分秒的成员
编写时间：2024.9.23
****************************************************************************/
void MjdTime2CommonTime(MJDTIME MJD, COMMONTIME& Ctime) {
	double JD = MJD.Days + MJD.FracDay + 2400000.5;
	int A = int(JD + 0.5);
	int b = A + 1537;
	int c = int((b - 122.1) / 365.25);
	int d = int(365.25 * c);
	int e = int((b - d) / 30.6001);
	double FracDay = JD + 0.5  - (int)(JD + 0.5);
	Ctime.Day = b - d - int(30.6001 * e);
	Ctime.Month = e - 1 - 12 * int(e / 14);
	Ctime.Year = c - 4715 - int((7 + Ctime.Month) / 10);
	Ctime.Hour = int(FracDay * 24);
	Ctime.Minute = int((FracDay * 24 - Ctime.Hour) * 60);
	Ctime.Second = double(((FracDay * 24 - Ctime.Hour) * 60 - Ctime.Minute) * 60);
}
/****************************************************************************
函数编号：    03
函数目的：    实现简化儒略日到GPS时的转换算法
变量含义：
MJDTIME       简化儒略日的结构体，包含整数部分和小数部分的成员
GPSTIME       GPS时的结构体，包含周和周内秒的成员
编写时间：2024.9.23
****************************************************************************/
void MjdTime2GPSTime(MJDTIME MJD, GPSTIME& GPSTime) {
	double Mjd = MJD.Days + MJD.FracDay;
	GPSTime.Week = int((Mjd - 44244) / 7);
	GPSTime.SecOfWeek = (Mjd - 44244 - GPSTime.Week * 7) * 86400;
}
/****************************************************************************
函数编号：    04
函数目的：    实现GPS时到简化儒略日的转换算法
变量含义：
GPSTIME       GPS时的结构体，包含周和周内秒的成员
MJDTIME       简化儒略日的结构体，包含整数部分和小数部分的成员
编写时间：2024.9.23
****************************************************************************/
void GPSTime2MjdTime(GPSTIME GPSTime, MJDTIME& MJD) {
	double Mjd = 44244 + GPSTime.Week * 7 + GPSTime.SecOfWeek / 86400;
	MJD.Days = (int)Mjd;
	MJD.FracDay = Mjd - MJD.Days;
}
/****************************************************************************
函数编号：    05
函数目的：    实现通用时时到GPS时的转换算法
变量含义：
COMMONTIME    通用时的结构体，包含年月日时分秒的成员
GPSTIME       GPS时的结构体，包含周和周内秒的成员
编写时间：2024.9.24
****************************************************************************/
void CommonTime2GPSTime(COMMONTIME Ctime, GPSTIME& GPSTime) {
	MJDTIME MJD;
	CommonTimeToMjdTime(Ctime, MJD);
	MjdTime2GPSTime(MJD, GPSTime);
}
/****************************************************************************
函数编号：    06
函数目的：    实现GPS时到通用时的转换算法
变量含义：
GPSTIME       GPS时的结构体，包含周和周内秒的成员
COMMONTIME    通用时的结构体，包含年月日时分秒的成员
编写时间：2024.9.24
****************************************************************************/
void GPSTime2CommonTime(GPSTIME GPSTime, COMMONTIME& Ctime) {
	MJDTIME MJD;
	GPSTime2MjdTime(GPSTime, MJD);
	MjdTime2CommonTime(MJD, Ctime);
}