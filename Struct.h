	#include <iostream>
	#include <Eigen/Dense>
	#include <Eigen/Core>
	#include <cmath>
	#include <iomanip>
	#include <vector>
	#include <fstream>
	#include<stdio.h>
	#include<windows.h>

	#pragma comment(lib,"WS2_32.lib")
	#pragma warning(disable:4996)

	#define Pi 3.1415926535898
	#define Gu 3.986005e14              //GM
	#define Gomegae 7.2921151467e-5     // GPS地球自转速度
	#define Bu 3.986004418e14           //GM
	#define Bomegae 7.2921150e-5        //BDS地球自转角速度
	#define GPSMAX 7500                 //GPS星历更新时间
	#define BDSMAX 4100                 //GPS星历更新时间
	#define C_Light 299792458.0         //光速
	#define F -4.442807633e-10          //相对论常数



	//数据解码常量
	#define OME7HEADLENTH 28                //头文件的字符数
	#define MAXCHANNUM 36                   //GPS和BDS组合的最大卫星数
	#define MAXRAWLEN 20480                 
	#define MAXGPSNUM 32                    //GPS最大卫星数
	#define MAXBDSNUM 63                    //BDS最大卫星数
	#define POLYCRC32 0xEDB88320u           //CRC32 polynomial

	#define  FG1_GPS  1575.42E6             //L1信号频率 
	#define  FG2_GPS  1227.60E6             //L2信号频率 
	#define  FG12R    (77/60.0)             //FG1_Freq/FG2_Freq
	#define  FG12R2   (5929/3600.0)
	#define  WL1_GPS  (C_Light/FG1_GPS)
	#define  WL2_GPS  (C_Light/FG2_GPS)

	#define  FG1_BDS  1561.098E6              //B1信号的基准频率 
	#define  FG2_BDS  1207.140E6              // B2信号的基准频率 
	#define  FG3_BDS  1268.520E6              //B3信号的基准频率 
	#define  FC12R    (FG1_BDS/FG2_BDS)       // FG1_BDS/FG2_BDS 
	#define  FC12R2   (FC12R*FC12R)           // FG1_BDS^2/FG2_BDS^2 
	#define  FC13R    (FG1_BDS/FG3_BDS)       // FG1_BDS^2/FG3_BDS^2 
	#define  FC13R2   (FC13R*FC13R)
	#define  WL1_BDS  (C_Light/FG1_BDS)
	#define  WL2_BDS  (C_Light/FG2_BDS)
	#define  WL3_BDS  (C_Light/FG3_BDS)       // 波长


	using namespace std;
	using namespace Eigen;

	//笛卡尔坐标
	struct XYZ {
		double x;
		double y;
		double z;
		XYZ() {
			x = 0;
			y = 0;
			z = 0;
		}
	};
	//大地坐标
	struct BLh {
		double longitude;//经度
		double latitude;
		double height;

		BLh() {
			longitude = 0;
			latitude = 0;
			height= 0;
		}
	};
	//测站地平坐标
	struct NEU {
		double dN;
		double dE;
		double dU;

		NEU() {
			dN = 0;
			dE = 0;
			dU = 0;
		}
	};

	//通用时间
	struct COMMONTIME
	{
		short Year;
		unsigned short Month;
		unsigned short Day;
		unsigned short Hour;
		unsigned short Minute;
		double         Second;

		COMMONTIME()
		{
			Year = 0;
			Month = 0;
			Day = 0;
			Hour = 0;
			Minute = 0;
			Second = 0.0;
		}
	};

	//GPS时间定义
	struct GPSTIME              
	{
		unsigned short Week;
		double         SecOfWeek;

		GPSTIME()
		{
			Week = 0;
			SecOfWeek = 0.0;
		}
	};

	//简化儒略日
	struct MJDTIME{
		int Days;
		double FracDay;

		MJDTIME()
		{
			Days = 0;
			FracDay = 0.0;
		}
	};

	//每颗卫星位置、速度和钟差等的中间计算结果 
	struct SATMIDRES
	{
		double SatPos[3], SatVel[3];
		double SatClkOft, SatClkSft;
		double Elevation, Azimuth;
		double TropCorr;
		double Tgd1, Tgd2;
		bool Valid;  //false=没有星历或星历过期,true-计算成功

		SATMIDRES()
		{
			SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
			SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
			Elevation = Pi / 2.0;
			SatClkOft = SatClkSft = 0.0;
			Tgd1 = Tgd2 = TropCorr = 0.0;
			Valid = false;
		}
	};

	/* 导航卫星系统定义 */
	enum GNSSSys { UNKS = 0, GPS, BDS, GLONASS, GALILEO, QZSS };

	/*  每颗卫星的单差观测数据定义  */
	struct SATOBSDATA
	{
		unsigned short Prn;
		GNSSSys System;
		double P[2], L[2], D[2];
		bool Valid;
		double   Cn0[2], LockTime[2], CodeLock[2];
		unsigned char Half[2];

		SATOBSDATA()
		{
			Prn = 0;
			System = UNKS;
			P[0] = L[0] = P[1] = L[1] = D[0] = D[1] = 0.0;
			Cn0[0] = Cn0[0] = LockTime[1] = LockTime[1] = CodeLock[0] = CodeLock[1] = 0;
			Half[0] = Half[1] = 0;
			Valid = true;
		}
	};

	/*每个历元定位结果*/
	struct POSRES
	{
		GPSTIME Time;
		double Pos[3], Vel[3];
		double PDOP, SigmaPos, SigmaVel;
		int SatNum;
		POSRES()
		{
			for (int i = 0; i < 3; i++)
			{
				Pos[i] = Vel[i] = 0.0;
			}
			PDOP = SigmaPos = SigmaVel = 0.0;
			SatNum = 0;
		}
	};


	struct MWGF
	{
		short Prn;//卫星号
		GNSSSys Sys;
		double MW, GF, PIF;

		int n;

		MWGF()
		{
			Prn = n = 0;
			Sys = UNKS;
			MW = GF = PIF = 0.0;
		}
	};

	/* 每个历元的观测数据定义 */
	struct EPOCHOBSDATA
	{

		GPSTIME        Time;
		short          SatNum;
		SATOBSDATA     SatObs[MAXCHANNUM];
		SATMIDRES      SatPVT[MAXCHANNUM]; // 卫星位置等计算结果，数组索引与SatObs相同
		MWGF           ComObs[MAXCHANNUM];  // 当前历元的组合观测值，数组索引与SatObs相同
		double		   Bestpos[3];

		EPOCHOBSDATA()
		{
			SatNum = 0;
			for (int i = 0; i < 3; i++) {
				Bestpos[i] = 0.0;
			}
		}
	};

	// GPS+BDS广播星历
	struct GPSEPHREC
	{
		short PRN;
		GNSSSys Sys;
		GPSTIME TOC, TOE;
		double ClkBias, ClkDrift, ClkDriftRate;
		double IODE, IODC;
		double SqrtA, M0, e, OMEGA, i0, omega;
		double Crs, Cuc, Cus, Cic, Cis, Crc;
		double DeltaN, OMEGADot, iDot;
		int SVHealth;
		double SVAccuracy;
		double TGD1, TGD2;

		GPSEPHREC() {
			PRN = SVHealth = 0;
			Sys = UNKS;
			ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
			SVAccuracy = SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
			Crs = Cuc = Cus = Cic = Cis = Crc = 0.0;
		}
	};

	/*  每颗卫星的单差观测数据定义  */
	struct SDSATOBS
	{
		short    Prn;
		GNSSSys  System;
		short    Valid;
		double   dP[2], dL[2];   // m
		short    nBas, nRov;   // 存储单差观测值对应的基准和流动站的数值索引号

		SDSATOBS()
		{
			Prn = nBas = nRov = 0;
			System = UNKS;
			dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
			Valid = -1;
		}
	};

	//每个历元的单差观测数据定义
	struct SDEPOCHOBS
	{
		GPSTIME    Time;
		short      SatNum;
		SDSATOBS   SdSatObs[MAXCHANNUM];
		MWGF       SdCObs[MAXCHANNUM];

		SDEPOCHOBS()
		{
			SatNum = 0;
		}
	};

	//双差相关的数据定义
	struct DDCOBS
	{
		int RefPrn[2], RefPos[2];         // 参考星卫星号与存储位置，0=GPS; 1=BDS
		int Sats, DDSatNum[2];            // 待估的双差模糊度数量，0=GPS; 1=BDS
		double FixedAmb[MAXCHANNUM * 4];  // 包括双频最优解[0,AmbNum]和次优解[AmbNum,2*AmbNum]
		double ResAmb[2], Ratio;          // LAMBDA浮点解中的模糊度残差
		float  FixRMS[2];                 // 固定解定位中rms误差
		double dPos[6];                   // 基线向量
		bool bFixed;                      // true为固定，false为未固定
		double Qxx[(MAXCHANNUM * 2 + 3) * (MAXCHANNUM * 2 + 3)];
		double dX_amb[MAXCHANNUM * 2 + 3];
		double W[MAXCHANNUM * 4];
		double Rov[3];
		double bestpos[3];
		DDCOBS()
		{
			int i;
			for (i = 0; i < 2; i++) {
				DDSatNum[i] = 0;    // 各卫星系统的双差数量
				RefPos[i] = RefPrn[i] = -1;
			}
			Sats = 0;              // 双差卫星总数
			dPos[0] = dPos[1] = dPos[2] = 0.0;
			dPos[3] = dPos[4] = dPos[5] = 0.0;
			ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = Ratio = 0.0;
			Rov[0] = Rov[1] = Rov[2] = 0.0;
			bestpos[0] = bestpos[1] = bestpos[2] = 0.0;
			bFixed = false;
			for (i = 0; i < MAXCHANNUM * 2; i++)
			{
				FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
			}
			for (int i = 0; i < (MAXCHANNUM * 2 + 3) * (MAXCHANNUM * 2 + 3); i++) {
				Qxx[i] = 0;
			}
			for (int i = 0; i < MAXCHANNUM * 2 + 3; i++) {
				Qxx[i] = 0;
			}
		}
	};

	//定义RTK的结构
	struct RAWDAT {
		EPOCHOBSDATA BasEpk;
		EPOCHOBSDATA RovEpk;
		SDEPOCHOBS SdObs;
		DDCOBS DDObs;
		GPSEPHREC GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM];
		POSRES pos;
	};


	/* 每个历元单点定位和测速的结果及其精度指标 */
	struct PPRESULT
	{
		GPSTIME Time;
		double Position[3];
		double Velocity[3];
		double RcvClkOft[2];               /* 0 为GPS钟差; 1=BDS钟差 */
		double RcvClkSft;
		double PDOP, SigmaPos, SigmaVel;  // 精度指标
		short  GPSSatNum, BDSSatNum;      /* 单点定位使用的GPS卫星数 */
		short  AllSatNum;                /* 观测历元的所有卫星数   */
		bool   IsSuccess;                /* 单点定位是否成功, 1为成功, 0为失败 */
		PPRESULT()
		{
			for (int i = 0; i < 3; i++)		Position[i] = Velocity[i] = 0.0;
			RcvClkOft[0] = RcvClkOft[1] = RcvClkSft = 0.0;
			PDOP = SigmaPos = SigmaVel = 999.9;
			GPSSatNum = BDSSatNum = AllSatNum = 0;
			IsSuccess = false;
		}
	};

	//坐标转换
	void BLHToXYZ(BLh Blh, XYZ& Xyz);
	void XYZToBLH(XYZ Xyz, BLh& Blh);
	void XYZ2ENU(XYZ Xr, XYZ Xs, NEU& neu);
	void CompSatElAz(XYZ Xr, XYZ Xs, double& Elev, double& Azim);
	void CompEnudPos(XYZ Xr, XYZ X0, NEU& neu);

	//时间系统
	void CommonTimeToMjdTime(COMMONTIME Ctime, MJDTIME& MJD);
	void MjdTime2CommonTime(MJDTIME MJD, COMMONTIME& Ctime);
	void MjdTime2GPSTime(MJDTIME MJD, GPSTIME& GPSTime);
	void GPSTime2MjdTime(GPSTIME GPSTime, MJDTIME& MJD);
	void CommonTime2GPSTime(COMMONTIME Ctime, GPSTIME& GPSTime);
	void GPSTime2CommonTime(GPSTIME GPSTime, COMMONTIME& Ctime);

	//矩阵计算
	int MatrixAddition(int Col1, int Row1, int Col2, int Row2, double Matrix1[], double Matrix2[], double Matrix3[]);
	int MatrixSubtraction(int Col1, int Row1, int Col2, int Row2, double Matrix1[], double Matrix2[], double Matrix3[]);
	void MatrixTranpose(int Col, int Row, double Matrix1[], double Matrix2[]);
	int MatrixMultiplies(int Col1, int Row1, int Col2, int Row2, double Matrix1[], double Matrix2[], double Matrix3[]);
	int MatrixInv(int Col, int Row, double Matrix1[], double Matrix2[]);
	void DeleteRowAndCol(int Col, int Row, int ColToDelete, int RowToDelete, int deleteType, double p[], double q[]);
	void PrintMatrix(int Col, int Row, double matrix[]);

	//向量计算
	int VectorAddtion(int Dim1, int Dim2, double Vector1[], double Vector2[], double Vector3[]);
	int VectorSubtraction(int Dim1, int Dim2, double Vector1[], double Vector2[], double Vector3[]);
	int VectorDotProduct(int Dim1, int Dim2, double Vector1[], double Vector2[], double& Answer);
	int VectorCrossProduct(int Dim1, int Dim2, double Vector1[], double Vector2[], double Vector3[]);

	//数据解码
	int DecodeNovOem7Dat(unsigned char* buff, int& NumWritten, EPOCHOBSDATA* obs, GPSEPHREC* geph, GPSEPHREC* beph, POSRES* pos);
	void DecodeRange(unsigned char* buff, EPOCHOBSDATA* obs);
	void DecodeGpsEphem(unsigned char* buff, GPSEPHREC* eph);
	void DecodeBdsEphem(unsigned char* buff, GPSEPHREC* eph);
	void DecodeBestPos(unsigned char* buff, POSRES* pos);

	//广播星历
	int CompGNSSatPVT(int Prn, GNSSSys Sys, GPSTIME* t, GPSEPHREC* Eph, GPSEPHREC* BEph, SATMIDRES* Mid);
	int CompSatClkOff(int Prn, GNSSSys Sys, GPSTIME* Gt, GPSEPHREC* Eph, GPSEPHREC* BEph, SATMIDRES* Mid);
	void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA* Epk, GPSEPHREC* Eph, GPSEPHREC* Beph, XYZ UserPos, SATMIDRES* Mid);
	double Hopfield(const double H, const double Elev);
	void DetectOutlier(EPOCHOBSDATA* Obs);

	//SPP
	bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GEph, GPSEPHREC* BEph, POSRES* Res, PPRESULT* Res1);
	void SPV(EPOCHOBSDATA* Epoch, POSRES* Res, PPRESULT* Res1);

	//RTK
	int GetSynObsFile(FILE* FBas, FILE* FRov, RAWDAT* Raw, PPRESULT* Res1); 
	int RealTimeSync(SOCKET baseSock, SOCKET roverSock, RAWDAT* raw);
	void FormSDEpochObs(EPOCHOBSDATA* EpkA, EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs);
	void DetectCycleSlip(SDEPOCHOBS* Obs); 
	void DetRefSat(EPOCHOBSDATA* EpkA, EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs);
	bool RTKFloat(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov);

	// lambda
	int LD(int n, const double* Q, double* L, double* D);
	void gauss(int n, double* L, double* Z, int i, int j);
	void perm(int n, double* L, double* D, int j, double del, double* Z);
	void reduction(int n, double* L, double* D, double* Z);
	int search(int n, int m, const double* L, const double* D, const double* zs, double* zn, double* s);
	int lambda(int n, int m, double* a, const double* Q, double* f, double* s);
	int RTKlambda(RAWDAT* Raw);

	// Socket网络
	bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);
	void CloseSocket(SOCKET& sock);