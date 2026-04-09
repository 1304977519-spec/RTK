#include "Struct.h"
/****************************************************************************
函数编号：    01
函数目的：    实现GNSSPVT的算法
变量含义：
Prn    卫星号
Gt     卫星发射时间
Eph    GNSS星历
Mid    中间解算结果
编写时间：2024.10.23
****************************************************************************/
int CompGNSSatPVT(int Prn, GNSSSys Sys, GPSTIME* Gt, GPSEPHREC* GEph, GPSEPHREC* BEph, SATMIDRES* Mid) {
	double omegae, u, A, n0, tk, n, Mk, Ek, vk, phik, duk, drk, dik, uk, rk, ik, xk0, yk0, omegak, EkDot, phikDot, ukDot, rkDot, ikDot, omegakDot, xk0Dot, yk0Dot, dtr, dtrdot;
	int flag = 0, Cnt = 0;
	GPSTIME t; 
	GPSEPHREC* Eph;
	Matrix<double, 3, 4> Rdot;
	Matrix<double, 4, 1> SatPosDot, SatPosGk;
	Matrix<double, 3, 1> SatVel, Xk, Xgk, GkDot;
	Matrix<double, 3, 3> Rx, Rz, RzDot;

	Eph = (Sys == GPS) ? GEph + Prn - 1 : BEph + Prn - 1;
	u = (Sys == GPS) ? Gu : Bu;
	omegae = (Sys == GPS) ? Gomegae : Bomegae;
	if (Sys == BDS) flag = (Eph->i0 > 30 * Pi / 180) ? 0 : 1;
	t.Week = (Sys == GPS) ? Eph->TOE.Week : Eph->TOE.Week + 1356;
	t.SecOfWeek = (Sys == GPS) ? Eph->TOE.SecOfWeek : Eph->TOE.SecOfWeek + 14;
	tk = (Gt->Week - t.Week) * 604800 + (Gt->SecOfWeek - t.SecOfWeek);

	if (flag == 0) {//MEO和IGSO
		omegak = Eph->OMEGA + (Eph->OMEGADot - omegae) * tk - omegae * Eph->TOE.SecOfWeek;
		omegakDot = Eph->OMEGADot - omegae;
	}
	else if (flag == 1) {
		omegak = Eph->OMEGA + Eph->OMEGADot * tk - omegae * Eph->TOE.SecOfWeek;
		omegakDot = Eph->OMEGADot;
	}
	else return 0;
	
	//计算卫星坐标方法
	A = Eph->SqrtA * Eph->SqrtA;
	n0 = sqrt(u / (A * A * A));
	n = n0 + Eph->DeltaN;
	Mk = Eph->M0 + n * tk;
	Ek = Mk;
	do {
		Cnt++;
		// 计算 f(Ek) 和 f'(Ek)
		double f_Ek = Ek - Eph->e * sin(Ek) - Mk;
		double f_prime_Ek = 1 - Eph->e * cos(Ek);
		// 计算下一个 E_k
		double Ek_next = Ek - f_Ek / f_prime_Ek;
		// 判断是否收敛
		if (fabs(Ek_next - Ek) < 1e-12) {
			break;
		}
		Ek = Ek_next;  // 更新 E_k
	} while (Cnt > 10);
	vk = atan2(sqrt(1 - Eph->e * Eph->e) * sin(Ek), cos(Ek) - Eph->e);
	phik = vk + Eph->omega;
	duk = Eph->Cus * sin(2 * phik) + Eph->Cuc * cos(2 * phik);
	drk = Eph->Crs * sin(2 * phik) + Eph->Crc * cos(2 * phik);
	dik = Eph->Cis * sin(2 * phik) + Eph->Cic * cos(2 * phik);
	uk = phik + duk;
	rk = A * (1 - Eph->e * cos(Ek)) + drk;
	ik = Eph->i0 + dik + Eph->iDot * tk;
	xk0 = rk * cos(uk);
	yk0 = rk * sin(uk);
	//计算卫星速度方法
	EkDot = n / (1 - Eph->e * cos(Ek));
	phikDot = sqrt(1 - Eph->e * Eph->e) / (1 - Eph->e * cos(Ek)) * EkDot;
	ukDot = 2 * (Eph->Cus * cos(2 * phik) - Eph->Cuc * sin(2 * phik)) * phikDot + phikDot;
	rkDot = A * Eph->e * sin(Ek) * EkDot + 2 * (Eph->Crs * cos(2 * phik) - Eph->Crc * sin(2 * phik)) * phikDot;
	ikDot = Eph->iDot + 2 * (Eph->Cis * cos(2 * phik) - Eph->Cic * sin(2 * phik)) * phikDot;
	Rdot << cos(omegak), -sin(omegak) * cos(ik), -(xk0 * sin(omegak) + yk0 * cos(omegak) * cos(ik)), yk0* sin(omegak)* sin(ik),
		sin(omegak), cos(omegak)* cos(ik), xk0* cos(omegak) - yk0 * sin(omegak) * cos(ik), -yk0 * cos(omegak) * sin(ik),
		0, sin(ik), 0, yk0* cos(ik);
	xk0Dot = rkDot * cos(uk) - rk * ukDot * sin(uk);
	yk0Dot = rkDot * sin(uk) + rk * ukDot * cos(uk);

	//计算钟差和钟速，包含相对论改正
	dtr = F * Eph->e * Eph->SqrtA * sin(Ek);
	Mid->SatClkOft = Eph->ClkBias + Eph->ClkDrift * tk + Eph->ClkDriftRate * tk * tk + dtr;
	dtrdot = F * Eph->e * Eph->SqrtA * cos(Ek) * EkDot;
	Mid->SatClkSft = Eph->ClkDrift + 2 * Eph->ClkDriftRate * tk + dtrdot;

	if (flag == 0) {
		//计算GPS, MEO/IGSO 卫星在 BDCS 坐标系中的坐标
		Mid->SatPos[0] = xk0 * cos(omegak) - yk0 * cos(ik) * sin(omegak);
		Mid->SatPos[1] = xk0 * sin(omegak) + yk0 * cos(ik) * cos(omegak);
		Mid->SatPos[2] = yk0 * sin(ik);

		//计算卫星速度方法
		SatPosDot << xk0Dot,
			yk0Dot,
			omegakDot,
			ikDot;
		SatVel = Rdot * SatPosDot;
		Mid->SatVel[0] = SatVel(0, 0);
		Mid->SatVel[1] = SatVel(1, 0);
		Mid->SatVel[2] = SatVel(2, 0);
	}
	else if (flag == 1) {
		//计算 GEO 卫星在 BDCS 坐标系中的坐标
		double phi;
		Xgk << xk0 * cos(omegak) - yk0 * cos(ik) * sin(omegak),
			xk0 * sin(omegak) + yk0 * cos(ik) * cos(omegak),
			yk0 * sin(ik);
		phi = -5 * Pi / 180;
		Rx << 1, 0, 0,
			0, cos(phi), sin(phi),
			0, -sin(phi), cos(phi);
		phi = omegae * tk;
		Rz << cos(phi), sin(phi), 0,
			-sin(phi), cos(phi), 0,
			0, 0, 1;
		Xk = Rz * Rx * Xgk;
		Mid->SatPos[0] = Xk(0, 0);
		Mid->SatPos[1] = Xk(1, 0);
		Mid->SatPos[2] = Xk(2, 0);

		//计算卫星速度方法
		SatPosGk << xk0Dot,
			yk0Dot,
			omegakDot,
			ikDot;
		GkDot = Rdot * SatPosGk;
		phi = omegae * tk;
		RzDot << -sin(phi), cos(phi), 0,
			-cos(phi), -sin(phi), 0,
			0, 0, 0;
		SatVel = Rz * Rx * GkDot + omegae * RzDot * Rx * Xgk;
		Mid->SatVel[0] = SatVel(0, 0);
		Mid->SatVel[1] = SatVel(1, 0);
		Mid->SatVel[2] = SatVel(2, 0);
	}
	else return 0;

	return 1;

}

/*
函数编号：    02
函数目的：    计算某一时刻某一颗卫星的钟差，不含相对论改正，考虑过期和健康状态
变量含义：
Prn    卫星号
Sys    卫星系统
Gt     卫星发射时间
Eph    GNSS星历
Mid    中间解算结果
编写时间：2024.10.29
*/
int CompSatClkOff(int Prn, GNSSSys Sys, GPSTIME* Gt, GPSEPHREC* GEph, GPSEPHREC* BEph, SATMIDRES* Mid) {
	double Tmax, tk, u;
	GPSTIME t;
	GPSEPHREC* Eph;
	Eph = (Sys == GPS) ? GEph + Prn - 1 : BEph + Prn - 1;
	if (Eph->PRN != Prn || Eph->Sys != Sys || Eph->SVHealth) return 0;
	u = (Sys == GPS) ? Gu : Bu;

	Tmax = (Sys == GPS) ? GPSMAX : BDSMAX;
	t.Week = (Sys == GPS) ? Eph->TOE.Week : Eph->TOE.Week + 1356;
	t.SecOfWeek = (Sys == GPS) ? Eph->TOE.SecOfWeek : Eph->TOE.SecOfWeek + 14;

	tk = (Gt->Week - t.Week) * 604800 + (Gt->SecOfWeek - t.SecOfWeek);
	if (fabs(tk) > Tmax) return 0;
	else {
		
		Mid->SatClkOft = Eph->ClkBias + Eph->ClkDrift * tk + Eph->ClkDriftRate * tk * tk;
		Mid->SatClkSft = Eph->ClkDrift + 2 * Eph->ClkDriftRate * tk;
		return 1;
	}

}

/*
函数编号：    03
函数目的：    计算某一时刻某一颗卫星的钟差，含相对论改正，考虑过期和健康状态
变量含义：
Prn    卫星号
Sys    卫星系统
Gt     卫星发射时间
Eph    GNSS星历
Mid    中间解算结果
编写时间：2024.11.22
*/
void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA* Epk, GPSEPHREC* Eph, GPSEPHREC* BEph, XYZ UserPos, SATMIDRES* Mid) {
	int prn;
	GPSTIME T;
	double dx, dy, dz, phi, dt, omegae;
	Matrix<double, 3, 1> X, Xdot, Pos, Vel, Xr;
	Matrix<double, 3, 3> Rz;
	XYZ Xs;
	BLh blh;
	T.Week = Epk->Time.Week;

	for (int i = 0; i < Epk->SatNum; i++) {
		prn = Epk->SatObs[i].Prn;


		//调用计算卫星钟差的函数，计算信号发射时刻的GPS时间或BDT，迭代计算两次，提高计算精度
		T.SecOfWeek = Epk->Time.SecOfWeek - Epk->ComObs[i].PIF / C_Light;
		Mid[i].Valid = CompSatClkOff(prn, Epk->SatObs[i].System, &T, Eph, BEph, &Mid[i]);
		if(Mid[i].Valid < 1) continue;
		
		T.SecOfWeek = Epk->Time.SecOfWeek - Epk->ComObs[i].PIF / C_Light - Mid[i].SatClkOft;
		Mid[i].Valid = CompSatClkOff(prn, Epk->SatObs[i].System, &T, Eph, BEph, &Mid[i]);
		if (Mid[i].Valid < 1) continue;

		T.SecOfWeek = Epk->Time.SecOfWeek - Epk->ComObs[i].PIF / C_Light - Mid[i].SatClkOft;

		//以𝑡r[𝐺𝑃𝑆]时刻调用卫星位置和钟差函数，计算卫星位置、速度和钟差钟速等
		Mid[i].Valid = CompGNSSatPVT(prn, Epk->SatObs[i].System, &T, Eph, BEph, &Mid[i]);

		if (Mid[i].Valid <= 0) continue;

		if (Epk->SatObs->System == GPS) omegae = Gomegae;
		else if (Epk->SatObs->System == BDS) omegae = Bomegae;
		else continue;

		//对卫星位置进行地球自转改正
		dx = Mid[i].SatPos[0] - UserPos.x;
		dy = Mid[i].SatPos[1] - UserPos.y;
		dz = Mid[i].SatPos[2] - UserPos.z;
		dt = sqrt(dx * dx + dy * dy + dz * dz) / C_Light;
		phi = omegae * dt;
		Rz << cos(phi), sin(phi), 0,
			-sin(phi), cos(phi), 0,
			0, 0, 1;
		Pos << Mid[i].SatPos[0],
			Mid[i].SatPos[1],
			Mid[i].SatPos[2];
		Vel << Mid[i].SatVel[0],
			Mid[i].SatVel[1],
			Mid[i].SatVel[2];
		X = Rz * Pos;
		Xdot = Rz * Vel;
		for (int j = 0; j < 3; j++) {
			Mid[i].SatPos[j] = X(j, 0);
			Mid[i].SatVel[j] = Xdot(j, 0);
		}

		Xs.x = Mid[i].SatPos[0];
		Xs.y = Mid[i].SatPos[1];
		Xs.z = Mid[i].SatPos[2];


		//对用户位置进行地球自转改正
		CompSatElAz(UserPos, Xs, Mid[i].Elevation, Mid[i].Azimuth);

		//对流层延迟改正
		XYZToBLH(UserPos, blh);
		Mid[i].TropCorr = Hopfield(blh.height, Mid[i].Elevation);

	}

}

