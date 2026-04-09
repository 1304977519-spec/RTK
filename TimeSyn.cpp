#include "Struct.h"
#include "sockets.h"
/****************************************************************************
函数编号：    01
函数目的：    实现RTK基站与流动站数据同步
变量含义：
FBas        基站的数据
FRov        流动站的数据
Raw         RTK的数据
编写时间：2025.2.19
****************************************************************************/
int GetSynObsFile(FILE* FBas, FILE* FRov, RAWDAT* Raw, PPRESULT* Res1) {
	static int LenReadRov = 0, NumWrittenRov = 0, LenReadBas = 0, NumWrittenBas = 0;
	static unsigned char buffRov[MAXRAWLEN], buffBas[MAXRAWLEN];

	double timediff;
	int isRovObs, isBasObs;
	//获取流动站的观测数据，若不成功，继续获取，若文件结束，返回文件结束标记，若成功，得到观测时刻
	while (!feof(FRov)) {
		LenReadRov = fread(buffRov + NumWrittenRov, 1, MAXRAWLEN - NumWrittenRov, FRov);
		if (LenReadRov < MAXRAWLEN - NumWrittenRov) return -1;
		NumWrittenRov += LenReadRov;
		isRovObs = DecodeNovOem7Dat(buffRov, NumWrittenRov, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph, &Raw->pos);
		if (isRovObs == 1) break;
	}
	//流动站观测时刻与当前基站观测时刻比较，在限差范围内，文件同步成功，返回1；
	timediff = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + (Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek);
	if (fabs(timediff) <= 0.5) return 1;
	//若不在限差范围内，获取基站观测数据，两站的观测时间求差，在限差范围内，同步成功返回1；
	else {
		while (!feof(FBas)) {
			LenReadBas = fread(buffBas + NumWrittenBas, 1, MAXRAWLEN - NumWrittenBas, FBas);
			if (LenReadBas < MAXRAWLEN - NumWrittenBas) return -1;
			NumWrittenBas += LenReadBas;
			isBasObs = DecodeNovOem7Dat(buffBas, NumWrittenBas, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph, &Raw->pos);
			if (isBasObs == 1) {
				timediff = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + (Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek);
				if (fabs(timediff) <= 0.5) return 1;
				//如果不在限差范围内，若基站时间在后，返回0；若基站时间在前，循环获取基站数据，直到成功。
				else if (timediff < -0.5) return 0;
			}
		}
	}
}

/****************************************************************************
函数编号：    02
函数目的：    实现RTK基站与流动站数据同步
Raw         RTK的数据

编写时间：    2025.5.8
****************************************************************************/
int RealTimeSync(SOCKET baseSock, SOCKET roverSock, RAWDAT* raw) {
    static unsigned char baseBuf[MAXRAWLEN], roverBuf[MAXRAWLEN];
    static int baseLen = 0, roverLen = 0;
    static int baseDecoded = 0; // 标记基站数据是否已解码

    // 1. 优先处理流动站数据
    int roverRecv = recv(roverSock, (char*)roverBuf + roverLen, sizeof(roverBuf) - roverLen, 0);
    if (roverRecv > 0) {
        roverLen += roverRecv;
        if (DecodeNovOem7Dat(roverBuf, roverLen, &raw->RovEpk,
            raw->GpsEph, raw->BdsEph, &raw->pos)) {
            roverLen = 0; // 解码成功重置缓冲区

            // 2. 检查与基站时间差
            if (baseDecoded) {
                double diff = (raw->RovEpk.Time.Week - raw->BasEpk.Time.Week) * 604800.0
                    + (raw->RovEpk.Time.SecOfWeek - raw->BasEpk.Time.SecOfWeek);

                if (fabs(diff) <= 15) {
                    baseDecoded = 0; // 重置标记
                    return 1;  // 同步成功
                }
            }
        }
        else if (roverLen >= sizeof(roverBuf)) {
            roverLen = 0; // 防止缓冲区溢出
        }
    }

    // 3. 处理基站数据（仅在需要时更新）
    int baseRecv = recv(baseSock, (char*)baseBuf + baseLen, sizeof(baseBuf) - baseLen, 0);
    if (baseRecv > 0) {
        baseLen += baseRecv;
        if (DecodeNovOem7Dat(baseBuf, baseLen, &raw->BasEpk,
            raw->GpsEph, raw->BdsEph, &raw->pos)) {
            baseLen = 0;
            baseDecoded = 1; // 标记基站数据有效
        }
        else if (baseLen >= sizeof(baseBuf)) {
            baseLen = 0;
        }
    }

    // 错误处理
    if (roverRecv == SOCKET_ERROR || baseRecv == SOCKET_ERROR) {
        return -1;
    }
    return 0;
}
