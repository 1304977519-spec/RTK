#include "Struct.h"
/****************************************************************************
函数编号：    01
函数目的：    实现2个字节的数据提取
变量含义：
buff    缓存区
result  数据提取的结果
编写时间：2024.10.10
****************************************************************************/
void R2(unsigned char* buff, short& result)
{
    memcpy(&result, buff, 2);
}
void R2(unsigned char* buff, unsigned short& result)
{
    memcpy(&result, buff, 2);
}
/****************************************************************************
函数编号：    02
函数目的：    实现4个字节的数据提取
变量含义：
buff    缓存区
result  数据提取的结果
编写时间：2024.10.10
****************************************************************************/
void R4(unsigned char* buff, unsigned int& result)
{
    memcpy(&result, buff, 4);
}

void R4(unsigned char* buff, int& result)
{
    memcpy(&result, buff, 4);
}

void R4(unsigned char* buff, float& result)
{
    memcpy(&result, buff, 4);
}

void R4(unsigned char* buff, unsigned long& result)
{
    memcpy(&result, buff, 4);
}
/****************************************************************************
函数编号：    03
函数目的：    实现CRC 校验
变量含义：
buff    缓存区
len     数据长度
编写时间：2024.10.10
****************************************************************************/
unsigned int crc32(const unsigned char* buff, int len) {
    int i, j;
    unsigned int crc = 0;

    for (i = 0; i < len; i++)
    {
        crc ^= buff[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}

/****************************************************************************
函数编号：    04
函数目的：    实现8个字节的数据提取
变量含义：
buff    缓存区
result  数据提取的结果
编写时间：2024.10.10
****************************************************************************/
void R8(unsigned char* buff, double& result)
{
    memcpy(&result, buff, 8);
}

/****************************************************************************
函数编号：    05
函数目的：    实现NovAtelOem7的解码
变量含义：
Buff[]        缓存区的数据
NumWritten    实际数据长度
obs	          历元数据
geph[]        GPS的卫星星历
beph[]        北斗的卫星星历
编写时间：2024.10.10
****************************************************************************/
int DecodeNovOem7Dat(unsigned char* buff, int& NumWritten, EPOCHOBSDATA* obs, GPSEPHREC* geph, GPSEPHREC* beph, POSRES* pos) {
    unsigned short MessageLenth, MessageID;
    int i = 0, flag = 0, j = 0;
    unsigned int Crc;
    unsigned char Buff[MAXRAWLEN];

    while (i < NumWritten) {

        //进行容错处理（判断是否超限）
        if (i + OME7HEADLENTH > NumWritten) break;

        //查看头文件
        if (buff[i] != 0xAA || buff[i + 1] != 0x44 || buff[i + 2] != 0x12) {
            i++;
            continue;
        }

        //解出数据长度
        R2(buff + i + 4, MessageID);
        R2(buff + i + 8, MessageLenth);

        //进行容错处理（判断是否超限）
        if (i + OME7HEADLENTH + MessageLenth + 4 > NumWritten) break;
        R4(buff + i + MessageLenth + OME7HEADLENTH, Crc);

        //检查Crc
        if (crc32(buff + i, MessageLenth + OME7HEADLENTH) != Crc) {
            i += 3;
            continue;
        }

        flag = 0;

        //进行数据拷贝
        memcpy(Buff, buff + i, OME7HEADLENTH + MessageLenth + 4);

        switch (MessageID) {
        case 43:
            DecodeRange(Buff, obs);  // 处理 RANGE
            flag = 1;
            break;
        case 7:
            DecodeGpsEphem(Buff, geph);  // 处理 GPSEPHEM
            break;
        case 1696:
            DecodeBdsEphem(Buff, beph);  // 处理 BDSEPHEMERIS
            break;
        case 42:
            DecodeBestPos(Buff, pos);  // 处理 BDSEPHEMERIS
            obs->Bestpos[0] = pos->Pos[0];
            obs->Bestpos[1] = pos->Pos[1];
            obs->Bestpos[2] = pos->Pos[2];
            break;
        default:
            break;
        }

        //查看下一组数据
        i += (OME7HEADLENTH + MessageLenth + 4);

        //对后续剩余的语句进行处理（避免出现后一缓冲区只有半句话）
        if (flag == 1) break;
    }

    // 移动未处理的数据
    for (j = 0; j < NumWritten - i; j++)
    {
        buff[j] = buff[i + j];
    }
    NumWritten = j;

    return flag;
}

/****************************************************************************
函数编号：    06
函数目的：    实现range的解码
变量含义：
buff    缓存区
obs     解码数据的存储
编写时间：2024.10.10
****************************************************************************/
void DecodeRange(unsigned char* p, EPOCHOBSDATA* obs) {
    int SignalType, SatSystem, Freq, k, SatNum = 0, ObsNum, SecOfWeek;
    unsigned short Prn = 0;
    unsigned long obsnum, ChTrStatus;
    float Dopp, Cn0, LockTime;
    double Psr, Adr, Wl;
    GNSSSys System;
    unsigned char* buff;

    obs->SatNum = 0;
    memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));
    R2(p + 14, obs->Time.Week);//GPSweek
    R4(p + 16, SecOfWeek);
    obs->Time.SecOfWeek = SecOfWeek * 0.001;//周内秒  ms->s
    buff = p + OME7HEADLENTH;
    R4(buff, obsnum);
    ObsNum = obsnum;
    for (int i = 0; i < ObsNum; i++, buff += 44) {
        R2(buff + 4, Prn);
        R8(buff + 8, Psr);
        R8(buff + 20, Adr);
        R4(buff + 32, Dopp);
        R4(buff + 36, Cn0);
        R4(buff + 40, LockTime);
        R4(buff + 44, ChTrStatus);

        SatSystem = (ChTrStatus >> 16) & 0x07;
        SignalType = (ChTrStatus >> 21) & 0x1F;

        switch (SatSystem)
        {
        case 0:
            System = GPS;
            break;
        case 1:
            System = GLONASS;
            break;
        case 3:
            System = GALILEO;
            break;
        case 4:
            System = BDS;
            break;
        case 5:
            System = QZSS;
            break;
        default:
            System = UNKS;
            break;
        }

        if (System != GPS && System != BDS) continue;


        if (System == GPS) {
            switch (SignalType)
            {
            case 0:
                Freq = 0;
                Wl = WL1_GPS;
                break;
            case 9:
                Freq = 1;
                Wl = WL2_GPS;
                break;

            default:
                Freq = 2;
                break;
            }
        }
        else if (System == BDS) {
            switch (SignalType)
            {
            case 0:
                Freq = 0;
                Wl = WL1_BDS;
                break;
            case 4:
                Freq = 0;
                Wl = WL1_BDS;
                break;
            case 2:
                Freq = 1;
                Wl = WL3_BDS;
                break;
            case 6:
                Freq = 1;
                Wl = WL3_BDS;
                break;
            default:
                Freq = 2;
                break;
            }
        }
        else continue;

        if (Freq != 0 && Freq != 1) continue;

        // 将当前解出来的obs放到对应数组，如果该卫星在数组已存在
        for (int j = 0; j < MAXCHANNUM; j++) {
            if (obs->SatObs[j].Prn == Prn && obs->SatObs[j].System == System) {
                k = j;
                break;
            }
            if (obs->SatObs[j].Prn == 0 && obs->SatObs[j].System == UNKS) {
                k = j;
                break;
            }
        }
        if (k > SatNum) SatNum = k;

        obs->SatObs[k].Prn = Prn;
        obs->SatObs[k].System = System;
        obs->SatObs[k].P[Freq] = Psr;
        obs->SatObs[k].L[Freq] = -1.0 * Adr * Wl;//cycle->m;
        obs->SatObs[k].D[Freq] = -1.0 * Dopp * Wl;//Hz->m/s
        obs->SatObs[k].Cn0[Freq] = Cn0;
        obs->SatObs[k].LockTime[Freq] = LockTime;
        obs->SatObs[k].Half[Freq] = (ChTrStatus >> 11) & 0x1;
        obs->SatObs[k].CodeLock[Freq] = (ChTrStatus >> 12) & 0x1;

    }
    obs->SatNum = SatNum + 1;
}

/****************************************************************************
函数编号：    07
函数目的：    实现GPS的解码
变量含义：
buff        缓存区
eph         解码数据的存储
编写时间：2024.10.10
****************************************************************************/
void DecodeGpsEphem(unsigned char* p, GPSEPHREC* Geph) {
    double A;
    int Prn, Week, SVHealth;
    unsigned long IODE, IODC;
    GPSEPHREC* eph;
    unsigned char* buff;

    buff = p + OME7HEADLENTH;
    R4(buff, Prn);
    //防止数组超限
    if (Prn < 0 || Prn > MAXGPSNUM) return;
    eph = Geph + Prn - 1;

    //只更新这一颗卫星的星历数据，其他卫星的不改变
    eph->Sys = GPS;
    eph->PRN = Prn;
    R4(buff + 12, SVHealth);
    eph->SVHealth = SVHealth;
    R4(buff + 16, IODE);
    eph->IODE = IODE;
    R4(buff + 24, Week);
    eph->TOE.Week = Week;
    eph->TOC.Week = Week;
    R8(buff + 32, eph->TOE.SecOfWeek);
    R8(buff + 40, A);
    eph->SqrtA = sqrt(A);
    R8(buff + 48, eph->DeltaN);
    R8(buff + 56, eph->M0);
    R8(buff + 64, eph->e);
    R8(buff + 72, eph->omega);
    R8(buff + 80, eph->Cuc);
    R8(buff + 88, eph->Cus);
    R8(buff + 96, eph->Crc);
    R8(buff + 104, eph->Crs);
    R8(buff + 112, eph->Cic);
    R8(buff + 120, eph->Cis);
    R8(buff + 128, eph->i0);
    R8(buff + 136, eph->iDot);
    R8(buff + 144, eph->OMEGA);
    R8(buff + 152, eph->OMEGADot);
    R4(buff + 160, IODC);
    eph->IODC = IODC;
    R8(buff + 164, eph->TOC.SecOfWeek);
    R8(buff + 172, eph->TGD1);
    R8(buff + 180, eph->ClkBias);
    R8(buff + 188, eph->ClkDrift);
    R8(buff + 196, eph->ClkDriftRate);
    R8(buff + 216, eph->SVAccuracy);
}

/****************************************************************************
函数编号：    08
函数目的：    实现BDS的解码
变量含义：
buff    缓存区
eph     解码数据的存储
编写时间：2024.10.10
****************************************************************************/
void DecodeBdsEphem(unsigned char* p, GPSEPHREC* Geph) {
    int Prn; 
    unsigned long IODE, IODC, Week, SecofWeek1, SecofWeek2;
    GPSEPHREC* eph;
    unsigned char* buff;

    buff = p + OME7HEADLENTH;
    R4(buff, Prn);
    if (Prn < 0 || Prn > MAXBDSNUM) return;

    eph = Geph + Prn - 1;
    eph->PRN = Prn;
    eph->Sys = BDS;
    R4(buff + 4, Week);
    eph->TOE.Week = Week;
    eph->TOC.Week = Week;
    R8(buff + 8, eph->SVAccuracy);
    R4(buff + 16, eph->SVHealth);
    R8(buff + 20, eph->TGD1);
    R8(buff + 28, eph->TGD2);
    R4(buff + 36, IODC);
    eph->IODC = IODC;
    R4(buff + 40, SecofWeek1);
    eph->TOC.SecOfWeek = SecofWeek1;
    R8(buff + 44, eph->ClkBias);
    R8(buff + 52, eph->ClkDrift);
    R8(buff + 60, eph->ClkDriftRate);
    R4(buff + 68, IODE);
    eph->IODE = IODE;
    R4(buff + 72, SecofWeek2);
    eph->TOE.SecOfWeek = SecofWeek2;
    R8(buff + 76, eph->SqrtA);
    R8(buff + 84, eph->e);
    R8(buff + 92, eph->omega);
    R8(buff + 100, eph->DeltaN);
    R8(buff + 108, eph->M0);
    R8(buff + 116, eph->OMEGA);
    R8(buff + 124, eph->OMEGADot);
    R8(buff + 132, eph->i0);
    R8(buff + 140, eph->iDot);
    R8(buff + 148, eph->Cuc);
    R8(buff + 156, eph->Cus);
    R8(buff + 164, eph->Crc);
    R8(buff + 172, eph->Crs);
    R8(buff + 180, eph->Cic);
    R8(buff + 188, eph->Cis);
}

/****************************************************************************
函数编号：    08
函数目的：    实现接收机定位的提取
变量含义：
buff    缓存区
pos     解码数据的存储
编写时间：2024.10.10
****************************************************************************/
void DecodeBestPos(unsigned char* buff, POSRES* pos)
{
    int j;
    float ud = 0.0;
    int SecofWeek;
    XYZ pos1;
    BLh blh;
    R2(buff + 14, pos->Time.Week);//GPSweek
    R4(buff + 16, SecofWeek);//周内秒  ms->s
    pos->Time.SecOfWeek = SecofWeek;
    pos->Time.SecOfWeek *= 0.001;
    buff += OME7HEADLENTH;
    for (j = 0; j < 3; j++)
    {
        R8(buff + 8 * (j + 1), pos->Pos[j]);//此时的h不是大地高是正高
    }
    R4(buff + 32, ud);
    pos->Pos[2] = pos->Pos[2] + ud;//转换到大地高
    blh.latitude = pos->Pos[0];
    blh.longitude = pos->Pos[1];
    blh.height = pos->Pos[2];

    BLHToXYZ(blh, pos1);
    pos->Pos[0] = pos1.x;
    pos->Pos[1] = pos1.y;
    pos->Pos[2] = pos1.z;
}