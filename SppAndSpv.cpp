#include "Struct.h"
/****************************************************************************
函数编号：    02
函数目的：    实现GNSSPVT的算法
变量含义：
Epoch    历元的观测数据
Eph      星历数据
Res      结算结果
编写时间：2024.11.22
****************************************************************************/
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GEph, GPSEPHREC* BEph, POSRES* Res, PPRESULT* Res1) {
    XYZ X0;
    double dx, dy, dz, p, tgd, dt, sum = 0, PDOP = 0;
    vector<double> l, m, n, w;
    vector<int> flag; // 使用 int 来表示卫星系统类型，更直观
    int Snum = 0, SGnum = 0, SBnum = 0, Prn, count, dim, cnt = 0;
    double B[MAXCHANNUM * 5] = { 0 }, W[MAXCHANNUM] = { 0 }, xr[5] = { 0 };
    double Bx[MAXCHANNUM] = { 0 }, Bx_W[MAXCHANNUM] = { 0 }, Bx_WT[MAXCHANNUM] = { 0 }, Bx_WTP[MAXCHANNUM] = { 0 }, Bx_WTPBx_WT[1] = {0};
    double BT[MAXCHANNUM * 5] = { 0 }, BTP[MAXCHANNUM * 5] = { 0 }, P[MAXCHANNUM * MAXCHANNUM] = { 0 };
    double BTPB[5 * 5] = { 0 }, BTPBInv[5 * 5] = { 0 }, BTPW[5] = { 0 };
    double BTPBver[4 * 4] = { 0 }, BTPWver[4] = { 0 };
    //double Xpre[5] = { 0 };
    X0.x = -2267335.4037;
    X0.y = 5008650.0645;
    X0.z = 3222376.0801;
    double Xpre[5] = { X0.x, X0.y, X0.z };
    // 粗差探测
    DetectOutlier(Epoch);
    
    do {
        cnt++;
        // 初始化计数器
        Snum = SGnum = SBnum = 0;

        // 计算卫星位置和其他修正量
        ComputeSatPVTAtSignalTrans(Epoch, GEph, BEph, X0, Epoch->SatPVT);

        // 对每颗卫星进行处理
        for (int i = 0; i < Epoch->SatNum; i++) {
            Prn = Epoch->SatObs[i].Prn;

            // 卫星位置计算失败、观测数据不完整或有粗差，不参与定位计算
            if (!(Epoch->SatPVT[i].Valid && Epoch->SatObs[i].Valid)) continue;

            // 处理不同卫星系统的情况
            if (Epoch->SatObs[i].System == GPS) {
                tgd = 0;
                dt = Xpre[3];
                SGnum++;
                flag.push_back(0);  // GPS
            }
            else if (Epoch->SatObs[i].System == BDS) {
                tgd = FG1_BDS * FG1_BDS / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS) * BEph[Prn - 1].TGD1 * C_Light;
                dt = Xpre[4];
                SBnum++;
                flag.push_back(1);  // BDS
            }
            else continue;  // 不处理其他系统
            Snum++;

            // 计算卫星与接收机之间的距离和观测方程系数
            dx = X0.x - Epoch->SatPVT[i].SatPos[0];
            dy = X0.y - Epoch->SatPVT[i].SatPos[1];
            dz = X0.z - Epoch->SatPVT[i].SatPos[2];
            p = sqrt(dx * dx + dy * dy + dz * dz);

            l.push_back(dx / p);
            m.push_back(dy / p);
            n.push_back(dz / p);
            w.push_back(Epoch->ComObs[i].PIF - (p + dt - C_Light * Epoch->SatPVT[i].SatClkOft + Epoch->SatPVT[i].TropCorr + tgd));
        }
        dim = (SGnum == 0 || SBnum == 0) ? 4 : 5;
        // 如果卫星数少于4颗，定位失败
        if (Snum < dim) return false;

        // 重置矩阵和向量
        memset(B, 0, sizeof(B));
        memset(W, 0, sizeof(W));
        memset(BT, 0, sizeof(BT));
        memset(BTP, 0, sizeof(BTP));
        memset(BTPB, 0, sizeof(BTPB));
        memset(BTPW, 0, sizeof(BTPW));
        memset(BTPBver, 0, sizeof(BTPBver));
        memset(BTPWver, 0, sizeof(BTPWver));
        memset(BTPBInv, 0, sizeof(BTPBInv));
        memset(xr, 0, sizeof(xr));
        memset(P, 0, sizeof(P));

        // 填充 B 矩阵和 W 向量
        for (int i = 0; i < Snum; i++) {
            if (!l.empty() && !m.empty() && !n.empty() && !w.empty()) {
                W[i] = w.front();
                B[i * 5] = l.front();
                B[i * 5 + 1] = m.front();
                B[i * 5 + 2] = n.front();
                if (flag.front() == 0) {
                    B[i * 5 + 3] = 1;
                    B[i * 5 + 4] = 0;
                }
                else {
                    B[i * 5 + 3] = 0;
                    B[i * 5 + 4] = 1;
                }

                l.erase(l.begin());
                m.erase(m.begin());
                n.erase(n.begin());
                flag.erase(flag.begin());
                w.erase(w.begin());
            }
            else {
                break;
            }
        }

        // 初始化 P 矩阵为单位矩阵
        for (int i = 0; i < Snum; i++) {
            for (int j = 0; j < Snum; j++) {
                P[i * Snum + j] = (i == j) ? 1 : 0;
            }
        }

        // 矩阵运算步骤
        MatrixTranpose(5, Snum, B, BT);
        MatrixMultiplies(Snum, 5, Snum, Snum, BT, P, BTP);
        MatrixMultiplies(Snum, 5, 5, Snum, BTP, B, BTPB);
        MatrixMultiplies(Snum, 5, 1, Snum, BTP, W, BTPW);

        // 根据卫星系统数量，删除相应行列并解方程
        if (SGnum == 0) {
            DeleteRowAndCol(5, 5, 4, 4, 2, BTPB, BTPBver);
            DeleteRowAndCol(1, 5, 0, 4, 0, BTPW, BTPWver);
            MatrixInv(4, 4, BTPBver, BTPBInv);
            MatrixMultiplies(4, 4, 1, 4, BTPBInv, BTPWver, xr);
            for (int i = 0; i < 3; i++)  Xpre[i] += xr[i];
            Xpre[4] += xr[3];
        }
        else if (SBnum == 0) {
            DeleteRowAndCol(5, 5, 5, 5, 2, BTPB, BTPBver);
            DeleteRowAndCol(1, 5, 0, 5, 0, BTPW, BTPWver);
            MatrixInv(4, 4, BTPBver, BTPBInv);
            MatrixMultiplies(4, 4, 1, 4, BTPBInv, BTPWver, xr);
            for (int i = 0; i < 3; i++)  Xpre[i] += xr[i];
            Xpre[3] += xr[3];
        }
        else {
            // 最小二乘解
            MatrixInv(5, 5, BTPB, BTPBInv);
            MatrixMultiplies(5, 5, 1, 5, BTPBInv, BTPW, xr);
            for (int i = 0; i < 5; i++)  Xpre[i] += xr[i];
        }

        X0.x = Xpre[0];
        X0.y = Xpre[1];
        X0.z = Xpre[2];

        sum = 0;
        for (int i = 0; i < 5; i++) {
            sum += xr[i] * xr[i];
        }

    } while (sqrt(sum) > 10e-6 && cnt < 10); // 检查平差是否收敛

    if (cnt >= 10) return false;

    Res->Pos[0] = X0.x;
    Res->Pos[1] = X0.y;
    Res->Pos[2] = X0.z;
    Res->SatNum = Snum;
    Res->Time = Epoch->Time;
    // 精度评定
    if (SGnum == 0 || SBnum == 0) count = 4;
    else count = 5;/*
    cout << "Qxx:" << endl;
    PrintMatrix(count, count, BTPBInv);*/
    MatrixMultiplies(5, Snum, 1, 5, B, xr, Bx);
    MatrixSubtraction(1, Snum, 1, Snum, Bx, W, Bx_W);
    MatrixTranpose(1, Snum, Bx_W, Bx_WT);
    MatrixMultiplies(Snum, 1, Snum, Snum, Bx_WT, P, Bx_WTP);
    MatrixMultiplies(Snum, 1, 1, Snum, Bx_WTP, Bx_W, Bx_WTPBx_WT);

    Res->SigmaPos = (Snum - 5 == 0) ? 999 : sqrt(Bx_WTPBx_WT[0] / (Snum - 5));
    PDOP = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (i == j) PDOP += BTPBInv[i * count + j];
    Res->PDOP = sqrt(PDOP);
    
    Res1->Time = Epoch->Time;
    Res1->Position[0] = X0.x;
    Res1->Position[1] = X0.y;
    Res1->Position[2] = X0.z;
    Res1->PDOP = sqrt(PDOP);
    Res1->SigmaPos = (Snum - 5 == 0) ? 999 : sqrt(Bx_WTPBx_WT[0] / (Snum - 5));
    Res1->GPSSatNum = SGnum;
    Res1->BDSSatNum = SBnum;
    Res1->AllSatNum = SGnum + SBnum;
    Res1->IsSuccess = true;

    return true;
}



void SPV(EPOCHOBSDATA* Epoch, POSRES* Res, PPRESULT* Res1) {
    vector<double> l, m, n, w;
    int Snum = 0;
    double dx, dy, dz, p, pdot, PDOP = 0;
    double B[MAXCHANNUM * 5] = { 0 }, W[MAXCHANNUM] = { 0 }, x[5] = { 0 };
    double BT[MAXCHANNUM * 5] = { 0 }, BTP[MAXCHANNUM * 5] = { 0 }, P[MAXCHANNUM * MAXCHANNUM] = { 0 };
    double BTPB[5 * 5] = { 0 }, BTPBInv[5 * 5] = { 0 }, BTPW[5] = { 0 };
    double Bx[MAXCHANNUM] = { 0 }, Bx_W[MAXCHANNUM] = { 0 }, Bx_WTP[MAXCHANNUM] = { 0 }, Bx_WTPBx_W[1] = { 0 }, Bx_WT[MAXCHANNUM] = { 0 };
    for (int i = 0; i < Epoch->SatNum; i++) {

        // 卫星位置计算失败、观测数据不完整或有粗差，不参与定位计算
        if (!(Epoch->SatPVT[i].Valid && Epoch->SatObs[i].Valid)) continue;
        Snum++;
        // 计算卫星与接收机之间的距离和观测方程系数
        dx = Res->Pos[0] - Epoch->SatPVT[i].SatPos[0];
        dy = Res->Pos[1] - Epoch->SatPVT[i].SatPos[1];
        dz = Res->Pos[2] - Epoch->SatPVT[i].SatPos[2];
        p = sqrt(dx * dx + dy * dy + dz * dz);
        pdot = -(dx * Epoch->SatPVT[i].SatVel[0] + dy * Epoch->SatPVT[i].SatVel[1] + Epoch->SatPVT[i].SatVel[2] * dz) / p;

        l.push_back(dx / p);
        m.push_back(dy / p);
        n.push_back(dz / p);
        w.push_back(Epoch->SatObs[i].D[0] - (pdot - C_Light * Epoch->SatPVT[i].SatClkSft));
    }

    // 如果卫星数少于4颗，定位失败
    if (Snum < 4) return;

    // 重置矩阵和向量
    memset(B, 0, sizeof(B));
    memset(W, 0, sizeof(W));
    memset(BT, 0, sizeof(BT));
    memset(BTP, 0, sizeof(BTP));
    memset(BTPB, 0, sizeof(BTPB));
    memset(BTPW, 0, sizeof(BTPW));
    memset(BTPBInv, 0, sizeof(BTPBInv));
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));

    // 填充 B 矩阵和 W 向量
    for (int i = 0; i < Snum; i++) {
        if (!l.empty() && !m.empty() && !n.empty() && !w.empty()) {
            W[i] = w.front();
            B[i * 4] = l.front();
            B[i * 4 + 1] = m.front();
            B[i * 4 + 2] = n.front();
            B[i * 4 + 3] = 1;

            l.erase(l.begin());
            m.erase(m.begin());
            n.erase(n.begin());
            w.erase(w.begin());
        }
        else {
            break;
        }
    }

    // 初始化 P 矩阵为单位矩阵
    for (int i = 0; i < Snum; i++) {
        for (int j = 0; j < Snum; j++) {
            P[i * Snum + j] = (i == j) ? 1 : 0;
        }
    }

    // 矩阵运算步骤
    MatrixTranpose(4, Snum, B, BT);
    MatrixMultiplies(Snum, 4, Snum, Snum, BT, P, BTP);
    MatrixMultiplies(Snum, 4, 4, Snum, BTP, B, BTPB);
    MatrixMultiplies(Snum, 4, 1, Snum, BTP, W, BTPW);

    // 最小二乘解
    MatrixInv(4, 4, BTPB, BTPBInv);
    MatrixMultiplies(4, 4, 1, 4, BTPBInv, BTPW, x);

    Res->Vel[0] = x[0];
    Res->Vel[1] = x[1];
    Res->Vel[2] = x[2];

    MatrixMultiplies(5, Snum, 1, 5, B, x, Bx);
    MatrixSubtraction(1, Snum, 1, Snum, Bx, W, Bx_W);
    MatrixTranpose(1, Snum, Bx_W, Bx_WT);
    MatrixMultiplies(Snum, 1, Snum, Snum, Bx_WT, P, Bx_WTP);
    MatrixMultiplies(Snum, 1, 1, Snum, Bx_WTP, Bx_W, Bx_WTPBx_W);

    Res->SigmaVel = (Snum - 5 == 0) ? 999 : sqrt(Bx_WTPBx_W[0] / (Snum - 5));

    Res1->Velocity[0] = x[0];
    Res1->Velocity[1] = x[1];
    Res1->Velocity[2] = x[2];
    Res1->SigmaVel = (Snum - 5 == 0) ? 999 : sqrt(Bx_WTPBx_W[0] / (Snum - 5));
}