#include "Struct.h"
#define PR_NOISE 0.3         // 伪距观测噪声(m)
#define CP_NOISE 0.01        // 载波相位观测噪声(m)
#define THRESHOLD 0.001     // 迭代收敛阈值(1mm)
#define MAX_ITER 5            // 最大迭代次数
/****************************************************************************
函数编号：    01
函数目的：    实现相对定位浮点解
变量含义：
Raw         RTK的数据
Base        历元单点定位的结果
Rov         历元单点定位的结果
编写时间：2025.3.3
****************************************************************************/
bool RTKFloat(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov) {
    double xBas[3] = { 0 }, xRov[3] = { 0 },
        rho_b[MAXCHANNUM] = { 0 }, rho_ref[2] = { 0 },
        wl1, wl2,
        dx, dy, dz,
        rho_r, dd_rho,
        l, m, n,
        l_ref[2] = { 0 }, m_ref[2] = { 0 }, n_ref[2] = { 0 },
        dX[3] = { 0 }, var_pos[3] = { 0 },
        B[(MAXCHANNUM * 2 + 3) * MAXCHANNUM * 4], W[MAXCHANNUM * 4], P[MAXCHANNUM * MAXCHANNUM * 4 * 4],
        Bt[(MAXCHANNUM * 2 + 3) * MAXCHANNUM * 4] = { 0 }, PB[(MAXCHANNUM * 2 + 3) * MAXCHANNUM * 4] = { 0 }, N[(MAXCHANNUM * 2 + 3) * (MAXCHANNUM * 2 + 3)] = { 0 },
        U[(MAXCHANNUM * 2 + 3)] = { 0 }, invN[(MAXCHANNUM * 2 + 3) * (MAXCHANNUM * 2 + 3)] = { 0 }, dX_amb[(MAXCHANNUM * 2 + 3)] = { 0 }, PW[4 * MAXCHANNUM], dX_result[(MAXCHANNUM * 2 + 3)] = { 0 };
    int iter = 0, dim = 0, k,
        num_params, SysID, SysID2,
        sat_count = 0, Sats = 0,
        SatNum[2] = { 0 };
    bool success = false;


    // 1. 初始化基站和流动站位置
    if (Raw->BasEpk.Bestpos[0] == 0 && Raw->BasEpk.Bestpos[1] == 0 && Raw->BasEpk.Bestpos[2] == 0) {
        xBas[0] = Base->Position[0];
        xBas[1] = Base->Position[1];
        xBas[2] = Base->Position[2];
    }
    else {
        xBas[0] = Raw->BasEpk.Bestpos[0];
        xBas[1] = Raw->BasEpk.Bestpos[1];
        xBas[2] = Raw->BasEpk.Bestpos[2];
    }

    if (Raw->RovEpk.Bestpos[0] == 0 && Raw->RovEpk.Bestpos[1] == 0 && Raw->RovEpk.Bestpos[2] == 0) {
        xRov[0] = Rov->Position[0];
        xRov[1] = Rov->Position[1];
        xRov[2] = Rov->Position[2];
    }
    else {
        xRov[0] = Raw->RovEpk.Bestpos[0];
        xRov[1] = Raw->RovEpk.Bestpos[1];
        xRov[2] = Raw->RovEpk.Bestpos[2];
    }

    // 2. 计算GPS和BDS双差卫星数（单双频混用：各系统每个频率的双差卫星数）
    // 3. 计算基站到所有卫星的几何距离
    for (int i = 0; i < Raw->SdObs.SatNum; i++) {
        if (Raw->SdObs.SdSatObs[i].Valid < 1 || Raw->BasEpk.SatPVT[Raw->SdObs.SdSatObs[i].nBas].Valid < 1 || Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[i].nRov].Valid < 1)
        {
            Raw->SdObs.SdSatObs[i].Valid = -1;
            continue;
        }

        if (Raw->SdObs.SdSatObs[i].System == GPS) SatNum[0]++;
        if (Raw->SdObs.SdSatObs[i].System == BDS) SatNum[1]++;

        dx = xBas[0] - Raw->BasEpk.SatPVT[Raw->SdObs.SdSatObs[i].nBas].SatPos[0];
        dy = xBas[1] - Raw->BasEpk.SatPVT[Raw->SdObs.SdSatObs[i].nBas].SatPos[1];
        dz = xBas[2] - Raw->BasEpk.SatPVT[Raw->SdObs.SdSatObs[i].nBas].SatPos[2];

        rho_b[i] = sqrt(dx * dx + dy * dy + dz * dz);
    }

    if ((SatNum[0] + SatNum[1]) < 6)
    {
        cout << "卫星数量不够，没有足够的双差观测值.GPS 卫星数:" << SatNum[0] << ", BDS 卫星数 :" << SatNum[1] << "." << endl;
        cout << endl;
        return false;
    }

    //计算浮点解的待估参数的维数
    num_params = 0;

    if (SatNum[0] < 2 || Raw->DDObs.RefPos[0] == -1)SatNum[0] = 0;
    else num_params += (SatNum[0] - 1) * 2;
    if (SatNum[1] < 2 || Raw->DDObs.RefPos[1] == -1)SatNum[1] = 0;
    else num_params += (SatNum[1] - 1) * 2;

    num_params += 3;


    //用伪距初始化模期度
    for (int i = 0; i < 3; i++) dX_amb[i] = xRov[i] - xBas[i];

    sat_count = 0;
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        SysID = Raw->SdObs.SdSatObs[i].System == GPS ? 0 : 1;
        if (Raw->SdObs.SdSatObs[i].Valid < 1)continue;
        if (Raw->DDObs.RefPos[SysID] == -1 || SatNum[SysID] == 0)continue;
        if (Raw->DDObs.RefPos[0] == i || Raw->DDObs.RefPos[1] == i)continue;
        wl1 = (SysID == 0 ? WL1_GPS : WL1_BDS);
        wl2 = (SysID == 0 ? WL2_GPS : WL3_BDS);
        dX_amb[3 + sat_count * 2 + 0] = (Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos
            [SysID]].dL[0] - Raw->SdObs.SdSatObs[i].dP[0] + Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[SysID]].dP
            [0]) / wl1;
        dX_amb[3 + sat_count * 2 + 1] = (Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos
            [SysID]].dL[1] - Raw->SdObs.SdSatObs[i].dP[1] + Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[SysID]].dP
            [1]) / wl2;
        sat_count++;
    }

    // 5-9. 迭代计算
    // 待估参数: 3个位置增量 + (GPS双差模糊度) + (BDS双差模糊度)
    memset(&P, 0, sizeof(P));
    do {
        Sats = 0;
        // 初始化矩阵大小
        memset(&B, 0, sizeof(B));
        memset(&W, 0, sizeof(W));

        // 4. 计算流动站到参考星的几何距离
        for (int i = 0; i < 2; i++) {
            if (Raw->DDObs.RefPos[i] == -1 || Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[i]].Valid < 1)
            {
                Raw->DDObs.RefPos[i] = Raw->DDObs.RefPrn[i] = -1;
                continue;
            }
            dx = xRov[0] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[i]].nRov].SatPos[0];
            dy = xRov[1] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[i]].nRov].SatPos[1];
            dz = xRov[2] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[i]].nRov].SatPos[2];
            rho_ref[i] = sqrt(dx * dx + dy * dy + dz * dz);
            l_ref[i] = dx / rho_ref[i];
            m_ref[i] = dy / rho_ref[i];
            n_ref[i] = dz / rho_ref[i];
        }


        // 填充B矩阵和W向量
        k = 0;
        for (int i = 0; i < Raw->SdObs.SatNum; i++) {

            // 获取卫星系统类型
            SysID = Raw->SdObs.SdSatObs[i].System == GPS ? 0 : 1;
            if (Raw->SdObs.SdSatObs[i].Valid < 1)continue;
            if (Raw->DDObs.RefPos[SysID] == -1 || SatNum[SysID] == 0)continue;
            if (Raw->DDObs.RefPos[0] == i || Raw->DDObs.RefPos[1] == i)continue;


            // 计算流动站到当前卫星的几何距离
            dx = xRov[0] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[i].nRov].SatPos[0];
            dy = xRov[1] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[i].nRov].SatPos[1];
            dz = xRov[2] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[i].nRov].SatPos[2];

            // 计算方向余弦
            rho_r = sqrt(dx * dx + dy * dy + dz * dz);
            l = dx / rho_r;
            m = dy / rho_r;
            n = dz / rho_r;

            //流动站到当前卫星的几何距离 - 基站到当前卫星的几何距离
            dd_rho = rho_r - rho_b[i];

            // 设计矩阵行
            B[k * num_params + 0] = l - l_ref[SysID]; // dX 系数
            B[k * num_params + 1] = m - m_ref[SysID]; // dY 系数
            B[k * num_params + 2] = n - n_ref[SysID]; // dZ 系数

            B[(k + 1) * num_params + 0] = l - l_ref[SysID]; // dX 系数
            B[(k + 1) * num_params + 1] = m - m_ref[SysID]; // dY 系数
            B[(k + 1) * num_params + 2] = n - n_ref[SysID]; // dZ 系数

            B[(k + 2) * num_params + 0] = l - l_ref[SysID]; // dX 系数
            B[(k + 2) * num_params + 1] = m - m_ref[SysID]; // dY 系数
            B[(k + 2) * num_params + 2] = n - n_ref[SysID]; // dZ 系数

            B[(k + 3) * num_params + 0] = l - l_ref[SysID]; // dX 系数
            B[(k + 3) * num_params + 1] = m - m_ref[SysID]; // dY 系数
            B[(k + 3) * num_params + 2] = n - n_ref[SysID]; // dZ 系数

            wl1 = (SysID == 0 ? WL1_GPS : WL1_BDS);
            wl2 = (SysID == 0 ? WL2_GPS : WL3_BDS);

            // 模糊度参数设为 0（伪距无模糊度）
            B[(k + 1) * num_params + 3 + 2 * Sats + 0] = wl1;
            B[(k + 3) * num_params + 3 + 2 * Sats + 1] = wl2;

            W[k + 0] = Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[SysID]].dP[0] -
                (rho_r - rho_b[i] - rho_ref[SysID] + rho_b[Raw->DDObs.RefPos[SysID]]);
            W[k + 1] = Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[SysID]].dL[0] -
                (rho_r - rho_b[i] - rho_ref[SysID] + rho_b[Raw->DDObs.RefPos[SysID]]) -
                dX_amb[3 + 2 * Sats + 0] * wl1;
            W[k + 2] = Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[SysID]].dP[1] -
                (rho_r - rho_b[i] - rho_ref[SysID] + rho_b[Raw->DDObs.RefPos[SysID]]);
            W[k + 3] = Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[SysID]].dL[1] -
                (rho_r - rho_b[i] - rho_ref[SysID] + rho_b[Raw->DDObs.RefPos[SysID]]) -
                dX_amb[3 + 2 * Sats + 1] * wl2;

            if (iter == 0)
            {
                for (int m = 0, j = 0; j < Raw->SdObs.SatNum; j++) {
                    SysID2 = Raw->SdObs.SdSatObs[j].System == GPS ? 0 : 1;
                    if (Raw->SdObs.SdSatObs[j].Valid < 1) continue;
                    if (Raw->DDObs.RefPos[SysID2] == -1 || SatNum[SysID2] == 0)continue;
                    if (Raw->DDObs.RefPos[0] == j || Raw->DDObs.RefPos[1] == j)continue;
                    int p = (SatNum[0] + SatNum[1] - 2) * 4;
                    if (i == j)
                    {
                        if (SysID2 == 0)
                        {
                            P[(k + 0) * p + 4 * m + 0] = (SatNum[SysID2] - 1) * 1.0 / SatNum[SysID2];
                            P[(k + 1) * p + 4 * m + 1] = (SatNum[SysID2] - 1) * 1000.0 / SatNum[SysID2];
                            P[(k + 2) * p + 4 * m + 2] = (SatNum[SysID2] - 1) * 1.0 / SatNum[SysID2];
                            P[(k + 3) * p + 4 * m + 3] = (SatNum[SysID2] - 1) * 1000.0 / SatNum[SysID2];
                        }
                        if (SysID2 == 1)
                        {
                            P[(k + 0) * p + 4 * m + 0] = (SatNum[SysID2] - 1) * 0.5 / SatNum[SysID2];
                            P[(k + 1) * p + 4 * m + 1] = (SatNum[SysID2] - 1) * 500.0 / SatNum[SysID2];
                            P[(k + 2) * p + 4 * m + 2] = (SatNum[SysID2] - 1) * 0.5 / SatNum[SysID2];
                            P[(k + 3) * p + 4 * m + 3] = (SatNum[SysID2] - 1) * 500.0 / SatNum[SysID2];
                        }
                    }
                    else
                    {
                        if (SysID == SysID2 && SysID2 == 0)
                        {
                            P[(k + 0) * p + 4 * m + 0] = -1.0 / SatNum[SysID2];
                            P[(k + 1) * p + 4 * m + 1] = -1000.0 / SatNum[SysID2];
                            P[(k + 2) * p + 4 * m + 2] = -1.0 / SatNum[SysID2];
                            P[(k + 3) * p + 4 * m + 3] = -1000.0 / SatNum[SysID2];
                        }
                        if (SysID == SysID2 && SysID2 == 1)
                        {
                            P[(k + 0) * p + 4 * m + 0] = -0.5 / SatNum[SysID2];
                            P[(k + 1) * p + 4 * m + 1] = -500.0 / SatNum[SysID2];
                            P[(k + 2) * p + 4 * m + 2] = -0.5 / SatNum[SysID2];
                            P[(k + 3) * p + 4 * m + 3] = -500.0 / SatNum[SysID2];
                        }
                    }
                    m++;
                }
            }
            Sats++;
            k += 4;
        }

        if (k < num_params) return false;

        // 7. 最小二乘解算

        memset(N, 0, sizeof(N));
        memset(U, 0, sizeof(U));

        // 计算 B转置
        MatrixTranpose(num_params, k, B, Bt);

        // 计算 P*B
        MatrixMultiplies(k, k, num_params, k, P, B, PB);

        // 计算 N = B'*P*B = Bt * PB
        MatrixMultiplies(k, num_params, num_params, k, Bt, PB, N);

        // 计算 U = B'*P*W = Bt * (P*W) 
        MatrixMultiplies(k, k, 1, k, P, W, PW);

        MatrixMultiplies(k, num_params, 1, k, Bt, PW, U);

        if (MatrixInv(num_params, num_params, N, invN) == 0) {
            if (iter > 2)
            {
                break;
            }
            else
            {
                cout << "求逆出现问题，卫星数:" << Sats << "." << endl;
                cout << endl;
                return false;
            }
        }

        // 计算解向量 dX_amb = invN * U
        MatrixMultiplies(num_params, num_params, 1, num_params, invN, U, dX_result);

        // 8. 更新流动站位置
        for (int j = 0; j < num_params; j++) dX_amb[j] += dX_result[j];
        for (int j = 0; j < 3; j++) xRov[j] += dX_result[j];

        iter++;

        // 检查是否收敛
        if (fabs(dX_result[0]) < THRESHOLD &&
            fabs(dX_result[1]) < THRESHOLD &&
            fabs(dX_result[2]) < THRESHOLD) {
            success = true;
            break;
        }

    } while (iter < MAX_ITER);


    Raw->DDObs.Sats = Sats;
    Raw->DDObs.DDSatNum[0] = SatNum[0] - 1;
    Raw->DDObs.DDSatNum[1] = SatNum[1] - 1;
    for (int i = 0; i < num_params * num_params; i++) Raw->DDObs.Qxx[i] = invN[i];
    for (int i = 0; i < num_params; i++) Raw->DDObs.dX_amb[i] = dX_amb[i];
    cout << setiosflags(ios::fixed) << setprecision(4) << "浮点解：" << dX_amb[0] << "  " << dX_amb[1] << "  " << dX_amb[2] << endl;
    for (int i = 0; i < 3; i++) Raw->DDObs.Rov[i] = xRov[i];

    return success;
}