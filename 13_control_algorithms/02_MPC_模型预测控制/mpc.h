#ifndef MPC_H
#define MPC_H

/*
 * 模型预测控制 MPC (Model Predictive Control) 嵌入式简化实现
 * 
 * 适用于离散状态空间模型: x(k+1) = A*x(k) + B*u(k)
 * 
 * 简化策略：
 * - 固定预测步长Np和控制步长Nc
 * - 二次规划采用在线递推求解（适用于小规模问题）
 * - 仅支持单输入单输出(SISO)系统
 * 
 * 适用场景：电机控制、过程控制
 * 参数整定：Np=10~30, Nc=3~5, Q越大越快响应, R越大控制越平滑
 */

#define MPC_NP  10   /* 预测步长 */
#define MPC_NC  3    /* 控制步长 */
#define MPC_NX  2    /* 状态维数 */

typedef struct {
    float A[MPC_NX][MPC_NX];  /* 状态矩阵 */
    float B[MPC_NX];           /* 输入矩阵 */
    float C[MPC_NX];           /* 输出矩阵 */
    float Q;                    /* 状态权重 */
    float R;                    /* 控制权重 */
    float x[MPC_NX];           /* 当前状态 */
    float u_min, u_max;        /* 控制约束 */
    float du_min, du_max;      /* 增量约束 */
    float dt;
    float du[MPC_NC];          /* 控制增量序列（避免static局部变量） */
    float u_total;             /* 累积控制量（避免static局部变量） */
} MPC_t;

void MPC_Init(MPC_t *mpc, float dt);
void MPC_SetModel(MPC_t *mpc, float A[2][2], float B[2], float C[2]);
void MPC_SetWeight(MPC_t *mpc, float Q, float R);
void MPC_SetConstraint(MPC_t *mpc, float u_min, float u_max, float du_min, float du_max);
float MPC_Update(MPC_t *mpc, float ref, float y_meas);

#endif
