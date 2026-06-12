#ifndef NN_PID_H
#define NN_PID_H

/*
 * 神经网络PID (Neural Network PID)
 * 
 * 原理：使用BP神经网络在线调整PID的Kp,Ki,Kd参数
 * 网络结构：3-5-3 (输入层3个神经元, 隐层5个, 输出层3个)
 * 
 * 输入层: [error, error_sum, delta_error]
 * 输出层: [Kp, Ki, Kd]
 * 
 * 适用场景：非线性系统、参数时变系统
 * 参数整定：学习率lr越大收敛越快但可能震荡，通常0.01~0.5
 */

#define NN_IN   3    /* 输入层节点数 */
#define NN_HID  5    /* 隐层节点数 */
#define NN_OUT  3    /* 输出层节点数 */

typedef struct {
    /* 权值矩阵 */
    float w_ih[NN_HID][NN_IN];    /* 输入->隐层权值 */
    float w_ho[NN_OUT][NN_HID];   /* 隐层->输出权值 */
    float w_ih_last[NN_HID][NN_IN];
    float w_ho_last[NN_OUT][NN_HID];
    
    /* 偏置 */
    float b_h[NN_HID];
    float b_o[NN_OUT];
    
    /* 神经元输出 */
    float o_i[NN_IN];
    float o_h[NN_HID];
    float o_o[NN_OUT];  /* 对应Kp, Ki, Kd */
    
    /* PID参数 */
    float Kp, Ki, Kd;
    float lr;          /* 学习率 */
    
    /* PID状态 */
    float error;
    float error_last;
    float error_sum;
    float u;
    float u_max;
} NN_PID_t;

void NN_PID_Init(NN_PID_t *nn, float lr, float Kp_init, float Ki_init, float Kd_init);
float NN_PID_Update(NN_PID_t *nn, float ref, float y);
void NN_PID_SetOutputLimit(NN_PID_t *nn, float max);

#endif
