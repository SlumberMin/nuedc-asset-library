#include "nn_pid.h"
#include <math.h>
#include <string.h>

/*
 * 神经网络PID实现
 * 使用3层BP神经网络在线自适应调整Kp,Ki,Kd
 * 激活函数: sigmoid (隐层), sigmoid (输出层，保证正值)
 */

static float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

/* 确定性初始化，避免rand()导致不可复现行为 */
static float weight_init(int i, int j, int layer) {
    /* 简单哈希产生[-0.15, 0.15]范围的确定性权重 */
    int hash = (i * 7 + j * 13 + layer * 31) & 0xFF;
    return ((float)hash / 255.0f - 0.5f) * 0.3f;
}

void NN_PID_Init(NN_PID_t *nn, float lr, float Kp_init, float Ki_init, float Kd_init)
{
    memset(nn, 0, sizeof(NN_PID_t));
    nn->lr = lr;
    nn->u_max = 100;
    
    /* 确定性初始化权值 */
    for (int i = 0; i < NN_HID; i++)
        for (int j = 0; j < NN_IN; j++)
            nn->w_ih[i][j] = weight_init(i, j, 0);
    for (int i = 0; i < NN_OUT; i++)
        for (int j = 0; j < NN_HID; j++)
            nn->w_ho[i][j] = weight_init(i, j, 1);
    
    /* 初始PID参数作为输出偏置 */
    nn->b_o[0] = Kp_init;
    nn->b_o[1] = Ki_init;
    nn->b_o[2] = Kd_init;
}

float NN_PID_Update(NN_PID_t *nn, float ref, float y)
{
    /* 计算误差 */
    nn->error_last = nn->error;
    nn->error = ref - y;
    float delta_e = nn->error - nn->error_last;
    nn->error_sum += nn->error;
    
    /* 防积分饱和 */
    if (nn->error_sum > 100) nn->error_sum = 100;
    if (nn->error_sum < -100) nn->error_sum = -100;
    
    /* ===== 前向传播 ===== */
    /* 输入层: 归一化 */
    nn->o_i[0] = nn->error * 0.01f;
    nn->o_i[1] = nn->error_sum * 0.001f;
    nn->o_i[2] = delta_e * 0.1f;
    
    /* 隐层 */
    for (int i = 0; i < NN_HID; i++) {
        float net = nn->b_h[i];
        for (int j = 0; j < NN_IN; j++)
            net += nn->w_ih[i][j] * nn->o_i[j];
        nn->o_h[i] = sigmoid(net);
    }
    
    /* 输出层 (Kp,Ki,Kd必须为正，用sigmoid保证) */
    for (int i = 0; i < NN_OUT; i++) {
        float net = nn->b_o[i];
        for (int j = 0; j < NN_HID; j++)
            net += nn->w_ho[i][j] * nn->o_h[j];
        nn->o_o[i] = sigmoid(net) * 10.0f;  /* 缩放到[0,10] */
    }
    
    nn->Kp = nn->o_o[0];
    nn->Ki = nn->o_o[1];
    nn->Kd = nn->o_o[2];
    
    /* ===== PID输出 ===== */
    nn->u = nn->Kp * nn->error 
          + nn->Ki * nn->error_sum 
          + nn->Kd * delta_e;
    
    /* 输出限幅 */
    if (nn->u > nn->u_max) nn->u = nn->u_max;
    if (nn->u < -nn->u_max) nn->u = -nn->u_max;
    
    /* ===== 反向传播 (BP) ===== */
    /* 以误差的平方对Kp的偏导作为学习信号 */
    float dJ_dy = -nn->error;  /* ∂J/∂y ≈ -error */
    float dy_du = 1.0f;        /* 简化假设 */
    
    /* 输出层梯度 */
    float delta_o[NN_OUT];
    delta_o[0] = dJ_dy * dy_du * nn->error;       /* ∂u/∂Kp = error */
    delta_o[1] = dJ_dy * dy_du * nn->error_sum;   /* ∂u/∂Ki = Σerror */
    delta_o[2] = dJ_dy * dy_du * delta_e;          /* ∂u/∂Kd = Δerror */
    
    /* 更新隐层->输出权值 */
    for (int i = 0; i < NN_OUT; i++) {
        float sigmoid_deriv = nn->o_o[i]/10.0f * (1.0f - nn->o_o[i]/10.0f);
        for (int j = 0; j < NN_HID; j++) {
            float dw = nn->lr * delta_o[i] * sigmoid_deriv * nn->o_h[j];
            nn->w_ho[i][j] += dw;
            /* 权值限制 */
            if (nn->w_ho[i][j] > 5) nn->w_ho[i][j] = 5;
            if (nn->w_ho[i][j] < -5) nn->w_ho[i][j] = -5;
        }
    }
    
    /* 隐层梯度 */
    for (int j = 0; j < NN_HID; j++) {
        float sum = 0;
        for (int i = 0; i < NN_OUT; i++)
            sum += delta_o[i] * nn->w_ho[i][j];
        float h_deriv = nn->o_h[j] * (1.0f - nn->o_h[j]);
        for (int k = 0; k < NN_IN; k++) {
            float dw = nn->lr * sum * h_deriv * nn->o_i[k];
            nn->w_ih[j][k] += dw;
            if (nn->w_ih[j][k] > 5) nn->w_ih[j][k] = 5;
            if (nn->w_ih[j][k] < -5) nn->w_ih[j][k] = -5;
        }
    }
    
    return nn->u;
}

void NN_PID_SetOutputLimit(NN_PID_t *nn, float max) { nn->u_max = max; }
