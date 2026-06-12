#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电赛控制算法快速选型决策树工具

通过问答方式推荐最佳控制算法，包括：
- 控制对象类型
- 系统阶数
- 模型精度
- 实时性要求
- 抗干扰要求
- 调试时间

推荐算法优先级：
- 时间紧张 → PID 或 LADRC
- 有精确模型 → LQR 或 MPC
- 强干扰 → ADRC 或 超螺旋SMC
- 非线性系统 → 模糊PID 或 神经网络PID
- 高精度 → 串级PID + 前馈
"""

import os
import sys

# ==================== 算法数据库 ====================

ALGORITHMS = {
    'position_pid': {
        'name': '位置式PID',
        'file': 'common/position_pid.h, position_pid.c',
        'description': '经典位置式PID，适用于大多数线性系统',
        'difficulty': '易',
        'parameters': {'Kp': '0.5-2.0', 'Ki': '0.1-1.0', 'Kd': '0.01-0.5'},
        'applicable': ['电机', '舵机', '电磁铁', '气泵', '温度', '位置'],
        'order': [1, 2, 3],
        'model_required': False,
        'realtime': '高',
        'anti_disturbance': '中',
        'tuning_time': '短',
        'features': ['简单', '稳定', '成熟'],
        'priority': 1
    },
    'incremental_pid': {
        'name': '增量式PID',
        'file': 'common/incremental_pid.h, incremental_pid.c',
        'description': '增量式PID，适用于步进电机等执行器',
        'difficulty': '易',
        'parameters': {'Kp': '0.5-2.0', 'Ki': '0.1-1.0', 'Kd': '0.01-0.5'},
        'applicable': ['步进电机', '电磁铁', '阀门'],
        'order': [1, 2],
        'model_required': False,
        'realtime': '高',
        'anti_disturbance': '中',
        'tuning_time': '短',
        'features': ['无积分饱和', '切换无冲击'],
        'priority': 2
    },
    'cascade_pid': {
        'name': '串级PID',
        'file': 'common/cascade_pid.h, cascade_pid.c',
        'description': '内外环串级控制，高精度位置/速度控制',
        'difficulty': '中',
        'parameters': {
            'Kp_outer': '0.5-2.0', 'Ki_outer': '0.1-1.0', 'Kd_outer': '0.01-0.5',
            'Kp_inner': '0.5-2.0', 'Ki_inner': '0.1-1.0', 'Kd_inner': '0.01-0.5'
        },
        'applicable': ['电机', '舵机', '机械臂'],
        'order': [2, 3],
        'model_required': False,
        'realtime': '高',
        'anti_disturbance': '强',
        'tuning_time': '中',
        'features': ['高精度', '快速响应', '抗扰动'],
        'priority': 3
    },
    'ladrc': {
        'name': '线性自抗扰控制(LADRC)',
        'file': 'common/ladrc.h, ladrc.c',
        'description': '只需两个参数，无需模型，自抗扰',
        'difficulty': '易',
        'parameters': {'wc': '10-100', 'wo': '30-300'},
        'applicable': ['电机', '舵机', '电磁铁', '气泵', '温度', '位置'],
        'order': [1, 2, 3],
        'model_required': False,
        'realtime': '高',
        'anti_disturbance': '强',
        'tuning_time': '短',
        'features': ['两参数整定', '自抗扰', '鲁棒性强'],
        'priority': 4
    },
    'adrc': {
        'name': '非线性自抗扰控制(ADRC)',
        'file': 'common/adrc.h, adrc.c',
        'description': '完整ADRC，性能最优但参数较多',
        'difficulty': '中',
        'parameters': {'β01': '10-100', 'β02': '5-50', 'β03': '1-10', 'α1': '0.5-0.9', 'α2': '0.25-0.5', 'δ': '0.01-0.1'},
        'applicable': ['电机', '舵机', '电磁铁', '气泵', '温度', '位置'],
        'order': [1, 2, 3],
        'model_required': False,
        'realtime': '高',
        'anti_disturbance': '强',
        'tuning_time': '中',
        'features': ['性能最优', '强鲁棒性', '复杂'],
        'priority': 5
    },
    'fuzzy_pid': {
        'name': '模糊PID',
        'file': 'common/fuzzy_pid.h, fuzzy_pid.c',
        'description': '模糊逻辑自适应调整PID参数',
        'difficulty': '中',
        'parameters': {
            'Kp_base': '0.5-2.0', 'Ki_base': '0.1-1.0', 'Kd_base': '0.01-0.5',
            'fuzzy_rules': '需定制'
        },
        'applicable': ['非线性系统', '电机', '舵机', '机器人'],
        'order': [1, 2, 3],
        'model_required': False,
        'realtime': '中',
        'anti_disturbance': '强',
        'tuning_time': '中',
        'features': ['非线性适应', '智能', '需规则库'],
        'priority': 6
    },
    'neural_pid': {
        'name': '神经网络PID',
        'file': 'common/neural_pid.h, neural_pid.c',
        'description': '单神经元在线学习PID参数',
        'difficulty': '中',
        'parameters': {'lr_p': '0.1-0.5', 'lr_i': '0.05-0.2', 'lr_d': '0.01-0.1'},
        'applicable': ['非线性系统', '时变系统', '电机', '机器人'],
        'order': [1, 2, 3],
        'model_required': False,
        'realtime': '中',
        'anti_disturbance': '强',
        'tuning_time': '短',
        'features': ['在线学习', '自适应', '无需调参'],
        'priority': 7
    },
    'lqr': {
        'name': 'LQR线性二次调节器',
        'file': 'common/lqr.h, lqr.c',
        'description': '最优状态反馈控制，需要精确模型',
        'difficulty': '中',
        'parameters': {'Q': '对角矩阵', 'R': '对角矩阵'},
        'applicable': ['已知模型系统', '电机', '机器人'],
        'order': [2, 3],
        'model_required': True,
        'realtime': '高',
        'anti_disturbance': '中',
        'tuning_time': '长',
        'features': ['最优', '稳定', '需要模型'],
        'priority': 8
    },
    'mpc': {
        'name': 'MPC模型预测控制',
        'file': 'common/mpc_simple.h, mpc_simple.c',
        'description': '滚动优化预测控制',
        'difficulty': '难',
        'parameters': {'N': '5-20步', 'Q': '对角矩阵', 'R': '对角矩阵'},
        'applicable': ['已知模型系统', '复杂约束系统'],
        'order': [2, 3],
        'model_required': True,
        'realtime': '低',
        'anti_disturbance': '中',
        'tuning_time': '长',
        'features': ['处理约束', '最优', '计算量大'],
        'priority': 9
    },
    'smc': {
        'name': '滑模控制(SMC)',
        'file': 'common/smc_sliding_mode.h, smc_sliding_mode.c',
        'description': '变结构控制，对不确定性强鲁棒',
        'difficulty': '中',
        'parameters': {'lambda': '5-20', 'epsilon': '0.1-1.0', 'k': '1-5'},
        'applicable': ['非线性系统', '不确定系统', '电机', '机器人'],
        'order': [2, 3],
        'model_required': False,
        'realtime': '高',
        'anti_disturbance': '强',
        'tuning_time': '中',
        'features': ['强鲁棒性', '抖振问题'],
        'priority': 10
    },
    'super_twisting_smc': {
        'name': '超螺旋滑模控制',
        'file': 'common/super_twisting_smc.h, super_twisting_smc.c',
        'description': '高阶滑模，抑制抖振',
        'difficulty': '难',
        'parameters': {'lambda': '5-20', 'alpha': '1-5', 'beta': '1-10'},
        'applicable': ['高精度非线性系统', '航天', '机器人'],
        'order': [2, 3],
        'model_required': False,
        'realtime': '高',
        'anti_disturbance': '强',
        'tuning_time': '长',
        'features': ['无抖振', '高精度', '复杂'],
        'priority': 11
    },
    'feedforward': {
        'name': '前馈控制',
        'file': 'common/feedforward.h, feedforward.c',
        'description': '基于模型的前馈，提高响应速度',
        'difficulty': '易',
        'parameters': {'ff_gain': '0.1-1.0'},
        'applicable': ['已知模型', '轨迹跟踪'],
        'order': [1, 2, 3],
        'model_required': True,
        'realtime': '高',
        'anti_disturbance': '弱',
        'tuning_time': '短',
        'features': ['提高响应速度', '需要模型', '无反馈'],
        'priority': 12
    }
}


def get_user_choice(options, prompt):
    """获取用户选择"""
    print(f"\n{'='*60}")
    print(f"{prompt}")
    print(f"{'='*60}")

    for i, opt in enumerate(options, 1):
        print(f"  {i}. {opt}")

    while True:
        try:
            choice = int(input("\n请输入选项编号: ").strip())
            if 1 <= choice <= len(options):
                return choice - 1
            else:
                print(f"错误：请输入 1-{len(options)} 之间的数字")
        except ValueError:
            print("错误：请输入有效的数字")


def ask_questions():
    """向用户提问并收集信息"""
    print("\n" + "="*60)
    print("电赛控制算法快速选型工具")
    print("="*60)
    print("回答以下问题，系统将为您推荐最佳算法\n")

    answers = {}

    # Q1: 控制对象类型
    options1 = ['电机', '舵机', '电磁铁', '气泵', '温度控制', '位置控制', '其他']
    idx = get_user_choice(options1, "Q1: 控制对象类型是什么？")
    answers['applicable'] = options1[idx]

    # Q2: 系统阶数
    options2 = ['一阶系统（如温控、液位）', '二阶系统（如电机位置、振荡系统）', '三阶及以上（如多关节机械臂）']
    idx = get_user_choice(options2, "Q2: 系统阶数？")
    if idx == 0:
        answers['order'] = 1
    elif idx == 1:
        answers['order'] = 2
    else:
        answers['order'] = 3

    # Q3: 是否有精确模型
    options3 = ['有精确的数学模型', '模型大致知道（参数不确定）', '完全未知，黑箱系统']
    idx = get_user_choice(options3, "Q3: 是否有精确的数学模型？")
    if idx == 0:
        answers['model_required'] = True
        answers['model_precise'] = True
    elif idx == 1:
        answers['model_required'] = False
        answers['model_precise'] = False
    else:
        answers['model_required'] = False
        answers['model_precise'] = False

    # Q4: 实时性要求
    options4 = ['高（< 1ms响应）', '中（1-10ms响应）', '低（> 10ms可以）']
    idx = get_user_choice(options4, "Q4: 实时性要求？")
    answers['realtime'] = ['高', '中', '低'][idx]

    # Q5: 抗干扰要求
    options5 = ['强抗干扰（复杂环境）', '中等抗干扰', '弱抗干扰（环境稳定）']
    idx = get_user_choice(options5, "Q5: 抗干扰要求？")
    answers['anti_disturbance'] = ['强', '中', '弱'][idx]

    # Q6: 调试时间
    options6 = ['充裕（可以仔细调试）', '紧张（需要快速出结果）', '极少（基本没法调试）']
    idx = get_user_choice(options6, "Q6: 调试时间充裕度？")
    answers['tuning_time'] = ['长', '中', '短'][idx]

    return answers


def score_algorithm(algo, answers):
    """计算算法与需求的匹配度分数"""
    score = 100

    # 1. 控制对象匹配 (20分)
    if answers['applicable'] in algo['applicable']:
        score += 0
    elif any(answers['applicable'].lower() in a.lower() for a in algo['applicable']):
        score += 0
    else:
        score -= 20

    # 2. 系统阶数匹配 (20分)
    if answers['order'] in algo['order']:
        score += 0
    else:
        score -= 20

    # 3. 模型要求匹配 (25分)
    if answers['model_required'] and not algo['model_required']:
        # 有模型但算法不需要 - 还好
        score += 0
    elif answers['model_required'] and algo['model_required']:
        # 有模型且算法需要 - 最优
        score += 10
    elif not answers['model_required'] and algo['model_required']:
        # 无模型但算法需要 - 最差
        score -= 25
    else:
        score += 5

    # 4. 实时性要求 (15分)
    realtime_map = {'高': 3, '中': 2, '低': 1}
    algo_rt = realtime_map.get(algo['realtime'], 2)
    need_rt = realtime_map.get(answers['realtime'], 2)
    if algo_rt >= need_rt:
        score += 0
    else:
        score -= 15

    # 5. 抗干扰要求 (15分)
    dist_map = {'强': 3, '中': 2, '弱': 1}
    algo_dist = dist_map.get(algo['anti_disturbance'], 2)
    need_dist = dist_map.get(answers['anti_disturbance'], 2)
    if algo_dist >= need_dist:
        score += 5
    else:
        score -= 15

    # 6. 调试时间 (15分)
    tuning_map = {'短': 1, '中': 2, '长': 3}
    algo_tuning = tuning_map.get(algo['tuning_time'], 2)
    need_tuning = tuning_map.get(answers['tuning_time'], 2)
    if algo_tuning <= need_tuning:
        score += 10
    else:
        score -= 10

    return score


def recommend_algorithms(answers):
    """推荐算法"""
    scores = []
    for key, algo in ALGORITHMS.items():
        score = score_algorithm(algo, answers)
        scores.append((score, key, algo))

    # 按分数降序排列
    scores.sort(key=lambda x: -x[0])

    return scores[:5]  # 返回前5个推荐


def print_recommendation(recommendations, answers):
    """打印推荐结果"""
    print("\n" + "="*60)
    print("算法推荐结果")
    print("="*60)

    print("\n根据您的需求，推荐以下算法（按优先级排序）：\n")

    for i, (score, key, algo) in enumerate(recommendations, 1):
        priority_tag = ""
        if i <= 2:
            priority_tag = " ★ 强烈推荐"
        elif i <= 4:
            priority_tag = " ☆ 推荐"
        else:
            priority_tag = "   可选"

        print(f"【{i}】{algo['name']}{priority_tag}")
        print(f"    文件路径: {algo['file']}")
        print(f"    难度: {algo['difficulty']} | 实时性: {algo['realtime']} | 抗干扰: {algo['anti_disturbance']}")
        print(f"    描述: {algo['description']}")
        print(f"    特点: {', '.join(algo['features'])}")
        print(f"    参数整定建议:")
        for param, value in algo['parameters'].items():
            print(f"      - {param}: {value}")
        print()

    # 额外建议
    print("\n" + "="*60)
    print("综合建议")
    print("="*60)

    if answers['tuning_time'] == '短':
        print("\n【时间紧迫】")
        print("  建议使用 PID 或 LADRC（仅需2个参数）")
        print("  文件: common/position_pid.h 或 common/ladrc.h")
    elif answers['model_precise']:
        print("\n【有精确模型】")
        print("  建议使用 LQR（线性二次最优）")
        print("  文件: common/lqr.h")
    elif answers['anti_disturbance'] == '强':
        print("\n【强抗干扰需求】")
        print("  建议使用 ADRC 或 超螺旋SMC")
        print("  文件: common/adrc.h 或 common/super_twisting_smc.h")
    else:
        print("\n【通用场景】")
        print("  建议使用 PID + 前馈 组合")
        print("  文件: common/position_pid.h + common/feedforward.h")


def print_code_example(algo_key):
    """打印代码使用示例"""
    algo = ALGORITHMS.get(algo_key, None)
    if not algo:
        return

    print("\n" + "="*60)
    print(f"{algo['name']} 使用示例")
    print("="*60)

    if algo_key == 'position_pid':
        print("""
// 1. 包含头文件
#include "position_pid.h"

// 2. 定义PID结构体
PositionPID_t pid;

// 3. 初始化PID参数
PositionPID_Init(&pid, 1.0f, 0.5f, 0.1f, -1000.0f, 1000.0f);
PositionPID_SetIntegralLimit(&pid, -500.0f, 500.0f);

// 4. 在控制循环中调用
float target = 100.0f;
float measurement = get_sensor_reading();
float output = PositionPID_Compute(&pid, target, measurement);
set_motor_output(output);
        """)
    elif algo_key == 'ladrc':
        print("""
// 1. 包含头文件
#include "ladrc.h"

// 2. 定义ADRC结构体
LADRC_t ladrc;

// 3. 初始化LADRC
LADRC_Init(&ladrc, 1.0f, 2, 0.001f);  // 控制带宽10, 观测器带宽30
LADRC_SetBandwidth(&ladrc, 10.0f, 30.0f);

// 4. 在控制循环中调用
float target = 100.0f;
float measurement = get_sensor_reading();
float output = LADRC_Compute(&ladrc, target, measurement);
set_motor_output(output);
        """)
    elif algo_key == 'neural_pid':
        print("""
// 1. 包含头文件
#include "neural_pid.h"

// 2. 定义神经元PID结构体
NeuralPID_t neural_pid;

// 3. 初始化神经元PID
NeuralPID_Init(&neural_pid);
NeuralPID_SetLearningRate(&neural_pid, 0.2f, 0.1f, 0.05f);

// 4. 在控制循环中调用
float target = 100.0f;
float measurement = get_sensor_reading();
float output = NeuralPID_Compute(&neural_pid, target, measurement);
set_motor_output(output);

// 5. 查看学习到的权重（即等效PID参数）
float w1, w2, w3;
NeuralPID_GetWeights(&neural_pid, &w1, &w2, &w3);
        """)
    else:
        print(f"  参考文件: {algo['file']}")
        print("  请查看头文件中的API说明和示例")


def main():
    """主函数"""
    try:
        answers = ask_questions()
        recommendations = recommend_algorithms(answers)
        print_recommendation(recommendations, answers)

        # 询问是否查看代码示例
        print("\n" + "="*60)
        choice = input("是否查看第一个推荐算法的代码示例？(y/n): ").strip().lower()
        if choice == 'y':
            print_code_example(recommendations[0][1])

        print("\n" + "="*60)
        print("推荐算法文件位置：D:\\Users\\Lie\\Desktop\\电赛\\nuedc-asset-library\\11_控制算法库\\")
        print("="*60)

    except KeyboardInterrupt:
        print("\n\n操作已取消")
        sys.exit(0)
    except Exception as e:
        print(f"\n错误：{e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
