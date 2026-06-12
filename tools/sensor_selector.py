#!/usr/bin/env python3
"""传感器选型工具 - 根据应用场景推荐传感器

用法:
    python sensor_selector.py --category temperature --precision high
    python sensor_selector.py --category motion --interface I2C
    python sensor_selector.py --list-categories
    python sensor_selector.py --interactive
"""

import argparse
import json
import sys

# ============================================================
# 传感器数据库（电赛常用传感器）
# ============================================================
SENSOR_DATABASE = [
    # ======== 温度传感器 ========
    {
        "型号": "DS18B20", "类别": "温度", "厂商": "Dallas/Maxim",
        "测量范围": "-55~125°C", "精度": "±0.5°C", "精度等级": "medium",
        "接口": "1-Wire", "供电电压V": [3.0, 5.5], "功耗": "低",
        "封装": "TO-92/SO8", "单价约": 3.0, "供货": "充足",
        "特点": "单总线，数字输出，接线简单，电赛经典"
    },
    {
        "型号": "DHT11", "类别": "温湿度", "厂商": "Aosong",
        "测量范围": "温度0~50°C,湿度20~90%RH", "精度": "±2°C,±5%RH", "精度等级": "low",
        "接口": "单总线", "供电电压V": [3.3, 5.5], "功耗": "低",
        "封装": "模块", "单价约": 5.0, "供货": "充足",
        "特点": "温湿度一体，价格低，适合入门"
    },
    {
        "型号": "DHT22/AM2302", "类别": "温湿度", "厂商": "Aosong",
        "测量范围": "温度-40~80°C,湿度0~100%RH", "精度": "±0.5°C,±2%RH", "精度等级": "high",
        "接口": "单总线", "供电电压V": [3.3, 6.0], "功耗": "低",
        "封装": "模块", "单价约": 12.0, "供货": "充足",
        "特点": "温湿度一体，精度高"
    },
    {
        "型号": "SHT30", "类别": "温湿度", "厂商": "Sensirion",
        "测量范围": "温度-40~125°C,湿度0~100%RH", "精度": "±0.3°C,±2%RH", "精度等级": "high",
        "接口": "I2C", "供电电压V": [2.4, 5.5], "功耗": "低",
        "封装": "DFN8", "单价约": 15.0, "供货": "充足",
        "特点": "高精度温湿度，工业级"
    },
    {
        "型号": "LM35", "类别": "温度", "厂商": "TI",
        "测量范围": "-55~150°C", "精度": "±0.5°C", "精度等级": "medium",
        "接口": "模拟", "供电电压V": [4.0, 30.0], "功耗": "低",
        "封装": "TO-92", "单价约": 2.0, "供货": "充足",
        "特点": "模拟输出，10mV/°C，无需校准"
    },
    {
        "型号": "MLX90614", "类别": "红外温度", "厂商": "Melexis",
        "测量范围": "环境-40~85°C,目标-70~380°C", "精度": "±0.5°C", "精度等级": "high",
        "接口": "I2C/SMBus", "供电电压V": [2.6, 3.6], "功耗": "低",
        "封装": "TO-39", "单价约": 25.0, "供货": "一般",
        "特点": "非接触红外测温，医疗级"
    },
    # ======== 距离/测距传感器 ========
    {
        "型号": "HC-SR04", "类别": "超声波测距", "厂商": "通用",
        "测量范围": "2cm~4m", "精度": "±3mm", "精度等级": "medium",
        "接口": "GPIO触发", "供电电压V": [4.5, 5.5], "功耗": "中",
        "封装": "模块", "单价约": 4.0, "供货": "充足",
        "特点": "超声波测距，经典方案，性价比极高"
    },
    {
        "型号": "VL53L0X", "类别": "激光测距", "厂商": "ST",
        "测量范围": "5cm~2m (ToF)", "精度": "±3%", "精度等级": "high",
        "接口": "I2C", "供电电压V": [2.6, 3.5], "功耗": "低",
        "封装": "模块", "单价约": 12.0, "供货": "充足",
        "特点": "ToF激光测距，精度高，体积小"
    },
    {
        "型号": "VL53L1X", "类别": "激光测距", "厂商": "ST",
        "测量范围": "5cm~4m (ToF)", "精度": "±3%", "精度等级": "high",
        "接口": "I2C", "供电电压V": [2.6, 3.5], "功耗": "低",
        "封装": "模块", "单价约": 18.0, "供货": "充足",
        "特点": "长距离ToF，快速测量"
    },
    {
        "型号": "TFmini-S", "类别": "激光雷达", "厂商": "北醒",
        "测量范围": "0.1~12m", "精度": "±1cm", "精度等级": "high",
        "接口": "UART/TTL", "供电电压V": [4.5, 6.0], "功耗": "中",
        "封装": "模块", "单价约": 70.0, "供货": "充足",
        "特点": "单点激光雷达，无人机避障常用"
    },
    # ======== 运动/惯性传感器 ========
    {
        "型号": "MPU6050", "类别": "六轴IMU", "厂商": "InvenSense/TDK",
        "测量范围": "±2~16g,±250~2000°/s", "精度": "中等", "精度等级": "medium",
        "接口": "I2C", "供电电压V": [2.375, 3.46], "功耗": "低",
        "封装": "QFN4x4", "单价约": 5.0, "供货": "充足",
        "特点": "6轴(加速度+陀螺仪)，DMP内置姿态解算，电赛经典"
    },
    {
        "型号": "MPU9250", "类别": "九轴IMU", "厂商": "InvenSense/TDK",
        "测量范围": "±16g,±2000°/s,±4800μT", "精度": "中等", "精度等级": "medium",
        "接口": "I2C/SPI", "供电电压V": [2.4, 3.6], "功耗": "低",
        "封装": "QFN3x3", "单价约": 12.0, "供货": "一般",
        "特点": "9轴(加速度+陀螺仪+磁力计)，全姿态解算"
    },
    {
        "型号": "ICM-42688-P", "类别": "六轴IMU", "厂商": "InvenSense/TDK",
        "测量范围": "±16g,±2000°/s", "精度": "高", "精度等级": "high",
        "接口": "I2C/SPI", "供电电压V": [1.71, 3.6], "功耗": "极低",
        "封装": "LGA14", "单价约": 15.0, "供货": "充足",
        "特点": "新一代高性能6轴IMU，低噪声"
    },
    {
        "型号": "QMC5883L", "类别": "磁力计", "厂商": "矽睿",
        "测量范围": "±8 Gauss", "精度": "±1°", "精度等级": "medium",
        "接口": "I2C", "供电电压V": [2.16, 3.6], "功耗": "低",
        "封装": "LGA16", "单价约": 3.0, "供货": "充足",
        "特点": "电子罗盘，航向角测量"
    },
    {
        "型号": "ADXL345", "类别": "加速度计", "厂商": "ADI",
        "测量范围": "±2/4/8/16g", "精度": "±0.5°倾斜", "精度等级": "high",
        "接口": "I2C/SPI", "供电电压V": [2.0, 3.6], "功耗": "低",
        "封装": "LGA14", "单价约": 8.0, "供货": "充足",
        "特点": "高精度3轴加速度计"
    },
    # ======== 光学/光电传感器 ========
    {
        "型号": "BH1750", "类别": "光照", "厂商": "Rohm",
        "测量范围": "1~65535 lux", "精度": "±20%", "精度等级": "medium",
        "接口": "I2C", "供电电压V": [2.4, 3.6], "功耗": "低",
        "封装": "模块", "单价约": 4.0, "供货": "充足",
        "特点": "数字光照传感器，量程宽"
    },
    {
        "型号": "TSL2561", "类别": "光照", "厂商": "AMS",
        "测量范围": "0.1~40000 lux", "精度": "±40%", "精度等级": "low",
        "接口": "I2C", "供电电压V": [2.7, 3.6], "功耗": "低",
        "封装": "TMB6", "单价约": 8.0, "供货": "一般",
        "特点": "宽带光谱，接近人眼响应"
    },
    {
        "型号": "APDS-9960", "类别": "手势/颜色/接近", "厂商": "Broadcom",
        "测量范围": "手势识别0~20cm,颜色RGBC", "精度": "中等", "精度等级": "medium",
        "接口": "I2C", "供电电压V": [2.4, 3.6], "功耗": "低",
        "封装": "LGA8", "单价约": 10.0, "供货": "充足",
        "特点": "手势+颜色+接近三合一"
    },
    # ======== 气体/环境传感器 ========
    {
        "型号": "MQ-2", "类别": "烟雾/可燃气体", "厂商": "通用",
        "测量范围": "可燃气体/烟雾", "精度": "定性检测", "精度等级": "low",
        "接口": "模拟", "供电电压V": [4.9, 5.1], "功耗": "中",
        "封装": "模块", "单价约": 5.0, "供货": "充足",
        "特点": "烟雾检测，需预热"
    },
    {
        "型号": "MQ-135", "类别": "空气质量", "厂商": "通用",
        "测量范围": "NH3,NOx,CO2,苯等", "精度": "定性检测", "精度等级": "low",
        "接口": "模拟", "供电电压V": [4.9, 5.1], "功耗": "中",
        "封装": "模块", "单价约": 6.0, "供货": "充足",
        "特点": "空气质量检测，多气体"
    },
    {
        "型号": "SGP30", "类别": "TVOC/eCO2", "厂商": "Sensirion",
        "测量范围": "TVOC 0~60000ppb, eCO2 400~60000ppm", "精度": "±15%", "精度等级": "medium",
        "接口": "I2C", "供电电压V": [1.62, 1.98], "功耗": "低",
        "封装": "DFN8", "单价约": 30.0, "供货": "一般",
        "特点": "TVOC+CO2双输出，室内空气品质"
    },
    {
        "型号": "BME280", "类别": "环境(温湿压)", "厂商": "Bosch",
        "测量范围": "温度-40~85°C,湿度0~100%RH,气压300~1100hPa", "精度": "±0.5°C,±3%RH,±1hPa", "精度等级": "high",
        "接口": "I2C/SPI", "供电电压V": [1.71, 3.6], "功耗": "低",
        "封装": "LGA8", "单价约": 12.0, "供货": "充足",
        "特点": "温湿度+气压三合一，适合气象/高度计"
    },
    # ======== 气压传感器 ========
    {
        "型号": "BMP280", "类别": "气压", "厂商": "Bosch",
        "测量范围": "300~1100hPa,温度-40~85°C", "精度": "±1hPa,±1°C", "精度等级": "high",
        "接口": "I2C/SPI", "供电电压V": [1.71, 3.6], "功耗": "低",
        "封装": "LGA8", "单价约": 5.0, "供货": "充足",
        "特点": "气压+温度，海拔测量"
    },
    # ======== 电流/电压传感器 ========
    {
        "型号": "INA219", "类别": "电流/功率", "厂商": "TI",
        "测量范围": "±3.2A,0~26V", "精度": "±0.5%", "精度等级": "high",
        "接口": "I2C", "供电电压V": [2.7, 5.5], "功耗": "低",
        "封装": "SOIC8", "单价约": 6.0, "供货": "充足",
        "特点": "电流/电压/功率监测，I2C数字输出"
    },
    {
        "型号": "ACS712", "类别": "电流", "厂商": "Allegro",
        "测量范围": "±5A/±20A/±30A", "精度": "±1.5%", "精度等级": "medium",
        "接口": "模拟", "供电电压V": [4.5, 5.5], "功耗": "中",
        "封装": "SOIC8", "单价约": 8.0, "供货": "充足",
        "特点": "霍尔效应电流检测，隔离测量"
    },
    # ======== 超声波/声学传感器 ========
    {
        "型号": "MAX9814", "类别": "麦克风/音频", "厂商": "Maxim",
        "测量范围": "音频40Hz~20kHz", "精度": "AGC自动增益", "精度等级": "medium",
        "接口": "模拟", "供电电压V": [2.2, 5.5], "功耗": "低",
        "封装": "模块", "单价约": 5.0, "供货": "充足",
        "特点": "驻极体麦克风+AGC，声源定位"
    },
    # ======== GPS/定位 ========
    {
        "型号": "NEO-6M", "类别": "GPS定位", "厂商": "u-blox",
        "测量范围": "全球定位", "精度": "2.5m CEP", "精度等级": "medium",
        "接口": "UART", "供电电压V": [2.7, 3.6], "功耗": "中",
        "封装": "模块", "单价约": 20.0, "供货": "充足",
        "特点": "GPS模块，定位导航"
    },
    {
        "型号": "AT6558R", "类别": "北斗定位", "厂商": "中科微",
        "测量范围": "全球定位(BDS+GPS)", "精度": "2.5m CEP", "精度等级": "medium",
        "接口": "UART", "供电电压V": [3.0, 3.6], "功耗": "中",
        "封装": "模块", "单价约": 15.0, "供货": "充足",
        "特点": "北斗+GPS双模，国产方案"
    },
    # ======== 摄像头/视觉 ========
    {
        "型号": "OV2640", "类别": "摄像头", "厂商": "OmniVision",
        "测量范围": "200万像素", "精度": "1600x1200", "精度等级": "medium",
        "接口": "DCMI/SPI", "供电电压V": [2.6, 3.0], "功耗": "中",
        "封装": "模块", "单价约": 15.0, "供货": "充足",
        "特点": "200万像素CMOS，JPEG输出，电赛常用"
    },
    {
        "型号": "OV7670", "类别": "摄像头", "厂商": "OmniVision",
        "测量范围": "30万像素", "精度": "640x480", "精度等级": "low",
        "接口": "SCCB", "供电电压V": [2.45, 3.0], "功耗": "中",
        "封装": "模块", "单价约": 8.0, "供货": "充足",
        "特点": "VGA分辨率，低成本视觉方案"
    },
    {
        "型号": "OpenMV4 H7", "类别": "视觉模块", "厂商": "OpenMV",
        "测量范围": "机器视觉", "精度": "QVGA@60fps", "精度等级": "high",
        "接口": "UART/SPI/I2C", "供电电压V": [3.3, 5.0], "功耗": "中",
        "封装": "模块", "单价约": 200.0, "供货": "一般",
        "特点": "MicroPython机器视觉，色块/人脸/二维码识别"
    },
    # ======== 颜色传感器 ========
    {
        "型号": "TCS34725", "类别": "颜色", "厂商": "AMS",
        "测量范围": "RGBC颜色识别", "精度": "16bit分辨率", "精度等级": "high",
        "接口": "I2C", "供电电压V": [2.7, 3.6], "功耗": "低",
        "封装": "模块", "单价约": 8.0, "供货": "充足",
        "特点": "高精度颜色识别，滤光片+ADC"
    },
    # ======== 旋转/编码器 ========
    {
        "型号": "AS5600", "类别": "角度编码器", "厂商": "AMS",
        "测量范围": "0~360°旋转角度", "精度": "0.087°(12bit)", "精度等级": "high",
        "接口": "I2C/模拟/PWM", "供电电压V": [3.3, 3.6], "功耗": "低",
        "封装": "TSSOP16", "单价约": 12.0, "供货": "充足",
        "特点": "磁性旋转编码器，非接触式"
    },
    # ======== 心率/生物传感器 ========
    {
        "型号": "MAX30102", "类别": "心率/血氧", "厂商": "Maxim",
        "测量范围": "心率/SpO2", "精度": "医疗级算法", "精度等级": "high",
        "接口": "I2C", "供电电压V": [1.8, 3.3], "功耗": "低",
        "封装": "模块", "单价约": 10.0, "供货": "充足",
        "特点": "心率+血氧检测，光电容积脉搏波"
    },
    # ======== 称重/压力 ========
    {
        "型号": "HX711+压力传感器", "类别": "称重", "厂商": "海芯",
        "测量范围": "根据传感器(几g~几百kg)", "精度": "24bit ADC", "精度等级": "high",
        "接口": "专用2线", "供电电压V": [2.6, 5.5], "功耗": "低",
        "封装": "SOP16+模块", "单价约": 5.0, "供货": "充足",
        "特点": "24位ADC，称重电子秤方案"
    },
    # ======== 雨滴/水位 ========
    {
        "型号": "雨滴传感器模块", "类别": "雨滴/湿度", "厂商": "通用",
        "测量范围": "有无雨滴", "精度": "开关量", "精度等级": "low",
        "接口": "模拟+数字", "供电电压V": [3.3, 5.0], "功耗": "低",
        "封装": "模块", "单价约": 3.0, "供货": "充足",
        "特点": "雨滴检测，数字/模拟双输出"
    },
    # ======== 超声波风速 ========
    {
        "型号": "FS3000", "类别": "风速", "厂商": "Renesas",
        "测量范围": "0~7.2m/s或0~15m/s", "精度": "±5%", "精度等级": "medium",
        "接口": "I2C/模拟", "供电电压V": [3.3, 5.5], "功耗": "低",
        "封装": "模块", "单价约": 60.0, "供货": "一般",
        "特点": "固态风速传感器，无机械部件"
    },
]


def find_sensors(requirements: dict) -> list:
    """根据需求筛选传感器"""
    results = []
    for sensor in SENSOR_DATABASE:
        score = 0
        reasons = []
        penalties = []

        # 硬性筛选
        if requirements.get("category"):
            cat = requirements["category"].lower()
            if cat not in sensor["类别"].lower():
                continue

        if requirements.get("interface"):
            iface = requirements["interface"].upper()
            if iface not in sensor["接口"].upper():
                continue

        if requirements.get("voltage"):
            v = requirements["voltage"]
            if not (sensor["供电电压V"][0] <= v <= sensor["供电电压V"][1]):
                continue

        # 精度等级筛选
        if requirements.get("precision"):
            prec = requirements["precision"].lower()
            level_map = {"low": 1, "medium": 2, "high": 3}
            sensor_level = level_map.get(sensor["精度等级"], 0)
            req_level = level_map.get(prec, 0)
            if sensor_level < req_level:
                continue

        # 软性评分
        # 精度加分
        if sensor["精度等级"] == "high":
            score += 20
            reasons.append("高精度")
        elif sensor["精度等级"] == "medium":
            score += 10

        # 价格加分
        if sensor["单价约"] <= 5:
            score += 15
            reasons.append(f"高性价比(¥{sensor['单价约']})")
        elif sensor["单价约"] <= 15:
            score += 8
        elif sensor["单价约"] <= 50:
            score += 3
        else:
            penalties.append(f"价格较高(¥{sensor['单价约']})")
            score -= 5

        # 供货加分
        if sensor["供货"] == "充足":
            score += 10
            reasons.append("供货充足")
        else:
            penalties.append("供货紧张")
            score -= 5

        # 功耗加分
        if sensor["功耗"] == "极低":
            score += 10
            reasons.append("超低功耗")
        elif sensor["功耗"] == "低":
            score += 5

        # 数字接口加分
        if "I2C" in sensor["接口"] or "SPI" in sensor["接口"] or "UART" in sensor["接口"]:
            score += 5
            reasons.append("数字接口")

        # 关键词匹配
        if requirements.get("keyword"):
            kw = requirements["keyword"]
            if kw in sensor["特点"] or kw in sensor["类别"]:
                score += 15
                reasons.append(f"关键词匹配: {kw}")

        results.append({
            "型号": sensor["型号"],
            "类别": sensor["类别"],
            "厂商": sensor["厂商"],
            "测量范围": sensor["测量范围"],
            "精度": sensor["精度"],
            "接口": sensor["接口"],
            "供电": f"{sensor['供电电压V'][0]}~{sensor['供电电压V'][1]}V",
            "功耗": sensor["功耗"],
            "单价约": f"¥{sensor['单价约']}",
            "供货": sensor["供货"],
            "特点": sensor["特点"],
            "匹配分": score,
            "优势": reasons,
            "不足": penalties,
        })

    results.sort(key=lambda x: x["匹配分"], reverse=True)
    return results


def print_results(results: list, top_n: int = 5):
    """格式化打印传感器推荐结果"""
    if not results:
        print("\n❌ 未找到满足条件的传感器，请放宽条件重试。")
        return

    print(f"\n{'='*70}")
    print(f"  传感器选型推荐结果（共 {len(results)} 款，显示前 {min(top_n, len(results))} 款）")
    print(f"{'='*70}")

    for i, s in enumerate(results[:top_n], 1):
        print(f"\n  ┌─ 第 {i} 名 ─ 匹配分: {s['匹配分']}分")
        print(f"  │ 型号: {s['型号']}  类别: {s['类别']}  厂商: {s['厂商']}")
        print(f"  │ 测量范围: {s['测量范围']}")
        print(f"  │ 精度: {s['精度']}  接口: {s['接口']}")
        print(f"  │ 供电: {s['供电']}  功耗: {s['功耗']}")
        print(f"  │ 单价: {s['单价约']}  供货: {s['供货']}")
        print(f"  │ 特点: {s['特点']}")
        if s["优势"]:
            print(f"  │ ✅ {'; '.join(s['优势'])}")
        if s["不足"]:
            print(f"  │ ⚠️ {'; '.join(s['不足'])}")
        print(f"  └{'─'*60}")

    print()


def interactive_mode():
    """交互式选型"""
    print("\n" + "="*60)
    print("  🔧 传感器交互式选型工具")
    print("="*60)

    categories = list(set(s["类别"] for s in SENSOR_DATABASE))
    categories.sort()
    print(f"  可选类别: {', '.join(categories)}")
    print("  （直接回车跳过）\n")

    req = {}
    try:
        cat = input("  传感器类别: ").strip()
        if cat:
            req["category"] = cat

        prec = input("  精度等级 (low/medium/high): ").strip()
        if prec:
            req["precision"] = prec

        iface = input("  接口类型 (I2C/SPI/UART/模拟): ").strip()
        if iface:
            req["interface"] = iface

        vcc = input("  工作电压 (V): ").strip()
        if vcc:
            req["voltage"] = float(vcc)

        kw = input("  关键词: ").strip()
        if kw:
            req["keyword"] = kw

        print(f"\n  正在筛选...")
        results = find_sensors(req)
        print_results(results)
    except (ValueError, KeyboardInterrupt):
        print("\n  已取消。")


def list_categories():
    """列出所有传感器类别"""
    cats = {}
    for s in SENSOR_DATABASE:
        cat = s["类别"]
        if cat not in cats:
            cats[cat] = 0
        cats[cat] += 1

    print(f"\n{'='*60}")
    print(f"  📦 可选传感器类别（共{len(cats)}类，{len(SENSOR_DATABASE)}款）")
    print(f"{'='*60}")
    for cat, cnt in sorted(cats.items()):
        print(f"  • {cat:20s}  共 {cnt} 款")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="传感器选型工具 - 根据应用场景推荐传感器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例:
  python sensor_selector.py --category 温度 --precision high
  python sensor_selector.py --category IMU --interface I2C
  python sensor_selector.py --list-categories
  python sensor_selector.py --interactive
"""
    )

    parser.add_argument("--category", help="传感器类别 (如: 温度, IMU, 测距, 光照, 气体)")
    parser.add_argument("--precision", choices=["low", "medium", "high"], help="精度等级")
    parser.add_argument("--interface", help="接口类型 (I2C/SPI/UART/模拟)")
    parser.add_argument("--voltage", type=float, help="工作电压 (V)")
    parser.add_argument("--keyword", help="关键词匹配")
    parser.add_argument("--top", type=int, default=5, help="显示前N个结果")
    parser.add_argument("--list-categories", action="store_true", help="列出所有传感器类别")
    parser.add_argument("--interactive", action="store_true", help="交互式选型")
    parser.add_argument("--json", action="store_true", help="输出JSON格式")

    args = parser.parse_args()

    if args.list_categories:
        list_categories()
        return

    if args.interactive:
        interactive_mode()
        return

    req = {}
    if args.category: req["category"] = args.category
    if args.precision: req["precision"] = args.precision
    if args.interface: req["interface"] = args.interface
    if args.voltage: req["voltage"] = args.voltage
    if args.keyword: req["keyword"] = args.keyword

    if not req:
        parser.print_help()
        print("\n💡 提示: 请至少指定一个条件，或使用 --interactive")
        return

    results = find_sensors(req)
    if args.json:
        print(json.dumps(results[:args.top], ensure_ascii=False, indent=2))
    else:
        print_results(results, top_n=args.top)


if __name__ == "__main__":
    main()
