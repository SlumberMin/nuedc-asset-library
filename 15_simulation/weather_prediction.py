#!/usr/bin/env python3
"""
气象预测仿真
功能：气压趋势分析、温度预测、简单气象模型（晴/雨/风速）
适用：电赛气象站/环境监测类题目
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ============== 气象模型参数 ==============
LATITUDE = 30.0         # 纬度 (°N)
ELEVATION = 50.0        # 海拔 (m)
SEASON = 'spring'       # 季节

# 物理常数
G = 9.80665             # 重力加速度 m/s²
R_DRY = 287.05          # 干空气气体常数 J/(kg·K)
LAPSE_RATE = 6.5        # 温度直减率 °C/km
STD_PRESSURE = 1013.25  # 标准大气压 hPa


def pressure_altitude(p_hpa, t_c=15.0):
    """由气压计算海拔 (m)"""
    return 44330 * (1 - (p_hpa / STD_PRESSURE) ** 0.1903)


def altitude_pressure(alt_m, t_c=15.0):
    """由海拔计算气压 (hPa)"""
    return STD_PRESSURE * (1 - alt_m / 44330) ** (1 / 0.1903)


def pressure_to_sealevel(p_hpa, alt_m, t_c=15.0):
    """气压归一到海平面"""
    t_k = t_c + 273.15
    return p_hpa * np.exp(G * alt_m / (R_DRY * t_k))


def dew_point(t_c, rh_percent):
    """由温度和相对湿度计算露点温度 (°C)"""
    a, b = 17.27, 237.7
    gamma = (a * t_c) / (b + t_c) + np.log(rh_percent / 100.0 + 1e-10)
    return (b * gamma) / (a - gamma)


def heat_index(t_c, rh_percent):
    """体感温度/热指数 (°C)"""
    t_f = t_c * 9 / 5 + 32  # 转华氏
    if t_f < 80:
        return t_c
    hi_f = (-42.379 + 2.04901523 * t_f + 10.14333127 * rh_percent
            - 0.22475541 * t_f * rh_percent - 6.83783e-3 * t_f ** 2
            - 5.481717e-2 * rh_percent ** 2 + 1.22874e-3 * t_f ** 2 * rh_percent
            + 8.5282e-4 * t_f * rh_percent ** 2 - 1.99e-6 * t_f ** 2 * rh_percent ** 2)
    return (hi_f - 32) * 5 / 9


def wind_chill(t_c, wind_ms):
    """风寒指数 (°C)"""
    wind_kph = wind_ms * 3.6
    if t_c > 10 or wind_kph < 4.8:
        return t_c
    wc = (13.12 + 0.6215 * t_c
          - 11.37 * wind_kph ** 0.16
          + 0.3965 * t_c * wind_kph ** 0.16)
    return wc


def generate_weather_data(n_days=7, dt_hours=1):
    """生成仿真气象数据（含日变化+天气系统）"""
    n_points = n_days * 24 // dt_hours
    t = np.arange(n_points) * dt_hours / 24  # 天数

    np.random.seed(42)

    # ---- 基础温度 (日变化 + 季节基线) ----
    season_base = {'spring': 18, 'summer': 30, 'autumn': 15, 'winter': 5}
    t_base = season_base.get(SEASON, 18)
    hour_of_day = (t * 24) % 24
    temp_daily = 6 * np.sin(2 * np.pi * (hour_of_day - 6) / 24)  # 最高温~14时
    temp_noise = 1.5 * np.random.randn(n_points)
    temperature = t_base + temp_daily + temp_noise

    # ---- 气压 (天气系统 + 日变化) ----
    # 天气系统：低压槽过境 (4天周期)
    pressure_system = 8 * np.sin(2 * np.pi * t / 4)
    # 日微变化
    pressure_daily = 1.5 * np.sin(2 * np.pi * hour_of_day / 24)
    pressure_noise = 0.8 * np.random.randn(n_points)
    pressure = STD_PRESSURE + pressure_system + pressure_daily + pressure_noise
    # 海拔修正
    p_station = altitude_pressure(ELEVATION, np.mean(temperature)) + \
                (pressure - STD_PRESSURE)

    # ---- 湿度 ----
    rh_base = 60 + 20 * np.sin(2 * np.pi * t / 4 + np.pi)  # 与气压反相
    rh_daily = -10 * np.sin(2 * np.pi * (hour_of_day - 6) / 24)  # 白天干
    rh_noise = 5 * np.random.randn(n_points)
    humidity = np.clip(rh_base + rh_daily + rh_noise, 10, 100)

    # ---- 风速 ----
    # 气压梯度 → 风速
    dp = np.gradient(pressure)
    wind_base = np.abs(dp) * 3
    wind_daily = 1.5 * np.sin(2 * np.pi * (hour_of_day - 12) / 24)
    wind_speed = np.clip(wind_base + wind_daily + 2 + 0.5 * np.random.randn(n_points), 0, 25)

    # ---- 天气状况判定 ----
    weather = []
    for i in range(n_points):
        p_trend = dp[i]  # 气压变化趋势
        rh = humidity[i]
        t_d = dew_point(temperature[i], rh)
        spread = temperature[i] - t_d

        if p_trend < -2 and rh > 80:
            weather.append('rain')
        elif p_trend < -1 and rh > 70:
            weather.append('cloudy')
        elif rh > 90 and temperature[i] < 2:
            weather.append('snow')
        elif spread < 3 and hour_of_day[i] >= 20:
            weather.append('fog')
        elif wind_speed[i] > 10:
            weather.append('windy')
        else:
            weather.append('clear')

    return {
        't': t, 'temperature': temperature, 'pressure': p_station,
        'humidity': humidity, 'wind_speed': wind_speed,
        'weather': weather, 'hour_of_day': hour_of_day,
        'dew_point': dew_point(temperature, humidity),
    }


def simple_forecast(pressure, window_hours=6):
    """简单气压趋势预测"""
    n = len(pressure)
    forecast = []
    for i in range(n):
        if i < window_hours:
            forecast.append('stable')
            continue
        dp = pressure[i] - pressure[i - window_hours]
        if dp < -3:
            forecast.append('falling_fast')
        elif dp < -1:
            forecast.append('falling')
        elif dp > 3:
            forecast.append('rising_fast')
        elif dp > 1:
            forecast.append('rising')
        else:
            forecast.append('stable')
    return forecast


def predict_next_hours(data, n_predict=24):
    """基于趋势外推的简单预测"""
    temp = data['temperature']
    pres = data['pressure']

    # 线性外推（最近6小时斜率）
    window = 6
    t_slope = (temp[-1] - temp[-window]) / window
    p_slope = (pres[-1] - pres[-window]) / window

    t_future = np.array([temp[-1] + t_slope * i for i in range(1, n_predict + 1)])
    p_future = np.array([pres[-1] + p_slope * i for i in range(1, n_predict + 1)])

    return t_future, p_future


def run_simulation():
    print("=" * 60)
    print("气象预测仿真系统")
    print("=" * 60)
    print(f"位置: 北纬{LATITUDE}° | 海拔: {ELEVATION}m | 季节: {SEASON}")

    # ---- 生成气象数据 ----
    data = generate_weather_data(n_days=7)
    forecasts = simple_forecast(data['pressure'])

    # ---- 统计 ----
    print(f"\n--- 7天气象统计 ---")
    print(f"温度: {np.min(data['temperature']):.1f}°C ~ {np.max(data['temperature']):.1f}°C "
          f"(均值 {np.mean(data['temperature']):.1f}°C)")
    print(f"气压: {np.min(data['pressure']):.1f} ~ {np.max(data['pressure']):.1f} hPa")
    print(f"湿度: {np.min(data['humidity']):.1f}% ~ {np.max(data['humidity']):.1f}%")
    print(f"风速: {np.min(data['wind_speed']):.1f} ~ {np.max(data['wind_speed']):.1f} m/s")

    # 天气统计
    from collections import Counter
    weather_count = Counter(data['weather'])
    print(f"\n天气分布:")
    for w, c in weather_count.most_common():
        print(f"  {w:8s}: {c:4d} 次 ({c/len(data['weather'])*100:.1f}%)")

    # ---- 预测 ----
    t_future, p_future = predict_next_hours(data, 24)
    print(f"\n--- 24小时预测 ---")
    print(f"温度: {data['temperature'][-1]:.1f}°C → {t_future[-1]:.1f}°C")
    print(f"气压: {data['pressure'][-1]:.1f} → {p_future[-1]:.1f} hPa")
    if p_future[-1] < data['pressure'][-1] - 2:
        print("⚠ 气压下降趋势，可能有降水")
    elif p_future[-1] > data['pressure'][-1] + 2:
        print("✓ 气压上升趋势，天气转好")

    # ---- 绘图 ----
    fig = plt.figure(figsize=(18, 18))
    fig.suptitle("气象预测仿真系统", fontsize=16, fontweight='bold')
    gs = GridSpec(5, 2, figure=fig, hspace=0.4, wspace=0.3)

    t_days = data['t']

    # (0,0) 温度
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(t_days, data['temperature'], 'r-', linewidth=0.8, alpha=0.7)
    # 平滑
    w = min(12, len(t_days) // 20)
    if w > 1:
        smooth = np.convolve(data['temperature'], np.ones(w)/w, mode='same')
        ax.plot(t_days, smooth, 'r-', linewidth=2, label='平滑')
    # 预测
    t_pred = np.arange(t_days[-1], t_days[-1] + 24/24, 1/24)[:len(t_future)]
    ax.plot(t_pred, t_future, 'r--', linewidth=2, alpha=0.5, label='预测')
    ax.axvline(x=t_days[-1], color='gray', linestyle=':', alpha=0.5)
    ax.set_title('温度变化')
    ax.set_xlabel('天数')
    ax.set_ylabel('温度 (°C)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # (0,1) 气压
    ax = fig.add_subplot(gs[0, 1])
    ax.plot(t_days, data['pressure'], 'b-', linewidth=0.8, alpha=0.7)
    if w > 1:
        smooth = np.convolve(data['pressure'], np.ones(w)/w, mode='same')
        ax.plot(t_days, smooth, 'b-', linewidth=2, label='平滑')
    ax.plot(t_pred, p_future, 'b--', linewidth=2, alpha=0.5, label='预测')
    ax.axvline(x=t_days[-1], color='gray', linestyle=':', alpha=0.5)
    ax.set_title('气压变化')
    ax.set_xlabel('天数')
    ax.set_ylabel('气压 (hPa)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # (1,0) 湿度 + 露点
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(t_days, data['humidity'], 'g-', linewidth=0.8, alpha=0.5, label='相对湿度')
    ax2 = ax.twinx()
    ax2.plot(t_days, data['dew_point'], 'purple', linewidth=0.8, alpha=0.7, label='露点温度')
    ax.set_title('湿度与露点')
    ax.set_xlabel('天数')
    ax.set_ylabel('相对湿度 (%)', color='g')
    ax2.set_ylabel('露点温度 (°C)', color='purple')
    ax.legend(loc='upper left', fontsize=7)
    ax2.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3)

    # (1,1) 风速
    ax = fig.add_subplot(gs[1, 1])
    ax.fill_between(t_days, 0, data['wind_speed'], alpha=0.3, color='steelblue')
    ax.plot(t_days, data['wind_speed'], 'steelblue', linewidth=0.8)
    ax.axhline(y=10, color='orange', linestyle='--', alpha=0.5, label='大风阈值')
    ax.set_title('风速')
    ax.set_xlabel('天数')
    ax.set_ylabel('风速 (m/s)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # (2,0) 天气状况时序图
    ax = fig.add_subplot(gs[2, 0])
    weather_map = {'clear': 0, 'cloudy': 1, 'windy': 2, 'fog': 3, 'rain': 4, 'snow': 5}
    weather_colors = {'clear': '#FFD700', 'cloudy': '#A9A9A9', 'windy': '#87CEEB',
                      'fog': '#D3D3D3', 'rain': '#4169E1', 'snow': '#FFFFFF'}
    weather_num = [weather_map.get(w, 0) for w in data['weather']]
    for i in range(len(t_days) - 1):
        w = data['weather'][i]
        ax.axvspan(t_days[i], t_days[i+1], alpha=0.6, color=weather_colors.get(w, 'gray'))
    ax.set_yticks(list(weather_map.values()))
    ax.set_yticklabels(list(weather_map.keys()))
    ax.set_title('天气状况')
    ax.set_xlabel('天数')
    ax.grid(True, alpha=0.3, axis='x')

    # (2,1) 气压趋势预测
    ax = fig.add_subplot(gs[2, 1])
    forecast_map = {'falling_fast': -2, 'falling': -1, 'stable': 0,
                    'rising': 1, 'rising_fast': 2}
    forecast_num = [forecast_map.get(f, 0) for f in forecasts]
    colors = ['red' if f < 0 else 'green' if f > 0 else 'gray' for f in forecast_num]
    ax.bar(t_days, forecast_num, width=1/24, color=colors, alpha=0.6)
    ax.set_title('气压趋势预测')
    ax.set_xlabel('天数')
    ax.set_yticks(list(forecast_map.values()))
    ax.set_yticklabels(list(forecast_map.keys()), fontsize=8)
    ax.grid(True, alpha=0.3)

    # (3,0) 温度-湿度散点图
    ax = fig.add_subplot(gs[3, 0])
    scatter = ax.scatter(data['temperature'], data['humidity'],
                         c=data['wind_speed'], cmap='coolwarm', s=5, alpha=0.5)
    plt.colorbar(scatter, ax=ax, label='风速 (m/s)')
    ax.set_title('温度-湿度关系')
    ax.set_xlabel('温度 (°C)')
    ax.set_ylabel('相对湿度 (%)')
    ax.grid(True, alpha=0.3)

    # (3,1) 体感温度
    ax = fig.add_subplot(gs[3, 1])
    hi = np.array([heat_index(t, rh) for t, rh in
                   zip(data['temperature'], data['humidity'])])
    wc = np.array([wind_chill(t, w) for t, w in
                    zip(data['temperature'], data['wind_speed'])])
    ax.plot(t_days, data['temperature'], 'gray', linewidth=0.5, alpha=0.5, label='实际温度')
    ax.plot(t_days, hi, 'r-', linewidth=0.8, alpha=0.7, label='热指数')
    ax.plot(t_days, wc, 'b-', linewidth=0.8, alpha=0.7, label='风寒指数')
    ax.set_title('体感温度')
    ax.set_xlabel('天数')
    ax.set_ylabel('温度 (°C)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # (4,0) 气压与降水量关系
    ax = fig.add_subplot(gs[4, 0])
    rain_hours = [i for i, w in enumerate(data['weather']) if w in ('rain', 'snow')]
    rain_amount = np.zeros(len(t_days))
    for i in rain_hours:
        rain_amount[i] = np.random.exponential(2)  # 简单降水模型
    ax.bar(t_days, rain_amount, width=1/24, color='dodgerblue', alpha=0.6, label='降水')
    ax2 = ax.twinx()
    ax2.plot(t_days, data['pressure'], 'b-', linewidth=1, alpha=0.5, label='气压')
    ax.set_title('气压与降水关系')
    ax.set_xlabel('天数')
    ax.set_ylabel('降水量 (mm/h)', color='dodgerblue')
    ax2.set_ylabel('气压 (hPa)', color='blue')
    ax.legend(loc='upper left', fontsize=7)
    ax2.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3)

    # (4,1) 传感器误差对预测的影响
    ax = fig.add_subplot(gs[4, 1])
    sensor_errors = [0.1, 0.5, 1.0, 2.0, 5.0]  # hPa
    pred_errors = []
    for err in sensor_errors:
        p_noisy = data['pressure'] + np.random.randn(len(data['pressure'])) * err
        _, p_pred = predict_next_hours({'temperature': data['temperature'],
                                         'pressure': p_noisy}, 24)
        _, p_clean = predict_next_hours({'temperature': data['temperature'],
                                          'pressure': data['pressure']}, 24)
        pred_errors.append(np.std(p_pred - p_clean))
    ax.plot(sensor_errors, pred_errors, 'ro-', markersize=8)
    ax.set_title('传感器精度 vs 预测误差')
    ax.set_xlabel('气压传感器误差 (hPa)')
    ax.set_ylabel('24h预测偏差 (hPa)')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('weather_prediction.png', dpi=150, bbox_inches='tight')
    print("\n图像已保存: weather_prediction.png")

    # ---- 气象站硬件参考 ----
    print("\n--- 气象站传感器推荐 ---")
    print("温度/湿度: SHT30 (±0.3°C, ±2%RH) 或 DHT22 (±0.5°C, ±5%RH)")
    print("气压: BMP280 (±1hPa) 或 BME280 (温度+湿度+气压一体)")
    print("风速: 脉冲计数式风速计 (霍尔传感器)")
    print("雨量: 翻斗式雨量计 (脉冲计数)")
    print("通信: RS485 / LoRa / WiFi (ESP8266)")

    plt.show()


if __name__ == '__main__':
    run_simulation()
