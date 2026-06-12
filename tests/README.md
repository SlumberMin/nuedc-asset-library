# ✅ tests（单元测试）

## 模块概述

本目录包含nuedc-asset-library各模块的单元测试程序，覆盖控制算法、视觉处理、传感器驱动、系统工具等模块。所有测试基于Python实现，支持批量运行和自动化验证。

## 目录结构

```
tests/
├── run_all_tests.py              # 批量运行所有测试
├── test_dashboard.html           # 测试结果可视化面板
│
├── # 控制算法测试
├── test_pid.py
├── test_position_pid.py
├── test_velocity_pid.py
├── test_incremental_pid.py
├── test_advanced_pid.py
├── test_fuzzy_pid.py
├── test_adaptive_pid.py
├── test_pid_auto_tune.py
├── test_pid_scheduled.py
├── test_pid_fusion.py
├── test_adrc.py
├── test_ladrc.py
├── test_kalman.py / test_kalman_filter.py
├── test_ekf.py
├── test_adaptive_kalman.py
├── test_lqr.py
├── test_mpc.py
├── test_sliding_mode.py
├── test_super_twisting_smc.py
├── test_biquad_filter.py
├── test_notch_filter.py
├── test_gain_scheduling_pid.py
├── test_backstepping.py
├── test_luenberger_observer.py
├── test_repetitive_control.py
├── test_h_infinity.py
├── test_optimal_control.py
├── test_feedback_linearization.py
├── test_adaptive_control.py
├── test_neural_pid.py
├── test_lead_lag.py
├── test_deadbeat.py
├── test_smith_predictor.py
├── test_repetitive.py
├── test_anti_saturation.py
├── test_anti_windup.py
├── test_bang_bang.py
├── test_feedforward.py
├── test_cascade_pid.py
├── test_impedance_control.py
├── test_force_control.py
├── test_disturbance_observer.py
├── test_input_shaper.py
├── test_observer_controller.py
├── test_state_feedback.py
├── test_decoupling.py
│
├── # 视觉模块测试
├── test_color_detect.py
├── test_line_detector.py
├── test_qr_code_detector.py
├── test_barcode_detector.py
├── test_polygon_detector.py
├── test_circle_fit.py
├── test_line_intersection.py
├── test_min_area_rect.py
├── test_connected_components.py
├── test_fit_ellipse.py
├── test_convex_hull.py
├── test_shape_matcher.py
├── test_contour_filter.py
├── test_contour_analyzer.py
├── test_edge_detector.py
├── test_adaptive_threshold.py
├── test_color_constancy.py
├── test_color_histogram.py
├── test_color_moment.py
├── test_color_calibration_gui.py
├── test_template_matcher.py
├── test_feature_matcher.py
├── test_feature_vector.py
├── test_hog_feature.py
├── test_lbp_feature.py
├── test_haar_feature.py
├── test_aruco_detector.py
├── test_perspective_transform.py
├── test_background_subtractor.py
├── test_optical_flow.py
├── test_image_enhancer.py
├── test_image_filter.py
├── test_image_morphology.py
├── test_image_histogram.py
├── test_image_segmentation.py
├── test_image_annotation.py
├── test_image_transformation.py
├── test_image_frequency.py
├── test_image_quality.py
├── test_image_watermark.py
├── test_image_compression.py
├── test_image_super_resolution.py
├── test_image_deblurring.py
├── test_image_pyramid.py
├── test_image_registration.py
├── test_image_fusion.py
├── test_multi_roi_detector.py
├── test_motion_detector.py
├── test_obstacle_detector.py
├── test_motion_detection_advanced.py
│
├── # 系统工具测试
├── test_camera.py
├── test_motor.py
├── test_servo.py
├── test_encoder.py
├── test_sensor_ir.py
├── test_math_utils.py
├── test_ring_buffer.py
├── test_state_machine.py
├── test_event_system.py
├── test_watchdog.py
├── test_task_scheduler.py
├── test_moving_average.py
├── test_speed_estimator.py
├── test_foc_simple.py
│
├── # 高级功能测试
├── test_motion_control.py
├── test_path_tracker.py
├── test_path_planning.py
├── test_obstacle_avoidance.py
├── test_trajectory_generator.py
├── test_trajectory_predictor.py
├── test_target_size_estimator.py
├── test_angle_estimator.py
├── test_camera_stabilizer.py
├── test_visual_servo.py
├── test_visual_debugger.py
├── test_tracking_kalman.py
├── test_multi_agent.py
├── test_simulation.py
├── test_performance_benchmark.py
├── test_system_integration.py
├── test_npu_models.py
├── test_auto_exposure.py
├── test_multi_frame_denoise.py
├── test_hdr_capture.py
├── test_fast_capture.py
│
└── __pycache__/                  # Python缓存目录
```

## 测试分类统计

| 类别 | 测试文件数 | 覆盖模块 |
|------|-----------|----------|
| 控制算法 | 45+ | PID/ADRC/Kalman/LQR/MPC/SMC等 |
| 视觉处理 | 50+ | 颜色/形状/特征/图像处理等 |
| 系统工具 | 15+ | 传感器/数据结构/调度器等 |
| 高级功能 | 15+ | 路径规划/视觉伺服/集成测试等 |
| **合计** | **130+** | **全模块覆盖** |

## 使用方法

### 运行单个测试
```bash
python test_pid.py
python test_color_detect.py
```

### 批量运行所有测试
```bash
python run_all_tests.py
```

### 查看测试报告
```bash
# 打开测试结果面板
start test_dashboard.html
```

### 运行特定类别测试
```bash
# 运行所有控制算法测试
python -m pytest test_pid.py test_adrc.py test_kalman.py

# 运行所有视觉测试
python -m pytest test_color_detect.py test_line_detector.py
```

## 依赖要求

```bash
pip install numpy matplotlib scipy opencv-python
```

## FAQ

**Q: 测试失败怎么办？**
A: 检查依赖是否安装完整，确认Python版本（推荐3.10+）。部分测试需要对应的源模块文件。

**Q: 如何添加新测试？**
A: 参照现有测试文件格式，使用 `unittest` 框架编写，文件名以 `test_` 开头。

**Q: __pycache__ 目录是什么？**
A: Python自动生成的字节码缓存目录，可以安全删除，运行测试时会自动重建。

**Q: test_dashboard.html 是什么？**
A: 测试结果的可视化HTML面板，用浏览器打开可查看测试通过率和详细结果。
