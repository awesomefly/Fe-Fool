import time

import numpy as np
from windows.window_robot import NINE_POINT, ENGINE_NUM
from robot.tools import FunctionFitter, TrilinearCalibrator
from robot import SERVER_ADDR, ROOT, LOG
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == "__main__":
    engine_real = [
        [-103, 144, 20],
        [-5, 146, 25],
        [94, 149, 22],
        [94, 196, 16],
        [-8, 196, 18],
        [-103, 190, 16],
        [-103, 215, 11],
        [-6, 218, 15],
        [92, 218, 12],
        [-92, 133, -90],
        [-3, 131, -87],
        [88, 136, -90],
        [87, 186, -90],
        [-3, 179, -91],
        [-96, 182, -92],
        [-97, 204, -92],
        [-5, 206, -91],
        [88, 207, -92],
    ]

    # loaded_fit0 = FunctionFitter.load(ROOT + "/calibration0.pickle")
    # loaded_fit1 = FunctionFitter.load(ROOT + "/calibration1.pickle")
    # loaded_fit2 = FunctionFitter.load(ROOT + "/calibration2.pickle")

    # err0 = loaded_fit0.y
    # err1 = loaded_fit1.y
    # err2 = loaded_fit2.y

    # X = np.array([it[0] for it in NINE_POINT] + [-120, 120])
    # Y = np.array([it[1] for it in NINE_POINT] + [120, 240])
    # Z = np.array([it[2] for it in NINE_POINT] + [20, 40])

    # engine_real0 = (err0 + X).tolist()
    # engine_real1 = (err1 + Y).tolist()
    # engine_real2 = (err2 + Z).tolist()

    # real = []
    # for i in range(len(NINE_POINT)):
    #     real.append([engine_real0[i], engine_real1[i], engine_real2[i]])

    # 1. 校准阶段：收集数据
    target_coords = NINE_POINT  # 期望到达的位置
    actual_coords = engine_real  # 校准补偿后的位置
    print(target_coords)
    print(actual_coords)

    # 2. 训练校准模型
    calibrator = TrilinearCalibrator(grid_resolution=10)
    stats = calibrator.train(target_coords, actual_coords)

    # 3. 实际使用：需要到达某个目标位置
    target = [100, 200, 300]
    calibrated_target = calibrator.calibrate(target)

    calibrator.save_model(ROOT + "/calibration_rbf_model.pickle")

    # fit0 = FunctionFitter(X, loaded_fit0.y)
    # fit1 = FunctionFitter(Y, loaded_fit1.y)
    # fit2 = FunctionFitter(Z, loaded_fit2.y)

    # # fit0.plot("X")
    # # fit1.plot("Y")
    # # fit2.plot("Z")

    # loaded_fit0.plot()
    # loaded_fit1.plot()
    # loaded_fit2.plot()

    # # 使用示例
    # if __name__ == "__main__":
    #     # 生成模拟测量数据
    #     np.random.seed(42)
    #     n_samples = 100

    #     # 目标坐标（在工作空间中随机分布）
    #     target_coords = np.random.uniform(-500, 500, (n_samples, 3))

    #     # 模拟实际坐标（带有空间变化的系统误差）
    #     errors = np.column_stack(
    #         [
    #             0.01 * target_coords[:, 0]
    #             + 0.005 * target_coords[:, 1]
    #             + 0.00001 * target_coords[:, 0] ** 2
    #             + 2,
    #             0.008 * target_coords[:, 1]
    #             + 0.003 * target_coords[:, 2]
    #             + 0.00002 * target_coords[:, 1] * target_coords[:, 2]
    #             - 1,
    #             0.012 * target_coords[:, 2]
    #             + 0.002 * target_coords[:, 0]
    #             + 0.00001 * target_coords[:, 2] ** 2
    #             + 1.5,
    #         ]
    #     )
    #     errors += np.random.normal(0, 0.3, (n_samples, 3))  # 测量噪声
    #     actual_coords = target_coords + errors

    #     # 创建校准器
    #     calibrator = TrilinearCalibration(grid_resolution=8)

    #     # 校准（可选 'rbf' 或 'idw' 方法）
    #     stats = calibrator.train(target_coords, actual_coords, method="rbf")

    #     print("\n" + "=" * 60)
    #     print("三线性插值校准结果:")
    #     print("=" * 60)
    #     print(f"网格分辨率: {stats['grid_size']}")
    #     print(f"均方根误差 (RMSE): {stats['rmse']:.4f} mm")
    #     print(f"最大残余误差: {stats['max_error']:.4f} mm")
    #     print(f"平均误差 [x, y, z]: {stats['mean_error']}")
    #     print(f"误差标准差 [x, y, z]: {stats['std_error']}")
    #     print(f"\n工作空间范围:")
    #     print(
    #         f"  X: [{stats['workspace_bounds']['x_min']:.1f}, {stats['workspace_bounds']['x_max']:.1f}]"
    #     )
    #     print(
    #         f"  Y: [{stats['workspace_bounds']['y_min']:.1f}, {stats['workspace_bounds']['y_max']:.1f}]"
    #     )
    #     print(
    #         f"  Z: [{stats['workspace_bounds']['z_min']:.1f}, {stats['workspace_bounds']['z_max']:.1f}]"
    #     )

    # 测试补偿
    print("\n" + "=" * 60)
    print("补偿测试:")
    print("=" * 60)
    test_targets = np.array([[100, 200, 300], [-200, 150, -100], [0, 0, 0]])

    for i, target in enumerate(test_targets):
        compensated = calibrator.calibrate(target)
        predicted_error = calibrator.predict(target)
        print(f"\n测试点 {i+1}:")
        print(f"  目标坐标: {target}")
        print(f"  预测误差: {predicted_error}")
        print(f"  补偿后坐标: {compensated}")
        print(f"  补偿量: {target - compensated}")

    #     # 保存模型
    #     calibrator.save_model("robot_calibration.npz")

    actual_coords = np.array(actual_coords)
    target_coords = np.array(target_coords)

    # 可视化
    fig = plt.figure(figsize=(18, 5))

    # 原始误差分布
    ax1 = fig.add_subplot(131, projection="3d")
    original_errors = actual_coords - target_coords
    error_norms = np.linalg.norm(original_errors, axis=1)
    scatter1 = ax1.scatter(
        target_coords[:, 0],
        target_coords[:, 1],
        target_coords[:, 2],
        c=error_norms,
        cmap="Reds",
        s=30,
    )
    ax1.grid(True, linestyle="--", alpha=0.7, color="gray")  # 开启网格，设置样式
    _ = [
        ax1.text(x, y, z, f"{x:.0f},{y:.0f},{z:.0f},{e:.2f}mm", fontsize=8)
        for (x, y, z), e in zip(target_coords, error_norms)
    ]  # 为每个点添加误差标签

    ax1.set_xlabel("X (mm)")
    ax1.set_ylabel("Y (mm)")
    ax1.set_zlabel("Z (mm)")
    ax1.set_title("原始误差分布")
    plt.colorbar(scatter1, ax=ax1, label="误差大小 (mm)", shrink=0.6)

    # 插值预测误差
    ax2 = fig.add_subplot(132, projection="3d")
    predicted_errors = calibrator.predict(target_coords)
    error_norms = np.linalg.norm(predicted_errors, axis=1)
    scatter2 = ax2.scatter(
        target_coords[:, 0],
        target_coords[:, 1],
        target_coords[:, 2],
        c=error_norms,
        cmap="Blues",
        s=30,
    )
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Y (mm)")
    ax2.set_zlabel("Z (mm)")
    ax2.set_title("插值预测误差")

    _ = [
        ax2.text(x, y, z, f"{x:.0f},{y:.0f},{z:.0f},{e:.2f}mm", fontsize=8)
        for (x, y, z), e in zip(target_coords, error_norms)
    ]  # 为每个点添加误差标签
    plt.colorbar(scatter2, ax=ax2, label="误差大小 (mm)", shrink=0.6)

    # 校准后残余误差
    ax3 = fig.add_subplot(133, projection="3d")
    residual_errors = original_errors - predicted_errors
    error_norms = np.linalg.norm(residual_errors, axis=1)
    scatter3 = ax3.scatter(
        target_coords[:, 0],
        target_coords[:, 1],
        target_coords[:, 2],
        c=error_norms,
        cmap="Greens",
        s=30,
    )
    ax3.set_xlabel("X (mm)")
    ax3.set_ylabel("Y (mm)")
    ax3.set_zlabel("Z (mm)")
    ax3.set_title("校准后残余误差")

    _ = [
        ax3.text(x, y, z, f"{x:.0f},{y:.0f},{z:.0f},{e:.2f}mm", fontsize=8)
        for (x, y, z), e in zip(target_coords, error_norms)
    ]  # 为每个点添加误差标签
    plt.colorbar(scatter3, ax=ax3, label="误差大小 (mm)", shrink=0.6)

    plt.rcParams["font.sans-serif"] = ["Arial Unicode MS", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False
    plt.tight_layout()
    # plt.savefig("trilinear_calibration.png", dpi=150, bbox_inches="tight")
    # print(f"\n可视化结果已保存为 'trilinear_calibration.png'")
    plt.show()

    time.sleep(100)
