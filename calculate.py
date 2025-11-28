import numpy as np

# ---------------------------------------------------------
# 1. 在这里填入你的数据（建议 6~12 个点）
#    x_data:  ADC raw 数据（来自 ADC 读数）
#    y_data:  实际电压（用万用表量的）
# ---------------------------------------------------------
x_data = np.array([
    0, 559, 1141, 1743, 2339, 2964, 3749
], dtype=float)

y_data = np.array([
    0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0
], dtype=float)
# x_data = np.array([
#     50, 80, 100, 120, 150, 180, 240
# ], dtype=float)

# y_data = np.array([
#     90, 110, 125, 130, 160, 190, 240
# ], dtype=float)
# ---------------------------------------------------------
# 2. 定义一个函数，计算 RMS 误差
# ---------------------------------------------------------
def rms_error(y_true, y_pred):
    return np.sqrt(np.mean((y_true - y_pred)**2))


# ---------------------------------------------------------
# 3. 一次、二次、三次多项式拟合
# ---------------------------------------------------------
print("\n=== Polynomial Fit Results ===\n")

# 线性拟合 (degree = 1)
p1 = np.polyfit(x_data, y_data, 1)
y1 = np.polyval(p1, x_data)
rms1 = rms_error(y_data, y1)
print("Linear fit (degree 1):")
print("  V = a*x + b")
print("  a = {:.10f}, b = {:.10f}".format(p1[0], p1[1]))
print("  RMS error = {:.10f} V\n".format(rms1))

# 二次拟合 (degree = 2)
p2 = np.polyfit(x_data, y_data, 2)
y2 = np.polyval(p2, x_data)
rms2 = rms_error(y_data, y2)
print("Quadratic fit (degree 2):")
print("  V = a2*x^2 + a1*x + a0")
print("  a2 = {:.10e}, a1 = {:.10e}, a0 = {:.10e}".format(p2[0], p2[1], p2[2]))
print("  RMS error = {:.10f} V\n".format(rms2))

# 三次拟合 (degree = 3) —— 如不需要可注释掉
p3 = np.polyfit(x_data, y_data, 3)
y3 = np.polyval(p3, x_data)
rms3 = rms_error(y_data, y3)
print("Cubic fit (degree 3):")
print("  V = a3*x^3 + a2*x^2 + a1*x + a0")
print("  a3 = {:.10e}, a2 = {:.10e}, a1 = {:.10e}, a0 = {:.10e}".format(
    p3[0], p3[1], p3[2], p3[3]))
print("  RMS error = {:.10f} V\n".format(rms3))



# ---------------------------------------------------------
# 4. 输出可用于 C 代码的格式
# ---------------------------------------------------------
print("=== C code coefficients ===\n")

print("// Linear:")
print("float a1 = {:.10f};".format(p1[0]))
print("float b1 = {:.10f};\n".format(p1[1]))

print("// Quadratic:")
print("float a2 = {:.10e};".format(p2[0]))
print("float b2 = {:.10e};".format(p2[1]))
print("float c2 = {:.10e};\n".format(p2[2]))

print("// Cubic:")
print("float a3 = {:.10e};".format(p3[0]))
print("float b3 = {:.10e};".format(p3[1]))
print("float c3 = {:.10e};".format(p3[2]))
print("float d3 = {:.10e};".format(p3[3]))

print("\nDone.")