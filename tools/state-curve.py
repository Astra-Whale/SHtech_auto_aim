import matplotlib.pyplot as plt
import re

# 配置变量
config = {
    'file_path': 'tools/data.txt',
    'data_per_curve': 100,   # 每条曲线的数据点数
    'num_curves': 2,         # 曲线数量
    'curve_group': 22,        # 选择第几组数据（从0开始），一组包含2条曲线（共200行）
}


def parse_line(line):
    """安全地解析一行文本为浮点数"""
    try:
        return float(line.strip())
    except (ValueError, TypeError):
        return None


def extract_curves(file_path, data_per_curve, num_curves, curve_group):
    """
    从文件中提取指定组的多条曲线。
    文件可能包含多个时间戳块，每个块内包含 num_curves 条曲线。
    第一行为时间戳，接下来每 data_per_curve 行为一条完整的曲线。
    跳过数据格式不符合的行。
    返回: (timestamp, [[curve1_data], [curve2_data], ...])
    """
    curves = []
    
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = [line.strip() for line in f if line.strip()]
    
    if not lines:
        print("文件为空")
        return None, []
    
    # 扫描所有时间戳位置
    timestamp_positions = []
    for i, line in enumerate(lines):
        time_match = re.match(r'(\d{2}):(\d{2}):(\d{3}): <MSG>:', line)
        if time_match:
            timestamp_positions.append((i, line))
    
    if not timestamp_positions:
        print("未找到时间戳")
        return None, []
    
    print(f"找到 {len(timestamp_positions)} 个时间戳块")
    
    # 检查是否有足够的块
    if curve_group >= len(timestamp_positions):
        print(f"错误: 只有 {len(timestamp_positions)} 个数据组 (0-{len(timestamp_positions)-1}), 无法访问第 {curve_group} 组")
        return None, []
    
    # 获取所选块的时间戳和起始位置
    start_idx, timestamp_line = timestamp_positions[curve_group]
    time_match = re.match(r'(\d{2}):(\d{2}):(\d{3}): <MSG>:', timestamp_line)
    hh, mm, ms = map(int, time_match.groups())
    timestamp = f"{hh}:{mm}:{ms}"
    print(f"选择第 {curve_group} 组数据，采集时间: {timestamp}")
    
    # 从时间戳的下一行开始提取曲线
    line_idx = start_idx + 1
    n = len(lines)
    
    for curve_idx in range(num_curves):
        curve_data = []
        
        # 逐行读取，直到收集够 data_per_curve 个有效数据
        while line_idx < n and len(curve_data) < data_per_curve:
            # 检查是否遇到下一个时间戳（表示新的数据组开始）
            if re.match(r'(\d{2}):(\d{2}):(\d{3}): <MSG>:', lines[line_idx]):
                print(f"警告: 遇到下一个时间戳块，未完整收集曲线 {curve_idx + 1}")
                break
            
            val = parse_line(lines[line_idx])
            if val is not None:
                curve_data.append(val)
            else:
                # 跳过格式不符的行
                pass
            
            line_idx += 1
        
        # 检查是否成功收集了完整的曲线
        if len(curve_data) == data_per_curve:
            curves.append(curve_data)
            print(f"成功提取第 {curve_idx + 1} 条曲线 ({data_per_curve} 个数据点)")
        else:
            print(f"警告: 数据不足，只提取了 {len(curve_data)} 个数据点，需要 {data_per_curve} 个")
            if curve_data:
                curves.append(curve_data)
            break
    
    return timestamp, curves


def plot_curves(file_path, data_per_curve, num_curves, curve_group):
    """
    读取指定组的数据并绘制多条曲线
    """
    timestamp, curves = extract_curves(file_path, data_per_curve, num_curves, curve_group)
    
    if not curves:
        print("没有成功提取到任何曲线数据")
        return
    
    # 创建图表
    plt.figure(figsize=(14, 8))
    
    # 绘制每条曲线
    for idx, curve_data in enumerate(curves):
        x = range(len(curve_data))
        plt.plot(x, curve_data, label=f'Curve {idx + 1}', marker='o', markersize=3)
    
    plt.xlabel('Data Point Index')
    plt.ylabel('Value')
    if timestamp:
        plt.title(f'Group {curve_group} Curves - Collected at {timestamp}')
    else:
        plt.title(f'Group {curve_group} Curves')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    plot_curves(
        config['file_path'],
        config['data_per_curve'],
        config['num_curves'],
        config['curve_group']
    )
