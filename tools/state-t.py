import matplotlib.pyplot as plt
import re

file_path = 'tools/data.txt'
num_data = 1

def parse_line(line):
    """安全地解析一行文本为浮点数"""
    try:
        return float(line.strip())
    except (ValueError, TypeError):
        return None


def extract_valid_data_blocks(file_path):
    """
    从文件中提取所有有效的数据块。
    每个数据块由一行时间戳 + <MSG>: 行 和其后 15 行数值组成。
    返回: [(time, [val1, val2, ..., val15]), ...]
    """
    data_blocks = []

    with open(file_path, 'r', encoding='utf-8') as f:
        lines = [line.strip() for line in f if line.strip()]  # 去除空行

    i = 0
    n = len(lines)

    while i < n:
        line = lines[i]

        # 匹配时间戳格式：xx:xx:xxx: <MSG>:
        time_match = re.match(r'(\d{2}):(\d{2}):(\d{3}): <MSG>:', line)
        if time_match:
            try:
                hh, mm, ms = map(int, time_match.groups())
                total_time = hh * 60 + mm + ms * 0.001

                # 检查后面是否有至少15个数据行
                if i + num_data >= n:
                    print("Reached end of file, incomplete block.")
                    break

                # 解析接下来的15个数据
                values = []
                for j in range(1, num_data + 1):
                    val = parse_line(lines[i + j])
                    if val is None:
                        raise ValueError(f"Invalid data at line {i + j}: '{lines[i + j]}'")
                    values.append(val)

                # 成功解析时间 + 15个数据
                data_blocks.append((total_time, values))
                i += num_data + 1  # 跳过当前块

            except Exception as e:
                print(f"Skipping invalid block starting at line {i}: {e}")
                i += 1  # 小步前进避免死循环
        else:
            i += 1  # 不匹配就继续找

    return data_blocks


def plot_data(file_path):
    """
    主读取函数：调用 extract_valid_data_blocks 提取数据并按变量返回
    """
    data_blocks = extract_valid_data_blocks(file_path)

    times = [t for t, _ in data_blocks]
    values_list = [v for _, v in data_blocks]

    plt.figure(figsize=(14, 8))

    for i in range(num_data):
        datas = [d[i] for d in values_list]
        plt.plot(times, datas, label=f'data{i}')

    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('Data Variation Over Time')
    plt.legend()
    plt.xticks(rotation=45)  # 旋转x轴标签以提高可读性
    plt.tight_layout()  # 自动调整子图参数,使之填充整个图像区域
    plt.show()

plot_data(file_path)
