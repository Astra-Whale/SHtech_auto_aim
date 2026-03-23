import re
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors

def clean_text(text):
    """
    清洗文本：移除 标签、ANSI 颜色码、时间戳头
    """
    # 1. 移除 引用标签
    # text = re.sub(r'\'', '', text)
    
    # 2. 移除 ANSI 颜色代码
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    text = ansi_escape.sub('', text)
    
    # 3. 移除行首的时间戳 (例如 00:03:971:) 以便后续匹配更干净，但需保留行结构
    # 我们只在parse里做针对性匹配，这里只做基础清洗
    return text.strip()

def parse_log(log_content):
    """
    解析Log文本，支持跨行 Gap 匹配
    """
    frames = []
    current_frame = {}
    
    # 关键正则定义
    re_index = re.compile(r'recording frame index:\s*(\d+)')
    re_start_time = re.compile(r'start time:\s*(\d+)')
    
    # 匹配: "StageName" 123μs
    re_stage_proc = re.compile(r'"(\w+)"\s+(\d+)[μµ]s')
    # 匹配: gap: 456μs
    re_stage_gap = re.compile(r'gap:\s+(\d+)[μµ]s')

    lines = log_content.split('\n')
    
    for line in lines:
        clean = clean_text(line)
        if not clean: continue
        
        # 1. 检测新Frame开始
        idx_match = re_index.search(clean)
        if idx_match:
            if current_frame: 
                frames.append(current_frame)
            current_frame = {
                'id': int(idx_match.group(1)),
                'start_time_us': 0,
                'stages': [] # List of dicts
            }
            continue
            
        if not current_frame: continue

        # 2. 获取 Frame 开始时间
        time_match = re_start_time.search(clean)
        if time_match:
            # Log中是ms，转为us
            current_frame['start_time_us'] = int(time_match.group(1)) * 1000
            continue

        # 3. 获取 Stage 处理时间 (例如: "Sensor" 45714μs)
        proc_match = re_stage_proc.search(clean)
        if proc_match:
            name = proc_match.group(1)
            proc_time = int(proc_match.group(2))
            current_frame['stages'].append({
                'name': name,
                'proc': proc_time,
                'gap': 0 # 默认为0，等待后续更新
            })
            # continue # 移除 continue，以便同一行也能匹配 gap

        # 4. 获取 Gap 时间 (可能在下一行)
        gap_match = re_stage_gap.search(clean)
        if gap_match and current_frame['stages']:
            # 假设 gap 总是紧跟在最近添加的 stage 之后
            gap_time = int(gap_match.group(1))
            current_frame['stages'][-1]['gap'] = gap_time

    if current_frame:
        frames.append(current_frame)
        
    return frames

def filter_frames(frames, start_idx, end_idx):
    if not frames: return []
    targets = []
    for f in frames:
        fid = f['id']
        if start_idx is not None and fid < start_idx: continue
        if end_idx is not None and fid > end_idx: continue
        targets.append(f)
    return targets

def visualize_by_packet(frames, start_idx=None, end_idx=None):
    """
    视图1：以数据包(Frame)为行。
    侧重展示：单个数据包在整个生命周期中的流转延迟。
    """
    target_frames = filter_frames(frames, start_idx, end_idx)
    if not target_frames:
        print("No frames to visualize (Packet View).")
        return

    fig, ax = plt.subplots(figsize=(12, 6))
    
    # 颜色定义
    stage_map = {
        'Entry':   '#1f77b4', 'Sensor':  '#ff7f0e',
        'Detect':  '#2ca02c', 'Predict': '#d62728',
        'Planner': '#9467bd'
    }
    
    global_start = min(f['start_time_us'] for f in target_frames)
    y_ticks, y_labels = [], []

    for i, frame in enumerate(target_frames):
        y_pos = frame['id']
        y_ticks.append(y_pos)
        y_labels.append(f"Frame {frame['id']}")
        
        curr_time = frame['start_time_us'] - global_start
        
        for stage in frame['stages']:
            color = stage_map.get(stage['name'], '#333')
            # Draw Gap
            if stage['gap'] > 0:
                ax.barh(y_pos, stage['gap'], left=curr_time, height=0.6,
                        color='#cccccc', hatch='///', alpha=0.8)
                curr_time += stage['gap']
            # Draw Process
            if stage['proc'] > 0:
                ax.barh(y_pos, stage['proc'], left=curr_time, height=0.6,
                        color=color, alpha=0.9)
                curr_time += stage['proc']

    ax.set_yticks(y_ticks)
    ax.set_yticklabels(y_labels)
    ax.invert_yaxis()
    ax.set_xlabel('Time (us)')
    ax.set_title('View 1: Packet Lifecycle (Row = Frame)')
    
    # Legend
    patches = [mpatches.Patch(color=c, label=n) for n,c in stage_map.items()]
    patches.append(mpatches.Patch(facecolor='#cccccc', hatch='///', label='Gap'))
    ax.legend(handles=patches)
    plt.tight_layout()
    plt.show()

def visualize_by_stage(frames, start_idx=None, end_idx=None):
    """
    视图2：以流水级(Stage)为行。
    侧重展示：各模块的负载情况、是否拥塞。
    颜色含义：不同的颜色代表不同的 Frame ID。
    """
    target_frames = filter_frames(frames, start_idx, end_idx)
    if not target_frames:
        print("No frames to visualize (Stage View).")
        return

    fig, ax = plt.subplots(figsize=(12, 6))
    
    # 提取所有可能的 Stage 名称以保持顺序
    stage_names = []
    if target_frames:
        stage_names = [s['name'] for s in target_frames[0]['stages']]
    
    # 建立 Stage -> Y轴坐标 的映射
    stage_y_map = {name: i for i, name in enumerate(stage_names)}
    
    global_start = min(f['start_time_us'] for f in target_frames)
    
    # 生成颜色池 (用于区分不同的Frame)
    colors = list(mcolors.TABLEAU_COLORS.values())
    
    for frame in target_frames:
        frame_color = colors[frame['id'] % len(colors)] # 循环使用颜色
        
        # 计算该 Frame 中各 Stage 的绝对时间位置
        curr_time = frame['start_time_us'] - global_start
        
        for stage in frame['stages']:
            s_name = stage['name']
            if s_name not in stage_y_map: continue
            y_pos = stage_y_map[s_name]
            
            # 1. 绘制 Process (实心颜色，代表该模块正在处理此Frame)
            if stage['proc'] > 0:
                # 在柱子上加个文字标记 Frame ID，方便辨认
                ax.barh(y_pos, stage['proc'], left=curr_time, height=0.5,
                        color=frame_color, edgecolor='black', alpha=0.8)
                
                # 如果格子够宽，把 Frame ID 写上去
                if stage['proc'] > 2000: # 2ms以上才写字
                    ax.text(curr_time + stage['proc']/2, y_pos, str(frame['id']), 
                            ha='center', va='center', color='white', fontsize=8, fontweight='bold')
                
                curr_time += stage['proc']

            # 2. 处理 Gap (不绘制，仅累加时间，因为Gap是当前Stage结束后的间隔)
            if stage['gap'] > 0:
                curr_time += stage['gap']

    ax.set_yticks(list(range(len(stage_names))))
    ax.set_yticklabels(stage_names)
    ax.invert_yaxis() # Entry 在最上面
    ax.set_xlabel('Time (us)')
    ax.set_title('View 2: Module Utilization (Row = Stage, Color = Frame ID)')
    
    # Legend (仅解释颜色代表Frame)
    legend_elements = [
        mpatches.Patch(facecolor='gray', label='Processing (Color by Frame)')
    ]
    ax.legend(handles=legend_elements, loc='upper right')
    
    plt.tight_layout()
    plt.show()

# ==========================================
# Main Execution
# ==========================================

# 模拟读取你的 log.txt 内容
with open('log/2026-03-20-20:56:28.txt', 'r', encoding='utf-8') as f: log_content = f.read()

if __name__ == "__main__":
    # 1. 解析
    parsed_data = parse_log(log_content)
    
    # 2. 视图1：原始视图 (以Packet为行)
    print("Generating Packet View...")
    visualize_by_packet(parsed_data, start_idx=0, end_idx=40)
    
    # 3. 视图2：新视图 (以流水级Stage为行，查看模块占用) //逻辑有误，暂停使用
    # print("Generating Stage View...")
    # visualize_by_stage(parsed_data, start_idx=0, end_idx=10)