import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import subprocess
import re
import time
import threading 

# configuration
# cpp should output data
num_data = 9

def parse_line(line):
    try:
        return float(line.strip())
    except (ValueError, TypeError):
        return None

def extract_valid_data_blocks_from_lines(lines_buffer, last_parsed_index=0):
    data_blocks = []
    i = last_parsed_index
    n = len(lines_buffer)

    while i < n:
        line = lines_buffer[i]
        time_match = re.search(r'(\d{2}):(\d{2}):(\d{3}): <MSG>:', line)
        if time_match:
            try:
                hh, mm, ms = map(int, time_match.groups())
                total_time = hh * 60 + mm + ms * 0.001

                if i + num_data >= n:
                    break

                values = []
                for j in range(1, num_data + 1):
                    val = parse_line(lines_buffer[i + j])
                    if val is None:
                        raise ValueError(f"Invalid data at line {i + j}: '{lines_buffer[i + j]}'")
                    values.append(val)

                data_blocks.append((total_time, values))
                i += num_data + 1
            except Exception as e:
                print(f"Skipping invalid block at line {i}: {e}")
                i += 1
        else:
            i += 1

    return data_blocks, i

class RealTimePlotter:
    def __init__(self, cpp_executable_path, time_window=10.0):
        # 启动 C++ 子进程
        self.cpp_process = subprocess.Popen(
            [cpp_executable_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )

        self.lines_buffer = []
        self.all_data_blocks = []
        self.last_parsed_index = 0
        self._running = True  # 控制线程退出
        self.paused = False   # 控制是否暂停绘图
        self.time_window = time_window  # in seconds

        self.reader_thread = threading.Thread(target=self._read_output_loop, daemon=True)
        self.reader_thread.start()

        # 初始化图形（主线程中）
        plt.ion()  # 交互模式
        self.fig, self.ax = plt.subplots(figsize=(14, 8))
        self.lines = [self.ax.plot([], [], label=f'data{i}', marker='o', markersize=1)[0] for i in range(num_data)]
        self.ax.set_xlabel('Time (seconds)')
        self.ax.set_ylabel('Value')
        self.ax.set_title('Real-Time Data Plot (Sliding Time Window)')
        self.ax.legend()
        self.ax.grid(True)
        plt.xticks(rotation=45)
        plt.tight_layout()

        # 绑定按键事件
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)

        plt.show(block=False)

    def _on_key_press(self, event):
        """Handle key press: 'p' to toggle pause"""
        print(f"[DEBUG] Key pressed: {event.key}")
        if event.key == 'p':
            self.paused = not self.paused
            status = "[PAUSED]" if self.paused else "[RUNNING]"
            self.ax.set_title(f'Real-Time Data Plot (Sliding Time Window) {status}')
            self.fig.canvas.draw()
            print(f"Plotting {'paused' if self.paused else 'resumed'} (press 'p' to toggle)")

    def update_plot(self):
        """主线程调用：解析新数据并更新绘图，仅显示最近 time_window 秒的数据"""
        if self.paused:
            return  # Skip updating if paused

        new_blocks, new_index = extract_valid_data_blocks_from_lines(
            self.lines_buffer, self.last_parsed_index
        )
        self.last_parsed_index = new_index

        if new_blocks:
            self.all_data_blocks.extend(new_blocks)
            # print(f"Parsed {len(new_blocks)} new blocks. Total: {len(self.all_data_blocks)}")

        if not self.all_data_blocks:
            return

        # Get current latest time
        current_max_time = self.all_data_blocks[-1][0]
        window_start = current_max_time - self.time_window

        # Filter data within time window
        visible_blocks = [(t, vals) for t, vals in self.all_data_blocks if t >= window_start]
        times = [t for t, _ in visible_blocks]
        values_list = [v for _, v in visible_blocks]

        if not times:
            return

        for i in range(num_data):
            data_i = [d[i] for d in values_list]
            self.lines[i].set_data(times, data_i)

                # 自动缩放 X 轴到窗口范围
        self.ax.set_xlim(window_start, current_max_time)

        # 手动计算当前窗口内所有数据的 Y 范围
        if values_list:
            all_y = [val for sublist in values_list for val in sublist]  # 展平所有数据
            y_min, y_max = min(all_y), max(all_y)
            margin = (y_max - y_min) * 0.05  # 5% 边距，避免曲线贴边
            if margin == 0:  # 防止所有值相等时 margin=0
                margin = 0.1
            self.ax.set_ylim(y_min - margin, y_max + margin)
        else:
            self.ax.set_ylim(-1, 1)  # 默认范围，防止空数据报错

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run(self, refresh_rate=1.0):
        """主线程循环：按频率刷新绘图"""
        try:
            while self._running:
                start_time = time.time()

                self.update_plot()

                elapsed = time.time() - start_time
                sleep_time = max(refresh_rate - elapsed, 0.01)
                plt.pause(sleep_time)
        except KeyboardInterrupt:
            print("Interrupted by user.")
        finally:
            self._running = False
            self.cpp_process.terminate()
            try:
                self.cpp_process.wait(timeout=2)
            except:
                self.cpp_process.kill()
    
    def _read_output_loop(self):
        """后台线程：持续读取 C++ 输出"""
        while self._running:
            line = self.cpp_process.stdout.readline()
            if not line:
                break
            line = line.strip()
            if line:
                # print(f"[Received] '{line}'")
                self.lines_buffer.append(line)
            time.sleep(0.001)

if __name__ == "__main__":
    cpp_path = "./build/auto-aim"  # 修改为你的路径

    plotter = RealTimePlotter(cpp_path, time_window=15.0)
    plotter.run(refresh_rate=3)  # 每0.5秒刷新一次绘图