import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


RUN_MODES = {
    "指定位置（百分比）": 11,
    "停止并释放": 0,
    "通电保持": 1,
    "检修 5 Hz": 2,
    "检修 100 Hz": 3,
    "低速 800 Hz": 4,
    "正常 3200 Hz": 5,
    "高速 6400 Hz": 6,
    "自定义 RPM": 7,
    "快速正转 → 慢速反转": 8,
    "任务9：加速到 60 RPM，保持 10 秒": 9,
    "任务10：120 RPM 全行程 75 秒": 10,
}


class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("DRV8825 步进电机上位机")
        self.root.geometry("700x620")
        self.ser = None
        self.rx_queue = queue.Queue()
        self.alive = True

        panel = ttk.Frame(root, padding=12)
        panel.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(panel, text="串口连接", padding=10)
        connection.pack(fill="x")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(connection, textvariable=self.port_var, width=22)
        self.port_box.grid(row=0, column=0, padx=4)
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(row=0, column=1, padx=4)
        self.connect_button = ttk.Button(connection, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=4)
        self.connection_label = ttk.Label(connection, text="未连接")
        self.connection_label.grid(row=0, column=3, padx=10)

        settings = ttk.LabelFrame(panel, text="运行参数", padding=10)
        settings.pack(fill="x", pady=10)

        ttk.Label(settings, text="运行模式").grid(row=0, column=0, sticky="w", pady=5)
        self.run_mode_var = tk.StringVar(value="自定义 RPM")
        ttk.Combobox(
            settings,
            textvariable=self.run_mode_var,
            values=tuple(RUN_MODES),
            state="readonly",
            width=20,
        ).grid(row=0, column=1, padx=6)
        ttk.Button(settings, text="设置模式", command=self.set_run_mode).grid(row=0, column=2, padx=6)

        ttk.Label(settings, text="机械转速 (RPM)").grid(row=1, column=0, sticky="w", pady=5)
        self.speed_var = tk.StringVar(value="30.0")
        ttk.Entry(settings, textvariable=self.speed_var, width=22).grid(row=1, column=1, padx=6)
        ttk.Button(settings, text="设置转速", command=self.set_speed).grid(row=1, column=2, padx=6)

        ttk.Label(settings, text="细分模式").grid(row=2, column=0, sticky="w", pady=5)
        self.microstep_var = tk.StringVar(value="32")
        ttk.Combobox(
            settings,
            textvariable=self.microstep_var,
            values=("1", "2", "4", "8", "16", "32"),
            state="readonly",
            width=20,
        ).grid(row=2, column=1, padx=6)
        ttk.Button(settings, text="设置细分", command=self.set_microstep).grid(row=2, column=2, padx=6)

        ttk.Label(settings, text="方向").grid(row=3, column=0, sticky="w", pady=5)
        self.direction_var = tk.StringVar(value="REV")
        ttk.Radiobutton(
            settings, text="正向", variable=self.direction_var, value="FWD"
        ).grid(row=3, column=1, sticky="w")
        ttk.Radiobutton(
            settings, text="反向", variable=self.direction_var, value="REV"
        ).grid(row=3, column=2, sticky="w")

        ttk.Label(settings, text="目标位置 (%)").grid(row=4, column=0, sticky="w", pady=5)
        self.position_var = tk.StringVar(value="50")
        ttk.Entry(settings, textvariable=self.position_var, width=22).grid(row=4, column=1, padx=6)
        ttk.Button(settings, text="设置位置", command=self.set_position).grid(row=4, column=2, padx=6)

        ttk.Button(settings, text="应用全部参数", command=self.apply_all).grid(
            row=5, column=0, columnspan=3, pady=10, sticky="ew"
        )

        actions = ttk.Frame(panel)
        actions.pack(fill="x")
        ttk.Button(actions, text="启动", command=lambda: self.send("RUN"), width=14).pack(side="left", padx=4)
        ttk.Button(actions, text="停止", command=lambda: self.send("STOP"), width=14).pack(side="left", padx=4)
        ttk.Button(actions, text="查询状态", command=lambda: self.send("STATUS"), width=14).pack(side="left", padx=4)
        ttk.Button(actions, text="清空日志", command=self.clear_log, width=14).pack(side="left", padx=4)

        log_frame = ttk.LabelFrame(panel, text="通信日志", padding=8)
        log_frame.pack(fill="both", expand=True, pady=10)
        self.log = tk.Text(log_frame, height=14, state="disabled")
        self.log.pack(fill="both", expand=True)

        self.refresh_ports()
        self.root.after(50, self.process_rx_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.connect_button.config(text="连接")
            self.connection_label.config(text="未连接")
            return
        if not self.port_var.get():
            messagebox.showwarning("提示", "请选择 COM 口")
            return
        try:
            self.ser = serial.Serial(self.port_var.get(), 115200, timeout=0.1)
            self.connect_button.config(text="断开")
            self.connection_label.config(text=f"已连接 {self.port_var.get()}")
            threading.Thread(target=self.reader, daemon=True).start()
        except serial.SerialException as exc:
            messagebox.showerror("连接失败", str(exc))

    def reader(self):
        while self.alive and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline()
                if line:
                    text = line.decode("utf-8", errors="replace").strip()
                    self.rx_queue.put("RX  " + text)
            except serial.SerialException as exc:
                self.rx_queue.put("ERR " + str(exc))
                break

    def send(self, command):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("提示", "请先连接串口")
            return False
        try:
            self.ser.write((command + "\r\n").encode("ascii"))
            self.append_log("TX  " + command)
            return True
        except serial.SerialException as exc:
            messagebox.showerror("发送失败", str(exc))
            return False

    def validated_speed(self):
        try:
            value = float(self.speed_var.get())
            if not 0.01 <= value <= 1875.0:
                raise ValueError
            return value
        except ValueError:
            messagebox.showwarning("参数错误", "RPM 请输入 0.01～1875 的数字")
            return None

    @staticmethod
    def format_rpm(value):
        return f"{value:.3f}".rstrip("0").rstrip(".")

    def set_speed(self):
        speed = self.validated_speed()
        if speed is not None:
            self.run_mode_var.set("自定义 RPM")
            self.send(f"RPM {self.format_rpm(speed)}")

    def set_microstep(self):
        self.send(f"MICROSTEP {self.microstep_var.get()}")

    def set_direction(self):
        self.send(f"DIR {self.direction_var.get()}")

    def set_run_mode(self):
        self.send(f"RUNMODE {RUN_MODES[self.run_mode_var.get()]}")

    def set_position(self):
        try:
            position = int(self.position_var.get())
            if not 0 <= position <= 100:
                raise ValueError
        except ValueError:
            messagebox.showwarning("参数错误", "目标位置请输入 0～100 的整数百分比")
            return
        self.run_mode_var.set(next(name for name, value in RUN_MODES.items() if value == 11))
        self.send(f"GOTO {position}")

    def apply_all(self):
        speed = self.validated_speed()
        if speed is None:
            return
        commands = (
            f"RPM {self.format_rpm(speed)}",
            f"MICROSTEP {self.microstep_var.get()}",
            f"DIR {self.direction_var.get()}",
            f"RUNMODE {RUN_MODES[self.run_mode_var.get()]}",
        )
        for command in commands:
            if not self.send(command):
                break

    def process_rx_queue(self):
        while not self.rx_queue.empty():
            self.append_log(self.rx_queue.get_nowait())
        self.root.after(50, self.process_rx_queue)

    def append_log(self, text):
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def clear_log(self):
        self.log.config(state="normal")
        self.log.delete("1.0", "end")
        self.log.config(state="disabled")

    def close(self):
        self.alive = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    window = tk.Tk()
    MotorControlApp(window)
    window.mainloop()
