#!/usr/bin/env python3
"""F407 串口监控/收发工具 (Lunarover_f407).

通过 USB CDC 虚拟串口(或 USB-TTL 物理串口)与 STM32F407 通信:
- 实时解析 $STD 运行数据帧
- 下发命令帧 $SPD / $ANG / $STOP / $ZERO

用法:
    python f407_monitor.py                          # 列出可用串口
    python f407_monitor.py COM3                     # 持续监听
    python f407_monitor.py COM3 --send '$ANG,90,90,0,0!'   # 发一帧后退出
    python f407_monitor.py COM3 --read 3            # 读 3 秒数据后退出

命令帧:
    $SPD,s0,s1,s2,s3!   速度模式(开环, -1~1)
    $ANG,a0,a1,a2,a3!   位置模式(轮舵角度, 度)
    $STOP!              停止
    $ZERO!              把当前位置设为零位

数据帧(固件每 500ms 发一次):
    $STD,fbA,fbB,encA,encB,angA,angB,spdA,spdB
"""
import sys
import time
import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("需要 pyserial: pip install pyserial")

BAUD = 115200  # USB CDC 虚拟串口忽略波特率, 仅物理串口需要


def _now():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def _open(port, baud):
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.6)  # 等 USB 枚举/CDC 就绪
    ser.reset_input_buffer()
    return ser


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("未找到任何串口设备.")
        return
    print("可用串口:")
    for p in ports:
        print(f"  {p.device:<10} {p.description}")


def parse_std(fields):
    """$STD,fbA,fbB,encA,encB,angA,angB,spdA,spdB -> 对齐的可读行."""
    if len(fields) < 9:
        return f"[{_now()}] RAW: {','.join(fields)}"
    return (f"[{_now()}] fb={fields[1]}/{fields[2]}  "
            f"enc={fields[3]:>7}/{fields[4]:>7}  "
            f"ang={fields[5]:>8}/{fields[6]:>8}  spd={fields[7]}/{fields[8]}")


def _print_line(line):
    if line.startswith("$STD,"):
        print(parse_std(line.split(",")), flush=True)
    else:
        print(f"[{_now()}] {line}", flush=True)


def send_once(port, baud, frame):
    ser = _open(port, baud)
    ser.write(frame.encode("ascii"))
    ser.flush()
    ser.close()
    print(f"已发送: {frame}")


def read_for(port, baud, seconds):
    ser = _open(port, baud)
    print(f"读取 {port} {seconds}s ...")
    buf = b""
    start = time.time()
    try:
        while time.time() - start < seconds:
            chunk = ser.read(1024)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip().decode("ascii", "replace").strip()
                if line:
                    _print_line(line)
    finally:
        ser.close()


def monitor(port, baud):
    ser = _open(port, baud)
    print(f"监听 {port} @ {baud} ... (Ctrl+C 退出)")
    print("字段: fb=反馈在线  enc=累计编码器  ang=轮舵角  spd=输出速度")
    print("-" * 70)
    buf = b""
    try:
        while True:
            chunk = ser.read(1024)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip().decode("ascii", "replace").strip()
                if line:
                    _print_line(line)
    except KeyboardInterrupt:
        print("\n已停止.")
    finally:
        ser.close()


def main():
    args = sys.argv[1:]
    if not args:
        list_serial_ports()
        sys.exit(0)

    port = args[0]
    baud = BAUD
    if "--baud" in args:
        baud = int(args[args.index("--baud") + 1])

    if "--send" in args:
        frame = args[args.index("--send") + 1]
        send_once(port, baud, frame)
    elif "--read" in args:
        seconds = float(args[args.index("--read") + 1])
        read_for(port, baud, seconds)
    else:
        monitor(port, baud)


if __name__ == "__main__":
    main()
