"""UART GUI for 4-stepper STM32 firmware.

Requires:
    pip install pyserial
"""

from __future__ import annotations

import queue
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk

from serial import Serial
from serial.tools import list_ports


SYNC = b"\xA5\x5A"
TYPE_TEXT = 3
TYPE_ACK = 0xFE
TYPE_NACK = 0xFF

DEFAULT_BAUD = 115200
DEFAULT_MIN_DEGREES = 0
DEFAULT_MAX_DEGREES = 90
DEGREE_STEP = 15
BASE_MOVE_RATE_HZ = 200
MIN_MOVE_RATE_MULTIPLIER = 1.0
MAX_MOVE_RATE_MULTIPLIER = 3.0
MOTOR_GEAR_RATIOS = {
    1: 17,
    2: 25,
    3: 25,
    4: 25,
}
MOTOR_COUNT = 4
BASE_LIMIT_DEG = 360.0
SHOULDER_LIMIT_DEG = 90.0
ELBOW_LIMIT_DEG = 135.0
WRIST_LIMIT_DEG = 135.0
JOINT_LIMIT_DEG = {
    1: BASE_LIMIT_DEG,
    2: SHOULDER_LIMIT_DEG,
    3: ELBOW_LIMIT_DEG,
    4: WRIST_LIMIT_DEG,
}
READY_X_MM = 0.0
READY_Y_MM = 200.0
READY_Z_MM = 200.0
DEFAULT_TIP_SPEED_MM_S = 20.0
MOTOR_LABELS = {
    1: "Motor 1 (base joint)",
    2: "Motor 2 (shoulder joint)",
    3: "Motor 3 (elbow joint)",
    4: "Motor 4 (wrist joint)",
}
MOTOR_AVAILABLE = {
    1: True,
    2: True,
    3: True,
    4: True,
}


def build_frame(frame_type: int, payload: bytes) -> bytes:
    header = SYNC + bytes([frame_type]) + len(payload).to_bytes(2, "little")
    return header + payload


def read_exact(ser: Serial, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
            continue
        data.extend(chunk)
    return bytes(data)


def find_sync(ser: Serial) -> bool:
    window = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            return False
        window.extend(b)
        if len(window) > 2:
            del window[:-2]
        if len(window) == 2 and bytes(window) == SYNC:
            return True


def read_packet(ser: Serial):
    if not find_sync(ser):
        return None

    header = read_exact(ser, 3)
    packet_type = header[0]
    payload_len = int.from_bytes(header[1:3], "little")
    payload = read_exact(ser, payload_len) if payload_len else b""
    return packet_type, payload


@dataclass
class MotorState:
    enabled_var: tk.BooleanVar
    direction_var: tk.BooleanVar
    degrees_var: tk.IntVar


class StepperGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()

        self.title("STM32 4-Motor Control")
        self.geometry("1160x760")
        self.minsize(940, 660)

        self.serial_port: Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.reader_stop = threading.Event()
        self.rx_queue: "queue.Queue[tuple[str, str]]" = queue.Queue()

        self.port_var = tk.StringVar(value=self._default_port())
        self.baud_var = tk.IntVar(value=DEFAULT_BAUD)
        self.connection_state_var = tk.StringVar(value="Disconnected")
        self.joystick_state_var = tk.StringVar(value="Joystick mode off")
        self.last_reply_var = tk.StringVar(value="No traffic yet")
        self.estimated_pose_var = tk.StringVar(value="Pose: waiting for firmware (press Set Home first)")
        self.target_x_var = tk.StringVar(value=f"{READY_X_MM:g}")
        self.target_y_var = tk.StringVar(value=f"{READY_Y_MM:g}")
        self.target_z_var = tk.StringVar(value=f"{READY_Z_MM:g}")
        self.speed_var = tk.StringVar(value=f"{DEFAULT_TIP_SPEED_MM_S:g}")
        self.joint_test_vars = [tk.StringVar(value="0") for _ in range(MOTOR_COUNT)]

        self.motor_states: list[MotorState] = []

        self._build_style()
        self._build_ui()
        self.after(80, self._drain_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _default_port(self) -> str:
        return "COM5"

    def _build_style(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        self.configure(bg="#0c1222")
        style.configure("Root.TFrame", background="#0c1222")
        style.configure("Card.TFrame", background="#11172a", relief="flat")
        style.configure("Title.TLabel", background="#0c1222", foreground="#f8fafc", font=("Segoe UI", 17, "bold"))
        style.configure("Section.TLabel", background="#11172a", foreground="#dbeafe", font=("Segoe UI", 11, "bold"))
        style.configure("Body.TLabel", background="#11172a", foreground="#bfdbfe", font=("Segoe UI", 10))
        style.configure("Muted.TLabel", background="#11172a", foreground="#93c5fd", font=("Segoe UI", 9))
        style.configure("Motor.TCheckbutton", background="#11172a", foreground="#e2e8f0")
        style.map(
            "Motor.TCheckbutton",
            background=[("active", "#1a243f"), ("!active", "#11172a")],
            foreground=[("disabled", "#64748b"), ("active", "#e2e8f0"), ("!active", "#e2e8f0")],
            indicatorcolor=[("active", "#94a3b8"), ("!active", "#dbeafe")],
        )
        style.configure("Primary.TButton", padding=(10, 5))
        style.configure("Danger.TButton", padding=(10, 5))

    def _build_ui(self) -> None:
        root = ttk.Frame(self, style="Root.TFrame", padding=10)
        root.pack(fill="both", expand=True)

        ttk.Label(root, text="STM32 Motor Driver Control", style="Title.TLabel").pack(anchor="w", pady=(0, 7))

        top = ttk.Frame(root, style="Root.TFrame")
        top.pack(fill="x", pady=(0, 6))

        self._build_connection_card(top)
        self._build_global_card(top)

        arm_row = ttk.Frame(root, style="Root.TFrame")
        arm_row.pack(fill="x", pady=(0, 6))
        self._build_arm_card(arm_row)

        motors_wrap = ttk.Frame(root, style="Root.TFrame")
        motors_wrap.pack(fill="x", pady=(0, 6))

        for idx in range(1, MOTOR_COUNT + 1):
            card = ttk.Frame(motors_wrap, style="Card.TFrame", padding=8)
            r = (idx - 1) // 2
            c = (idx - 1) % 2
            card.grid(row=r, column=c, sticky="nsew", padx=4, pady=4)
            motors_wrap.columnconfigure(c, weight=1)
            self._build_motor_card(card, idx)

        lower = ttk.Frame(root, style="Root.TFrame")
        lower.pack(fill="both", expand=True)

        status_card = ttk.Frame(lower, style="Card.TFrame", padding=8)
        status_card.pack(fill="x", pady=(0, 6))
        ttk.Label(status_card, text="Latest Reply", style="Section.TLabel").pack(anchor="w")
        ttk.Label(status_card, textvariable=self.last_reply_var, style="Body.TLabel").pack(anchor="w", pady=(3, 0))
        ttk.Label(status_card, textvariable=self.estimated_pose_var, style="Muted.TLabel").pack(anchor="w", pady=(2, 0))

        log_card = ttk.Frame(lower, style="Card.TFrame", padding=8)
        log_card.pack(fill="both", expand=True)
        ttk.Label(log_card, text="Event Log", style="Section.TLabel").pack(anchor="w")

        self.log_text = tk.Text(
            log_card,
            height=6,
            bg="#0a1020",
            fg="#dbeafe",
            insertbackground="#dbeafe",
            relief="flat",
            borderwidth=0,
            wrap="word",
        )
        self.log_text.pack(fill="both", expand=True, pady=(4, 0))
        self.log_text.configure(state="disabled")

        self._log("Ready. Connect and use Motor 1 panel now.")

    def _build_connection_card(self, parent: ttk.Frame) -> None:
        card = ttk.Frame(parent, style="Card.TFrame", padding=8)
        card.pack(side="left", fill="both", expand=True, padx=(0, 6))

        ttk.Label(card, text="Connection", style="Section.TLabel").grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(card, text="Port", style="Body.TLabel").grid(row=1, column=0, sticky="w", pady=(5, 0))
        ttk.Label(card, text="Baud", style="Body.TLabel").grid(row=1, column=1, sticky="w", pady=(5, 0))

        ttk.Entry(card, textvariable=self.port_var, width=14).grid(row=2, column=0, sticky="we", padx=(0, 6))
        ttk.Entry(card, textvariable=self.baud_var, width=10).grid(row=2, column=1, sticky="we", padx=(0, 6))
        ttk.Button(card, text="Refresh", command=self._refresh_ports).grid(row=2, column=2, sticky="we", padx=(0, 6))
        self.connect_button = ttk.Button(card, text="Connect", style="Primary.TButton", command=self._toggle_connection)
        self.connect_button.grid(row=2, column=3, sticky="we")

        ttk.Label(card, textvariable=self.connection_state_var, style="Muted.TLabel").grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(5, 0)
        )

    def _build_global_card(self, parent: ttk.Frame) -> None:
        card = ttk.Frame(parent, style="Card.TFrame", padding=8)
        card.pack(side="left", fill="both", expand=True, padx=(6, 0))

        ttk.Label(card, text="Global Controls", style="Section.TLabel").grid(row=0, column=0, columnspan=4, sticky="w")

        ttk.Button(card, text="Enable All", command=self._enable_all).grid(row=1, column=0, padx=4, pady=(10, 4), sticky="we")
        ttk.Button(card, text="Disable All", command=lambda: self._send("ALL_EN=0")).grid(row=1, column=1, padx=4, pady=(10, 4), sticky="we")
        ttk.Button(card, text="Stop All", style="Danger.TButton", command=self._stop_all).grid(row=1, column=2, padx=4, pady=(10, 4), sticky="we")
        ttk.Button(card, text="Joystick On", command=self._enable_joystick).grid(row=1, column=3, padx=4, pady=(10, 4), sticky="we")
        ttk.Button(card, text="Joystick Off", command=self._disable_joystick).grid(row=2, column=3, padx=4, pady=(4, 4), sticky="we")
        ttk.Button(card, text="Joystick Status", command=lambda: self._send("JOY_STATUS")).grid(row=3, column=3, padx=4, pady=(4, 4), sticky="we")
        ttk.Label(card, textvariable=self.joystick_state_var, style="Muted.TLabel").grid(row=2, column=0, columnspan=3, sticky="w", padx=4, pady=(4, 4))

    def _build_arm_card(self, parent: ttk.Frame) -> None:
        card = ttk.Frame(parent, style="Card.TFrame", padding=8)
        card.pack(fill="x", expand=True)

        ttk.Label(card, text="Arm Control (inverse kinematics runs on the STM32)", style="Section.TLabel").grid(
            row=0, column=0, columnspan=10, sticky="w"
        )

        ttk.Button(card, text="1. Set Home (Calibrate 0)", command=self._set_home_pose).grid(
            row=1, column=0, columnspan=2, sticky="we", pady=(6, 0), padx=(0, 2)
        )
        ttk.Button(card, text="Go Home (0,0,0,0)", command=self._go_home).grid(
            row=1, column=2, columnspan=2, sticky="we", pady=(6, 0), padx=2
        )
        ttk.Button(card, text="2. Ready Pose", style="Primary.TButton", command=self._go_ready).grid(
            row=1, column=4, columnspan=2, sticky="we", pady=(6, 0), padx=2
        )
        ttk.Button(card, text="Stop Arm", style="Danger.TButton", command=self._stop_all).grid(
            row=1, column=6, columnspan=2, sticky="we", pady=(6, 0), padx=2
        )
        ttk.Button(card, text="Request Pose", command=lambda: self._send("POSE")).grid(
            row=1, column=8, columnspan=2, sticky="we", pady=(6, 0), padx=(2, 0)
        )

        ttk.Label(card, text="Go to X / Y / Z (mm)", style="Body.TLabel").grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Entry(card, textvariable=self.target_x_var, width=8).grid(row=3, column=0, sticky="w")
        ttk.Entry(card, textvariable=self.target_y_var, width=8).grid(row=3, column=1, sticky="w")
        ttk.Entry(card, textvariable=self.target_z_var, width=8).grid(row=3, column=2, sticky="w")
        ttk.Button(card, text="Move To XYZ", style="Primary.TButton", command=self._move_to_xyz).grid(
            row=3, column=3, columnspan=2, sticky="we", padx=4
        )
        ttk.Label(card, text="Tip speed (mm/s)", style="Body.TLabel").grid(row=3, column=5, sticky="e", padx=(8, 4))
        ttk.Entry(card, textvariable=self.speed_var, width=6).grid(row=3, column=6, sticky="w")
        ttk.Button(card, text="Set Speed", command=self._set_speed).grid(row=3, column=7, columnspan=2, sticky="we", padx=4)

        ttk.Label(card, text="Joint test: Base / Shoulder / Elbow / Wrist (deg from home)", style="Body.TLabel").grid(
            row=4, column=0, columnspan=6, sticky="w", pady=(8, 0)
        )
        for idx, var in enumerate(self.joint_test_vars):
            ttk.Entry(card, textvariable=var, width=8).grid(row=5, column=idx, sticky="w")
        ttk.Button(card, text="Joint Move", command=self._joint_move).grid(row=5, column=4, columnspan=2, sticky="we", padx=4)

        ttk.Label(
            card,
            text="Open loop: the pose is counted from commanded steps. Re-home after a stall or power cycle.",
            style="Muted.TLabel",
        ).grid(row=6, column=0, columnspan=10, sticky="w", pady=(8, 0))

        for col in range(10):
            card.columnconfigure(col, weight=1)

    def _build_motor_card(self, card: ttk.Frame, motor_id: int) -> None:
        enabled_var = tk.BooleanVar(value=False)
        direction_var = tk.BooleanVar(value=False)
        degrees_var = tk.IntVar(value=45)
        motor_available = MOTOR_AVAILABLE.get(motor_id, True)

        ttk.Label(card, text=MOTOR_LABELS.get(motor_id, f"Motor {motor_id}"), style="Section.TLabel").grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(card, text=f"Commands: M{motor_id}_EN / M{motor_id}_DIR / M{motor_id}_DEG", style="Muted.TLabel").grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(2, 5)
        )

        enable_check = ttk.Checkbutton(
            card,
            text="Enable",
            variable=enabled_var,
            style="Motor.TCheckbutton",
            command=lambda m=motor_id, v=enabled_var: self._send(f"M{m}_EN={1 if v.get() else 0}"),
        )
        enable_check.grid(row=2, column=0, sticky="w")

        direction_check = ttk.Checkbutton(
            card,
            text="Forward Dir",
            variable=direction_var,
            style="Motor.TCheckbutton",
            command=lambda m=motor_id, v=direction_var: self._send(f"M{m}_DIR={1 if v.get() else 0}"),
        )
        direction_check.grid(row=2, column=1, sticky="w")

        if not motor_available:
            ttk.Label(card, text="Driver not populated", style="Muted.TLabel").grid(row=2, column=2, columnspan=2, sticky="e")

        ttk.Label(card, text="Joint Degrees (0 to 90)", style="Body.TLabel").grid(row=3, column=0, sticky="w", pady=(5, 0))

        degree_select = tk.Scale(
            card,
            variable=degrees_var,
            from_=DEFAULT_MIN_DEGREES,
            to=DEFAULT_MAX_DEGREES,
            orient="horizontal",
            resolution=DEGREE_STEP,
            tickinterval=DEGREE_STEP,
            length=320,
            bg="#11172a",
            fg="#dbeafe",
            highlightthickness=0,
            troughcolor="#223255",
        )
        degree_select.grid(row=4, column=0, columnspan=4, sticky="we")

        apply_button = ttk.Button(
            card,
            text="Apply Speed",
            command=lambda m=motor_id, dv=degrees_var, dirv=direction_var: self._send_motor_degrees(m, dv, dirv),
        )
        apply_button.grid(row=5, column=0, padx=(0, 4), pady=(5, 0), sticky="we")

        stop_button = ttk.Button(card, text="Stop", style="Danger.TButton", command=lambda m=motor_id: self._stop_motor(m))
        stop_button.grid(row=5, column=1, padx=4, pady=(5, 0), sticky="we")

        for c in range(4):
            card.columnconfigure(c, weight=1)

        if not motor_available:
            enable_check.configure(state="disabled")
            direction_check.configure(state="disabled")
            degree_select.configure(state="disabled")
            apply_button.configure(state="disabled")
            stop_button.configure(state="disabled")

        self.motor_states.append(
            MotorState(
                enabled_var=enabled_var,
                direction_var=direction_var,
                degrees_var=degrees_var,
            )
        )

    def _refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        if not ports:
            self._log("No serial ports detected.")
            return

        self.port_var.set(ports[0])
        self._log(f"Detected ports: {', '.join(ports)}")

    def _set_connected_ui(self, connected: bool) -> None:
        self.connection_state_var.set("Connected" if connected else "Disconnected")
        self.connect_button.configure(text="Disconnect" if connected else "Connect")

    def _toggle_connection(self) -> None:
        if self.serial_port is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        if self.serial_port is not None:
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Connection error", "Please enter a COM port.")
            return

        try:
            baud = int(self.baud_var.get())
        except tk.TclError:
            messagebox.showerror("Connection error", "Baud rate must be a number.")
            return

        try:
            self.serial_port = Serial(port, baud, timeout=0.1)
        except Exception as exc:
            self.serial_port = None
            messagebox.showerror("Connection error", f"Failed to open {port}: {exc}")
            return

        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, name="StepperGuiSerialReader", daemon=True)
        self.reader_thread.start()
        self._set_connected_ui(True)
        self._log(f"Connected to {port} @ {baud} baud")

    def _disconnect(self) -> None:
        self.reader_stop.set()
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self._set_connected_ui(False)
        self._log("Disconnected")

    def _reader_loop(self) -> None:
        while not self.reader_stop.is_set():
            ser = self.serial_port
            if ser is None:
                break

            try:
                if ser.in_waiting == 0:
                    time.sleep(0.02)
                    continue

                packet = read_packet(ser)
                if packet is None:
                    continue

                packet_type, payload = packet
                if packet_type == TYPE_ACK:
                    self.rx_queue.put(("reply", "ACK received"))
                elif packet_type == TYPE_NACK:
                    self.rx_queue.put(("reply", "NACK received"))
                elif packet_type == TYPE_TEXT:
                    text = payload.decode("utf-8", errors="replace")
                    self.rx_queue.put(("reply", text))
                    self.rx_queue.put(("status", text))
                    self.rx_queue.put(("log", f"STM32: {text}"))
                else:
                    self.rx_queue.put(("log", f"Unknown packet type {packet_type} ({len(payload)} bytes)"))
            except Exception as exc:
                self.rx_queue.put(("log", f"Serial error: {exc}"))
                self.rx_queue.put(("disconnect", ""))
                break

    def _drain_queue(self) -> None:
        try:
            while True:
                kind, text = self.rx_queue.get_nowait()
                if kind == "reply":
                    self.last_reply_var.set(text)
                    self._log(text)
                elif kind == "status":
                    self._apply_status_text(text)
                elif kind == "log":
                    self._log(text)
                elif kind == "disconnect":
                    self._disconnect()
        except queue.Empty:
            pass

        self.after(80, self._drain_queue)

    def _send(self, command: str) -> None:
        if self.serial_port is None:
            messagebox.showwarning("Not connected", "Connect to the STM32 first.")
            return

        frame = build_frame(TYPE_TEXT, command.encode("utf-8"))
        try:
            self.serial_port.write(frame)
            self.serial_port.flush()
            self._log(f"TX: {command}")
        except Exception as exc:
            self._log(f"TX failed: {exc}")
            self._disconnect()

    def _set_home_pose(self) -> None:
        if not messagebox.askyesno(
            "Set home",
            "Is the arm physically in the home pose (base, shoulder, elbow and wrist all pointing straight up)?",
        ):
            return
        self._send("HOME")
        self._log("Home set. The firmware now counts joint angles from this pose.")

    def _command_joint_relative(self, motor_id: int, delta_deg: float) -> None:
        self._command_joint_group({motor_id: delta_deg})

    def _command_joint_group(self, joint_deltas: dict[int, float]) -> None:
        moves = []
        for motor_id, delta_deg in joint_deltas.items():
            if abs(delta_deg) < 1e-6 or not MOTOR_AVAILABLE.get(motor_id, True):
                continue

            direction = 1 if delta_deg >= 0.0 else 0
            joint_limit = JOINT_LIMIT_DEG.get(motor_id, DEFAULT_MAX_DEGREES)
            move_deg = min(abs(delta_deg), joint_limit)
            motor_degrees = move_deg * MOTOR_GEAR_RATIOS.get(motor_id, 25)
            moves.append((motor_id, direction, move_deg, motor_degrees))

        if not moves:
            return

        maximum_motor_degrees = max(move[3] for move in moves)
        maximum_rate = round(BASE_MOVE_RATE_HZ * MAX_MOVE_RATE_MULTIPLIER)
        minimum_rate = round(BASE_MOVE_RATE_HZ * MIN_MOVE_RATE_MULTIPLIER)
        for motor_id, direction, _, motor_degrees in moves:
            rate = max(minimum_rate, round(maximum_rate * motor_degrees / maximum_motor_degrees))
            self._send(f"M{motor_id}_DIR={direction}")
            self._send(f"M{motor_id}_RATE={rate}")

        for motor_id, _, _, motor_degrees in moves:
            self._send(f"M{motor_id}_DEG={round(motor_degrees)}")

    def _read_floats(self, values: list[str], names: str) -> list[float] | None:
        try:
            return [float(value) for value in values]
        except ValueError:
            messagebox.showerror("Input error", f"{names} must be numeric values.")
            return None

    def _move_to_xyz(self) -> None:
        values = self._read_floats(
            [self.target_x_var.get(), self.target_y_var.get(), self.target_z_var.get()], "X, Y, Z"
        )
        if values is None:
            return
        self._send(f"GOTO={values[0]:g},{values[1]:g},{values[2]:g}")

    def _set_speed(self) -> None:
        values = self._read_floats([self.speed_var.get()], "Speed")
        if values is None:
            return
        self._send(f"SPEED={values[0]:g}")

    def _go_ready(self) -> None:
        self._send("READY")

    def _go_home(self) -> None:
        self._send("JMOVE=0,0,0,0")

    def _joint_move(self) -> None:
        values = self._read_floats([var.get() for var in self.joint_test_vars], "Joint angles")
        if values is None:
            return
        for motor_id, angle in enumerate(values, start=1):
            if abs(angle) > JOINT_LIMIT_DEG[motor_id]:
                messagebox.showerror("Limit error", f"Motor {motor_id} angle must be within +/-{JOINT_LIMIT_DEG[motor_id]:g} deg.")
                return
        self._send("JMOVE=" + ",".join(f"{angle:g}" for angle in values))

    def _send_motor_degrees(self, motor_id: int, degrees_var: tk.IntVar, direction_var: tk.BooleanVar) -> None:
        joint_degrees = self._sync_motor_degree_input(degrees_var)
        if joint_degrees is None:
            return

        signed_joint_delta = float(joint_degrees if direction_var.get() else -joint_degrees)
        self._command_joint_relative(motor_id, signed_joint_delta)

    def _sync_motor_degree_input(self, degrees_var: tk.IntVar) -> int | None:
        try:
            degrees = int(degrees_var.get())
        except (ValueError, tk.TclError):
            messagebox.showerror("Degrees error", "Degrees must be a number.")
            return None

        degrees = int((degrees + (DEGREE_STEP // 2)) / DEGREE_STEP) * DEGREE_STEP
        degrees = max(DEFAULT_MIN_DEGREES, min(DEFAULT_MAX_DEGREES, degrees))
        degrees_var.set(degrees)
        return degrees

    def _stop_motor(self, motor_id: int) -> None:
        self._send(f"M{motor_id}_STEP=0")

    def _stop_all(self) -> None:
        self._send("STOPALL")

    def _enable_all(self) -> None:
        self._send("STOPALL")
        self._send("ALL_EN=1")

    def _enable_joystick(self) -> None:
        self._send("JOY=1")
        self.joystick_state_var.set("Joystick mode on")

    def _disable_joystick(self) -> None:
        self._send("JOY=0")
        self.joystick_state_var.set("Joystick mode off")

    def _apply_status_text(self, text: str) -> None:
        pose = re.match(r"POSE H(\d) M(\d) P(\d) X(-?\d+) Y(-?\d+) Z(-?\d+) \| B(-?\d+) S(-?\d+) E(-?\d+) W(-?\d+)", text)
        if pose:
            homed, mode, horizontal = (int(pose.group(i)) for i in (1, 2, 3))
            x, y, z, base, shoulder, elbow, wrist = (int(pose.group(i)) / 10.0 for i in range(4, 11))
            mode_names = {0: "idle", 1: "joystick", 2: "goto", 3: "joint move", 4: "settling"}
            self.estimated_pose_var.set(
                f"{'HOME SET' if homed else 'HOME NOT SET'} | {mode_names.get(mode, mode)} | "
                f"pointer {'horizontal' if horizontal else 'NOT horizontal'} | "
                f"Tip X={x:.1f} Y={y:.1f} Z={z:.1f} mm | "
                f"B={base:.1f} S={shoulder:.1f} E={elbow:.1f} W={wrist:.1f} deg"
            )
            self.joystick_state_var.set("Joystick mode on" if mode == 1 else "Joystick mode off")
            return

        matches = re.findall(r"M([1-4]) E([01]) D([01])", text)
        if len(matches) != MOTOR_COUNT:
            return

        for m_id_str, en_str, dir_str in matches:
            idx = int(m_id_str) - 1
            motor = self.motor_states[idx]
            motor.enabled_var.set(en_str == "1")
            motor.direction_var.set(dir_str == "1")

    def _log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {message}\n"
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _on_close(self) -> None:
        self.reader_stop.set()
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.destroy()


def main() -> None:
    app = StepperGui()
    app.mainloop()


if __name__ == "__main__":
    main()
