from __future__ import annotations

import math
import random
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
# fix dumbass bug
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from coprocessor.protocol import bcnp, schema


FIELD_LENGTH_MM = 16541
FIELD_WIDTH_MM = 8211
MAX_PHASE_SEQ = 16
MAX_WAYPOINTS_PER_PHASE = 64
MAX_TOTAL_WAYPOINTS = 512
MAX_VELOCITY_MM_S = 5000

DEFAULT_SCHEMA = Path("src/main/deploy/bcnp/messages.json")


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def _clampf(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class PlannerSimulator:
    def __init__(self, host: str, port: int, schema_path: Path) -> None:
        self.host = host
        self.port = port
        self.schema_path = schema_path
        self.contract = schema.load_contract(schema_path)
        self.schema_hash = schema.schema_crc32(schema_path.read_bytes())
        self.wire_sizes = self.contract.wire_sizes_by_id()

        self._server_sock: socket.socket | None = None
        self._client_sock: socket.socket | None = None
        self._client_addr: tuple[str, int] | None = None
        self._rx = bytearray()
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._heartbeat_seq = 0
        self._last_hb_ms = 0
        self._last_plan_id = 1000

        self._script: list[tuple[float, float, int]] = []
        self._last_world_update: dict[str, int] = {}
        self._events: list[str] = []
        self._send_shot_hint = False
        self._shot_rpm = 2600
        self._shot_hood_permille = 320
        self._shot_conf_permille = 800

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        with self._lock:
            if self._client_sock is not None:
                try:
                    self._client_sock.close()
                except OSError:
                    pass
                self._client_sock = None
            if self._server_sock is not None:
                try:
                    self._server_sock.close()
                except OSError:
                    pass
                self._server_sock = None
            self._client_addr = None

    def connected(self) -> bool:
        with self._lock:
            return self._client_sock is not None

    def client_peer(self) -> str:
        with self._lock:
            if self._client_addr is None:
                return "none"
            return f"{self._client_addr[0]}:{self._client_addr[1]}"

    def set_script(self, commands: list[tuple[float, float, int]]) -> None:
        with self._lock:
            self._script = list(commands)

    def set_shot_hint(self, enabled: bool, rpm: int, hood_permille: int, confidence_permille: int) -> None:
        with self._lock:
            self._send_shot_hint = enabled
            self._shot_rpm = _clamp(rpm, 1000, 6000)
            self._shot_hood_permille = _clamp(hood_permille, 0, 1000)
            self._shot_conf_permille = _clamp(confidence_permille, 0, 1000)

    def pop_events(self) -> list[str]:
        with self._lock:
            events = self._events[:]
            self._events.clear()
            return events

    def last_world(self) -> dict[str, int]:
        with self._lock:
            return dict(self._last_world_update)

    def _log(self, msg: str) -> None:
        with self._lock:
            self._events.append(msg)
            if len(self._events) > 200:
                self._events = self._events[-200:]

    def _loop(self) -> None:
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(1)
            server.settimeout(0.2)
            with self._lock:
                self._server_sock = server
            self._log(f"listening {self.host}:{self.port}")

            while self._running:
                self._accept_if_needed(server)
                self._recv_packets()
                self._send_heartbeat_if_due()
                time.sleep(0.01)
        except Exception as exc:
            self._log(f"server error: {exc}")
        finally:
            self.stop()

    def _accept_if_needed(self, server: socket.socket) -> None:
        with self._lock:
            has_client = self._client_sock is not None
        if has_client:
            return
        try:
            client, addr = server.accept()
        except TimeoutError:
            return
        except OSError:
            return

        client.setblocking(False)
        with self._lock:
            self._client_sock = client
            self._client_addr = addr
            self._rx.clear()
            self._heartbeat_seq = 0
            self._last_hb_ms = 0
        self._log(f"client connected {addr[0]}:{addr[1]}")

        # Send server handshake first.
        self._send_raw(bcnp.build_handshake(self.schema_hash))

    def _recv_packets(self) -> None:
        sock = None
        with self._lock:
            sock = self._client_sock
        if sock is None:
            return

        try:
            chunk = sock.recv(4096)
        except BlockingIOError:
            return
        except OSError:
            self._disconnect("recv_error")
            return

        if not chunk:
            self._disconnect("remote_closed")
            return

        with self._lock:
            self._rx.extend(chunk)
        self._parse_rx()

    def _parse_rx(self) -> None:
        while True:
            with self._lock:
                if len(self._rx) < bcnp.HANDSHAKE_SIZE:
                    break
                if bcnp.is_handshake_magic(self._rx):
                    if len(self._rx) < bcnp.HANDSHAKE_SIZE:
                        break
                    remote_hash = bcnp.read_handshake_schema_hash(self._rx[: bcnp.HANDSHAKE_SIZE])
                    del self._rx[: bcnp.HANDSHAKE_SIZE]
                    if remote_hash != self.schema_hash:
                        self._log(f"schema mismatch robot=0x{remote_hash:08X} panel=0x{self.schema_hash:08X}")
                    else:
                        self._log("handshake validated")
                    continue
                frame = bytes(self._rx)

            decoded = bcnp.decode_packet(frame, wire_sizes=self.wire_sizes)
            if decoded.error == bcnp.DecodeError.INCOMPLETE:
                break
            if not decoded.is_ok:
                with self._lock:
                    if decoded.consumed_bytes > 0:
                        del self._rx[: decoded.consumed_bytes]
                    else:
                        del self._rx[:1]
                self._log(f"decode error: {decoded.error.value}")
                continue

            with self._lock:
                del self._rx[: decoded.consumed_bytes]
            self._handle_packet(decoded)

    def _handle_packet(self, decoded: bcnp.DecodedPacket) -> None:
        if decoded.message_type == bcnp.MSG_AUTO_PLAN_REQUEST:
            if len(decoded.payload) != self.wire_sizes[bcnp.MSG_AUTO_PLAN_REQUEST]:
                self._log("bad plan request size")
                return
            requested_profile, alliance, _reserved, pose_x_mm, pose_y_mm, heading_mrad = struct.unpack(">HBBiii", decoded.payload)
            self._emit_plan(
                requested_profile=requested_profile,
                alliance=alliance,
                pose_x_mm=pose_x_mm,
                pose_y_mm=pose_y_mm,
                heading_mrad=heading_mrad,
            )
            return

        if decoded.message_type == bcnp.MSG_AUTO_WORLD_UPDATE:
            if len(decoded.payload) == self.wire_sizes[bcnp.MSG_AUTO_WORLD_UPDATE]:
                (
                    fuel_held,
                    last_shot_success,
                    phase_seq_completed,
                    event_flags,
                    robot_vx,
                    robot_vy,
                    pose_x,
                    pose_y,
                    heading_mrad,
                    _reserved,
                ) = struct.unpack(">BBHHhhiihi", decoded.payload)
                with self._lock:
                    self._last_world_update = {
                        "fuelHeld": fuel_held,
                        "shotSuccess": last_shot_success,
                        "phaseSeqCompleted": phase_seq_completed,
                        "eventFlags": event_flags,
                        "vx": robot_vx,
                        "vy": robot_vy,
                        "x": pose_x,
                        "y": pose_y,
                        "heading": heading_mrad,
                    }
            return

    def _emit_plan(self, requested_profile: int, alliance: int, pose_x_mm: int, pose_y_mm: int, heading_mrad: int) -> None:
        with self._lock:
            self._last_plan_id += 1
            plan_id = self._last_plan_id
            script = list(self._script)
            send_shot_hint = self._send_shot_hint
            shot_rpm = self._shot_rpm
            shot_hood = self._shot_hood_permille
            shot_conf = self._shot_conf_permille

        waypoints = self._script_to_waypoints(script, pose_x_mm, pose_y_mm, heading_mrad)
        phase_seq = 3
        phase_count = 5 if waypoints else 4
        plan_checksum = random.getrandbits(32)
        objective_id = 200 + (requested_profile % 100)
        policy_source = 1
        global_conf = 800

        plan_payload = struct.pack(
            ">IHHIHBBHH",
            plan_id & 0xFFFFFFFF,
            0,
            phase_count,
            plan_checksum & 0xFFFFFFFF,
            objective_id & 0xFFFF,
            policy_source & 0xFF,
            0,
            global_conf & 0xFFFF,
            0,
        )
        self._send_packet(bcnp.MSG_AUTO_PLAN_RESPONSE, plan_payload)

        if send_shot_hint:
            hint_payload = struct.pack(
                ">hHHHHHHH",
                0,
                shot_rpm & 0xFFFF,
                shot_hood & 0xFFFF,
                80,
                shot_conf & 0xFFFF,
                3000,
                0,
                0,
            )
            self._send_packet(bcnp.MSG_AUTO_SHOT_HINT, hint_payload)

        for idx, wp in enumerate(waypoints[:MAX_TOTAL_WAYPOINTS]):
            wp_payload = struct.pack(
                ">IHBBiihH",
                plan_id & 0xFFFFFFFF,
                phase_seq & 0xFFFF,
                idx & 0xFF,
                len(waypoints) & 0xFF,
                wp[0],
                wp[1],
                wp[2],
                wp[3] & 0xFFFF,
            )
            self._send_packet(bcnp.MSG_AUTO_WAYPOINT_DELTA, wp_payload)

        self._log(f"plan {plan_id} sent (wps={len(waypoints)} profile={requested_profile} alliance={alliance})")

    def _script_to_waypoints(
        self,
        script: list[tuple[float, float, int]],
        start_x_mm: int,
        start_y_mm: int,
        start_heading_mrad: int,
    ) -> list[tuple[int, int, int, int]]:
        if not script:
            return []
        x = float(start_x_mm)
        y = float(start_y_mm)
        heading = float(start_heading_mrad) / 1000.0
        out: list[tuple[int, int, int, int]] = []

        for vx, omega, duration_ms in script:
            dt = max(0.01, duration_ms / 1000.0)
            vx = _clampf(vx, -5.0, 5.0)
            omega = _clampf(omega, -4.0, 4.0)
            x += math.cos(heading) * vx * dt * 1000.0
            y += math.sin(heading) * vx * dt * 1000.0
            heading += omega * dt

            x_mm = _clamp(int(round(x)), 0, FIELD_LENGTH_MM)
            y_mm = _clamp(int(round(y)), 0, FIELD_WIDTH_MM)
            heading_mrad = _clamp(int(round(heading * 1000.0)), -3142, 3142)
            vel_mm_s = _clamp(int(round(abs(vx) * 1000.0)), 100, MAX_VELOCITY_MM_S)
            out.append((x_mm, y_mm, heading_mrad, vel_mm_s))

            if len(out) >= MAX_WAYPOINTS_PER_PHASE:
                break
        return out

    def _send_heartbeat_if_due(self) -> None:
        now_ms = int(time.time() * 1000)
        if now_ms - self._last_hb_ms < 100:
            return
        with self._lock:
            has_client = self._client_sock is not None
        if not has_client:
            return
        payload = struct.pack(">III", 1, self._heartbeat_seq & 0xFFFFFFFF, now_ms & 0xFFFFFFFF)
        self._send_packet(bcnp.MSG_AUTO_HEARTBEAT, payload)
        self._heartbeat_seq += 1
        self._last_hb_ms = now_ms

    def _send_packet(self, msg_type: int, payload: bytes) -> None:
        frame = bcnp.encode_packet(msg_type, 0, 1, payload)
        self._send_raw(frame)

    def _send_raw(self, frame: bytes) -> None:
        with self._lock:
            sock = self._client_sock
        if sock is None:
            return
        try:
            sock.sendall(frame)
        except OSError:
            self._disconnect("send_error")

    def _disconnect(self, reason: str) -> None:
        with self._lock:
            if self._client_sock is not None:
                try:
                    self._client_sock.close()
                except OSError:
                    pass
            self._client_sock = None
            self._client_addr = None
            self._rx.clear()
        self._log(f"client disconnected ({reason})")


class RobotUI(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("BCNP Planner Control Panel")
        self.geometry("860x840")
        self.minsize(740, 680)

        self.host_var = tk.StringVar(value="0.0.0.0")
        self.port_var = tk.StringVar(value="5801")
        self.schema_var = tk.StringVar(value=str(DEFAULT_SCHEMA))
        self.status_var = tk.StringVar(value="stopped")
        self.peer_var = tk.StringVar(value="none")
        self.world_var = tk.StringVar(value="no world updates")

        self.shot_hint_enabled = tk.BooleanVar(value=False)
        self.shot_rpm = tk.IntVar(value=2600)
        self.shot_hood = tk.IntVar(value=320)
        self.shot_conf = tk.IntVar(value=800)

        self.vx_var = tk.DoubleVar(value=0.8)
        self.omega_var = tk.DoubleVar(value=0.0)
        self.dur_var = tk.IntVar(value=200)
        self.clear_queue_flag = tk.BooleanVar(value=True)

        self.cont_running = False
        self.cont_thread: threading.Thread | None = None
        self.cont_vx = tk.DoubleVar(value=0.5)
        self.cont_omega = tk.DoubleVar(value=0.0)
        self.cont_duration = tk.IntVar(value=200)
        self.cont_period_ms = tk.IntVar(value=100)

        self.server: PlannerSimulator | None = None

        self._build_connection_panel()
        self._build_plan_controls()
        self._build_batch_panel()
        self._build_continuous_panel()
        self._build_log_panel()

        self.after(100, self._ui_tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_connection_panel(self) -> None:
        frm = ttk.LabelFrame(self, text="Planner Server")
        frm.pack(fill="x", padx=10, pady=8)

        ttk.Label(frm, text="Host").grid(row=0, column=0, padx=6, pady=6, sticky="w")
        ttk.Entry(frm, textvariable=self.host_var, width=16).grid(row=0, column=1, padx=6, pady=6, sticky="w")
        ttk.Label(frm, text="Port").grid(row=0, column=2, padx=6, pady=6, sticky="w")
        ttk.Entry(frm, textvariable=self.port_var, width=8).grid(row=0, column=3, padx=6, pady=6, sticky="w")
        ttk.Label(frm, text="Schema").grid(row=1, column=0, padx=6, pady=6, sticky="w")
        ttk.Entry(frm, textvariable=self.schema_var, width=52).grid(row=1, column=1, columnspan=3, padx=6, pady=6, sticky="we")

        ttk.Button(frm, text="Start", command=self._start_server).grid(row=0, column=4, padx=6)
        ttk.Button(frm, text="Stop", command=self._stop_server).grid(row=1, column=4, padx=6)

        ttk.Label(frm, text="Status:").grid(row=0, column=5, padx=(20, 4), sticky="e")
        ttk.Label(frm, textvariable=self.status_var).grid(row=0, column=6, sticky="w")
        ttk.Label(frm, text="Robot:").grid(row=1, column=5, padx=(20, 4), sticky="e")
        ttk.Label(frm, textvariable=self.peer_var).grid(row=1, column=6, sticky="w")

    def _build_plan_controls(self) -> None:
        frm = ttk.LabelFrame(self, text="Plan/Hint Controls")
        frm.pack(fill="x", padx=10, pady=8)
        ttk.Checkbutton(frm, text="Emit AutoShotHint", variable=self.shot_hint_enabled).grid(row=0, column=0, padx=6, pady=6, sticky="w")
        ttk.Label(frm, text="RPM").grid(row=0, column=1, padx=6, sticky="e")
        ttk.Entry(frm, textvariable=self.shot_rpm, width=8).grid(row=0, column=2, padx=6, sticky="w")
        ttk.Label(frm, text="Hood permille").grid(row=0, column=3, padx=6, sticky="e")
        ttk.Entry(frm, textvariable=self.shot_hood, width=8).grid(row=0, column=4, padx=6, sticky="w")
        ttk.Label(frm, text="Confidence").grid(row=0, column=5, padx=6, sticky="e")
        ttk.Entry(frm, textvariable=self.shot_conf, width=8).grid(row=0, column=6, padx=6, sticky="w")
        ttk.Label(frm, textvariable=self.world_var).grid(row=1, column=0, columnspan=7, padx=6, pady=(0, 6), sticky="w")

    def _build_batch_panel(self) -> None:
        frm = ttk.LabelFrame(self, text="Batch Commands (vx, omega, duration)")
        frm.pack(fill="both", expand=True, padx=10, pady=8)

        inputs = ttk.Frame(frm)
        inputs.pack(fill="x", padx=6, pady=6)
        ttk.Label(inputs, text="vx (m/s)").pack(side="left")
        ttk.Entry(inputs, textvariable=self.vx_var, width=8).pack(side="left", padx=6)
        ttk.Label(inputs, text="omega (rad/s)").pack(side="left")
        ttk.Entry(inputs, textvariable=self.omega_var, width=8).pack(side="left", padx=6)
        ttk.Label(inputs, text="duration (ms)").pack(side="left")
        ttk.Entry(inputs, textvariable=self.dur_var, width=8).pack(side="left", padx=6)
        ttk.Button(inputs, text="Add", command=self._add_cmd).pack(side="left", padx=6)

        btns = ttk.Frame(frm)
        btns.pack(fill="x", padx=6, pady=6)
        ttk.Button(btns, text="Remove Selected", command=self._remove_selected).pack(side="left", padx=4)
        ttk.Button(btns, text="Clear List", command=self._clear_list).pack(side="left", padx=4)
        ttk.Checkbutton(btns, text="Clear Queue (UI only)", variable=self.clear_queue_flag).pack(side="left", padx=12)
        ttk.Button(btns, text="Apply Script", command=self._apply_script).pack(side="right", padx=4)

        listfrm = ttk.Frame(frm)
        listfrm.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        self.queue_list = tk.Listbox(listfrm, height=12)
        self.queue_list.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(listfrm, orient="vertical", command=self.queue_list.yview)
        sb.pack(side="right", fill="y")
        self.queue_list.configure(yscrollcommand=sb.set)

        presets = ttk.Frame(frm)
        presets.pack(fill="x", padx=6, pady=6)
        ttk.Label(presets, text="Presets:").pack(side="left")
        ttk.Button(presets, text="Square", command=self._preset_square).pack(side="left", padx=4)
        ttk.Button(presets, text="Smooth Curve", command=self._preset_curve).pack(side="left", padx=4)
        ttk.Button(presets, text="Stress (10k)", command=self._preset_stress).pack(side="left", padx=4)
        ttk.Button(presets, text="Random (30k)", command=self._preset_random).pack(side="left", padx=4)

    def _build_continuous_panel(self) -> None:
        frm = ttk.LabelFrame(self, text="Continuous Control (updates script)")
        frm.pack(fill="x", padx=10, pady=8)

        ttk.Label(frm, text="vx").grid(row=0, column=0, padx=6, pady=6)
        ttk.Entry(frm, textvariable=self.cont_vx, width=8).grid(row=0, column=1, padx=6, pady=6)
        ttk.Label(frm, text="omega").grid(row=0, column=2, padx=6, pady=6)
        ttk.Entry(frm, textvariable=self.cont_omega, width=8).grid(row=0, column=3, padx=6, pady=6)
        ttk.Label(frm, text="duration ms").grid(row=0, column=4, padx=6, pady=6)
        ttk.Entry(frm, textvariable=self.cont_duration, width=8).grid(row=0, column=5, padx=6, pady=6)
        ttk.Label(frm, text="period ms").grid(row=0, column=6, padx=6, pady=6)
        ttk.Entry(frm, textvariable=self.cont_period_ms, width=8).grid(row=0, column=7, padx=6, pady=6)
        ttk.Button(frm, text="Start", command=self._start_continuous).grid(row=0, column=8, padx=6)
        ttk.Button(frm, text="Stop", command=self._stop_continuous).grid(row=0, column=9, padx=6)

    def _build_log_panel(self) -> None:
        frm = ttk.LabelFrame(self, text="Events")
        frm.pack(fill="both", expand=True, padx=10, pady=8)
        self.log = tk.Text(frm, height=8, wrap="word")
        self.log.pack(fill="both", expand=True, padx=6, pady=6)

    def _add_cmd(self) -> None:
        try:
            vx = float(self.vx_var.get())
            omega = float(self.omega_var.get())
            dur = int(self.dur_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "vx/omega must be numeric and duration must be integer.")
            return
        self.queue_list.insert(tk.END, f"{vx:.3f},{omega:.3f},{dur}")

    def _remove_selected(self) -> None:
        sel = list(self.queue_list.curselection())
        for idx in reversed(sel):
            self.queue_list.delete(idx)

    def _clear_list(self) -> None:
        self.queue_list.delete(0, tk.END)

    def _collect_script(self) -> list[tuple[float, float, int]]:
        out: list[tuple[float, float, int]] = []
        for i in range(self.queue_list.size()):
            row = self.queue_list.get(i).split(",")
            try:
                out.append((float(row[0]), float(row[1]), int(row[2])))
            except (ValueError, IndexError):
                continue
        return out

    def _apply_script(self) -> None:
        if self.server is None:
            messagebox.showwarning("Server not running", "Start the planner server first.")
            return
        script = self._collect_script()
        self.server.set_script(script)
        self._append_log(f"script applied ({len(script)} cmds)")

    def _preset_square(self) -> None:
        seq = [
            (1.0, 0.0, 1500),
            (0.0, 1.2, 900),
            (1.0, 0.0, 1500),
            (0.0, 1.2, 900),
            (1.0, 0.0, 1500),
            (0.0, 1.2, 900),
            (1.0, 0.0, 1500),
            (0.0, 1.2, 900),
            (0.0, 0.0, 100),
        ]
        for vx, om, dur in seq:
            self.queue_list.insert(tk.END, f"{vx:.3f},{om:.3f},{dur}")

    def _preset_curve(self) -> None:
        for i in range(24):
            t = i / 24.0
            self.queue_list.insert(tk.END, f"0.800,{(1.0 * (1.0 - t)):.3f},200")
        self.queue_list.insert(tk.END, "0.000,0.000,100")

    def _preset_stress(self) -> None:
        if not messagebox.askyesno("Stress", "Add 10,000 commands?"):
            return
        for i in range(10000):
            t = i * 0.01
            vx = 0.5 + 0.5 * math.sin(t)
            om = math.cos(t * 0.5)
            self.queue_list.insert(tk.END, f"{vx:.3f},{om:.3f},10")

    def _preset_random(self) -> None:
        if not messagebox.askyesno("Random", "Add 30,000 random commands?"):
            return
        for _ in range(30000):
            vx = (random.random() * 10.0) - 5.0
            om = (random.random() * 8.0) - 4.0
            self.queue_list.insert(tk.END, f"{vx:.3f},{om:.3f},20")

    def _start_continuous(self) -> None:
        if self.cont_running:
            return
        self.cont_running = True
        self.cont_thread = threading.Thread(target=self._continuous_loop, daemon=True)
        self.cont_thread.start()

    def _stop_continuous(self) -> None:
        self.cont_running = False

    def _continuous_loop(self) -> None:
        while self.cont_running:
            if self.server is not None:
                self.server.set_script([(float(self.cont_vx.get()), float(self.cont_omega.get()), int(self.cont_duration.get()))])
            time.sleep(max(0.02, int(self.cont_period_ms.get()) / 1000.0))

    def _start_server(self) -> None:
        if self.server is not None:
            self._append_log("server already running")
            return
        try:
            schema_path = Path(self.schema_var.get())
            host = self.host_var.get().strip()
            port = int(self.port_var.get())
            self.server = PlannerSimulator(host=host, port=port, schema_path=schema_path)
            self.server.start()
            self.status_var.set("running")
            self._append_log(f"server started on {host}:{port}")
        except Exception as exc:
            messagebox.showerror("Start failed", str(exc))
            self.server = None
            self.status_var.set("stopped")

    def _stop_server(self) -> None:
        if self.server is None:
            return
        self.server.stop()
        self.server = None
        self.status_var.set("stopped")
        self.peer_var.set("none")
        self._append_log("server stopped")

    def _append_log(self, line: str) -> None:
        self.log.insert(tk.END, line + "\n")
        self.log.see(tk.END)

    def _ui_tick(self) -> None:
        if self.server is not None:
            self.server.set_shot_hint(
                enabled=bool(self.shot_hint_enabled.get()),
                rpm=int(self.shot_rpm.get()),
                hood_permille=int(self.shot_hood.get()),
                confidence_permille=int(self.shot_conf.get()),
            )
            self.peer_var.set(self.server.client_peer())
            world = self.server.last_world()
            if world:
                self.world_var.set(
                    f"world fuel={world.get('fuelHeld', 0)} phase={world.get('phaseSeqCompleted', 0)} "
                    f"pose=({world.get('x', 0)},{world.get('y', 0)})"
                )
            for event in self.server.pop_events():
                self._append_log(event)
        else:
            self.peer_var.set("none")
            self.world_var.set("no world updates")
        self.after(100, self._ui_tick)

    def _on_close(self) -> None:
        self._stop_continuous()
        self._stop_server()
        self.destroy()


if __name__ == "__main__":
    app = RobotUI()
    app.mainloop()
