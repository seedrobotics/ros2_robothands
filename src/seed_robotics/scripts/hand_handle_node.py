#!/usr/bin/env python3

# ROS 2 Jazzy port of the RH8D hand controller.
# Original ROS 1 author: Lucas Combe
# Ported to ROS 2 rclpy with timer-based main loop.

import math
import time

import rclpy
from rclpy.node import Node

from dynamixel_sdk import (
    PortHandler, PacketHandler,
    GroupSyncRead, GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD,
)
from seed_robotics.msg import (
    AllJoints, AllMainBoards,
    LoneJoint, LoneMainBoard,
    JointListSetSpeedPos,
    ClearHWError,
    JointListSetStiffness,
    SetShutdownCond,
)

# ── Control-table addresses ───────────────────────────────────────────────────
ADDR_FIRMWARE_VERSION       = 2
ADDR_BUS_ID                 = 3
ADDR_D                      = 26
ADDR_I                      = 27
ADDR_P                      = 28
ADDR_OL_FILTER_THRESHOLD    = 66
ADDR_GEARTRAIN_MODEL        = 102
ADDR_ELECTRONICS_MODEL      = 103
ADDR_PERMISSION             = 23
LEN_PERMISSION              = 1
PERMISSION_ENABLE           = 0
ADDR_TORQUE_ENABLE          = 24
ADDR_TARGET_POSITION        = 30
LEN_TARGET_POSITION         = 2
ADDR_TARGET_SPEED           = 32
LEN_TARGET_SPEED            = 2
ADDR_TORQUE_LIMIT           = 34
LEN_TORQUE_LIMIT            = 2
ADDR_PRESENT_POSITION       = 36
LEN_PRESENT_POSITION        = 2
ADDR_PRESENT_SPEED          = 38
LEN_PRESENT_SPEED           = 2
ADDR_TEMPERATURE            = 43
LEN_TEMPERATURE             = 1
ADDR_MOVING                 = 46
LEN_MOVING                  = 1
ADDR_HW_ERROR_COND          = 47
LEN_HW_ERROR_COND           = 1
ADDR_OL_FILTER_VALUE        = 67
LEN_OL_FILTER_VALUE         = 1
ADDR_SHUTDOWN_COND          = 18
LEN_SHUTDOWN_COND           = 1
DEFAULT_SHUTDOWN_COND       = 36
DISABLE_TEMP_COND           = 32
DISABLE_OL_COND             = 4
DISABLE_OL_AND_TEMP         = 0
ADDR_CURRENT                = 68
LEN_CURRENT                 = 2
ADDR_CURRENT_LIMIT          = 71

ADDR_PALM_IR_SENSOR         = 94
LEN_PALM_IR_SENSOR          = 2
ADDR_CAPACITIVE_SENSOR_1    = 98
ADDR_CAPACITIVE_SENSOR_2    = 100
LEN_CAPACITIVE_SENSOR       = 2

ADDR_START_SYNC_READ        = 30
LEN_SYNC_READ_FULL          = 40   # normal mode: 40 bytes from addr 30
LEN_SYNC_READ_LIGHT         = 10   # light mode:  10 bytes from addr 30

ADDR_START_SYNC_WRITE       = 30
LEN_SYNC_WRITE              = 4

ADDR_SYNC_WRITE_STIFFNESS   = 26
LEN_SYNC_WRITE_STIFFNESS    = 3

ADDR_START_SYNC_READ_MB     = 94
LEN_SYNC_READ_MB            = 8

PROTOCOL_VERSION            = 2.0
TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0


# ── Helper data classes ───────────────────────────────────────────────────────

class _Security:
    def __init__(self, dxl_id: int):
        self.id = dxl_id
        self.time_last_clear: float | None = None
        self.time_last_set_stiffness: float | None = None


class _Flags:
    def __init__(self):
        self.WRITE_SPEED_POS     = False
        self.WRITE_CLEAR_ERROR   = False
        self.WRITE_STIFFNESS     = False
        self.FORBIDDEN_STIFFNESS = False
        self.WRITE_SHUTDOWN_COND = False


class _Info:
    def __init__(self, model_number: int = 0, dxl_id: int = -1):
        self.model_number    = model_number
        self.id              = dxl_id
        self.firmware        = 0
        self.bus_id          = -1
        self.geartrain_model = None
        self.electronics_model = None
        self.P               = None
        self.I               = None
        self.D               = None
        self.OL_filt_threshold = None
        self.current_limit   = None

    def log(self, logger):
        logger.info('  model=%d  fw=%d  bus_id=%d  geartrain=%s  electronics=%s' % (
            self.model_number, self.firmware, self.bus_id,
            self.geartrain_model, self.electronics_model))


class _Joint:
    def __init__(self, info: _Info, joint_dict: dict, logger):
        self.info   = info
        self.logger = logger
        try:
            self.name = [n for n, v in joint_dict.items() if v == info.id][0]
        except IndexError:
            self.name = 'None'
            logger.warn('Joint ID %d has no mapping in YAML, name set to "None"' % info.id)
        self.bus_id               = 0
        self.stiffness            = 8
        self.stress_level         = 0
        self.target_pos           = 0
        self.target_speed         = 0
        self.torque_limit         = 0
        self.pres_pos             = 0
        self.pres_speed           = 0
        self.temperature          = 0
        self.moving               = 0
        self.HW_err_cond          = 0
        self.overload_filter_value = 0
        self.pres_current         = 0

    def print_info(self):
        self.logger.info('Joint: %s (ID %d)' % (self.name, self.info.id))
        self.info.log(self.logger)

    def set_stress_level(self, ol_threshold, current_limit):
        if ol_threshold and current_limit:
            tmp = (self.overload_filter_value / (ol_threshold * 0.8) +
                   self.pres_current / (current_limit * 0.9))
            self.stress_level = int(tmp / 2)


class _MainBoard:
    def __init__(self, bus_id: int, joint_dict: dict, logger):
        self.id = bus_id
        try:
            self.name = [n for n, v in joint_dict.items() if v == bus_id][0]
        except IndexError:
            self.name = 'None'
            logger.warn('Main board ID %d has no mapping in YAML' % bus_id)
        self.palm_IR_sensor      = 0
        self.capacitive_sensor_1 = 0
        self.capacitive_sensor_2 = 0


# ── Node ─────────────────────────────────────────────────────────────────────

class HandControllerNode(Node):

    def __init__(self):
        super().__init__(
            'seed_hand',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self._load_parameters()
        self._open_port()
        self._create_sync_objects()
        self._setup_ros_interfaces()
        self._scan_and_init_joints()
        period = 1.0 / self.FREQUENCY
        self._timer = self.create_timer(period, self._main_loop)

    # ── Parameter loading ─────────────────────────────────────────────────────

    def _load_parameters(self):
        def _get(name, default):
            if self.has_parameter(name):
                return self.get_parameter(name).value
            self.declare_parameter(name, default)
            return default

        self.LIGHT_MODE  = _get('light_mode', False)
        self.PREFIX      = _get('prefix', 'R_')
        self.BAUDRATE    = _get('baudrate', 1000000)
        self.DEVICENAME  = _get('port', '/dev/ttyUSB0')
        self.FREQUENCY   = _get('frequency', 20)

        joint_params = self.get_parameters_by_prefix('joint_mapping')
        self.joint_dict = {k: v.value for k, v in joint_params.items()}

        if not self.joint_dict:
            self.get_logger().error(
                'joint_mapping not found in parameters — '
                'the node will only work with numeric joint IDs')
        else:
            self.get_logger().info('joint_mapping: %s' % str(self.joint_dict))

        self.get_logger().info(
            'baudrate=%d  port=%s  freq=%d  light_mode=%s  prefix="%s"' %
            (self.BAUDRATE, self.DEVICENAME, self.FREQUENCY,
             self.LIGHT_MODE, self.PREFIX))

        self._LEN_SYNC_READ = LEN_SYNC_READ_LIGHT if self.LIGHT_MODE else LEN_SYNC_READ_FULL
        self.PERIOD_MS = 1000.0 / self.FREQUENCY

    # ── Serial port ───────────────────────────────────────────────────────────

    def _open_port(self):
        self._port   = PortHandler(self.DEVICENAME)
        self._packet = PacketHandler(PROTOCOL_VERSION)
        if not self._port.openPort():
            self.get_logger().fatal('Failed to open port %s' % self.DEVICENAME)
            raise SystemExit(1)
        self.get_logger().info('Port opened: %s' % self.DEVICENAME)
        if not self._port.setBaudRate(self.BAUDRATE):
            self.get_logger().fatal('Failed to set baudrate %d' % self.BAUDRATE)
            raise SystemExit(1)
        self.get_logger().info('Baudrate set: %d' % self.BAUDRATE)

    # ── Dynamixel group-sync objects ──────────────────────────────────────────

    def _create_sync_objects(self):
        ph, pk = self._port, self._packet
        self._gsr       = GroupSyncRead(ph, pk, ADDR_START_SYNC_READ, self._LEN_SYNC_READ)
        self._gsr_mb    = GroupSyncRead(ph, pk, ADDR_START_SYNC_READ_MB, LEN_SYNC_READ_MB)
        self._gsw_sp    = GroupSyncWrite(ph, pk, ADDR_START_SYNC_WRITE, LEN_SYNC_WRITE)
        self._gsw_err   = GroupSyncWrite(ph, pk, ADDR_HW_ERROR_COND, LEN_HW_ERROR_COND)
        self._gsw_stiff = GroupSyncWrite(ph, pk, ADDR_SYNC_WRITE_STIFFNESS, LEN_SYNC_WRITE_STIFFNESS)
        self._gsw_perm  = GroupSyncWrite(ph, pk, ADDR_PERMISSION, LEN_PERMISSION)
        self._gsw_shut  = GroupSyncWrite(ph, pk, ADDR_SHUTDOWN_COND, LEN_SHUTDOWN_COND)

    # ── ROS publishers / subscribers ─────────────────────────────────────────

    def _setup_ros_interfaces(self):
        P = self.PREFIX
        self._pub_joints = self.create_publisher(AllJoints,    P + 'Joints',      10)
        self._pub_mb     = self.create_publisher(AllMainBoards, P + 'Main_Boards', 10)
        self.create_subscription(JointListSetSpeedPos, P + 'speed_position',   self._cb_speed_pos,    10)
        self.create_subscription(ClearHWError,         P + 'clear_error',      self._cb_clear_error,  10)
        self.create_subscription(JointListSetStiffness, P + 'stiffness',       self._cb_stiffness,    10)
        self.create_subscription(SetShutdownCond,      P + 'shutdown_condition', self._cb_shutdown,   10)

        self._flags = _Flags()
        self._sp_ids:    list = []
        self._sp_params: list = []
        self._stiff_ids:   list = []
        self._stiff_params: list = []
        self._perm_ids:    list = []
        self._perm_params: list = []

    # ── Bus scan and joint initialisation ────────────────────────────────────

    def _scan_and_init_joints(self):
        self._joint_ids:     list[int]    = []
        self._joint_models:  list[int]    = []
        self._mb_ids:        list[int]    = []
        self._security:      list[_Security] = []
        self.infos:          list[_Info]  = []
        self.joints:         list[_Joint] = []
        self.main_boards:    list[_MainBoard] = []

        dict_ids = list(self.joint_dict.values())

        self.get_logger().info('Scanning bus (IDs 0–252)…')
        for dxl_id in range(253):
            model, res, err = self._packet.ping(self._port, dxl_id)
            if res == COMM_SUCCESS and err == 0:
                self.get_logger().info('[ID:%03d] model=%d' % (dxl_id, model))
                if model != 405:
                    self._joint_ids.append(dxl_id)
                    self._joint_models.append(model)
                    self._security.append(_Security(dxl_id))
                else:
                    self._mb_ids.append(dxl_id)

        found = set(self._joint_ids + self._mb_ids)
        if found != set(dict_ids):
            self.get_logger().warn('Bus scan did not find every joint in the YAML dictionary')
        else:
            self.get_logger().info('Bus scan went well, every joint in YAML dictionary was found')

        # Read detailed info and enable torque
        for idx, dxl_id in enumerate(self._joint_ids):
            info = _Info(self._joint_models[idx], dxl_id)
            info.firmware          = self._r1(dxl_id, ADDR_FIRMWARE_VERSION)
            info.D                 = self._r1(dxl_id, ADDR_D)
            info.I                 = self._r1(dxl_id, ADDR_I)
            info.P                 = self._r1(dxl_id, ADDR_P)
            info.bus_id            = self._r1(dxl_id, ADDR_BUS_ID)
            info.OL_filt_threshold = self._r1(dxl_id, ADDR_OL_FILTER_THRESHOLD)
            info.geartrain_model   = self._r1(dxl_id, ADDR_GEARTRAIN_MODEL)
            info.electronics_model = self._r1(dxl_id, ADDR_ELECTRONICS_MODEL)

            perm = self._r1(dxl_id, ADDR_PERMISSION)
            if perm == 0:
                self._flags.FORBIDDEN_STIFFNESS = True

            cl, res, err = self._packet.read2ByteTxRx(self._port, dxl_id, ADDR_CURRENT_LIMIT)
            if res == COMM_SUCCESS and err == 0:
                info.current_limit = cl

            res, err = self._packet.write1ByteTxRx(
                self._port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if res == COMM_SUCCESS and err == 0:
                self.get_logger().info('Dynamixel#%d torque enabled' % dxl_id)
            else:
                self.get_logger().warn('Torque enable failed on ID %d' % dxl_id)

            if not self._gsr.addParam(dxl_id):
                self.get_logger().error('[ID:%03d] groupSyncRead addParam failed' % dxl_id)
                raise SystemExit(1)

            self.infos.append(info)

        if self._flags.FORBIDDEN_STIFFNESS:
            self.get_logger().warn(
                'Package started after a stiffness change. '
                'Stiffness commands will be ignored until next power cycle.')

        # Main boards
        self.get_logger().info('Main board IDs: %s' % str(self._mb_ids))
        for dxl_id in self._mb_ids:
            self.main_boards.append(_MainBoard(dxl_id, self.joint_dict, self.get_logger()))
            if not self._gsr_mb.addParam(dxl_id):
                self.get_logger().error('[ID:%03d] groupSyncReadMainBoard addParam failed' % dxl_id)
                raise SystemExit(1)

        # Build Joint list from Info list
        for info in self.infos:
            j = _Joint(info, self.joint_dict, self.get_logger())
            j.bus_id = info.bus_id
            self.joints.append(j)
            j.print_info()

        # Pre-allocate ROS message objects (reused every cycle to avoid GC pressure)
        self._lone_joints   = [LoneJoint()     for _ in self._joint_ids]
        self._all_joints    = AllJoints()
        self._lone_mbs      = [LoneMainBoard() for _ in self._mb_ids]
        self._all_mbs       = AllMainBoards()

    # ── Low-level read helpers ────────────────────────────────────────────────

    def _r1(self, dxl_id: int, addr: int):
        """Read 1 byte; returns value or None on error."""
        val, res, err = self._packet.read1ByteTxRx(self._port, dxl_id, addr)
        if res != COMM_SUCCESS:
            self.get_logger().warn('r1 [ID:%d addr:%d]: %s' %
                                   (dxl_id, addr, self._packet.getTxRxResult(res)))
            return None
        if err != 0:
            self.get_logger().warn('r1 [ID:%d addr:%d] err: %s' %
                                   (dxl_id, addr, self._packet.getRxPacketError(err)))
            return None
        return val

    # ── Name → ID helper ─────────────────────────────────────────────────────

    def _id(self, name: str):
        if name.isnumeric():
            return int(name)
        try:
            return self.joint_dict[name]
        except KeyError:
            self.get_logger().warn(
                'No mapping for joint "%s". '
                'Ignore if running 2 hands on different ports.' % name)
            return 'None'

    # ── Stiffness → PID helper ────────────────────────────────────────────────

    def _pid_from_stiffness(self, stiffness: int, dxl_id: int):
        matches = [j for j in self.joints if j.bus_id == dxl_id]
        if not matches:
            return None, None, None
        j = matches[0]
        if   stiffness == 1: P = math.ceil(j.info.P / 2)
        elif stiffness == 8: P = j.info.P
        elif 1 < stiffness < 8: P = math.ceil(1.42 * stiffness + 8.5)
        elif stiffness == 9: P = math.ceil(25 * stiffness - 180)
        else: return None, None, None
        return P, j.info.I, j.info.D

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _cb_clear_error(self, msg: ClearHWError):
        self._gsw_err.clearParam()
        dxl_id = self._id(msg.name)
        if dxl_id == 'None':
            return
        now = time.time_ns() / 1e9
        for sec in self._security:
            if sec.id == dxl_id:
                if sec.time_last_clear is not None and now - sec.time_last_clear < 30:
                    self.get_logger().warn('Clear error on "%s" too frequent — discarded' % msg.name)
                    return
                sec.time_last_clear = now
        if not self._gsw_err.addParam(dxl_id, [DXL_LOBYTE(DXL_LOWORD(0))]):
            self.get_logger().error('gsw_err addParam failed for "%s"' % msg.name)
            return
        self._flags.WRITE_CLEAR_ERROR = True

    def _cb_shutdown(self, msg: SetShutdownCond):
        self._gsw_shut.clearParam()
        dxl_id = self._id(msg.name)
        if dxl_id == 'None':
            return
        if   msg.temperature and     msg.overload: val = DEFAULT_SHUTDOWN_COND
        elif not msg.temperature and msg.overload:  val = DISABLE_TEMP_COND
        elif msg.temperature and not msg.overload:  val = DISABLE_OL_COND
        else:                                        val = DISABLE_OL_AND_TEMP
        if not self._gsw_shut.addParam(dxl_id, [DXL_LOBYTE(DXL_LOWORD(val))]):
            self.get_logger().error('gsw_shut addParam failed for "%s"' % msg.name)
            return
        self._flags.WRITE_SHUTDOWN_COND = True

    def _cb_stiffness(self, msg: JointListSetStiffness):
        if self._flags.FORBIDDEN_STIFFNESS:
            self.get_logger().warn(
                'Stiffness commands ignored (package started after a stiffness change). '
                'Power cycle the unit to restore.')
            return
        self._gsw_stiff.clearParam()
        self._gsw_perm.clearParam()
        self._stiff_ids.clear(); self._stiff_params.clear()
        self._perm_ids.clear();  self._perm_params.clear()

        now = time.time_ns() / 1e9
        for joint in msg.joints:
            dxl_id = self._id(joint.name)
            if dxl_id == 'None':
                return
            for sec in self._security:
                if sec.id == dxl_id:
                    if sec.time_last_set_stiffness is not None and \
                            now - sec.time_last_set_stiffness < 30:
                        self.get_logger().warn('Set stiffness on "%s" too frequent — discarded' % joint.name)
                        return
                    sec.time_last_set_stiffness = now
            stiffness = joint.stiffness
            if not (1 <= stiffness <= 9):
                self.get_logger().warn('Stiffness %d out of range [1–9]' % stiffness)
                return
            P, I, D = self._pid_from_stiffness(stiffness, dxl_id)
            if P is None:
                return
            self._stiff_ids.append(dxl_id)
            self._stiff_params.append([DXL_LOBYTE(DXL_LOWORD(D)),
                                        DXL_LOBYTE(DXL_LOWORD(I)),
                                        DXL_LOBYTE(DXL_LOWORD(P))])
            self._perm_ids.append(dxl_id)
            self._perm_params.append([DXL_LOBYTE(DXL_LOWORD(PERMISSION_ENABLE))])
            for idx, j in enumerate(self.joints):
                if j.name == joint.name:
                    self.joints[idx].stiffness = stiffness

        self.get_logger().info('Stiffness write IDs: %s' % str(self._stiff_ids))
        for idx, dxl_id in enumerate(self._stiff_ids):
            if not self._gsw_stiff.addParam(dxl_id, self._stiff_params[idx]):
                self.get_logger().error('[ID:%d] gsw_stiff addParam failed' % dxl_id)
                return
        for idx, dxl_id in enumerate(self._perm_ids):
            if not self._gsw_perm.addParam(dxl_id, self._perm_params[idx]):
                self.get_logger().error('[ID:%d] gsw_perm addParam failed' % dxl_id)
                return
        self._flags.WRITE_STIFFNESS = True

    def _cb_speed_pos(self, msg: JointListSetSpeedPos):
        self._gsw_sp.clearParam()
        self._sp_ids.clear(); self._sp_params.clear()

        for joint in msg.joints:
            dxl_id = self._id(joint.name)
            self.get_logger().info('speed_pos: "%s" → ID %s' % (joint.name, dxl_id))
            if dxl_id == 'None':
                return
            self._sp_ids.append(dxl_id)
            target_pos = joint.target_pos
            if joint.target_speed < 0:
                # Keep the last known speed from the joint state
                target_speed = 0
                for lj in self._lone_joints:
                    if lj.bus_id == dxl_id:
                        target_speed = lj.target_speed
                        break
            else:
                target_speed = joint.target_speed
            self._sp_params.append([
                DXL_LOBYTE(DXL_LOWORD(target_pos)),
                DXL_HIBYTE(DXL_LOWORD(target_pos)),
                DXL_LOBYTE(DXL_LOWORD(target_speed)),
                DXL_HIBYTE(DXL_LOWORD(target_speed)),
            ])

        for idx, dxl_id in enumerate(self._sp_ids):
            if not self._gsw_sp.addParam(dxl_id, self._sp_params[idx]):
                self.get_logger().error('[ID:%d] gsw_sp addParam failed' % dxl_id)
                return
        self._flags.WRITE_SPEED_POS = True

    # ── Message fill helpers ──────────────────────────────────────────────────

    def _fill_joints_msg(self):
        for idx, lj in enumerate(self._lone_joints):
            j = self.joints[idx]
            lj.name             = j.name
            lj.bus_id           = j.bus_id
            lj.stiffness        = j.stiffness
            lj.target_position  = j.target_pos
            lj.target_speed     = j.target_speed
            lj.torque_limit     = j.torque_limit
            lj.present_position = j.pres_pos
            lj.present_speed    = j.pres_speed
            lj.temperature      = j.temperature
            lj.hw_error_condition = j.HW_err_cond
            if not self.LIGHT_MODE:
                lj.stress_level = j.stress_level
                lj.moving       = j.moving
                lj.current      = j.pres_current
        self._all_joints.header.stamp = self.get_clock().now().to_msg()
        self._all_joints.length       = len(self._joint_ids)
        self._all_joints.joints       = self._lone_joints

    def _fill_mb_msg(self):
        for idx, lmb in enumerate(self._lone_mbs):
            mb = self.main_boards[idx]
            lmb.name               = mb.name
            lmb.id                 = mb.id
            lmb.palm_ir_sensor      = mb.palm_IR_sensor
            lmb.capacitive_sensor_1 = mb.capacitive_sensor_1
            lmb.capacitive_sensor_2 = mb.capacitive_sensor_2
        self._all_mbs.header.stamp = self.get_clock().now().to_msg()
        self._all_mbs.length       = len(self._mb_ids)
        self._all_mbs.boards       = self._lone_mbs

    # ── Main loop (timer callback — fires at 1/FREQUENCY Hz) ─────────────────

    def _main_loop(self):
        t0  = time.time_ns() / 1_000_000   # ms
        t_end = t0 + self.PERIOD_MS

        # ── 1. Sync-read all joints ──────────────────────────────────────────
        res = self._gsr.txRxPacket()
        if res != COMM_SUCCESS:
            self.get_logger().warn('SyncRead joints: %s' % self._packet.getTxRxResult(res))

        for joint, dxl_id in zip(self.joints, self._joint_ids):
            if not self._gsr.isAvailable(dxl_id, ADDR_START_SYNC_READ, self._LEN_SYNC_READ):
                self.get_logger().error('[ID:%d] SyncRead isAvailable failed' % dxl_id)
                break
            joint.target_pos  = self._gsr.getData(dxl_id, ADDR_TARGET_POSITION, LEN_TARGET_POSITION)
            joint.target_speed = self._gsr.getData(dxl_id, ADDR_TARGET_SPEED,    LEN_TARGET_SPEED)
            joint.torque_limit = self._gsr.getData(dxl_id, ADDR_TORQUE_LIMIT,    LEN_TORQUE_LIMIT)
            joint.pres_pos    = self._gsr.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            joint.pres_speed  = self._gsr.getData(dxl_id, ADDR_PRESENT_SPEED,    LEN_PRESENT_SPEED)
            if not self.LIGHT_MODE:
                joint.temperature           = self._gsr.getData(dxl_id, ADDR_TEMPERATURE,    LEN_TEMPERATURE)
                joint.moving                = self._gsr.getData(dxl_id, ADDR_MOVING,          LEN_MOVING)
                joint.HW_err_cond           = self._gsr.getData(dxl_id, ADDR_HW_ERROR_COND,   LEN_HW_ERROR_COND)
                joint.overload_filter_value = self._gsr.getData(dxl_id, ADDR_OL_FILTER_VALUE, LEN_OL_FILTER_VALUE)
                joint.pres_current          = self._gsr.getData(dxl_id, ADDR_CURRENT,          LEN_CURRENT)
                joint.set_stress_level(joint.info.OL_filt_threshold, joint.info.current_limit)
                if joint.HW_err_cond != 0:
                    self.get_logger().warn('Joint "%s" HW error: %d' % (joint.name, joint.HW_err_cond))

        # ── 2. Publish joint states ──────────────────────────────────────────
        self._fill_joints_msg()
        self._pub_joints.publish(self._all_joints)

        if self.LIGHT_MODE and time.time_ns() / 1_000_000 > t_end:
            self.get_logger().warn('TIME PERIOD EXCEEDED by %d ms' %
                                   (time.time_ns() / 1_000_000 - t_end))

        # ── 3. Priority-ordered writes ───────────────────────────────────────
        if self._flags.WRITE_CLEAR_ERROR:
            res = self._gsw_err.txPacket()
            if res != COMM_SUCCESS:
                self.get_logger().warn('gsw_err: %s' % self._packet.getTxRxResult(res))
            self.get_logger().info('CLEARING ERROR')
            self._flags.WRITE_CLEAR_ERROR = False

        elif self._flags.WRITE_SHUTDOWN_COND:
            res = self._gsw_shut.txPacket()
            if res != COMM_SUCCESS:
                self.get_logger().warn('gsw_shut: %s' % self._packet.getTxRxResult(res))
            self.get_logger().info('SETTING SHUTDOWN CONDITIONS')
            self._flags.WRITE_SHUTDOWN_COND = False

        elif self._flags.WRITE_STIFFNESS:
            res = self._gsw_perm.txPacket()
            if res != COMM_SUCCESS:
                self.get_logger().warn('gsw_perm: %s' % self._packet.getTxRxResult(res))
            res = self._gsw_stiff.txPacket()
            if res != COMM_SUCCESS:
                self.get_logger().warn('gsw_stiff: %s' % self._packet.getTxRxResult(res))
            self.get_logger().info('SETTING STIFFNESS')
            self._flags.WRITE_STIFFNESS = False

        elif self._flags.WRITE_SPEED_POS:
            self.get_logger().info(str(self._gsw_sp.data_dict))
            res = self._gsw_sp.txPacket()
            if res != COMM_SUCCESS:
                self.get_logger().warn('gsw_sp: %s' % self._packet.getTxRxResult(res))
            self.get_logger().info('WRITING SPEED/POS')
            self._flags.WRITE_SPEED_POS = False

        # ── 4. Read main board (only if time remains and not in light mode) ──
        if not self.LIGHT_MODE:
            now_ms = time.time_ns() / 1_000_000
            if now_ms > t_end:
                self.get_logger().warn(
                    'TIME PERIOD EXCEEDED — consider enabling light_mode. '
                    'Exceeded by %d ms' % (now_ms - t_end))
            else:
                res = self._gsr_mb.txRxPacket()
                if res != COMM_SUCCESS:
                    self.get_logger().warn('SyncRead MB: %s' % self._packet.getTxRxResult(res))
                else:
                    for mb, dxl_id in zip(self.main_boards, self._mb_ids):
                        if not self._gsr_mb.isAvailable(
                                dxl_id, ADDR_START_SYNC_READ_MB, LEN_SYNC_READ_MB):
                            self.get_logger().error('[ID:%d] SyncReadMB isAvailable failed' % dxl_id)
                            break
                        #self.get_logger().info(f"{dxl_id}")
                        mb.palm_IR_sensor      = self._gsr_mb.getData(dxl_id, ADDR_PALM_IR_SENSOR,      LEN_PALM_IR_SENSOR)
                        mb.capacitive_sensor_1 = self._gsr_mb.getData(dxl_id, ADDR_CAPACITIVE_SENSOR_1, LEN_CAPACITIVE_SENSOR)
                        mb.capacitive_sensor_2 = self._gsr_mb.getData(dxl_id, ADDR_CAPACITIVE_SENSOR_2, LEN_CAPACITIVE_SENSOR)
                    self._fill_mb_msg()
                    self._pub_mb.publish(self._all_mbs)

                if time.time_ns() / 1_000_000 > t_end:
                    self.get_logger().warn(
                        'TIME PERIOD EXCEEDED — consider enabling light_mode. '
                        'Exceeded by %d ms' % (time.time_ns() / 1_000_000 - t_end))

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('Closing port…')
        self._port.closePort()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HandControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
