import math

from ik import InverseKinematics
from dynamixel_sdk import *

_PRESENT_TEMP = 43
_PRESENT_LOAD = 40
_PRESENT_POSITION = 30


class ARMIKError(Exception):
    def __init__(self, x, z, angel):
        self.x = x
        self.z = z
        self.angel = angel
        text = f"x: {self.x} || y: {self.z} || angel: {self.angel}"
        super().__init__(text)

def _to_bytes(value, size):
    """Число -> список байт (младший байт первым) для addParam()."""
    mask = (1 << (8 * size)) - 1
    return list((int(value) & mask).to_bytes(size, "little"))


class RobotARM:
    def __init__(self, device: str, speed: int = 5, dxl_num: int = 6, baud_rate: int = 1000000, timeout=1000.0) -> None:
        self.device = device
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.protocol = 1.0
        self.dxl_num = dxl_num
        self.ik = InverseKinematics()
        self.port_handler = PortHandler(self.device)
        self.packet_handler = PacketHandler(self.protocol)
        self.port_handler.setPacketTimeoutMillis(self.timeout)
        self.speed = speed
        self.port_handler.openPort()
        if not self.port_handler.is_open:
            raise Exception("Port not open")
        self.port_handler.setBaudRate(self.baud_rate)

        for i in range(1, dxl_num + 1):
            self.packet_handler.write2ByteTxRx(self.port_handler, i, 6, 0)
            self.packet_handler.write2ByteTxRx(self.port_handler, i, 8, 4095)
            self.packet_handler.write2ByteTxRx(self.port_handler, i, 24, 1)
            self.set_speed(i, self.speed)
        self.start_position()

    def set_speed(self, dxl_id: int, speed_percent: int) -> None:
        self.speed = speed_percent
        speed = int(speed_percent * 1023 / 100)
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, 32, speed)

    def set_speed_arm(self, speed_percent: int):
        for i in range(1, self.dxl_num + 1):
            self.set_speed(i, speed_percent)

    def set_sync_pos(self, positions: list[int]) -> None:
        goal = dict(enumerate(positions, start=1))
        group = GroupSyncWrite(self.port_handler, self.packet_handler, 30, 2)
        for dxl_id, pos in goal.items():
            group.addParam(dxl_id, _to_bytes(pos, 2))
        group.txPacket()
        group.clearParam()


    def read_temp(self, dxl_id: int):
        present_temp = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, _PRESENT_TEMP)
        print(present_temp)
        return present_temp[0]

    def read_pos(self, dxl_id: int):
        present_pos = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, _PRESENT_POSITION)
        return present_pos[0]

    def read_load(self, dxl_id: int):
        present_load = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, _PRESENT_LOAD)
        return present_load[0]

    def get_motors_temp(self):
        temps = []
        for i in range(1, self.dxl_num + 1):
            temps.append({i: self.read_temp(i)})
        return temps

    def get_motors_load(self):
        loads = []
        for i in range(1, self.dxl_num + 1):
            loads.append({i: self.read_load(i)})
        return loads

    def get_motors_pos(self):
        pos = []
        for i in range(1, self.dxl_num + 1):
            pos.append({i: self.read_pos(i)})
        return pos

    def move_arm(self, x, z, angel, gripper_pov, gripper_state:bool):
        pos_servo = self.ik.calculate(x, z, angel)
        if pos_servo is None:
            raise ARMIKError(x, z, angel)
        pos_servo.append(math.ceil(gripper_pov * 3.41))
        if gripper_state:
            print("True gripper")
            pos_servo.append(370)
        else:
            print("False gripper")
            pos_servo.append(710)

        self.set_sync_pos(pos_servo)
        for i in range(1, self.dxl_num + 1):
            self.wait_move(i)

    def wait_move(self, dxl_id):
        while True:
            status = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, 46)[0]
            if status == 0:
                print("Move stop")
                break


    def start_position(self):
        self.move_arm(200, 200, 180, 150, False)
