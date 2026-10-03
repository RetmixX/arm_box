import math
import time

from ik import InverseKinematics
import serial


class ARMIKError(Exception):
    def __init__(self, x, z, angel):
        self.x = x
        self.z = z
        self.angel = angel
        text = f"x: {self.x} || y: {self.z} || angel: {self.angel}"
        super().__init__(text)

class ARMSerialError(Exception):
    pass


class RobotARM:
    def __init__(self, device: str, speed: int = 3, dxl_num: int = 6, baud_rate: int = 115200, timeout = 1.0) -> None:
        self.ser = serial.Serial(device, baud_rate, timeout=timeout)
        self.timeout = timeout
        self.dxl_num = dxl_num
        self.ik = InverseKinematics()
        self.speed = speed
        time.sleep(1.5)
        self.ser.reset_input_buffer()
        self.set_speed(self.speed)
        self.start_position()

    def close(self) -> None:
        self.ser.close()

    def _readline(self):
        raw = self.ser.readline()
        return raw.decode("utf-8", errors="replace").strip()

    def _query(self, cmd, args=""):
        self.ser.reset_input_buffer()
        line = f"{cmd} {args}".strip() + "\n"
        self.ser.write(line.encode("ascii"))
        self.ser.flush()

        deadline = time.time() + self.timeout + 1.0
        while time.time() < deadline:
            resp = self._readline()
            if not resp:
                continue
            if resp.startswith("ERR"):
                raise ARMSerialError(resp)
            if resp[0].upper() == cmd.upper():
                return resp
        raise ARMSerialError(f"Нет ответа на команду '{cmd}' (таймаут)")

    @staticmethod
    def _values(resp):
        """'G 2048 2051 ...' -> [2048, 2051, ...]; ERR от мотора -> None."""
        out = []
        for tok in resp.split()[1:]:
            out.append(None if tok == "ERR" else int(tok))
        return out

    def set_speed(self, speed_percent: int):
        self.speed = speed_percent
        speed = int(speed_percent * 1023 / 100)
        resp = self._query("S", f" {speed}")
        return self._values(resp[:1] + resp[4:])


    def get_motors_temp(self):
        return self._values(self._query("T"))

    def get_motors_load(self):
        return self._values(self._query("L"))

    def get_motors_pos(self):
        return self._values(self._query("G"))

    def move_arm(self, x, z, angel, gripper_pov, gripper_state:bool):
        pos_servo = self.ik.calculate(x, z, angel)
        if pos_servo is None:
            raise ARMIKError(x, z, angel)
        pos_servo.append(math.ceil(gripper_pov * 3.41))
        if gripper_state:
            pos_servo.append(370)
        else:
            pos_servo.append(710)

        res = self._query("P", " ".join(str(int(p)) for p in pos_servo))
        return self._values(res[:1] + res[4:])



    def start_position(self):
        self.move_arm(200, 200, 180, 150, False)
