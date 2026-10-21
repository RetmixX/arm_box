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
    def __init__(self, device: str, speed: int = 10, dxl_num: int = 6, baud_rate: int = 115200,
                 timeout=1.0, move_timeout=35.0) -> None:
        self.ser = serial.Serial(device, baud_rate, timeout=timeout)
        self.timeout = timeout
        # Сколько ждать ответа на P: плата отвечает только после прихода руки в цель.
        # Должно быть больше MOVE_TIMEOUT_MAX_MS в скетче (30 с).
        self.move_timeout = move_timeout
        self.dxl_num = dxl_num
        self.ik = InverseKinematics()
        self.speed = speed
        time.sleep(1.5)
        self.ser.reset_input_buffer()
        self.set_speed(self.speed)
        try:
            self.start_position()
        except ARMSerialError as e:
            # Не роняем запуск сервиса, если рука не дошла до стартовой позиции
            print(f"Предупреждение: стартовая позиция не достигнута: {e}")

    def close(self) -> None:
        self.ser.close()

    def _readline(self):
        raw = self.ser.readline()
        return raw.decode("utf-8", errors="replace").strip()

    def _query(self, cmd, args="", wait=None):
        """Отправляет команду и ждёт ответ, начинающийся с той же буквы или с ERR.
        wait - сколько ждать ответа, с (по умолчанию timeout + 1)."""
        self.ser.reset_input_buffer()
        line = f"{cmd} {args}".strip() + "\n"
        self.ser.write(line.encode("ascii"))
        self.ser.flush()

        deadline = time.time() + (wait if wait is not None else self.timeout + 1.0)
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

    def _to_dicts(self, resp):
        """'G 2048 2051 ...' или 'P OK 2048 ...' -> [{"1": 2048}, {"2": 2051}, ...]"""
        tokens = resp.split()[1:]
        if tokens and tokens[0] == "OK":
            tokens = tokens[1:]
        if len(tokens) != self.dxl_num:
            raise ARMSerialError(f"Неожиданный ответ: {resp!r}")

        result = []
        for servo_id, tok in zip(range(1, self.dxl_num + 1), tokens):
            result.append({str(servo_id): None if tok == "ERR" else int(tok)})
        return result

    def _move(self, pos_servo):
        """Отправляет P и ждёт, пока рука придёт в цель.
        Возвращает фактические позиции после прихода.
        Если рука не дошла за отведённое время - ARMSerialError('ERR P: timeout ...')."""
        targets = [int(p) for p in pos_servo]
        try:
            res = self._query("P", " ".join(str(p) for p in targets), wait=self.move_timeout)
        except ARMSerialError as e:
            # Добавляем цели, чтобы было видно, какой сустав не дошёл
            raise ARMSerialError(f"{e} | цели: {' '.join(map(str, targets))}") from e
        return self._values(res[:1] + res[4:])   # убираем "OK"

    def set_speed(self, speed_percent: int):
        self.speed = speed_percent
        speed = int(speed_percent * 1023 / 100)
        return self._to_dicts(self._query("S", f" {speed}"))

    def get_motors_temp(self):
        return self._to_dicts(self._query("T"))

    def get_motors_load(self):
        return self._to_dicts(self._query("L"))

    def get_motors_pos(self):
        return self._to_dicts(self._query("G"))

    def move_arm(self, x, z, angel, gripper_pov, gripper_state: bool):
        pos_servo = self.ik.calculate(x, z, angel)
        if pos_servo is None:
            raise ARMIKError(x, z, angel)
        pos_servo.append(math.ceil(gripper_pov * 3.41))
        if gripper_state:
            pos_servo.append(370)
        else:
            pos_servo.append(710)

        return self._move(pos_servo)

    def move_arm_paint(self, x, z, angel, gripper_pov, pain_pov):
        pos_servo = self.ik.calculate(x, z, angel)
        if pos_servo is None:
            raise ARMIKError(x, z, angel)
        pos_servo.append(math.ceil(gripper_pov * 3.41))
        pos_servo.append(math.ceil(pain_pov * 3.41))

        return self._move(pos_servo)

    def start_position(self):
        self.move_arm(200, 200, 180, 150, False)