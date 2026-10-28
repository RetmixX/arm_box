import math
import threading
import time

from ik import InverseKinematics
from ik_parallel import ParallelInverseKinematics
import serial


# Профили манипуляторов. Тип отправляется на плату командой A при старте:
#   serial   - A 1, 6 моторов: база, плечо, локоть, кисть, поворот кисти, захват
#   parallel - A 2, 5 моторов: база, плечо, предплечье, поворот захвата, захват
ARM_PROFILES = {
    "serial": {
        "fw_type": 1,
        "ik": InverseKinematics,
        "dxl_num": 6,
        "park": (160, 200, 280, 150),     # x, z, angle, поворот захвата
        "start": (200, 200, 180, 150),
    },
    "parallel": {
        "fw_type": 2,
        "ik": ParallelInverseKinematics,
        "pump": True,                     # gripper_state управляет помпой
        "dxl_num": 5,
        "park": (250, 300, 280, 150),     # середина рабочей зоны
        "start": (250, 200, 180, 150),
    },
}

GRIP_OPEN = 710
GRIP_CLOSED = 370


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
    def __init__(self, device: str, speed: int = 10, arm_type: str = "serial",
                 baud_rate: int = 115200, timeout=1.0, move_timeout=35.0) -> None:
        if arm_type not in ARM_PROFILES:
            raise ValueError(f"arm_type должен быть одним из {list(ARM_PROFILES)}")
        self.arm_type = arm_type
        self.profile = ARM_PROFILES[arm_type]
        self.ser = serial.Serial(device, baud_rate, timeout=timeout)
        # Один обмен с платой за раз. Защищает порт, даже если веб-запрос отменили,
        # а поток с движением ещё работает (asyncio.Lock в этом случае уже отпущен).
        self._io_lock = threading.Lock()
        self.timeout = timeout
        # Сколько ждать ответа на P: плата отвечает только после прихода руки в цель.
        # Должно быть больше MOVE_TIMEOUT_MAX_MS в скетче (30 с).
        self.move_timeout = move_timeout
        self.dxl_num = self.profile["dxl_num"]
        self.ik = self.profile["ik"]()
        self.speed = speed
        self.pump_on = False               # последнее отправленное состояние помпы
        time.sleep(1.5)
        self.ser.reset_input_buffer()
        self.set_arm_type()            # сообщаем плате, какая рука подключена
        self.set_speed(self.speed)
        try:
            self.park_pos()
        except (ARMSerialError, ARMIKError) as e:
            # Не роняем запуск сервиса, если рука не дошла до стартовой позиции
            print(f"Предупреждение: стартовая позиция не достигнута: {e}")

    def close(self) -> None:
        with self._io_lock:
            self.ser.close()

    def _readline(self):
        raw = self.ser.readline()
        return raw.decode("utf-8", errors="replace").strip()

    def _query(self, cmd, args="", wait=None):
        """Отправляет команду и ждёт ответ, начинающийся с той же буквы или с ERR.
        wait - сколько ждать ответа, с (по умолчанию timeout + 1).
        Если плата перезагрузилась и забыла тип руки - заново задаёт тип
        и скорость и повторяет команду один раз."""
        with self._io_lock:
            try:
                return self._query_locked(cmd, args, wait)
            except ARMSerialError as e:
                if cmd.upper() == "A" or "arm type not set" not in str(e):
                    raise
                print("Плата не знает тип руки (перезагрузка?) - отправляю заново")
                self._send_arm_type_locked()
                self._query_locked("S", str(self._speed_units()))
                return self._query_locked(cmd, args, wait)

    # Инициализация моторов на плате занимает до нескольких секунд
    ARM_TYPE_WAIT = 10.0

    def _send_arm_type_locked(self):
        fw_type = self.profile["fw_type"]
        resp = self._query_locked("A", str(fw_type), wait=self.ARM_TYPE_WAIT)
        # Ответ: "A OK <тип> <число моторов>"
        parts = resp.split()
        if len(parts) < 4 or parts[1] != "OK" or int(parts[2]) != fw_type:
            raise ARMSerialError(f"Плата не приняла тип руки: {resp!r}")
        if int(parts[3]) != self.dxl_num:
            raise ARMSerialError(f"Число моторов на плате ({parts[3]}) "
                                 f"не совпадает с профилем ({self.dxl_num})")
        return resp

    def set_arm_type(self):
        """Сообщает плате тип руки (A 1 / A 2). Плата заново инициализирует моторы."""
        with self._io_lock:
            return self._send_arm_type_locked()

    def _query_locked(self, cmd, args="", wait=None):
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

    def _speed_units(self):
        return max(1, int(self.speed * 1023 / 100))

    def set_speed(self, speed_percent: int):
        self.speed = speed_percent
        return self._to_dicts(self._query("S", str(self._speed_units())))

    def get_motors_temp(self):
        return self._to_dicts(self._query("T"))

    def get_motors_load(self):
        return self._to_dicts(self._query("L"))

    def get_motors_pos(self):
        return self._to_dicts(self._query("G"))

    def set_pump(self, enabled: bool):
        """Включает или выключает помпу (присоску). Команда V прошивки.
        Если у руки нет помпы, будет ARMSerialError."""
        resp = self._query("V", "1" if enabled else "0")
        self.pump_on = resp.split()[-1] == "1"
        return self.pump_on

    # ---------- Присоска (параллельная рука) ----------

    def _suction_stroke(self):
        ik = self.ik
        return getattr(ik, "TOOL_Z", 0) - getattr(ik, "TOOL_Z_COMPRESSED", getattr(ik, "TOOL_Z", 0))

    def pick(self, x, z_top, angel, rot=150, approach=50.0, press=0.7, hold_s=0.3):
        """Взять деталь присоской.
        z_top    - высота верха детали, мм
        approach - на сколько выше детали подходить и отходить, мм
        press    - насколько поджать присоску (доля хода 10 мм; 0.7 = 7 мм)
        hold_s   - пауза после включения помпы, чтобы набрался вакуум"""
        z_contact = z_top - self._suction_stroke() * press
        self.move_arm(x, z_top + approach, angel, rot, False)   # над деталью, помпа выкл
        self.move_arm(x, z_contact, angel, rot, True)           # помпа вкл, прижать присоску
        time.sleep(hold_s)                                      # набор вакуума
        self.move_arm(x, z_top + approach, angel, rot, True)    # поднять с деталью

    def place(self, x, z_top, angel, rot=150, approach=50.0, press=0.3, release_s=0.3):
        """Положить деталь. z_top - высота, на которой должен оказаться верх детали."""
        z_contact = z_top - self._suction_stroke() * press
        self.move_arm(x, z_top + approach, angel, rot, True)    # над местом, деталь держим
        self.move_arm(x, z_contact, angel, rot, True)           # опустить
        self.move_arm(x, z_contact, angel, rot, False)          # на месте: помпа выкл
        time.sleep(release_s)
        self.move_arm(x, z_top + approach, angel, rot, False)   # уйти вверх

    def _arm_positions(self, x, z, angel, gripper_pov):
        """Позиции суставов по IK + поворот захвата (AX-18A, градусы -> позиция).
        serial:   [база, плечо, локоть, кисть] + поворот кисти
        parallel: [база, плечо, предплечье]    + поворот захвата
        (у параллельной руки захват всегда вертикален - кисть держит механика)"""
        pos_servo = self.ik.calculate(x, z, angel)
        if pos_servo is None:
            raise ARMIKError(x, z, angel)
        pos_servo.append(math.ceil(gripper_pov * 3.41))
        return pos_servo

    def move_arm(self, x, z, angel, gripper_pov, gripper_state: bool):
        """Перемещает руку. gripper_state - держать деталь:
        - последовательная рука: закрыть / открыть захват;
        - паллетайзер: включить / выключить помпу.
          Включение - ДО движения (присоска уже тянет, когда коснётся детали),
          выключение - ПОСЛЕ прихода в точку (деталь не упадёт по дороге)."""
        pos_servo = self._arm_positions(x, z, angel, gripper_pov)
        if self.arm_type == "serial":
            pos_servo.append(GRIP_CLOSED if gripper_state else GRIP_OPEN)
        else:
            pos_servo.append(200)

        use_pump = self.profile.get("pump", False)
        if use_pump and gripper_state and not self.pump_on:
            self.set_pump(True)
        result = self._move(pos_servo)
        if use_pump and not gripper_state and self.pump_on:
            self.set_pump(False)
        return result

    def move_arm_paint(self, x, z, angel, gripper_pov, pain_pov):
        pos_servo = self._arm_positions(x, z, angel, gripper_pov)
        pos_servo.append(math.ceil(pain_pov * 3.41))
        return self._move(pos_servo)

    def start_position(self):
        x, z, a, g = self.profile["start"]
        self.move_arm(x, z, a, g, False)

    def park_pos(self):
        x, z, a, g = self.profile["park"]
        self.move_arm(x, z, a, g, False)