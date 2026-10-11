import math


class InverseKinematics:
    """
    Обратная кинематика манипулятора (порт arm_IK_lib.cpp, исправленная версия).

    calculate(x, z, angle) -> [pos1, pos2, pos3, pos4] (порядок ID) или None.
        x     - расстояние от оси базы до середины схвата, мм
        z     - высота середины схвата над низом базы, мм
        angle - угол поворота базы, градусы (180 = позиция 2048)
    Схват всегда смотрит вертикально вниз.
    """

    # Геометрия манипулятора
    BASE_HEIGHT = 163.5

    L1A = 195.81
    L1B = 67.36

    L1 = 207.07
    L2 = 134.10
    L3 = 170.0

    POSITION_OFFSET_DXL4 = 30

    POS_MIN = 0
    POS_MAX = 4095

    def __init__(self):
        # Геометрические смещения (рад): излом звена L1 + ноль серво = π/2
        self.ik_offsets = [0.0, 0.0, 0.0, 0.0]

        # Калибровочные поправки пользователя (рад), по умолчанию 0
        self.calibration_offsets = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Последние рассчитанные углы (рад)
        self.q1 = 0.0  # база
        self.q2 = 0.0  # плечо
        self.q3 = 0.0  # локоть
        self.q4 = 0.0  # кисть

        # Последние позиции Dynamixel
        self.pos1 = 0
        self.pos2 = 0
        self.pos3 = 0
        self.pos4 = 0

        # Константы, считаются один раз
        self._c_sum = self.L1 * self.L1 + self.L2 * self.L2
        self._c_inv = 1.0 / (2.0 * self.L1 * self.L2)
        self._z_off = self.L3 - self.BASE_HEIGHT
        self._reach_max = self.L1 + self.L2
        self._reach_min = abs(self.L1 - self.L2)

        self.calc_ik_angular_offsets()

    # --------------------------------------------------------
    # Геометрические смещения
    # --------------------------------------------------------

    def calc_ik_angular_offsets(self):
        """Повторный вызов безопасен: смещения не накапливаются."""
        alpha = math.atan(self.L1B / self.L1A)
        half_pi = math.pi / 2
        self.ik_offsets = [
            0.0,
            half_pi + alpha,
            half_pi - alpha + math.pi,
            half_pi,
        ]

    # --------------------------------------------------------
    # Калибровка сервоприводов
    # --------------------------------------------------------

    def set_calibration(self, dxl, angle):
        """
        Установить калибровочную поправку.

        dxl: номер сервопривода (2, 3 или 4)
        angle: поправка в градусах (0 = без поправки)
        """
        if dxl not in (2, 3, 4):
            raise ValueError("Допустимые сервоприводы: 2, 3, 4")
        self.calibration_offsets[dxl - 1] = math.radians(angle)

    def add_calibration(self, dxl, angle):
        """Добавить калибровочную поправку в градусах."""
        if dxl not in (2, 3, 4):
            raise ValueError("Допустимые сервоприводы: 2, 3, 4")
        self.calibration_offsets[dxl - 1] += math.radians(angle)

    def reset_calibration(self):
        """Сбросить поправки. Геометрические смещения не трогаются."""
        self.calibration_offsets = [0.0, 0.0, 0.0, 0.0, 0.0]

    # --------------------------------------------------------
    # Угол в позицию Dynamixel и обратно
    # --------------------------------------------------------

    @staticmethod
    def angle_to_pos(angle):
        return int(2048.0 * angle / math.pi)

    @staticmethod
    def pos_to_angle(pos):
        return pos * math.pi / 2048.0

    # --------------------------------------------------------
    # Рабочая зона
    # --------------------------------------------------------

    def is_reachable(self, x, z):
        r = math.hypot(x, z + self._z_off)
        return x > 0 and self._reach_min <= r <= self._reach_max

    def max_distance(self, z):
        """Максимальное x на высоте z, мм."""
        zz = z + self._z_off
        return math.sqrt(max(self._reach_max ** 2 - zz * zz, 0.0))

    # --------------------------------------------------------
    # Обратная кинематика
    # --------------------------------------------------------

    def calculate(self, x, z, angle):
        """
        Возвращает [pos1, pos2, pos3, pos4] или None, если точка недостижима.
        """
        L1, L2 = self.L1, self.L2
        z = z + self._z_off  # высота кисти относительно оси плеча

        c = (x * x + z * z - self._c_sum) * self._c_inv  # cos(q3)
        if c < -1.0 or c > 1.0 or x <= 0:
            return None

        s = -math.sqrt(1.0 - c * c)  # sin(q3), локоть вверх
        k1 = L1 + L2 * c
        k2 = L2 * s

        q1 = math.radians(angle)
        q2 = math.atan2(z * k1 - x * k2, x * k1 + z * k2)
        q3 = math.atan2(s, c)
        q4 = -(q2 + q3)  # схват строго вертикально

        off, cal = self.ik_offsets, self.calibration_offsets
        pos1 = self.angle_to_pos(q1 + off[0] + cal[0])
        pos2 = self.angle_to_pos(q2 + off[1] + cal[1])
        pos3 = self.angle_to_pos(q3 + off[2] + cal[2])
        pos4 = self.angle_to_pos(q4 + off[3] + cal[3]) + self.POSITION_OFFSET_DXL4

        for p in (pos1, pos2, pos3, pos4):
            if p < self.POS_MIN or p > self.POS_MAX:
                return None

        self.q1, self.q2, self.q3, self.q4 = q1, q2, q3, q4
        self.pos1, self.pos2, self.pos3, self.pos4 = pos1, pos2, pos3, pos4
        return self.get_result()

    # --------------------------------------------------------
    # Прямая кинематика (проверка / чтение Present_Position)
    # --------------------------------------------------------

    def forward(self, positions):
        """
        [pos1, pos2, pos3, pos4] -> (x мм, z мм, angle градусы).
        Учитывает наклон кисти, если схват не вертикален.
        """
        off, cal = self.ik_offsets, self.calibration_offsets
        p4 = positions[3] - self.POSITION_OFFSET_DXL4
        q = [self.pos_to_angle(p) - off[i] - cal[i]
             for i, p in enumerate((positions[0], positions[1], positions[2], p4))]
        a = q[1] + q[2]
        tilt = a + q[3]
        x = self.L1 * math.cos(q[1]) + self.L2 * math.cos(a) + self.L3 * math.sin(tilt)
        z = (self.L1 * math.sin(q[1]) + self.L2 * math.sin(a)
             + self.BASE_HEIGHT - self.L3 * math.cos(tilt))
        return x, z, math.degrees(q[0])

    # --------------------------------------------------------
    # Результат
    # --------------------------------------------------------

    def get_result(self):
        return [self.pos1, self.pos2, self.pos3, self.pos4]


if __name__ == "__main__":
    ik = InverseKinematics()
    pos = ik.calculate(200, 200, 180)
    print("calculate(200, 200, 180):", pos)
    print("forward:", tuple(round(v, 2) for v in ik.forward(pos)))
    print("calculate(400, 60, 180):", ik.calculate(400, 60, 180))