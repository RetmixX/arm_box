import math


class ParallelInverseKinematics:
    """
    Обратная кинематика плоско-параллельного манипулятора (паллетайзер).

    Устройство: база (ID1) поворачивает всю руку; два мотора у основания
    приводят плечо и (через параллелограмм) предплечье. Мотор предплечья
    задаёт АБСОЛЮТНЫЙ угол предплечья относительно горизонта, а не угол
    относительно плеча. Какой ID за что отвечает - UPPER_ID / FORE_ID. Второй
    параллелограмм держит захват всегда вертикально, поэтому угол кисти
    не считается - его задаёт механика.

    calculate(x, z, angle) -> [pos1, pos2, pos3] (порядок ID) или None.
        x     - расстояние от оси базы до середины захвата, мм
        z     - высота середины захвата над низом базы, мм
        angle - угол поворота базы, градусы (180 = позиция 2048)
    Интерфейс совпадает с InverseKinematics последовательной руки.

    Все размеры и калибровку ниже нужно заполнить под свою руку
    (порядок калибровки - в calibration_help()).
    """

    # ---------------- Геометрия, мм (ЗАМЕРЬТЕ) ----------------
    BASE_HEIGHT = 163.5   # высота оси плеча над низом базы (как у последовательной руки)
    SHOULDER_X = 0.0      # смещение оси плеча от оси поворота базы по горизонтали
    L1 = 225.0            # плечо: ось плеча -> ось локтя
    L2 = 280.0            # предплечье: ось локтя -> ось крепления кисти
    TOOL_X = 0.0          # от оси кисти до оси присоски по горизонтали (ЗАМЕРЬТЕ, если не 0)
    TOOL_Z = 130.0        # от оси кисти вниз до кончика присоски (свободная присоска)
    # Присоска сжимается при упоре: высота 130 -> 120 мм, ход 10 мм.
    # z во всех расчётах - высота КОНЧИКА свободной присоски.
    TOOL_Z_COMPRESSED = 120.0

    # ---------------- Назначение моторов ----------------
    # Плечо (L1, почти вертикальное звено) наклоняется вперёд-назад - это ВЫЛЕТ.
    # Предплечье (L2, почти горизонтальное) поднимается-опускается - это ВЫСОТА.
    # Проверено на руке: ID2 -> плечо (вылет), ID3 -> предплечье (высота).
    UPPER_ID = 2          # мотор, наклоняющий плечо (вылет)
    FORE_ID = 3           # мотор предплечья через параллелограмм (высота)

    # ---------------- Калибровка моторов (ЗАПОЛНИТЕ) ----------------
    # Позиция мотора UPPER_ID, когда плечо стоит строго вертикально
    P_UPPER_VERTICAL = 2048
    # Позиция мотора FORE_ID, когда предплечье строго горизонтально
    P_FORE_HORIZONTAL = 2048
    # Направление: +1, если позиция растёт, когда плечо наклоняется НАЗАД
    # (угол от горизонта растёт), иначе -1
    # Проверено: ID2 уменьшается -> плечо уходит назад, значит -1
    DIR_UPPER = -1
    # +1, если позиция растёт, когда предплечье поднимается, иначе -1
    # Проверено: ID3 уменьшается -> рука опускается, значит +1
    DIR_FORE = 1
    # Поправка базы, тиков (если 2048 не соответствует 180°)
    P1_OFFSET = 0

    # ---------------- Ограничения механики, градусы (ПРОВЕРЬТЕ) ----------------
    A1_MIN, A1_MAX = 30.0, 150.0         # угол плеча от горизонта
    A2_MIN, A2_MAX = -60.0, 45.0         # угол предплечья от горизонта
    # Раствор параллелограмма: угол плеча минус угол предплечья.
    # Слишком маленький или большой - звенья упираются или параллелограмм складывается.
    DELTA_MIN, DELTA_MAX = 20.0, 150.0

    TICKS_PER_REV = 4096                 # MX: 4096 тиков на оборот
    POS_MIN = 0
    POS_MAX = 4095

    def __init__(self):
        self.q1 = self.a1 = self.a2 = 0.0   # последние углы, рад
        self.pos1 = self.pos2 = self.pos3 = 0
        self._k = self.TICKS_PER_REV / (2.0 * math.pi)   # тиков на радиан

    # --------------------------------------------------------
    # Углы <-> позиции
    # --------------------------------------------------------

    def _angles_to_pos(self, base_deg, a1, a2):
        """Углы -> (pos ID1, pos ID2, pos ID3) с учётом назначения моторов."""
        k = self._k
        pos1 = int(round(base_deg / 360.0 * self.TICKS_PER_REV)) + self.P1_OFFSET
        p_upper = int(round(self.P_UPPER_VERTICAL + self.DIR_UPPER * (a1 - math.pi / 2) * k))
        p_fore = int(round(self.P_FORE_HORIZONTAL + self.DIR_FORE * a2 * k))
        by_id = {self.UPPER_ID: p_upper, self.FORE_ID: p_fore}
        return pos1, by_id[2], by_id[3]

    def _pos_to_angles(self, pos1, pos2, pos3):
        k = self._k
        by_id = {2: pos2, 3: pos3}
        base_deg = (pos1 - self.P1_OFFSET) * 360.0 / self.TICKS_PER_REV
        a1 = math.pi / 2 + self.DIR_UPPER * (by_id[self.UPPER_ID] - self.P_UPPER_VERTICAL) / k
        a2 = self.DIR_FORE * (by_id[self.FORE_ID] - self.P_FORE_HORIZONTAL) / k
        return base_deg, a1, a2

    # --------------------------------------------------------
    # Ограничения
    # --------------------------------------------------------

    def _within_limits(self, a1, a2):
        d1, d2 = math.degrees(a1), math.degrees(a2)
        delta = d1 - d2
        return (self.A1_MIN <= d1 <= self.A1_MAX
                and self.A2_MIN <= d2 <= self.A2_MAX
                and self.DELTA_MIN <= delta <= self.DELTA_MAX)

    def is_reachable(self, x, z):
        return self._solve_plane(x, z) is not None

    def max_distance(self, z, step=1.0):
        """Максимальное x на высоте z с учётом ограничений, мм (0 - если недостижимо)."""
        x = self.SHOULDER_X + self.TOOL_X + self.L1 + self.L2
        while x > 0:
            if self._solve_plane(x, z) is not None:
                return x
            x -= step
        return 0.0

    # --------------------------------------------------------
    # Обратная кинематика
    # --------------------------------------------------------

    def _solve_plane(self, x, z):
        """(x, z) середины захвата -> (a1, a2) в радианах или None."""
        r = x - self.SHOULDER_X - self.TOOL_X     # до оси кисти, от оси плеча
        h = z + self.TOOL_Z - self.BASE_HEIGHT
        d2 = r * r + h * h
        d = math.sqrt(d2)
        if d < 1e-6 or d > self.L1 + self.L2 or d < abs(self.L1 - self.L2):
            return None

        cos_b = (self.L1 ** 2 + d2 - self.L2 ** 2) / (2.0 * self.L1 * d)
        cos_b = max(-1.0, min(1.0, cos_b))
        a1 = math.atan2(h, r) + math.acos(cos_b)          # локоть вверх
        ex, ez = self.L1 * math.cos(a1), self.L1 * math.sin(a1)
        a2 = math.atan2(h - ez, r - ex)

        if not self._within_limits(a1, a2):
            return None
        return a1, a2

    def calculate(self, x, z, angle):
        """Возвращает [pos1, pos2, pos3] или None, если точка недостижима."""
        sol = self._solve_plane(x, z)
        if sol is None:
            return None
        a1, a2 = sol
        pos = self._angles_to_pos(angle, a1, a2)
        if any(p < self.POS_MIN or p > self.POS_MAX for p in pos):
            return None

        self.q1, self.a1, self.a2 = math.radians(angle), a1, a2
        self.pos1, self.pos2, self.pos3 = pos
        return list(pos)

    # --------------------------------------------------------
    # Прямая кинематика
    # --------------------------------------------------------

    def forward(self, positions):
        """[pos1, pos2, pos3, ...] -> (x мм, z мм, angle градусы)."""
        base_deg, a1, a2 = self._pos_to_angles(*positions[:3])
        x = (self.SHOULDER_X + self.L1 * math.cos(a1) + self.L2 * math.cos(a2)
             + self.TOOL_X)
        z = (self.BASE_HEIGHT + self.L1 * math.sin(a1) + self.L2 * math.sin(a2)
             - self.TOOL_Z)
        return x, z, base_deg

    def get_result(self):
        return [self.pos1, self.pos2, self.pos3]

    # --------------------------------------------------------
    # Ограничение параллелограмма для прошивки
    # --------------------------------------------------------

    def firmware_parallel_limits(self):
        """
        Значения для скетча (ARM_TYPE 2), соответствующие DELTA_MIN/DELTA_MAX:
        (PAR_A, PAR_B, PAR_DIR_A, PAR_DIR_B, PAR_D_MIN, PAR_D_MAX).
        В прошивке: d = PAR_DIR_A * pos[PAR_A] - PAR_DIR_B * pos[PAR_B],
        PAR_A - индекс мотора плеча, PAR_B - предплечья (индекс = ID - 1).
        """
        k = self._k
        base = self.DIR_UPPER * self.P_UPPER_VERTICAL - self.DIR_FORE * self.P_FORE_HORIZONTAL
        d_min = base + k * (math.radians(self.DELTA_MIN) - math.pi / 2)
        d_max = base + k * (math.radians(self.DELTA_MAX) - math.pi / 2)
        return (self.UPPER_ID - 1, self.FORE_ID - 1, self.DIR_UPPER, self.DIR_FORE,
                int(math.floor(d_min)), int(math.ceil(d_max)))

    @staticmethod
    def calibration_help():
        return """\
Калибровка (Serial Monitor, команды G и P, скорость S 50):
 0. Проверьте назначение: измените в P только ID3 на +100. Если рука
    уехала вперёд-назад - ID3 это плечо (UPPER_ID = 3), если вверх-вниз -
    ID3 это предплечье (тогда UPPER_ID = 2, FORE_ID = 3).
 1. Поставьте плечо строго вертикально (по уровню), отправьте G.
    Значение мотора UPPER_ID -> P_UPPER_VERTICAL.
 2. Поставьте предплечье строго горизонтально, отправьте G.
    Значение мотора FORE_ID -> P_FORE_HORIZONTAL.
 3. Дайте мотору плеча +100. Если плечо наклонилось НАЗАД - DIR_UPPER = 1,
    вперёд - DIR_UPPER = -1. Дайте мотору предплечья +100: поднялось -
    DIR_FORE = 1, опустилось - DIR_FORE = -1.
 4. Сведите и разведите звенья до упора (осторожно, малыми шагами),
    запишите углы - это DELTA_MIN / DELTA_MAX. Проверьте A1/A2 пределы.
 5. Измерьте BASE_HEIGHT, SHOULDER_X, L1, L2, TOOL_X, TOOL_Z.
 6. Запустите python3 ik_parallel.py - он напечатает PAR_* для скетча.
"""


if __name__ == "__main__":
    ik = ParallelInverseKinematics()
    for args in [(250, 150, 180), (350, 50, 90), (200, 250, 180), (600, 0, 180)]:
        pos = ik.calculate(*args)
        if pos is None:
            print(f"calculate{args}: недостижимо")
            continue
        fk = tuple(round(v, 2) for v in ik.forward(pos))
        print(f"calculate{args}: {pos}  forward: {fk}")
    print("max x на высоте 100 мм:", ik.max_distance(100))
    ia, ib, a, b, dmin, dmax = ik.firmware_parallel_limits()
    print("Для скетча (ARM_TYPE 2):")
    print(f"  const uint8_t  PAR_A = {ia}, PAR_B = {ib};")
    print(f"  const int8_t   PAR_DIR_A = {a}, PAR_DIR_B = {b};")
    print(f"  const int32_t  PAR_D_MIN = {dmin};")
    print(f"  const int32_t  PAR_D_MAX = {dmax};")