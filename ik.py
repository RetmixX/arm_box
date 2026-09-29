
import math


class InverseKinematics:
    # Геометрия манипулятора
    BASE_HEIGHT = 163.5

    L1A = 195.81
    L1B = 67.36

    L1 = 207.07
    L2 = 134.10
    L3 = 170.0

    POSITION_OFFSET_DXL4 = 30

    def __init__(self):
        # Геометрические смещения
        self.ik_offsets = [0.0, 0.0, 0.0, 0.0]

        # Калибровочные смещения
        self.calibration_offsets = [0.0, math.pi/2, math.pi/2, math.pi/2, 0.0]

        # Последние рассчитанные углы
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.q4 = 0.0

        # Последние позиции Dynamixel
        self.pos1 = 0
        self.pos2 = 0
        self.pos3 = 0
        self.pos4 = 0

        # Рассчитать геометрические смещения
        self.calc_ik_angular_offsets()

    # --------------------------------------------------------
    # Геометрические смещения
    # --------------------------------------------------------

    def calc_ik_angular_offsets(self):
        alpha = math.atan(self.L1B / self.L1A)

        self.ik_offsets[1] += alpha
        self.ik_offsets[2] = (
            self.ik_offsets[2] - alpha + math.pi
        )

    # --------------------------------------------------------
    # Калибровка сервоприводов
    # --------------------------------------------------------

    def set_calibration(self, dxl, angle):
        """
        Установить калибровочное смещение.

        dxl: номер сервопривода (2, 3 или 4)
        angle: смещение в градусах
        """
        if dxl not in (2, 3, 4):
            raise ValueError("Допустимые сервоприводы: 2, 3, 4")

        self.calibration_offsets[dxl - 1] = math.radians(angle)

    def add_calibration(self, dxl, angle):
        """
        Добавить калибровочное смещение в градусах.
        """
        if dxl not in (2, 3, 4):
            raise ValueError("Допустимые сервоприводы: 2, 3, 4")

        self.calibration_offsets[dxl - 1] += math.radians(angle)

    # --------------------------------------------------------
    # Угол в позицию Dynamixel
    # --------------------------------------------------------

    @staticmethod
    def angle_to_pos(angle):
        return int(2048.0 * angle / math.pi)

    # --------------------------------------------------------
    # Обратная кинематика
    # --------------------------------------------------------

    def calculate(self, x, z, angle):
        """
        x     - координата X, мм
        z     - координата Z, мм
        angle - угол базы в градусах

        Возвращает словарь с углами и позициями
        или None, если точка недостижима.
        """

        # Коррекция координаты Z
        z = z - self.BASE_HEIGHT + self.L3

        # Расчёт D
        D = (
            x * x + z * z
            - self.L1 * self.L1
            - self.L2 * self.L2
        ) / (2.0 * self.L1 * self.L2)

        # Проверка рабочей области
        if D < -1.0 or D > 1.0:
            return None

        # Угол базы
        self.q1 = math.radians(angle)

        # Угол локтя
        self.q3 = -math.acos(D)

        # Угол плеча
        self.q2 = (
            math.atan2(z, x)
            - math.atan2(
                self.L2 * math.sin(self.q3),
                self.L1 + self.L2 * math.cos(self.q3)
            )
        )

        # Угол кисти
        sin_q4 = (
            self.L1 * math.sin(self.q2) - z
        ) / self.L2

        if not -1.0 <= sin_q4 <= 1.0:
            return None

        self.q4 = math.asin(sin_q4)

        # Позиции Dynamixel
        self.pos1 = int(angle * 4096 / 360)

        self.pos2 = self.angle_to_pos(
            self.q2
            + self.ik_offsets[1]
            + self.calibration_offsets[1]
        )

        self.pos3 = self.angle_to_pos(
            self.q3
            + self.ik_offsets[2]
            + self.calibration_offsets[2]
        )

        self.pos4 = (
            self.angle_to_pos(
                self.q4
                + self.ik_offsets[3]
                + self.calibration_offsets[3]
            )
            + self.POSITION_OFFSET_DXL4
        )

        return self.get_result()

    # --------------------------------------------------------
    # Результат
    # --------------------------------------------------------

    def get_result(self):
        return [self.pos1, self.pos2, self.pos3, self.pos4]

    # --------------------------------------------------------
    # Сброс калибровки
    # --------------------------------------------------------

    def reset_calibration(self):
        self.calibration_offsets = [0.0, 0.0, 0.0, 0.0]


ik = InverseKinematics()
print(ik.calculate(200, 200, 180))