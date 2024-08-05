"""
Program PID Controller
Author: Fahmi Fathur Rohman
"""

class PID:
    # Inisialisasi variabel kesalahan integral, kesalahan derivatif, dan kesalahan terakhir
    i_err = 0
    d_err = 0
    last_err = 0

    def __init__(self, kp=0, ki=0, kd=0) -> None:
        # Inisialisasi gain (Kp, Ki, Kd) untuk kontroler PID
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self, error, limit) -> float:
        # Menghitung kesalahan derivatif dan integral
        self.d_err = error - self.last_err
        self.i_err += error
        self.last_err = error
        # Menghitung output PID berdasarkan gain dan kesalahan
        result_pid = (self.kp * error) + (self.ki * self.i_err) + (self.kd * self.d_err)
        # Memastikan hasil PID berada dalam batas yang ditentukan
        if result_pid >= 0:
            return limit if result_pid > limit else result_pid
        else:
            return -limit if result_pid < -limit else result_pid

    def reset_err(self):
        # Mengatur ulang kesalahan integral, kesalahan derivatif, dan kesalahan terakhir
        self.i_err = 0
        self.d_err = 0
        self.last_err = 0
