"""
Program CAN-Bus
Author: Fahmi Fathur Rohman
"""

import can
import numpy as np
import math

# Konstanta untuk ID CAN dan rasio konversi
CAN_ID = [0x401, 0x402, 0x403, 0x404]
CAN_ID_feedback = [0x381, 0x382, 0x383, 0x384]
RATIO = 655350


class CAN_setting():
    def __init__(self):
        # Inisialisasi status koneksi dan data posisi
        self.can_open = False
        self.firt_data = [False, False, False, False]
        self.data_pre_position = np.zeros(4).astype(int)
        self.data_new_position = np.zeros(4).astype(int)
        self.motor_position = np.zeros(4)

        try:
            # Membuka bus CAN dengan antarmuka socketcan, channel can0, dan bitrate 500000
            self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)
            self.can_open = True
        except OSError:
            # Jika gagal membuka bus, set status koneksi ke False dan keluar dari program
            self.can_open = False
            exit()

    def send_data_can(self, data):
        for i in range(len(data)):
            # Mengirim data posisi ke motor dengan format CAN
            self.bus.send(can.Message(arbitration_id=CAN_ID[i], data=[0x0f, 0x00, 0x03,
                                                                      int(hex(data[i] & 0xff), 16),
                                                                      int(hex(data[i] >> 8 & 0xff), 16),
                                                                      int(hex(data[i] >> 16 & 0xff), 16),
                                                                      int(hex(data[i] >> 32 & 0xff), 16), 0x00],
                                      is_extended_id=False))
            if self.can_open:
                # Menerima pesan umpan balik dari motor
                msg_recv = self.bus.recv()
                if hex(msg_recv.arbitration_id) == hex(CAN_ID_feedback[i]):
                    a = msg_recv.data[msg_recv.dlc - 6]
                    b = msg_recv.data[msg_recv.dlc - 5] * 0x100
                    c = msg_recv.data[msg_recv.dlc - 4] * 0x10000
                    d = msg_recv.data[msg_recv.dlc - 3] * 0x1000000
                    if not self.firt_data[i]:
                        # Menyimpan posisi awal jika data pertama diterima
                        self.data_pre_position[i] = a + b + c + d
                        self.firt_data[i] = True
                    # Menyimpan posisi baru
                    self.data_new_position[i] = a + b + c + d
                    # Menghitung posisi motor dalam radian
                    self.motor_position[i] = self.read_position(self.data_pre_position[i],
                                                                self.data_new_position[i]) / RATIO * math.pi * 2
                    # Update posisi sebelumnya dengan posisi baru
                    self.data_pre_position[i] = self.data_new_position[i]

    def read_position(self, previous_position, new_position):
        # Menghitung selisih posisi dengan mempertimbangkan wrap-around
        diff = new_position - previous_position
        if diff > 0:
            if abs(diff) <= 2**31:
                return diff
            else:
                return diff - 2**32
        elif diff < 0:
            if abs(diff) <= 2**31:
                return diff
            else:
                return diff + 2**32
        else:
            return 0

    def read_data_pre_position(self):
        # Mengembalikan posisi sebelumnya
        return self.data_pre_position

    def read_data_new_position(self):
        # Mengembalikan posisi baru
        return self.data_new_position

    def read_motor_position(self):
        # Mengembalikan posisi motor dalam radian
        return self.motor_position
