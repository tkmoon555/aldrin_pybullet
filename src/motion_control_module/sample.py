#!/usr/bin/env python
# coding: utf-8
 
import math
import numpy as np


class SimpleSimulator(object):
    def __init__(self, step_time, ref_step_data, initial_centroid_position, initial_step_position):
 
        self.gravity = 9.8 # m/s2
        self.delta_time = 0.01 # s
        self.centroid_height = 0.5 # m
 
        self.step_time = step_time
        self.ref_step_data = np.array(ref_step_data, dtype=np.float32)
 
        self.centroid_position = np.array(initial_centroid_position, dtype=np.float32)
        self.centroid_velocity = np.array([0, 0], dtype=np.float32)
 
        self.step_position = np.array(initial_step_position, dtype=np.float32)
        self.ref_step_position = np.array(initial_step_position, dtype=np.float32)
 
        self.step_count = 0

        self.stock_ref = []
        self.stock_land = []
    def spin(self):

        for _ in range(len(self.ref_step_data) + 1):

            # 支持脚の描写
            position = [self.step_position[0], self.step_position[1], 0.]

            # 線形倒立振子の方程式を積分して1歩分の重心運動を求める
            for i in range(int(self.step_time/self.delta_time)):
                centroid_acceleration = self.gravity / self.centroid_height * (self.centroid_position - self.step_position)
                self.centroid_velocity += centroid_acceleration * self.delta_time
                self.centroid_position += self.centroid_velocity * self.delta_time

                # 重心運動の描写
                position = [self.centroid_position[0], self.centroid_position[1], self.centroid_height]

            # 1歩更新
            self.step_count += 1

            # 次の目標着地点を求める
            if self.step_count - 1 >= len(self.ref_step_data):
                current_ref_step_data = np.array([0, 0, 0], dtype=np.float32)
            else:
                current_ref_step_data = self.ref_step_data[self.step_count - 1]
            current_ref_step_angle = current_ref_step_data[2]
            current_ref_step_size = np.array([
                current_ref_step_data[0] * np.cos(current_ref_step_angle) - current_ref_step_data[1] * np.sin(current_ref_step_angle),
                current_ref_step_data[0] * np.sin(current_ref_step_angle) + current_ref_step_data[1] * np.cos(current_ref_step_angle)])
            self.ref_step_position = self.ref_step_position + current_ref_step_size

            # 次の1歩行周期後の重心座標と重心速度の目標値を求める
            if self.step_count >= len(self.ref_step_data):
                next_ref_step_data = np.array([0, 0, 0], dtype=np.float32)
            else:
                next_ref_step_data = self.ref_step_data[self.step_count]
            next_ref_step_angle = next_ref_step_data[2]
            next_ref_step_size = np.array([
                next_ref_step_data[0] * np.cos(next_ref_step_angle) - next_ref_step_data[1] * np.sin(next_ref_step_angle),
                next_ref_step_data[0] * np.sin(next_ref_step_angle) + next_ref_step_data[1] * np.cos(next_ref_step_angle)])

            Tc = math.sqrt(self.centroid_height / self.gravity)
            C = math.cosh(self.step_time / Tc)
            S = math.sinh(self.step_time / Tc)

            h_xy = next_ref_step_size / 2
            h_v_xy = np.array((
                (C + 1) / (Tc * S) * h_xy[0],
                (C - 1) / (Tc * S) * h_xy[1]
            ), dtype=np.float32)
            next_target_position = self.ref_step_position + h_xy
            next_target_velocity = h_v_xy

            # 次の修正した着地点を求める
            a = 10.
            b = 1.
            D = a * (C - 1)**2 + b * (S / Tc)**2

            xyi = self.centroid_position
            vxyi = self.centroid_velocity
            xyd = next_target_position
            vxyd = next_target_velocity
            next_step_position = -a * (C - 1) / D * (xyd - C * xyi - Tc * S * vxyi) - b * S / (Tc * D) * (vxyd - S / Tc * xyi - C * vxyi)
            self.step_position = next_step_position
            self.stock_land.append(next_step_position.copy())

if __name__ == '__main__':

    simulator = SimpleSimulator(
        step_time = 0.5,
        ref_step_data = [
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.0],
            [0.1, 0.1, 0.0]],
        initial_centroid_position = [0, 0.04],
        initial_step_position = [0, 0.05]
    )

    simulator.spin()
