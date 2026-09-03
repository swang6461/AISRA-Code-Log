import numpy as np


class Compensator:
    def __init__(self):
        # System Constants
        self.MICROSTEPPING = 1.0
        self.STEPS_PER_REV_123 = 200.0 * self.MICROSTEPPING
        self.STEPS_PER_REV_0 = 6400.0  # NEMA 34

        self.GEAR_0 = 1.0
        self.GEAR_1 = 20.0
        self.GEAR_2 = 10.0
        self.GEAR_3 = 5.0

        self.K_12 = 1.0
        self.K_13 = 3.0
        self.K_23 = 2.0

        # Degrees per step for actual FK translation
        self.deg_per_step_0 = 360.0 / (self.STEPS_PER_REV_0 * self.GEAR_0)
        self.deg_per_step_1 = 360.0 / (self.STEPS_PER_REV_123 * self.GEAR_1)
        self.deg_per_step_2 = 360.0 / (self.STEPS_PER_REV_123 * self.GEAR_2)
        self.deg_per_step_3 = 360.0 / (self.STEPS_PER_REV_123 * self.GEAR_3)

    def calculate_steps(self, delta_thetas_deg):
        """
        Takes [d0, d1, d2, d3] in degrees.
        Returns exact integer steps for motors and the ACTUAL angle changes achieved.
        """
        d0, d1, d2, d3 = delta_thetas_deg

        # Base is uncoupled
        steps_0 = round(d0 / self.deg_per_step_0)

        # Coupled Links (Your exact math)
        m1_deg = d1
        m2_deg = d2 + (self.K_12 * d1)
        m3_deg = d3 + (self.K_23 * d2) + (self.K_13 * d1)

        steps_1 = round((m1_deg / 360.0) * self.STEPS_PER_REV_123 * self.GEAR_1)
        steps_2 = round((m2_deg / 360.0) * self.STEPS_PER_REV_123 * self.GEAR_2)
        steps_3 = round((m3_deg / 360.0) * self.STEPS_PER_REV_123 * self.GEAR_3)

        actual_d0 = steps_0 * self.deg_per_step_0
        actual_d1 = (steps_1 / (self.STEPS_PER_REV_123 * self.GEAR_1)) * 360.0

        # Reverse the compensation to find actual d2 and d3 achieved by the steps
        actual_m2_deg = (steps_2 / (self.STEPS_PER_REV_123 * self.GEAR_2)) * 360.0
        actual_d2 = actual_m2_deg - (self.K_12 * actual_d1)

        actual_m3_deg = (steps_3 / (self.STEPS_PER_REV_123 * self.GEAR_3)) * 360.0
        actual_d3 = actual_m3_deg - (self.K_23 * actual_d2) - (self.K_13 * actual_d1)

        return [steps_0, steps_1, steps_2, steps_3], [actual_d0, actual_d1, actual_d2, actual_d3]

    def calculate_timing(self, steps, speeds_deg_sec):
        """
        Calculates time required for Arduino 1 to move synchronously.
        """
        s0, s1, s2, s3 = speeds_deg_sec
        steps_1, steps_2, steps_3 = steps[1:4]

        # Find time required for each joint based on requested speed
        t1 = abs(steps_1 * self.deg_per_step_1) / s1 if s1 else 0
        t2 = abs(steps_2 * self.deg_per_step_2) / s2 if s2 else 0
        t3 = abs(steps_3 * self.deg_per_step_3) / s3 if s3 else 0

        # The longest time dictates the master time for Arduino 1
        max_time_sec = max(t1, t2, t3)
        time_ms = int(max_time_sec * 1000)

        # Arduino 2 (NEMA) speed formatting (steps per second)
        speed_steps_0 = int(s0 / self.deg_per_step_0)

        return time_ms, speed_steps_0

