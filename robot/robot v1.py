pimport
numpy as np
from IK_FK import KinematicsSolver  # Assuming you wrap your IK code in a class
from compensator import Compensator
from communications import RobotComms


def main():
    solver = KinematicsSolver()
    comp = Compensator()
    comms = RobotComms()

    # Track absolute current state in DEGREES
    current_angles_deg = np.zeros(4)

    while True:
        user_input = input("\nEnter <X Y Z Spd0 Spd1 Spd2 Spd3> (or 'q' to quit): ")
        if user_input.lower() == 'q':
            break

        try:
            parts = list(map(float, user_input.split()))
            target_xyz = parts[0:3]
            speeds = parts[3:7]  # deg/sec
        except ValueError:
            print("Invalid input format.")
            continue

        # 1. Run IK
        ik_rad = solver.solve_ik(target_xyz)
        # Assuming IK returns [origin, joint0, joint1, joint2, joint3, tip]
        ik_target_deg = np.degrees([ik_rad[1], ik_rad[2], ik_rad[3], ik_rad[4]])

        # 2. Calculate ideal Delta
        delta_deg_ideal = ik_target_deg - current_angles_deg

        # 3. Compensator: Get raw steps and actual achievable Delta
        motor_steps, delta_deg_actual = comp.calculate_steps(delta_deg_ideal)

        # Calculate timing
        time_ms, spd_steps_0 = comp.calculate_timing(motor_steps, speeds)

        # 4. Calculate actual new angles and run FK to show user reality
        actual_new_angles = current_angles_deg + delta_deg_actual
        # Run FK (you'll need to wrap your FK logic in IK_FK.py to take these angles)
        actual_xyz = solver.solve_fk(np.radians(actual_new_angles))

        print(f"\n--- Target vs Reality ---")
        print(f"Target XYZ: {target_xyz}")
        print(f"Actual Reachable XYZ: [{actual_xyz[0]:.4f}, {actual_xyz[1]:.4f}, {actual_xyz[2]:.4f}]")
        print(f"Motor Steps: Base:{motor_steps[0]}, L1:{motor_steps[1]}, L2:{motor_steps[2]}, L3:{motor_steps[3]}")
        print(f"Movement Time (L1-L3): {time_ms}ms")

        confirm = input("Execute move? (y/n): ")
        if confirm.lower() == 'y':
            # 5. Dispatch to Arduinos
            comms.send_motion(
                motor_steps[0], motor_steps[1], motor_steps[2], motor_steps[3],
                time_ms, spd_steps_0
            )
            comms.wait_for_done()

            # 6. Update internal state ONLY after successful move
            current_angles_deg = actual_new_angles


if __name__ == "__main__":
    main()

