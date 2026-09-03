'''Streamlined User Interface (robot.py)
Removed Data Entry Clutter: You no longer need to type out 7 numbers (X, Y, Z, Spd0, Spd1, Spd2, Spd3) for every single move.
Default Speeds: The system now initializes with a baseline speed array (40, 100, 100, 75), which is automatically fed into
your compensator for timing calculations.
On-the-Fly Configuration: Added a SPEED s0 s1 s2 s3 command to update those default speeds mid-session without restarting the script.
Clean Movement Commands: Standard movement now only requires entering the target X Y Z coordinates.
Universal Motor Control (communications.py & robot.py)
New Broadcast Function: Added the set_motor_state(state) method to your communications pipeline.
Global Toggles: Typing ON or OFF in the Python terminal now instantly broadcasts that string to both Arduinos simultaneously,
allowing you to quickly kill power to the arm or lock it in place.
The POS Command (Position Tracking): Typing POS now calculates and prints the current real-world XYZ coordinates of your end
effector (using your Forward Kinematics solver) along with the exact angles of all four joints. It does this instantly without
sending any movement commands to the motors.
The HOME Command (Quick Reset): Typing HOME acts as a shortcut. It automatically sets your target to your designated starting
coordinates [0.0262, 0.5761, 0.0] and routes it right through your standard kinematics and compensator pipeline, saving you from
having to type those precise decimals every time you want to reset the arm.
'''

import numpy as np
from IK_FK import KinematicsSolver
from compensator import Compensator
from communications import RobotComms


def main():
    solver = KinematicsSolver()
    comp = Compensator()
    comms = RobotComms()

    # Track absolute current state in DEGREES
    current_angles_deg = np.zeros(4)

    # Default Speeds setup
    speeds = [40.0, 100.0, 100.0, 75.0]

    # --- NEW: Welcome Screen & Command List ---
    print("\n" + "=" * 55)
    print("      CONTROL INTERFACE")
    print("=" * 55)
    print("Available Commands:")
    print("  X Y Z               : Move to target coordinates")
    print("  HOME                : Move to default start [0.0262, 0.5761, 0.0]")
    print("  SPEED s0 s1 s2 s3   : Update motor speeds (e.g., SPEED 40 100 100 75)")
    print("  POS                 : Show current joint angles and XYZ position")
    print("  ON                  : Engage and lock all motors")
    print("  OFF                 : Disengage all motors (free spin)")
    print("  Q                   : Quit the program")
    print("=" * 55)

    while True:
        print(f"\nCurrent Speeds: {speeds}")
        user_input = input("Enter command: ").strip().upper()

        if user_input == 'Q':
            break

        # Handle Position Request
        if user_input == 'POS':
            current_xyz = solver.solve_fk(np.radians(current_angles_deg))
            print(f"\n--- Current Arm Status ---")
            print(
                f"Joint Angles (Deg): [{current_angles_deg[0]:.2f}, {current_angles_deg[1]:.2f}, {current_angles_deg[2]:.2f}, {current_angles_deg[3]:.2f}]")
            print(f"End Effector XYZ:   [{current_xyz[0]:.4f}, {current_xyz[1]:.4f}, {current_xyz[2]:.4f}]\n")
            continue

        # Handle Motor Toggles
        if user_input == 'OFF':
            comms.set_motor_state("OFF")
            continue
        if user_input == 'ON':
            comms.set_motor_state("ON")
            continue

        # Handle Speed Updates
        if user_input.startswith("SPEED"):
            parts = user_input.split()[1:]
            if len(parts) == 4:
                try:
                    speeds = list(map(float, parts))
                    print("Speeds successfully updated.")
                except ValueError:
                    print("Error: Speeds must be numbers.")
            else:
                print("Error: You must provide exactly 4 speeds.")
            continue

        # Handle Homing Command
        if user_input == 'HOME':
            target_xyz = [0.0262, 0.5761, 0.0]
            print(f"Homing arm to: {target_xyz}")

        # If it wasn't a special command, assume it's an X Y Z coordinate
        else:
            try:
                target_xyz = list(map(float, user_input.split()))
                if len(target_xyz) != 3:
                    print("Error: Please enter exactly 3 coordinates for movement.")
                    continue
            except ValueError:
                print("Unrecognized command or invalid coordinate format.")
                continue

        # --- Proceed with movement math ---

        # 1. Run IK
        ik_rad = solver.solve_ik(target_xyz)
        # Assuming IK returns [origin, joint0, joint1, joint2, joint3, tip]
        ik_target_deg = np.degrees([ik_rad[1], ik_rad[2], ik_rad[3], ik_rad[4]])

        # 2. Calculate ideal Delta
        delta_deg_ideal = ik_target_deg - current_angles_deg

        # 3. Compensator: Get raw steps and actual achievable Delta
        motor_steps, delta_deg_actual = comp.calculate_steps(delta_deg_ideal)

        # Calculate timing using the current 'speeds' list
        time_ms, spd_steps_0 = comp.calculate_timing(motor_steps, speeds)

        # 4. Calculate actual new angles and run FK to show user reality
        actual_new_angles = current_angles_deg + delta_deg_actual
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


