import numpy as np
import time
from IK_FK import KinematicsSolver
from compensator import Compensator
from communications import RobotComms


# --- DEFINED LOOP SEQUENCE (XYZ) ---
LOOP_SEQUENCE = [


   [0.5, 0.2, 0.1],


   [.4, 0, 0.4],


   [0.0, 0.5, 0.0],


   [0.0262, 0.5761, 0.0]  # Return to home


]




def process_and_move(target_xyz, current_angles, solver, comp, comms, speeds, auto_confirm=False):
   """
   Handles the IK -> Compensator -> Communication pipeline.
   Returns the updated current_angles_deg.
   """
   # 1. Run IK
   ik_rad = solver.solve_ik(target_xyz)
   ik_target_deg = np.degrees([ik_rad[1], ik_rad[2], ik_rad[3], ik_rad[4]])


   # 2. Calculate ideal Delta
   delta_deg_ideal = ik_target_deg - current_angles


   # 3. Compensator: Get raw steps and actual achievable Delta
   motor_steps, delta_deg_actual = comp.calculate_steps(delta_deg_ideal)
   time_ms, spd_steps_0 = comp.calculate_timing(motor_steps, speeds)


   # 4. Calculate reality
   actual_new_angles = current_angles + delta_deg_actual
   actual_xyz = solver.solve_fk(np.radians(actual_new_angles))


   # Detailed Printout
   print(f"\n--- Target vs Reality ---")
   print(f"Target XYZ: {target_xyz}")
   print(f"Actual Reachable XYZ: [{actual_xyz[0]:.4f}, {actual_xyz[1]:.4f}, {actual_xyz[2]:.4f}]")
   print(f"Motor Steps: Base:{motor_steps[0]}, L1:{motor_steps[1]}, L2:{motor_steps[2]}, L3:{motor_steps[3]}")
   print(f"Movement Time: {time_ms}ms")


   if auto_confirm:
       confirm = 'y'
   else:
       confirm = input("Execute move? (y/n): ").lower()


   if confirm == 'y':
       comms.send_motion(motor_steps[0], motor_steps[1], motor_steps[2], motor_steps[3], time_ms, spd_steps_0)
       comms.wait_for_done()
       return actual_new_angles


   return current_angles




def main():
   solver = KinematicsSolver()
   comp = Compensator()
   comms = RobotComms()


   current_angles_deg = np.zeros(4)
   speeds = [40.0, 100.0, 100.0, 100]


   print("\n" + "=" * 55)
   print("      CONTROL INTERFACE - VERSION 2.0")
   print("=" * 55)
   print("  X Y Z               : Move to target coordinates")
   print("  LOOP                : Play the XYZ sequence")
   print("  HOME                : Move to default start")
   print("  POS                 : Show current joint angles and XYZ")
   print("  SPEED s0 s1 s2 s3   : Update motor speeds")
   print("  ON / OFF            : Engage/Disengage motors")
   print("  Q                   : Quit")
   print("=" * 55)


   while True:
       print(f"\nCurrent Speeds: {speeds}")
       user_input = input("Enter command: ").strip().upper()


       if user_input == 'Q':
           break


       # --- 1. POS Command (RESTORED) ---
       if user_input == 'POS':
           current_xyz = solver.solve_fk(np.radians(current_angles_deg))
           print(f"\n--- Current Arm Status ---")
           print(f"Joint Angles (Deg): {np.round(current_angles_deg, 2)}")
           print(f"End Effector XYZ:   {np.round(current_xyz, 4)}\n")
           continue


       if user_input == 'OFF':
           comms.set_motor_state("OFF")
           continue


       if user_input == 'ON':
           comms.set_motor_state("ON")
           continue


       # --- 2. SPEED Update (RESTORED) ---
       if user_input.startswith("SPEED"):
           parts = user_input.split()[1:]
           if len(parts) == 4:
               speeds = list(map(float, parts))
               print("Speeds updated.")
           continue


       # --- 3. LOOP Command (NEW) ---
       if user_input == 'LOOP':
           print(f"Starting loop of {len(LOOP_SEQUENCE)} points.")
           try:
               for pt in LOOP_SEQUENCE:
                   # auto_confirm=True so it doesn't ask 'y/n' for every point in the loop
                   current_angles_deg = process_and_move(pt, current_angles_deg, solver, comp, comms, speeds,
                                                         auto_confirm=True)
                   time.sleep(0.5)
               print("Loop finished.")
           except KeyboardInterrupt:
               print("\nLoop stopped.")
           continue


       # --- 4. Homing & Manual Coordinates ---
       if user_input == 'HOME':
           target_xyz = [0.0262, 0.5761, 0.0]
       else:
           try:
               target_xyz = list(map(float, user_input.split()))
               if len(target_xyz) != 3:
                   print("Error: Need 3 coordinates.")
                   continue
           except ValueError:
               print("Unrecognized command.")
               continue


       # Execute Manual Move (auto_confirm=False so you can review the 'Target vs Reality')
       current_angles_deg = process_and_move(target_xyz, current_angles_deg, solver, comp, comms, speeds,
                                             auto_confirm=False)




if __name__ == "__main__":
   main()

