import numpy as np
from ikpy.chain import Chain




class KinematicsSolver:
   def __init__(self):
       # URDF Configuration
       self.urdf_path = r"C:\Users\Hride\Downloads\protArmURDF\urdf\robotarm_assem_for_urdfv7.urdf"
       self.tip_offset = [0.1905, 0.0, 0.0]


       # Initialize the IK chain
       self.chain = Chain.from_urdf_file(
           self.urdf_path,
           base_elements=["link_static"],
           last_link_vector=self.tip_offset,
           active_links_mask=[False, True, True, True, True, False]
       )




def solve_ik(self, target_xyz):
   """
   Takes a target [x, y, z] in meters.
   Returns the full 6-element IK solution in radians.
   """
   initial_position = np.zeros(len(self.chain.links))


   solution = self.chain.inverse_kinematics(
       target_position=target_xyz,
       initial_position=initial_position
   )
   return solution




def solve_fk(self, active_angles_rad):
   """
   Takes the 4 active joint angles [base, L1, L2, L3] in radians.
   Returns the actual [x, y, z] position in meters.
   """
   # Pad the 4 active angles back into the 6-element array ikpy expects:
   # [origin (False), joint0 (True), joint1 (True), joint2 (True), joint3 (True), tip (False)]
   full_solution = [
       0.0,  # origin
       active_angles_rad[0],  # joint0 (Base)
       active_angles_rad[1],  # joint1 (L1)
       active_angles_rad[2],  # joint2 (L2)
       active_angles_rad[3],  # joint3 (L3)
       0.0  # tip
   ]


   # Calculate FK
   T = self.chain.forward_kinematics(full_solution)


   # Return only the X, Y, Z coordinates
   return T[:3, 3]


