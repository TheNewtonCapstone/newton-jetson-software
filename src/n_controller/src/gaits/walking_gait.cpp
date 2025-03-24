#include "gaits/walking_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;

WalkingGait::WalkingGait(const rclcpp::NodeOptions &options)
    : BaseGait("harmonic_gait", true, options),
      frequency(1), phase(0), time_step(0.01), swing_height(0.5), step_length(0.25), steps(0), forward_speed(0.08)

{
  Logger::INFO("harmonic_gait", "time,angular_velocity_x,angular_velocity_y,angular_velocity_z,projected_gravity_x,projected_gravity_y,projected_gravity_z,linear_velocity_x,linear_velocity_y,angular_velocity_z,"
                                "joint_delta_1,joint_delta_2,joint_delta_3,joint_delta_4,joint_delta_5,joint_delta_6,joint_delta_7,joint_delta_8,joint_delta_9,joint_delta_10,joint_delta_11,joint_delta_12,"
                                "joint_velocity_1,joint_velocity_2,joint_velocity_3,joint_velocity_4,joint_velocity_5,joint_velocity_6,joint_velocity_7,joint_velocity_8,joint_velocity_9,joint_velocity_10,joint_velocity_11,joint_velocity_12"
                                "action_1,action_2,action_3,action_4,action_5,action_6,action_7,action_8,action_9,action_10,action_11,action_12");
  BaseGait::init();
};

result<void> WalkingGait::move()
{
      // update the phase 
      phase = (phase + time_step * frequency) % 1.0f;
      // get a copy of the standing pose
      auto swing_pos = leg_standing_positions;

      // determ which leg is in swing phase
      // each leg gets 1/4 of the cycle, with 25% duty factor
      auto leg_phase_duration = 1.0 / num_legs;

      // determine which leg is in swing phase
      auto swing_leg_idx = static_cast<int>(phase / leg_phase_duration);
      Logger::ERROR("Walking Gaits", "Swing Leg Index: %s", std::to_string(swing_leg_idx));
      

      return result<void>::success();





        // leg_phase_duration = 1.0 / self.num_legs
        // swing_leg_idx = int(self.phase / leg_phase_duration)
        // swing_leg = self.leg_sequence[swing_leg_idx]

        // # Calculate normalized phase for the current swing leg
        // swing_phase = (self.phase - swing_leg_idx * leg_phase_duration) / leg_phase_duration

        // # All legs except the swing leg are in stance phase
        // stance_legs = [leg for leg in self.leg_sequence if leg != swing_leg]

        // # Apply leg motions for STANCE legs - provide propulsion and stability
        // for leg in stance_legs:
        //     # In stance phase, legs should push backward to propel the robot forward
        //     # For front legs: negative hip adjustment pushes backward
        //     # For hind legs: positive hip adjustment pushes backward
        //     if leg.startswith("f"):  # Front legs
        //         direction = -1.0  # Push backward
        //     else:  # Hind legs
        //         direction = 1.0  # Push backward

        //     # Create a backward-pushing motion during stance
        //     # For walking, we want a smoother, more continuous push
        //     stance_phase = 0.5  # Middle of stance
        //     hip_stance_adjust = direction * self.step_length * 0.5  # Consistent push

        //     # Apply the stance motion (push backward)
        //     pose[leg][1] = self.stance_pose[leg][1] + hip_stance_adjust

        //     # Apply slight downward pressure during stance for better grip
        //     pose[leg][2] = self.stance_pose[leg][2] - 0.05

        //     # Set higher forces for stance legs through joints
        //     if leg == "fl":
        //         for joint, angle in zip(self.robot.FL_joints, pose[leg]):
        //             p.setJointMotorControl2(
        //                 self.robot.id, joint,
        //                 p.POSITION_CONTROL,
        //                 targetPosition=angle,
        //                 force=self.stance_force
        //             )
        //     elif leg == "fr":
        //         for joint, angle in zip(self.robot.FR_joints, pose[leg]):
        //             p.setJointMotorControl2(
        //                 self.robot.id, joint,
        //                 p.POSITION_CONTROL,
        //                 targetPosition=angle,
        //                 force=self.stance_force
        //             )
        //     elif leg == "hl":
        //         for joint, angle in zip(self.robot.HL_joints, pose[leg]):
        //             p.setJointMotorControl2(
        //                 self.robot.id, joint,
        //                 p.POSITION_CONTROL,
        //                 targetPosition=angle,
        //                 force=self.stance_force
        //             )
        //     elif leg == "hr":
        //         for joint, angle in zip(self.robot.HR_joints, pose[leg]):
        //             p.setJointMotorControl2(
        //                 self.robot.id, joint,
        //                 p.POSITION_CONTROL,
        //                 targetPosition=angle,
        //                 force=self.stance_force
        //             )

        // # Apply leg motion for the SWING leg
        // # For the height (knee) motion, use a parabolic trajectory
        // height_factor = 4 * swing_phase * (1 - swing_phase)  # Parabolic curve: 0->1->0
        // lift_amount = self.swing_height * height_factor

        // # For front/back (hip) motion, we want a smooth trajectory:
        // # Start backward, move forward during swing, end forward
        // if swing_leg.startswith("f"):  # Front legs
        //     direction = -1.0  # Moving from back to front
        // else:  # Hind legs
        //     direction = 1.0  # Moving from front to back

        // # Create a smooth forward motion during swing
        // # From back to front during swing (linear interpolation)
        // forward_factor = swing_phase
        // hip_swing_adjust = direction * self.step_length * (1 - 2 * forward_factor)

        // # Apply the swing trajectory to the current swing leg
        // pose[swing_leg][1] = self.stance_pose[swing_leg][1] + hip_swing_adjust
        // pose[swing_leg][2] = self.stance_pose[swing_leg][2] + lift_amount

        // # Set lower forces for swing leg through joints
        // if swing_leg == "fl":
        //     for joint, angle in zip(self.robot.FL_joints, pose[swing_leg]):
        //         p.setJointMotorControl2(
        //             self.robot.id, joint,
        //             p.POSITION_CONTROL,
        //             targetPosition=angle,
        //             force=self.swing_force
        //         )
        // elif swing_leg == "fr":
        //     for joint, angle in zip(self.robot.FR_joints, pose[swing_leg]):
        //         p.setJointMotorControl2(
        //             self.robot.id, joint,
        //             p.POSITION_CONTROL,
        //             targetPosition=angle,
        //             force=self.swing_force
        //         )
        // elif swing_leg == "hl":
        //     for joint, angle in zip(self.robot.HL_joints, pose[swing_leg]):
        //         p.setJointMotorControl2(
        //             self.robot.id, joint,
        //             p.POSITION_CONTROL,
        //             targetPosition=angle,
        //             force=self.swing_force
        //         )
        // elif swing_leg == "hr":
        //     for joint, angle in zip(self.robot.HR_joints, pose[swing_leg]):
        //         p.setJointMotorControl2(
        //             self.robot.id, joint,
        //             p.POSITION_CONTROL,
        //             targetPosition=angle,
        //             force=self.swing_force
        //         )

        // # Increment debug counter and print info periodically
        // self.steps += 1
        // if self.steps % 50 == 0:
        //     print(f"Phase: {self.phase:.2f}")
        //     print(f"Swing Leg: {swing_leg}")
        //     print(f"Stance Legs: {stance_legs}")
        //     print(f"Normalized Swing Phase: {swing_phase:.2f}")

        //     # Get base position and velocity for monitoring
        //     pos, orn = p.getBasePositionAndOrientation(self.robot.id)
        //     linear_vel, angular_vel = p.getBaseVelocity(self.robot.id)
        //     print(f"Position: {pos}, Forward velocity: {linear_vel[0]:.4f} m/s")

        // return pose
        
}
