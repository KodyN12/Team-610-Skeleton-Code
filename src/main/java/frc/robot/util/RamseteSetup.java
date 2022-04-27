package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

import static frc.robot.Constants.DriveTrain.*;

public final class RamseteSetup {
    public static Drivetrain m_driveInst = Drivetrain.getInstance();

    private static RamseteController s_controller = new RamseteController(VAL_RAMSETE_B, VAL_RAMSETE_ZETA);
    private static SimpleMotorFeedforward s_feedForward = new SimpleMotorFeedforward(VAL_KS, VAL_KV, VAL_KA);
    private static PIDController s_PID = new PIDController(VAL_KP, 0, VAL_KD);
    private static RamseteCommand s_ramseteCommand;


    public static Trajectory initializeTrajectory(List<Pose2d> waypoints, boolean reversed){
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(VAL_KS,VAL_KV,VAL_KA),DRIVE_KINEMATICS,10);
        
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                VAL_MAX_VELO,
                VAL_MAX_ACCEL)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DRIVE_KINEMATICS)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        config.setReversed(reversed);
        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    /**
     * Creates a new Ramsete path for the robot to follow with the passed in waypoints
     * @param waypoints The list of waypoints you would like to pass through
     * @param reversed If you want to drive backwards, this should be true
     * @return A RamseteCommand, a.k.a the path for the robot to follow
     */
    public static RamseteCommand initializeRamsetePath(Trajectory exampleTrajectory) {
        // Create a voltage constraint to ensure we don't accelerate too fast 
        s_ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                m_driveInst::getPose,
                s_controller,
                s_feedForward,
                DRIVE_KINEMATICS,
                m_driveInst::getWheelSpeeds,
                s_PID,
                s_PID,
                // RamseteCommand passes volts to the callback
                m_driveInst::tankDriveVolts,
                m_driveInst);

        // Reset odometry to the starting pose of the trajectory.
        m_driveInst.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        // ramseteCommand.andThen(() -> (driveTrain).tankDriveVolts(0, 0));
        return s_ramseteCommand;
    }
}