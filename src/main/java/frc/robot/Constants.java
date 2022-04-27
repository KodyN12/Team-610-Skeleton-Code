// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //TODO: Common
    public static final int PORT_DRIVER = 0;
    public static final int PORT_OPERATOR = 1;

    //TODO: DriveTrain
    public static class DriveTrain {
        public static final int CAN_LEFT_BATMAN = 0;
        public static final int CAN_LEFT_ROBIN = 0;
        public static final int CAN_RIGHT_BATMAN = 0;
        public static final int CAN_RIGHT_ROBIN = 0;
        public static final int CAN_PIGEON = 0;

        //* Units for Ramsete are in meters
        public static final double VAL_KS = 0;
        public static final double VAL_KV = 0;  // Less = less agressive turns
        public static final double VAL_KA = 0;
        public static final double VAL_KP = 0;
        public static final double VAL_KD = 0;

        public static final double VAL_TRACK_WIDTH = 0;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(VAL_TRACK_WIDTH);

        public static final double VAL_WHEEL_DIA = 0; // in meters
        public static final double VAL_MAX_VELO = 0; // in meters/second
        public static final double VAL_MAX_ACCEL = VAL_MAX_VELO*0.5; // in meters/second/second

        public static final double VAL_RAMSETE_B = 0;
        public static final double VAL_RAMSETE_ZETA = 0;

        public static final double UNIT_FEET_IN_METERS = 3.280;
        public static final double UNIT_METERS_IN_FEET = 0.305;

        public static final double UNIT_TICKS_PER_REV = 0;
        public static final double UNIT_DIST_PER_REV = 0;

        public static final double VAL_DEADBAND = 0.03;
        public static final double VAL_TURNRATE = 0.75;
    }

    //TODO: Vision
    public static class Vision {
        public static final double VAL_KP = 0;
        public static final double VAL_KI = 0;
        public static final double VAL_KD = 0;
    }

    //TODO: XBOX
    public static class XBoxConstants {
        public static final int BTN_X = 3;
        public static final int BTN_A = 1;
        public static final int BTN_B = 2;
        public static final int BTN_Y = 4;
        public static final int BTN_RB = 6;
        public static final int AXIS_RT = 3;
        public static final int BTN_LB = 5;
        public static final int AXIS_LT = 2;
        public static final int BTN_BACK = 7;
        public static final int BTN_START = 8;
        
        public static final int BTN_DOWN = 180;

        public static final int AXIS_LEFT_X = 0;
        public static final int AXIS_LEFT_Y = 1;
        public static final int AXIS_RIGHT_X = 4;
        public static final int AXIS_RIGHT_Y = 5;

        public static final int BTN_LEFT_JOYSTICK = 9;
        public static final int BTN_RIGHT_JOYSTICK = 10;
    }
}
