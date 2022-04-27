package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.MotorConfig;
import frc.robot.util.Subsystem610;

import static frc.robot.Constants.DriveTrain.*;

public class Drivetrain extends Subsystem610 {
    private static Drivetrain s_driveInst;
    private final DifferentialDriveOdometry m_odometry;
    // Batman motors are leaders, Robin motors are followers
    private TalonFX m_leftBatman, m_leftRobin, m_rightBatman, m_rightRobin;
    private PigeonIMU m_pidgey;

    // ! constructor
    private Drivetrain() {
        super("Drivetrain");
        m_leftBatman = MotorConfig.configDriveMotor(CAN_LEFT_BATMAN, false, false);
        m_leftRobin = MotorConfig.configDriveFollower(CAN_LEFT_ROBIN, CAN_LEFT_BATMAN, false, false);
        m_rightBatman = MotorConfig.configDriveMotor(CAN_RIGHT_BATMAN, true, false);
        m_rightRobin = MotorConfig.configDriveFollower(CAN_RIGHT_ROBIN, CAN_RIGHT_BATMAN, true, false);

        m_pidgey = new PigeonIMU(CAN_PIGEON);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
    }

    /**
     * Singleton getInstance class
     * @return Singleton instance of the drivetrain
     */
    public static Drivetrain getInstance() {
        if (s_driveInst == null) {
            s_driveInst = new Drivetrain();
        }
        return s_driveInst;
    }

    //! Motor Control
    /**
     * Sets all drivetrain motors to coast mode
     */
    public void setCoast() {
        m_leftBatman.setNeutralMode(NeutralMode.Coast);
        m_leftRobin.setNeutralMode(NeutralMode.Coast);
        m_rightBatman.setNeutralMode(NeutralMode.Coast);
        m_rightRobin.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets all drivetrain motors to brake mode
     */
    public void setBrake() {
        m_leftBatman.setNeutralMode(NeutralMode.Brake);
        m_leftRobin.setNeutralMode(NeutralMode.Brake);
        m_rightBatman.setNeutralMode(NeutralMode.Brake);
        m_rightRobin.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the left batman to a desired output percentage
     * @param output Desired left side output as a percentage
     */
    public void setLeft(double output) {
        m_leftBatman.set(ControlMode.PercentOutput, output);
    }

    /**
     * Sets the right batman to a desired output percentage
     * @param output Desired right side output as a percenge
     */
    public void setRight(double output) {
        m_rightBatman.set(ControlMode.PercentOutput, output);
    }

    /**
     * Sets the left batman to a desired output percentage, overloaded
     * with desired control mode
     * @param mode Mode of output metric
     * @param output Desired output in percentage
     */
    public void setLeft(ControlMode mode, double output) {
        m_leftBatman.set(mode, output);
    }

    /**
     * Sets the right batman to a desired output percentage, overloaded
     * with desired control mode
     * @param mode Mode of output metric
     * @param output Desired output in percentage
     */
    public void setRight(ControlMode mode, double output) {
        m_rightBatman.set(mode, output);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        setLeft(leftVolts / 12.0);
        setRight(rightVolts / 12.0);
    }

    /**
     * Resets the drivetrain motor sensors to a value of 0
     */
    public void resetSensors() {
        m_leftBatman.setSelectedSensorPosition(0);
        m_rightBatman.setSelectedSensorPosition(0);
    }

    /**
     * Resets the odometry of the whole robot, a.k.a the sensor values as well as
     * its idea of its location on the field
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        resetSensors();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(m_pidgey.getFusedHeading()));
    }

    /**
     * @return The number of meters the left batman has travelled
     */
    public double getLeftMeters() {
        return m_leftBatman.getSelectedSensorPosition() / UNIT_TICKS_PER_REV * UNIT_DIST_PER_REV;
    }

    /**
     * @return The number of meters the right batman has travelled
     */
    public double getRightMeters() {
        return m_rightBatman.getSelectedSensorPosition() / UNIT_TICKS_PER_REV * UNIT_DIST_PER_REV;
    }

    //! Default Methods
    @Override
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(m_pidgey.getFusedHeading()), getLeftMeters(), getRightMeters());
    }

    /**
     * @return The position of the robot on the field
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * @return The current speed of the wheels of the robot
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                m_leftBatman.getSelectedSensorVelocity() / UNIT_TICKS_PER_REV * UNIT_DIST_PER_REV * 10,
                m_rightBatman.getSelectedSensorVelocity() / UNIT_TICKS_PER_REV * UNIT_DIST_PER_REV * 10);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
    }

    @Override
    public void addToDriveTab(ShuffleboardTab tab) {
        // TODO Auto-generated method stub
    }

}