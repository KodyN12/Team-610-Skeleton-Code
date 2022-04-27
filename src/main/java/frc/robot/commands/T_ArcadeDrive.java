package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.DriveTrain.*;

public class T_ArcadeDrive extends CommandBase {
    private Drivetrain m_driveInst;
    private XboxController driver;
    // private SlewRateLimiter m_turnRate;

    public T_ArcadeDrive(){
        m_driveInst = Drivetrain.getInstance();
        driver = RobotContainer.s_driver;
        // m_turnRate = new SlewRateLimiter(VAL_TURNRATE);
        this.addRequirements(m_driveInst);
    }

    @Override
    public void initialize(){

    }

    /**
     * controls the drivetrain to move around according to the driver's controls
     */
    @Override
    public void execute() {
        double y = MathUtil.applyDeadband(driver.getLeftY(), VAL_DEADBAND);
        double x = MathUtil.applyDeadband(driver.getRightX(), VAL_DEADBAND);
        boolean turbo = RobotContainer.s_driver.getLeftBumper();

        y = y * y * y;
        x = x * x * x;

        y *= turbo ? 0.75 : 0.6;
        x *= 0.7;
        double leftSpeed = -y + x;
        double rightSpeed = -y - x;
        m_driveInst.setLeft(leftSpeed);
        m_driveInst.setRight(rightSpeed);
    }
}