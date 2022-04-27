package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisTrigger extends Trigger{
    private XboxController m_joystick;
    private int m_axis;
    private BooleanSupplier m_auxCon;
    private boolean m_inverted;

    public AxisTrigger(XboxController joystick, int axis, BooleanSupplier method, boolean inverted){
        m_joystick = joystick;
        m_axis = axis;
        m_auxCon = method;
        m_inverted = inverted;
    }

    @Override
    public boolean get() {
        return (MathUtil.applyDeadband(m_joystick.getRawAxis(m_axis), 0.8) != 0) && (m_inverted ? !m_auxCon.getAsBoolean() : m_auxCon.getAsBoolean());
    }
}