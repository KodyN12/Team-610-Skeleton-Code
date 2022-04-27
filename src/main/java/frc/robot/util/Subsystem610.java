package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class Subsystem610 implements Subsystem, Sendable {
    //! Class Members
    private int m_loopCounter;
    
    /**
     * Register the given subsystem on the command scheduler using the name provided
     * @param subsytemName Name to use to register the subsystem
     */
    public Subsystem610(String subsytemName) {
        SendableRegistry.addLW(this, subsytemName, subsytemName);
        CommandScheduler.getInstance().registerSubsystem(this);
        m_loopCounter = 0;
    }

    //! Methods all subsystems inherit
    /**
     * Increment the loop counter for this subsystem, should be done once per code cycle
     */
    public synchronized void incrementLoopCount() {
        m_loopCounter++;
    }

    //! Accessors/Mutators
    /**
     * Get the current loop counter value for this subsystem
     * @return The loop counter value, should be the number of code cycles iterated
     */
    public synchronized int getLoopCount() {
        return m_loopCounter;
    }

    /**
     * Reset the loop counter to 0 for this subsystem
     */
    public synchronized void resetLoopCount() {
        m_loopCounter = 0;
    }

    public abstract void addToDriveTab(ShuffleboardTab tab);
}