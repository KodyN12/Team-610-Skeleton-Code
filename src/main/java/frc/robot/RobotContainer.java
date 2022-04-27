// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.XBoxConstants.*;
import static frc.robot.Constants.*;

public class RobotContainer {
  public static XboxController s_driver;
  public static XboxController s_operator;

  public RobotContainer() {
    s_driver = new XboxController(PORT_DRIVER);
    s_operator = new XboxController(PORT_OPERATOR);
    configureButtonBindings();
  }

  /**
   * All controls for driver and operator
   */
  private void configureButtonBindings() {
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
