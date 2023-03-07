// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that has balance code in .execute and finish code in .isFinished */
public class BalanceCommand extends CommandBase {
  private DriveSubsystem m_drive;

  /**
  * @param drive is the DriveSubsystem that the command should attempt balancing on
  */
  public BalanceCommand(DriveSubsystem drive) {
    m_drive = drive;
  }

  @Override
  public void execute() {
    double pitchAngleRadians = m_drive.getPitch() * (Math.PI / 180.0);
    double xAxisRate = Math.sin(pitchAngleRadians);
    m_drive.tankDriveVolts(xAxisRate * 7, xAxisRate * 7);
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(m_drive.getPitch()) < 3) {
      return true;
    }
    return false;
  }
}
