// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(DriveSubsystem subsystem) {
    return Commands.waitSeconds(0); // Replace with real auto
  }

  public static CommandBase spinAuto(DriveSubsystem m_drivetrain) {
    return Commands.repeatingSequence(
        turnToAngle(90, m_drivetrain), 
        turnToAngle(271, m_drivetrain));
  }

  public static CommandBase goForSeconds(Double seconds, DriveSubsystem m_Drivetrain) {
    return Commands.sequence(
      Commands.runOnce(() -> m_Drivetrain.setSpeed(() -> 1.0, () -> 0.0, () -> false), m_Drivetrain),
      Commands.waitSeconds(seconds), 
      Commands.runOnce(() -> m_Drivetrain.stop(), m_Drivetrain));
  }

  public static CommandBase turnToAngle(double targetAngleDegrees, DriveSubsystem m_Drivetrain) {
    //new PIDCommand(null, null, null, null, null)
    return Commands.runOnce(() -> {
      PIDCommand pidCom = new PIDCommand(new PIDController(AutoConstants.kTurnP, AutoConstants.kTurnI, AutoConstants.kTurnD),
        () -> m_Drivetrain.getHeading().getDegrees(), targetAngleDegrees, (output) -> m_Drivetrain.setSpeed(() -> 0.0, () -> output, () -> false), m_Drivetrain);
      pidCom.getController().enableContinuousInput(-180, 180);
      pidCom.getController().setTolerance(AutoConstants.kTurnToleranceDeg, AutoConstants.kTurnRateToleranceDegPerS);
    });
}

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
