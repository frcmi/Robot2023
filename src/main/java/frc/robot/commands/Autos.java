// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(DriveSubsystem subsystem) {
    return Commands.waitSeconds(0); // Replace with real auto
  }

  public static CommandBase moveSeconds(DriveSubsystem driveSub, double speed) {
    return Commands.run(() -> {
      driveSub.setSpeed(speed);
    }).withTimeout(2.5).andThen(driveSub.stop());
  }

  // Right now only cone l3
  public static CommandBase score(IntakeSubsystem intake, ArmSubsystem subsystem, ElevatorSubsystem elevator) {
      return Setpoints.L3(subsystem, elevator)
        .andThen(intake.intake().withTimeout(1))
        .andThen(Setpoints.Stow(subsystem, elevator));
  }

  public static CommandBase scoreThenMove(IntakeSubsystem intake, ArmSubsystem subsystem, ElevatorSubsystem elevator, DriveSubsystem drive) {
    return score(intake, subsystem, elevator).andThen(moveSeconds(drive, -0.5));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
