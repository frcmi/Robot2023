// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(DriveSubsystem subsystem) {
    return Commands.waitSeconds(0); // Replace with real auto
  }

  public static CommandBase moveSeconds(DriveSubsystem driveSub, double speed, double seconds) {
    return Commands.run(() -> {
      driveSub.setSpeed(speed);
    }).withTimeout(seconds).andThen(driveSub.stop());
  }

  public static CommandBase scoreThenBalance(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm, ElevatorSubsystem elevator) {
    return score(intake, arm, elevator)
      .andThen(moveSeconds(drive, 0.75, 2))
      .andThen(Commands.waitSeconds(1))
      .andThen(() -> drive.setBrakes(IdleMode.kBrake))
      .andThen(moveSeconds(drive, 0.5, 1.63))
      .andThen(drive.balanceCommand());
  }

  public static CommandBase scoreMidThenBalance(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm, ElevatorSubsystem elevator) {
    return scoreMid(intake, arm, elevator)
      .andThen(moveSeconds(drive, 0.75, 2))
      .andThen(Commands.waitSeconds(1))
      .andThen(() -> drive.setBrakes(IdleMode.kBrake))
      .andThen(moveSeconds(drive, 0.5, 1.63))
      .andThen(drive.balanceCommand());
  }

  public static CommandBase scoreThenBalanceMobility(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm, ElevatorSubsystem elevator) {
    return score(intake, arm, elevator)
      .andThen(moveSeconds(drive, 0.75, 2))
      .andThen(Commands.waitSeconds(0.8)) // wait for charge station to tilt
      .andThen(moveSeconds(drive, 0.5, 0.2)) // go over charge station for mobility
      .andThen(Commands.waitSeconds(0.8)) // wait for charge station to tilt
      .andThen(() -> drive.setBrakes(IdleMode.kBrake))
      .andThen(moveSeconds(drive, -0.75, 1)) // back up on to charge station
      .andThen(Commands.waitSeconds(0.8)) // wait
      .andThen(drive.balanceCommand());
  }

  // Right now only cone l3
  public static CommandBase score(IntakeSubsystem intake, ArmSubsystem subsystem, ElevatorSubsystem elevator) {
      return Setpoints.L3(subsystem, elevator).withTimeout(2.5)
        .andThen(Commands.waitSeconds(0.5))
        .andThen(intake.intake().withTimeout(0.25))
        .andThen(intake.stopCommand())
        .andThen(Setpoints.Stow(subsystem, elevator))
        .andThen(Commands.waitSeconds(1));
  }

  public static CommandBase scoreMid(IntakeSubsystem intake, ArmSubsystem subsystem, ElevatorSubsystem elevator) {
    return Setpoints.L2(subsystem, elevator).withTimeout(2.5)
      .andThen(Commands.waitSeconds(0.5))
      .andThen(intake.intake().withTimeout(0.25))
      .andThen(intake.stopCommand())
      .andThen(Setpoints.Stow(subsystem, elevator))
      .andThen(Commands.waitSeconds(0.75));
}

  public static CommandBase scoreThenMove(IntakeSubsystem intake, ArmSubsystem subsystem, ElevatorSubsystem elevator, DriveSubsystem drive) {
    return score(intake, subsystem, elevator).andThen(moveSeconds(drive, 0.3, 5));
  }

  public static CommandBase scoreMidThenMove(IntakeSubsystem intake, ArmSubsystem subsystem, ElevatorSubsystem elevator, DriveSubsystem drive) {
    return scoreMid(intake, subsystem, elevator).andThen(moveSeconds(drive, 0.3, 5));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
