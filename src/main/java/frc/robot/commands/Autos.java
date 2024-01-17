// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PositionEstimationSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(DriveSubsystem subsystem) {
    return Commands.waitSeconds(0); // Replace with real auto
  }

  public static Command moveSeconds(DriveSubsystem driveSub, double speed, double seconds) {
    return Commands.run(() -> {
      driveSub.setSpeed(speed);
    }).withTimeout(seconds).andThen(driveSub.stop());
  }

  public static Command moveThenBalance(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm, ElevatorSubsystem elevator) {
    return score(intake, arm, elevator)
      .andThen(moveSeconds(drive, 0.75, 2))
      .andThen(Commands.waitSeconds(1))
      .andThen(() -> drive.setBrakes(IdleMode.kBrake))
      .andThen(moveSeconds(drive, 0.5, 1.63))
      .andThen(drive.balanceCommand());
  }

  public static Command moveThenBalanceMobility(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm, ElevatorSubsystem elevator) {
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
  public static Command score(IntakeSubsystem intake, ArmSubsystem subsystem, ElevatorSubsystem elevator) {
      return Setpoints.L3(subsystem, elevator).withTimeout(2.5)
        .andThen(Commands.waitSeconds(0.5))
        .andThen(intake.intake().withTimeout(0.25))
        .andThen(intake.stopCommand())
        .andThen(Setpoints.Stow(subsystem, elevator));
  }

  public static Command scoreThenMove(IntakeSubsystem intake, ArmSubsystem subsystem, ElevatorSubsystem elevator, DriveSubsystem drive) {
    return score(intake, subsystem, elevator).andThen(moveSeconds(drive, 0.5, 5));
  }

  public static Command moveToCircle(PositionEstimationSubsystem vision, DriveSubsystem drive, double speed, Translation2d targetCenter, double targetRadius) {
    return Commands.run(() -> {
      drive.setSpeed(speed);
    }).onlyWhile(() -> {
      var pose = vision.getPose();
      if (pose.isPresent()) {
        var poseValue = pose.get();

        double distance2 = Math.pow(poseValue.getX() - targetCenter.getX(), 2) + Math.pow(poseValue.getY() - targetCenter.getY(), 2);
        return distance2 > targetRadius * targetRadius;
      }

      return false;
    }).finallyDo(() -> drive.setSpeed(0));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
