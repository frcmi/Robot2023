// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(DriveSubsystem subsystem) {
    return Commands.waitSeconds(0); // Replace with real auto
  }

  public static CommandBase moveFiveSeconds(DriveSubsystem driveSub, double voltage, boolean reversed) {
    return Commands.run(() -> {
      Timer timer = new Timer();
      timer.start();
      while (timer.get() < 5.0) {
        if (reversed) {
          driveSub.tankDriveVolts(-voltage, -voltage);
        } else {
          driveSub.tankDriveVolts(voltage, voltage);
        }
      }
    });
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
