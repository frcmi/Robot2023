// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kSpeedMultiplier = 0.2;
    public static final double kRotationMultiplier = 0.2;
    public static final double kElevatorSpeed = 0.3;
  }

  public static class DriveConstants {
    //Currently placeholder values, make sure to update before testing
    public static final int kFrontLeftMotorId = 1;
    public static final int kRearLeftMotorId = 2;
    public static final int kFrontRightMotorId = 3;
    public static final int kRearRightMotorId = 4;
  }

  public static class ManipulatorConstants {
    public static final int kMotorId = 7;
    public static final double kIntakeSpeed = 0.3;
    public static final double kReleaseSpeed = 0.2 * -1;
    public static final double kReleaseTime = 1.0;
    // Amps drawn by manipulator showing current spike (to detect if grabbed object) 
    public static final double kCurrentThreshold = 2.0;
  }
  
  public static class ElevatorConstants {
    public static final int kElevatorLeftMotorId = 5;
    public static final int kElevatorRightMotorId = 6;
  }

  public static class MiscellaneousConstants {
    //Currently placeholder values, make sure to update before testing
    public static final int kLightPWMPort = 5;
  }

  public static class ColorConstants {
    public static final Color8Bit kPurple = new Color8Bit(191, 64, 191);
    public static final Color8Bit kYellow = new Color8Bit(255, 234, 0);
    public static final Color8Bit kMaroon = new Color8Bit(128, 0, 0);
    public static final Color8Bit kWhite = new Color8Bit(255, 255, 255);
  }

}
