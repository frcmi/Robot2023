// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Setpoints {
    private Setpoints() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Command Ground(ArmSubsystem arm, ElevatorSubsystem elevator) {
        return arm.moveTo(Math.toRadians(-90)).alongWith(elevator.lower());
    }

    public static Command Stow(ArmSubsystem arm, ElevatorSubsystem elevator) {
        return arm.moveTo(Math.toRadians(155)).alongWith(elevator.lower());
    }

    public static Command L2(ArmSubsystem arm, ElevatorSubsystem elevator) {
        return arm.moveTo(Math.toRadians(50)).alongWith(elevator.lower());
    }

    public static Command L3(ArmSubsystem arm, ElevatorSubsystem elevator) {
        return arm.moveTo(Math.toRadians(50)).alongWith(elevator.raise());
    }
}
