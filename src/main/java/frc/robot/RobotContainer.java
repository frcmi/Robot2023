// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Setpoints;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final PositionEstimationSubsystem m_positionEstimationSubsystem = new PositionEstimationSubsystem();
  //private final LEDControllerSubsystem m_ledSubsystem = new LEDControllerSubsystem(LEDConstants.kLightPorts, LEDConstants.kLightsLengthsArray);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Drive bindings
    m_driveSubsystem.setDefaultCommand(m_driveSubsystem
      .setSpeed(m_driverController::getLeftY, m_driverController::getRightX, m_driverController.leftBumper()));

    // Intake bindings
    m_driverController.rightTrigger()
      .onTrue(m_intakeSubsystem.intake())
      .onFalse(m_intakeSubsystem.stopCommand());
    m_driverController.leftTrigger()
      .onTrue(m_intakeSubsystem.reverseIntake())
      .onFalse(m_intakeSubsystem.stopCommand());

    // Elevator bindings
    m_driverController.povUp().onTrue(m_elevatorSubsystem.raise());
    m_driverController.povDown().onTrue(m_elevatorSubsystem.lower());

    // Arm bindings
    m_armSubsystem.setDefaultCommand(m_armSubsystem.stop());
    // Ground
    m_driverController.a().onTrue(Setpoints.Ground(m_armSubsystem, m_elevatorSubsystem));
    // Stow
    m_driverController.y().onTrue(Setpoints.Stow(m_armSubsystem, m_elevatorSubsystem));
    // L2/Substation
    m_driverController.b().onTrue(Setpoints.L2(m_armSubsystem, m_elevatorSubsystem));
    // L3
    m_driverController.x().onTrue(Setpoints.L3(m_armSubsystem, m_elevatorSubsystem));

    m_driverController.rightBumper().onTrue(m_driveSubsystem.balanceCommand());
    // LED Bindings
    // m_driverController.povUp()
    //   .onTrue(m_ledSubsystem.maroonLEDCommand());
    // m_driverController.povRight()
    //   .onTrue(m_ledSubsystem.purpleLEDCommand());
    // m_driverController.povLeft()
    //   .onTrue(m_ledSubsystem.yellowLEDCommand());
  }

  public DoubleSupplier axisFromButtons(BooleanSupplier firstButton, BooleanSupplier secondButton) {
    // Boolean as num is 0 or 1, this prevents fighting when both are pressed and normalizes [-1, 1]!
    return () -> Boolean.compare(firstButton.getAsBoolean(), false) - Boolean.compare(secondButton.getAsBoolean(), false);
  }

  public void robotInit () {
    SparkMax.burnFlashInSync();
  }

  public void teleopPeriodic() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var aprilTag = new Translation2d(14.02, 34.79);

    return Autos.rotateToFaceTarget(m_driveSubsystem, m_positionEstimationSubsystem, aprilTag, 0.25, Math.PI / 36).onlyWhile(() -> m_positionEstimationSubsystem.getPose().isPresent()).repeatedly();
  }
}
