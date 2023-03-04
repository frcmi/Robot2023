// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.BreathingMaroonLEDCommand;
import frc.robot.commands.PurpleLEDCommand;
import frc.robot.commands.YellowLEDCommand;
import frc.robot.subsystems.*;
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
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final PurpleLEDCommand m_purpleLEDCommand = new PurpleLEDCommand(OperatorConstants.kLightPorts, new int[] {OperatorConstants.kLightsPerFoot, OperatorConstants.kLightsPerFoot});
  private final YellowLEDCommand m_yellowLEDCommand = new YellowLEDCommand(OperatorConstants.kLightPorts, new int[] {OperatorConstants.kLightsPerFoot, OperatorConstants.kLightsPerFoot});
  private final BreathingMaroonLEDCommand m_maroonLEDCommand = new BreathingMaroonLEDCommand(OperatorConstants.kLightPorts, new int[] {OperatorConstants.kLightsPerFoot, OperatorConstants.kLightsPerFoot});


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
      .setSpeed(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController.leftBumper()));

    // Intake bindings
    m_driverController.rightTrigger()
      .onTrue(m_intakeSubsystem.intake())
      .onFalse(m_intakeSubsystem.stopCommand());
    m_driverController.leftTrigger()
      .onTrue(m_intakeSubsystem.reverseIntake())
      .onFalse(m_intakeSubsystem.stopCommand());
    m_driverController.povUp()
      .onTrue(Commands.runOnce(() -> m_maroonLEDCommand.setBreathing(true))
      .andThen(Commands.runOnce(m_maroonLEDCommand::instantiateControllers)));
    
    m_driverController.povRight()
      .onTrue(Commands.runOnce(() -> m_maroonLEDCommand.setBreathing(false))
      .andThen(Commands.runOnce(m_purpleLEDCommand::instantiateControllers)));

    m_driverController.povLeft()
      .onTrue(Commands.runOnce(() -> m_maroonLEDCommand.setBreathing(false))
      .andThen(Commands.runOnce(m_yellowLEDCommand::instantiateControllers)));

    // Elevator bindings
    m_elevatorSubsystem.setDefaultCommand(m_elevatorSubsystem.manualMotors(m_driverController::getRightY));

    // Arm bindings
    m_armSubsystem.setDefaultCommand(m_armSubsystem.manualMove(m_driverController::getRightX));
    m_driverController.a().onTrue(m_armSubsystem.moveArmToRelative(10));
    m_driverController.b().onTrue(m_armSubsystem.moveArmToRelative(-10));
  }

  public void teleopPeriodic() {

  }

  public void periodic() {
    if (m_maroonLEDCommand.getBreathing()) {
      m_maroonLEDCommand.periodic();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_driveSubsystem);
  }
}
