// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.EndEffector.IntakingNote;
import frc.robot.commands.EndEffector.ShootingNote;
import frc.robot.commands.Pivot.PivotGoToAmpPositionCommand;
import frc.robot.commands.Pivot.PivotGoToIntakePositionCommand;
import frc.robot.commands.Pivot.PivotGoToSpeakerPositionCommand;
import frc.robot.commands.Pivot.pivotMoveCommand;
import frc.robot.commands.Auto.AutoShootingNote;
import frc.robot.commands.Auto.AutoIntakeNote;
import frc.robot.commands.EndEffector.IntakeAndShooting;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// import static frc.robot.subsystems.drive.DriveConstants.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser;
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick EndEffectorcontroller = new Joystick(1);

  private final JoystickButton blue_button = new JoystickButton(EndEffectorcontroller, 1);
  private final JoystickButton green_button = new JoystickButton(EndEffectorcontroller, 2);
  private final JoystickButton red_button = new JoystickButton(EndEffectorcontroller, 3);
  private final JoystickButton yellow_button = new JoystickButton(EndEffectorcontroller, 4);
  private final JoystickButton left_bumper = new JoystickButton(EndEffectorcontroller, 5);
  private final JoystickButton right_bumper = new JoystickButton(EndEffectorcontroller, 6);
  private final JoystickButton left_trigger = new JoystickButton(EndEffectorcontroller, 7);
  private final JoystickButton right_trigger = new JoystickButton(EndEffectorcontroller, 8);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // initialize the dashboard auto chooser
    m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    NamedCommands.registerCommand("GoToSpeakerPosition", new PivotGoToSpeakerPositionCommand(m_pivotSubsystem));
    NamedCommands.registerCommand("GoToAmpPosition", new PivotGoToAmpPositionCommand(m_pivotSubsystem));
    NamedCommands.registerCommand("GoToIntakePosition", new PivotGoToIntakePositionCommand(m_pivotSubsystem));
    NamedCommands.registerCommand("Shoot Note Full Speed", new AutoShootingNote(m_endEffectorSubsystem, 1));
    NamedCommands.registerCommand("Shoot Note Half Speed", new AutoShootingNote(m_endEffectorSubsystem, 0.65));
    NamedCommands.registerCommand("Intake Note", new AutoIntakeNote(m_endEffectorSubsystem));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    defaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Controller - Right Trigger makes wheels go to X-formation
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // EndEffector Controller - 8 Buttons Configured
    blue_button.whileTrue(new pivotMoveCommand(m_pivotSubsystem, EndEffectorcontroller));
    green_button.onTrue(new PivotGoToIntakePositionCommand(m_pivotSubsystem));
    red_button.onTrue(new PivotGoToAmpPositionCommand(m_pivotSubsystem));
    yellow_button.onTrue(new PivotGoToSpeakerPositionCommand(m_pivotSubsystem));
    left_bumper.whileTrue(new IntakeAndShooting(m_endEffectorSubsystem, EndEffectorcontroller, 0.65));
    right_bumper.whileTrue(new IntakingNote(m_endEffectorSubsystem, EndEffectorcontroller));
    left_trigger.whileTrue(new IntakeAndShooting(m_endEffectorSubsystem, EndEffectorcontroller, 1));
    right_trigger.whileTrue(new ShootingNote(m_endEffectorSubsystem, EndEffectorcontroller));
  }

  private void defaultCommands() {

    m_robotDrive.setDefaultCommand(
        new RunCommand(() -> {
          if (m_driverController.getAButton()) {
            m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    Math.signum(m_driverController.getLeftY()) * Math.pow(m_driverController.getLeftY(), 2),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    Math.signum(m_driverController.getLeftX()) * Math.pow(m_driverController.getLeftX(), 2),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX() + m_driverController.getRightTriggerAxis()
                    - m_driverController.getLeftTriggerAxis()) * 1, OIConstants.kDriveDeadband),
                false, true);
          } else {
            m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    Math.signum(m_driverController.getLeftY()) * Math.pow(m_driverController.getLeftY(), 2) * 0.5,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    Math.signum(m_driverController.getLeftX()) * Math.pow(m_driverController.getLeftX(), 2) * 0.5,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX() + m_driverController.getRightTriggerAxis()
                    - m_driverController.getLeftTriggerAxis()) * 1, OIConstants.kDriveDeadband),
                false, true);
          }
        }, m_robotDrive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * '
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
    // return new PivotGoToSpeakerPositionCommand(m_pivotSubsystem).andThen(
    // new AutoShootingNote(m_endEffectorSubsystem).andThen(
    // new PivotGoToAmpPositionCommand(m_pivotSubsystem)
    // )
    // );
  }
}