// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.EndEffector.IntakingNote;
import frc.robot.commands.EndEffector.ShootingNote;
import frc.robot.commands.Pivot.PivotGoToAmpPositionCommandencoder;
import frc.robot.commands.Pivot.PivotGoToIntakePositionCommandencoder;
import frc.robot.commands.Pivot.PivotGoToSpeakerPositionCommandencoder;
import frc.robot.commands.Pivot.pivotMoveCommand;
import frc.robot.commands.Auto.AutoPivotMove;
import frc.robot.commands.Auto.AutoShootingNote;
import frc.robot.commands.Auto.AutoIntakeNote;
import frc.robot.commands.EndEffector.IntakeAndShooting;
import frc.robot.subsystems.DriveSubsystem;
//// import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import frc.robot.subsystems.EndEffectorSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private static SendableChooser<String> m_autoChooser;
  private static SendableChooser<String> m_DriveModeChooser;

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();

  private final Joystick EndEffectorcontroller = new Joystick(1);
  // the a button on the controller
  //// private JoystickButton A_BUTTON = new JoystickButton(EndEffectorcontroller,1);

  private final JoystickButton blue_button = new JoystickButton(EndEffectorcontroller, 1);
  private final JoystickButton green_button = new JoystickButton(EndEffectorcontroller, 2);
  private final JoystickButton red_button = new JoystickButton(EndEffectorcontroller, 3);
  private final JoystickButton yellow_button = new JoystickButton(EndEffectorcontroller, 4);
  private final JoystickButton left_bumper = new JoystickButton(EndEffectorcontroller, 5);
  private final JoystickButton right_bumper = new JoystickButton(EndEffectorcontroller, 6);
  private final JoystickButton left_trigger = new JoystickButton(EndEffectorcontroller, 7);
  private final JoystickButton right_trigger = new JoystickButton(EndEffectorcontroller, 8);
  
  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Getting Autonomous Routine from Driver Station
    m_autoChooser = new SendableChooser<String>();
    m_autoChooser.setDefaultOption("Shooting Note", null);
    //? Use the Field Map provided on the dashboard to know what the top, bottom, left and right signify
    m_autoChooser.addOption("Top Auto", "TA");
    m_autoChooser.addOption("Middle Auto", "MA");
    m_autoChooser.addOption("Bottom Auto", "BA");

    SmartDashboard.putData("Auto Selection", m_autoChooser);

    // Getting Drive Mode from Driver Station
    m_DriveModeChooser = new SendableChooser<String>();
    m_DriveModeChooser.setDefaultOption("Robot-Oriented", "R-O");
    m_DriveModeChooser.addOption("Field-Oriented", "F-O");

    SmartDashboard.putData("Drive Mode Selection", m_DriveModeChooser);
    
    // put commands here that should run by default
    
    
    String driveChoice = m_DriveModeChooser.getSelected();
    
    boolean selectedDriveMode = switch (driveChoice) {
      case "F-O" -> true;
      case "R-O" -> false;
      default -> true;
    };
    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands
    defaultCommands(selectedDriveMode);
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
    new JoystickButton(m_driverController, Button.kR1.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));

    // EndEffector Controller - 8 Buttons Configured
    blue_button.whileTrue(new pivotMoveCommand(m_pivotSubsystem, EndEffectorcontroller));
    green_button.onTrue(new PivotGoToIntakePositionCommandencoder(m_pivotSubsystem));
    red_button.onTrue(new PivotGoToAmpPositionCommandencoder(m_pivotSubsystem));
    yellow_button.onTrue(new PivotGoToSpeakerPositionCommandencoder(m_pivotSubsystem));
    left_bumper.whileTrue(new IntakeAndShooting(m_endEffectorSubsystem, EndEffectorcontroller, 0.5));
    right_bumper.whileTrue(new IntakingNote(m_endEffectorSubsystem, EndEffectorcontroller));
    left_trigger.whileTrue(new IntakeAndShooting(m_endEffectorSubsystem, EndEffectorcontroller, 1));
    right_trigger.whileTrue(new ShootingNote(m_endEffectorSubsystem, EndEffectorcontroller));
  }

  private void defaultCommands(boolean driveMode) {
    
    ////m_pivotSubsystem.setDefaultCommand(new pivotMoveCommand(m_pivotSubsystem, EndEffectorcontroller));
    
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X of the right stick.
        new RunCommand(() -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                driveMode, true), m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * '
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    String trajectoryJSON = "paths";
    Trajectory trajectory = new Trajectory();

    String autoChoice = m_autoChooser.getSelected();

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    switch (SmartDashboard.getString("Alliance Color", "N/A")) {
      case "Red":
        switch (autoChoice) {
          case "TA" -> trajectoryJSON = "paths/RedUpPath.wpilib.json";
          case "MA" -> trajectoryJSON = "paths/RedMiddlePath.wpilib.json";
          case "BA" -> trajectoryJSON = "paths/RedDownPath.wpilib.json";
        }
      case "Blue":
        switch (autoChoice) {
          case "TA" -> trajectoryJSON = "paths/BlueUpPath.wpilib.json";
          case "MA" -> trajectoryJSON = "paths/BlueMiddlePath.wpilib.json";
          case "BA" -> trajectoryJSON = "paths/BlueDownPath.wpilib.json";
        }
      case "N/A":
        trajectoryJSON = null;
    };

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
       trajectory,
       m_robotDrive::getPose, // Functional interface to feed supplier
       DriveConstants.kDriveKinematics,

        // Position controllers
       new PIDController(AutoConstants.kPXController, 0, 0),
       new PIDController(AutoConstants.kPYController, 0, 0),
       thetaController,
       m_robotDrive::setModuleStates,
       m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());


    // Run path following command, then stop at the end.
    return new AutoPivotMove(m_pivotSubsystem, 10).andThen(
      new AutoShootingNote(m_endEffectorSubsystem).andThen(
        new AutoPivotMove(m_pivotSubsystem, 10). andThen(
          swerveControllerCommand.andThen(
            () -> m_robotDrive.drive(0, 0, 0, false, false)))));
  }
}