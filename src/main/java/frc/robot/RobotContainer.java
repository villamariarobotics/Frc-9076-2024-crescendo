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
import frc.robot.commands.Pivot.pivotMoveCommand;
import frc.robot.commands.EndEffector.IntakeAndShooting;
import frc.robot.subsystems.DriveSubsystem;
//// import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotLimitSwitchSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PivotLimitSwitchSubsystem m_pivotSubsystem = new PivotLimitSwitchSubsystem();
  private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();

  private final Joystick EndEffectorcontroller = new Joystick(1);
  // the a button on the controller
  //// private JoystickButton A_BUTTON = new JoystickButton(EndEffectorcontroller,1);

  private final JoystickButton left_bumper = new JoystickButton(EndEffectorcontroller, 5); //! TO CHANGE
  private final JoystickButton right_bumper = new JoystickButton(EndEffectorcontroller, 6); //! TO CHANGE
  private final JoystickButton left_trigger = new JoystickButton(EndEffectorcontroller, 7); //! TO CHANGE
  private final JoystickButton right_trigger = new JoystickButton(EndEffectorcontroller, 8); //! TO CHANGE
  
  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Getting Autonomous Routine from SmartDashboard
    m_autoChooser = new SendableChooser<String>();
    m_autoChooser.setDefaultOption("Shooting Note", null);
    //? Use the Field Map provided on the dashboard to know what the top, bottom, left and right signify
    m_autoChooser.addOption("Blue Top Auto", "BTA");
    m_autoChooser.addOption("Blue Middle Auto", "BMA");
    m_autoChooser.addOption("Blue Bottom Auto", "BBA");
    m_autoChooser.addOption("Red Top Auto", "RTA"); 
    m_autoChooser.addOption("Red Middle Auto", "RMA");
    m_autoChooser.addOption("Red Bottom Auto", "RBA");

    SmartDashboard.putData("Auto Selection", m_autoChooser);
    


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

    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    left_bumper.whileTrue(new IntakeAndShooting(m_endEffectorSubsystem, EndEffectorcontroller));
    right_bumper.whileTrue(new IntakingNote(m_endEffectorSubsystem, EndEffectorcontroller));
    left_trigger.whileTrue(new ShootingNote(m_endEffectorSubsystem, EndEffectorcontroller)); //! TO CHANGE
    right_trigger.whileTrue(new ShootingNote(m_endEffectorSubsystem, EndEffectorcontroller));
  }

  private void defaultCommands() {
    // put commands here that should run by default
    m_pivotSubsystem.setDefaultCommand(new pivotMoveCommand(m_pivotSubsystem, EndEffectorcontroller));
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X of the right stick.
        new RunCommand(() -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false, true), m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * '
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    String autoChoice = m_autoChooser.getSelected();

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Command placeholderCommand = new pivotMoveCommand(m_pivotSubsystem, EndEffectorcontroller);

    Command selectedAutoCommand = switch (autoChoice) {
      case "BTA" -> placeholderCommand; //! CHANGE TO SPECIFIED AUTO COMMAND
      case "BMA" -> placeholderCommand; //! CHANGE TO SPECIFIED AUTO COMMAND
      case "BBA" -> placeholderCommand; //! CHANGE TO SPECIFIED AUTO COMMAND
      case "RTA" -> placeholderCommand; //! CHANGE TO SPECIFIED AUTO COMMAND
      case "RMA" -> placeholderCommand; //! CHANGE TO SPECIFIED AUTO COMMAND
      case "RBA" -> placeholderCommand; //! CHANGE TO SPECIFIED AUTO COMMAND 
      default -> placeholderCommand;
    }; 
    return selectedAutoCommand;
    // Create config for trajectory
    ////TrajectoryConfig config = new TrajectoryConfig(
    ////    AutoConstants.kMaxSpeedMetersPerSecond,
    ////    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
    ////    .setKinematics(DriveConstants.kDriveKinematics);
    
    // An example trajectory to follow. All units in meters.
    ////Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        ////new Pose2d(0, 0, new Rotation2d(0)),
        //// Pass through these two interior waypoints, making an 's' curve path
        ////List.of(new Translation2d(-1, 0), new Translation2d(-2, 0.1)),
        // End 3 meters straight ahead of where we started, facing forward
        ////new Pose2d(-2, 0, new Rotation2d(0)),
        ////config)
    // .concatenate(
    // TrajectoryGenerator.generateTrajectory(
    // List.of(new Pose2d(3, 0, new Rotation2d(0)), new Pose2d(3, 0, new
    // Rotation2d(0))),
    // config))

    ////var thetaController = new ProfiledPIDController(
    ////    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    ////thetaController.enableContinuousInput(-Math.PI, Math.PI);

    ////SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    ////    exampleTrajectory,
    ////    m_robotDrive::getPose, // Functional interface to feed supplier
    ////    DriveConstants.kDriveKinematics,

        // Position controllers
    ////    new PIDController(AutoConstants.kPXController, 0, 0),
    ////    new PIDController(AutoConstants.kPYController, 0, 0),
    ////    thetaController,
    ////    m_robotDrive::setModuleStates,
    ////    m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    ////m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    ////return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
