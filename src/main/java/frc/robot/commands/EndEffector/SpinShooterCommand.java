// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
// No changes needed, removing unused import statement.

public class SpinShooterCommand extends Command {
  private EndEffectorSubsystem m_EndEffectorSubsystem;
  private Joystick controller;
  double speed;

  /** Creates a new SpinShooterCommand. */
  public SpinShooterCommand(EndEffectorSubsystem shooter, Joystick con) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_EndEffectorSubsystem = shooter;
    this.controller = con;
    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = controller.getRawAxis(3);
    m_EndEffectorSubsystem.setShooterSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
