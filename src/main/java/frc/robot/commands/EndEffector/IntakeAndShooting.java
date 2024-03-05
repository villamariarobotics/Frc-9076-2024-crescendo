// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class IntakeAndShooting extends Command {
  private EndEffectorSubsystem m_EndEffectorSubsystem;
  private Joystick controller;
  double shooterSpeed;
  double intakeSpeed;

  /** Creates a new Note intaking and shooting command. */
  public IntakeAndShooting(EndEffectorSubsystem endeffector, Joystick con) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_EndEffectorSubsystem = endeffector;
    this.controller = con;
    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSpeed = 0.5;
    m_EndEffectorSubsystem.setShooterSpeed(shooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSpeed = controller.getRawAxis(3);
    m_EndEffectorSubsystem.setIntakeSpeed(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffectorSubsystem.setShooterSpeed(0);
    m_EndEffectorSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
