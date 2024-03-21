// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ShootingNote extends Command {
  private EndEffectorSubsystem m_EndEffectorSubsystem;
  private Joystick controller;
  double shooterSpeed;

  /** Creates a new Note shooting command. */
  public ShootingNote(EndEffectorSubsystem shooter, Joystick con) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_EndEffectorSubsystem = shooter;
    this.controller = con;
    //// addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSpeed = controller.getRawAxis(3);
    m_EndEffectorSubsystem.setShooterSpeed(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffectorSubsystem.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
