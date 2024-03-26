// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AutoShootingNote extends Command {
  
  private EndEffectorSubsystem m_EndEffectorSubsystem;

  double shooterSpeed;
  double intakeSpeed;
  long startTime;

  /** Creates a new Note intaking and shooting command. */
  public AutoShootingNote(EndEffectorSubsystem endeffector, double shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSpeed = shooterSpeed;
    this.m_EndEffectorSubsystem = endeffector;
    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSpeed = -1;
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_EndEffectorSubsystem.setShooterSpeed(shooterSpeed);

    if (System.currentTimeMillis() - startTime > 2000) {
      m_EndEffectorSubsystem.setIntakeSpeed(intakeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffectorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime > 3000;
  }
}