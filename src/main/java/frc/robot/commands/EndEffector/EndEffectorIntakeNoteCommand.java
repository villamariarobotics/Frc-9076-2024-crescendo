// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorIntakeNoteCommand extends Command {
  private EndEffectorSubsystem m_EndEffectorSubsystem;
  double speed;

  /** Creates a new Note intaking command. */
  public EndEffectorIntakeNoteCommand(EndEffectorSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_EndEffectorSubsystem = intake;
    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0.2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_EndEffectorSubsystem.setIntakeSpeed(speed);
  }

  public void end(boolean interrupted) {
    m_EndEffectorSubsystem.setIntakeSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public boolean isFinished() {
    return false;
  }
}
