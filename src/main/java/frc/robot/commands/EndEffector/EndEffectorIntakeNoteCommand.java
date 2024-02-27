// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorIntakeNoteCommand extends Command {
  private EndEffectorSubsystem m_EndEffectorSubsystem;
  double intakeSpeed = 0.4; // ! Change this to the desired speed

  /** Creates a new IntakeNoteCommand. */
  public EndEffectorIntakeNoteCommand(EndEffectorSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_EndEffectorSubsystem = intake;
    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_EndEffectorSubsystem.setIntakeSpeed(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public boolean isFinished() {
    double maxVoltage = 5.0; // Change this to the desired maximum voltage
    if (m_EndEffectorSubsystem.getIntakePower() > maxVoltage) {
      return true;
    }
    return false; // Continue the command
  }
}
