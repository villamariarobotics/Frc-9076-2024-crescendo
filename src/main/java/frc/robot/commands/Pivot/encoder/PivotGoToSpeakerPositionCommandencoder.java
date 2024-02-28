// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot.encoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystemEncoder;

public class PivotGoToSpeakerPositionCommandencoder extends Command {

  private PivotSubsystemEncoder m_pivotSubsystem;
  double target_angle = 90; // ! Change this to the desired angle

  /** Creates a new PivotGoToSpeakerPositionCommand. */
  public PivotGoToSpeakerPositionCommandencoder(PivotSubsystemEncoder pivot, double target_angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_pivotSubsystem = pivot;
    this.target_angle = target_angle;
    addRequirements(m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivotSubsystem.moveMotorToAngle(target_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivotSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
