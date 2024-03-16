// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystemEncoder;

public class pivotMoveCommand extends Command {
  private PivotSubsystemEncoder m_pivotSubsystem;
  private Joystick controller;
  double pivotSpeed;

  /** Creates a new PivotTestCommand. */
  public pivotMoveCommand(PivotSubsystemEncoder pivot, Joystick con) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_pivotSubsystem = pivot;
    this.controller = con;
    addRequirements(m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivotSubsystem.setMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSpeed = (controller.getRawAxis(1)); // y axis (hopefully)

    if (pivotSpeed < 0) {
      if (m_pivotSubsystem.getPivotPosition() >= 0) {
        m_pivotSubsystem.setMotorSpeed(0);
      } else {
        m_pivotSubsystem.setMotorSpeed(pivotSpeed);
      }
    if (pivotSpeed > 0) {
      if (m_pivotSubsystem.getPivotPosition() >= 90) {
        m_pivotSubsystem.setMotorSpeed(0);
      } else {
        m_pivotSubsystem.setMotorSpeed(pivotSpeed);
      }
    }
    //? m_pivotSubsystem.getPivotPosition
    }
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
