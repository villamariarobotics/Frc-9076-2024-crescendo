// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotLimitSwitchSubsystem;

public class pivotMoveCommand extends Command {

  private PivotLimitSwitchSubsystem m_pivotSubsystem;
  private Joystick controller;

  double speed;

  SlewRateLimiter speedLimiter = new SlewRateLimiter(1);

  /** Creates a new PivotTestCommand. */
  public pivotMoveCommand(PivotLimitSwitchSubsystem pivot, Joystick con) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_pivotSubsystem = pivot;
    this.controller = con;

    addRequirements(m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = speedLimiter.calculate(controller.getRawAxis(1)); // y axis (hopefully)

    m_pivotSubsystem.setMotorSpeed(speedLimiter.calculate(speed));
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
