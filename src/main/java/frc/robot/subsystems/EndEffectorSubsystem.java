// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotor = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax intakeMotor = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);
  private PIDController ShooterPIDController; // Declare the ShooterPIDController variable at the class level
  private RelativeEncoder shooterEncoder; // Declare the shooterEncoder variable at the class level
  // PID controller for precise positioning

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.burnFlash();

    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setInverted(true);
    shooterMotor.burnFlash();
    shooterEncoder = shooterMotor.getEncoder();
    // Configure the PID controller gains
    double kP = 0.02;
    double kI = 0;
    double kD = 0;
    ShooterPIDController = new PIDController(kP, kI, kD);

    ShooterPIDController.setTolerance(20, 100); // Set the tolerance to 100 RPM
    ShooterPIDController.setSetpoint(shooterEncoder.getPosition()); // Set the setpoint to the current position

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getVelocity());
    SmartDashboard.putBoolean("shooterAtSpeed", ShooterPIDController.atSetpoint());
  }

  public void setShooterRPM(double rpm) {

    ShooterPIDController.setSetpoint(rpm);
    shooterMotor.set(ShooterPIDController.calculate(shooterEncoder.getVelocity()));
    SmartDashboard.putNumber("Shooter Setpoint", rpm);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public double getIntakePower() {
    return intakeMotor.getBusVoltage();
  }

  public void stop() {
    intakeMotor.set(0);
    shooterMotor.set(0);
  }
}
