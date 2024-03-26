// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
  
  private CANSparkMax shooterMotor = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax intakeMotor = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
  
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.burnFlash();

    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setInverted(true);
    shooterMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
