// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  private CANSparkMax pivotMotor = new CANSparkMax(14, MotorType.kBrushed);
//  private encoder  pivotEncoder = new CANSparkMax(14, MotorType.kBrushless);
  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("pivot motor speed", pivotMotor.get());

  }
  public void setPivotSpeed( double speed) {
    pivotMotor.set(speed);

  }

  public void stop() {
    pivotMotor.set(0);
  }
}
  