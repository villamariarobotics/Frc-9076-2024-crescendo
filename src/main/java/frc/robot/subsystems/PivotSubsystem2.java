// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem2 extends SubsystemBase {

  private CANSparkMax pivotMotor = new CANSparkMax(14, MotorType.kBrushless); //! change to kBrushed before running (changed for simulation)
  private AbsoluteEncoder pivotEncoder;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem2() {
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setInverted(false);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }
  
  public void moveMotorToAngle(double angle) {
    double currentAngle = getPivotPosition();
    double angleDifference = angle - currentAngle;
    double speed = angleDifference * 0.1; // Adjust the speed multiplier as needed

    setPivotSpeed(speed);
  
  }
  public void stop() {
    pivotMotor.set(0);
  }
}
  