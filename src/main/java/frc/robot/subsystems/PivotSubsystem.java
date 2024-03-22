// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  
  //public CANSparkMax pivotMotor = new CANSparkMax(14, MotorType.kBrushless); // ! Change to kBrushed before running


  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    
  }
  
  ////public CANSparkMax returnMotor() {
      ////return pivotMotor;
  ////}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
