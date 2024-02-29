// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotLimitSwitchSubsystem extends SubsystemBase {
  private CANSparkMax pivotMotor = new CANSparkMax(14, MotorType.kBrushed);
  private DigitalInput upperLimitSwitch;
  private DigitalInput lowerLimitSwitch;
  Joystick joystick = new Joystick(1);

  /** Creates a new PivotLimitSwitchSubsystem. */
  public PivotLimitSwitchSubsystem() {
    pivotMotor.setInverted(false);

    upperLimitSwitch = new DigitalInput(0);
    lowerLimitSwitch = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed) {
    if (speed > 0) {
      if (upperLimitSwitch.get()) {
        // We are going up and top limit is tripped so stop
        pivotMotor.set(0);
      } else {
        // We are going up but top limit is not tripped so go at commanded speed
        pivotMotor.set(speed);
      }
    } else {
      if (lowerLimitSwitch.get()) {
        // We are going down and bottom limit is tripped so stop
        pivotMotor.set(0);
      } else {
        // We are going down but bottom limit is not tripped so go at commanded speed
        pivotMotor.set(speed);
      }
    }
  }

  public void stop() {
    pivotMotor.set(0);
  }
}