package frc.robot.subsystems;

import com.fasterxml.jackson.core.type.TypeReference;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class PivotSubsystem extends SubsystemBase {

    private CANSparkMax pivotMotor;
    private AbsoluteEncoder pivotEncoder;

    double currentPivotAngle;

    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    // PID controller for precise positioning
    private PIDController pidController = new PIDController(0.02, 0, 0);
    
    /**
     * Creates a new PivotSubsystem.
     */
    public PivotSubsystem() { 

        pivotMotor = new CANSparkMax(14, MotorType.kBrushed);
        pivotMotor.setInverted(true);
        pivotMotor.burnFlash();

        upperLimitSwitch = new DigitalInput(1);
        lowerLimitSwitch = new DigitalInput(0);

        // Ensure the encoder is in absolute mode
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // Set PID setpoint to the current angle initially
        pidController.setSetpoint(pivotEncoder.getPosition());
        pidController.setTolerance(2);
    }

    @Override
    public void periodic() {
        // Calculate and apply speed using PID controller
        currentPivotAngle = pivotEncoder.getPosition() / 4 ;
        
        SmartDashboard.putNumber("Motor Rotations", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Angle", currentPivotAngle);
    }

    public void setMotorSpeed(double speed) {
        
        if (lowerLimitSwitch.get() || upperLimitSwitch.get()) {
            if (speed > 0) {
                if (lowerLimitSwitch.get()) {
                  // We are going up and top limit is tripped so stop
                    pivotMotor.set(0);
                } else {
                  // We are going up but top limit is not tripped so go at commanded speed
                    pivotMotor.set(speed);
                }
            } else {
                if (upperLimitSwitch.get()) {
                  // We are going down and bottom limit is tripped so stop
                    pivotMotor.set(0);
                } else {
                  // We are going down but bottom limit is not tripped so go at commanded speed
                    pivotMotor.set(speed);
                }
            }
        } else {
            if (speed > 0) {
                if (currentPivotAngle < 0) {
                  // We are going up and top limit is tripped so stop
                    pivotMotor.set(0);
                } else {
                  // We are going up but top limit is not tripped so go at commanded speed
                    pivotMotor.set(speed);
                }
            } else {
                if (currentPivotAngle > 87.5) {
                  // We are going down and bottom limit is tripped so stop
                    pivotMotor.set(0);
                } else {
                  // We are going down but bottom limit is not tripped so go at commanded speed
                    pivotMotor.set(speed);
                }
            }
        }
        
    }
    
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
    
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    public boolean getLowerLimitSwitch() {
        return lowerLimitSwitch.get();
    }

    /**
     * Moves the motor to a given angle using PID control.
     *
     * @param targetAngle The desired angle in degrees.
     */
    public void moveMotorToAngle(double targetAngle) {
        
        pidController.setSetpoint(targetAngle);
        double output = -pidController.calculate(currentPivotAngle);
        setMotorSpeed(output);
    }

    public void stop() {
        pivotMotor.set(0);
    }
}
