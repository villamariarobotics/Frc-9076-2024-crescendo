package frc.robot.subsystems;

import com.fasterxml.jackson.core.type.TypeReference;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class PivotSubsystem extends SubsystemBase {

    private CANSparkMax pivotMotor = new CANSparkMax(14, MotorType.kBrushless); // ! Change to kBrushed before running
    private AbsoluteEncoder pivotEncoder;

    double targetAngle;

    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    // PID controller for precise positioning
    private PIDController pidController = new PIDController(0.1, 0, 0); // Adjust as needed
    
    /**
     * Creates a new PivotSubsystem.
     */
    public PivotSubsystem() {
        
        upperLimitSwitch = new DigitalInput(1);
        lowerLimitSwitch = new DigitalInput(0);
        

        pivotMotor.setInverted(true);
        pivotMotor.burnFlash();

        // Ensure the encoder is in absolute mode
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // Set PID setpoint to the current angle initially
        pidController.enableContinuousInput(0.1, 0.5);
        pidController.setSetpoint(pivotEncoder.getPosition());
        pidController.setTolerance(5);
    }

    @Override
    public void periodic() {
        // Calculate and apply speed using PID controller
        double currentAngle = pivotEncoder.getPosition();
        double speed = pidController.calculate(currentAngle, targetAngle);

        // Limit speed to prevent overshoot
        speed = Math.max(-1, Math.min(speed, 1));
        
        ////pivotMotor.set(speed);
    }

    public void setMotorSpeed(double speed) {
        ////if (upperLimitSwitch.get() || lowerLimitSwitch.get()){
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
        ////} else {
        ////    if (speed < 0) {
        ////        if (pivotEncoder.getPosition() >= 90) {
        ////            pivotMotor.set(0);
        ////        } else {
        ////            pivotMotor.set(speed);
        ////        }
        ////    } else {
        ////        if (pivotEncoder.getPosition() >= 0) {
        ////            pivotMotor.set(0);
        ////        } else {
        ////            pivotMotor.set(speed);;
        ////        }
        ////    }
        ////}
    }
    

    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    /**
     * Moves the motor to a given angle using PID control.
     *
     * @param targetAngle The desired angle in degrees.
     */
    public void moveMotorToAngle(double target_angle) {
        
        targetAngle = target_angle;
        
        ////double error = pivotEncoder.getPosition() - targetAngle;
        ////pivotMotor.set(error*0.00464159);
    }

    public void stop() {
        pivotMotor.set(0);
    }
}
