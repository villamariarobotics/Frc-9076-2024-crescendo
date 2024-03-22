package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("")
public class PivotManualSubsystem extends SubsystemBase {

    // private CANSparkMax pivotMotor = new CANSparkMax(14, MotorType.kBrushed); // ! Change to kBrushed before running
    private CANSparkMax pivotMotor;
    private AbsoluteEncoder pivotEncoder;

    double targetAngle;
    double speed;
    boolean atSetpoint;

    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    // PID controller for precise positioning
    private PIDController pidController = new PIDController(0.1, 0, 0); // Adjust as needed
    
    /**
     * Creates a new PivotSubsystem.
     */
    public PivotManualSubsystem(CANSparkMax pivotMotor) {
        upperLimitSwitch = new DigitalInput(1);
        lowerLimitSwitch = new DigitalInput(0);

        this.pivotMotor = pivotMotor;        

        pivotMotor.setInverted(true);
        pivotMotor.burnFlash();

        // Ensure the encoder is in absolute mode
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    @Override
    public void periodic() {
    
        // Limit speed to prevent overshoot
        speed = Math.max(-1, Math.min(speed, 1));
        
        pivotMotor.set(speed);
    }

    public void setMotorSpeed(double motorSpeed) {
        if (motorSpeed > 0) {
            if (lowerLimitSwitch.get()) {
              // We are going up and top limit is tripped so stop
              speed = 0;
            } else {
              // We are going up but top limit is not tripped so go at commanded speed
              speed = motorSpeed;
            }
        } else {
            if (upperLimitSwitch.get()) {
              // We are going down and bottom limit is tripped so stop
              speed = 0;
            } else {
              // We are going down but bottom limit is not tripped so go at commanded speed
              speed = motorSpeed;
            }
        }
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
        double currentAngle = pivotEncoder.getPosition();
        speed = pidController.calculate(currentAngle, targetAngle);
    }

    public void stop() {
        pivotMotor.set(0);
    }
}
