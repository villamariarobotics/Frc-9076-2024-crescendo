package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

    private CANSparkMax pivotMotor = new CANSparkMax(14, MotorType.kBrushed); // ! Change to kBrushed before running
                                                                              // (changed for simulation)
    private AbsoluteEncoder pivotEncoder;

    // PID controller for precise positioning
    private PIDController pidController = new PIDController(0.1, 0, 0); // Adjust as needed

    /**
     * Creates a new PivotSubsystem.
     */
    public PivotSubsystem() {
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setInverted(false);

        // Ensure the encoder is in absolute mode
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // Set PID setpoint to the current angle initially
        pidController.setSetpoint(pivotEncoder.getPosition());
    }

    @Override
    public void periodic() {
        // Calculate and apply speed using PID controller
        double currentAngle = pivotEncoder.getPosition();
        double speed = pidController.calculate(currentAngle);

        // Limit speed to prevent overshoot
        speed = Math.max(-1, Math.min(speed, 1));

        setPivotSpeed(speed);
    }

    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    /**
     * Moves the motor to a given angle using PID control.
     *
     * @param targetAngle The desired angle in degrees.
     */
    public void moveMotorToAngle(double targetAngle) {
        pidController.setSetpoint(targetAngle);
    }

    public void stop() {
        pivotMotor.set(0);
    }
}
