package frc.robot.subsystems;

// import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonUtils;
// import edu.wpi.first.wpilibj.util.Units;

public class PivotTurretModeSubsystem extends SubsystemBase {
  private CANSparkMax pivotMotor = new CANSparkMax(14, MotorType.kBrushed); // Change to kBrushed before running
  private final PhotonCamera camera = new PhotonCamera("FrontCamera");
  private AbsoluteEncoder pivotEncoder;

  // PID controller for precise positioning
  private PIDController pidController = new PIDController(0.1, 0, 0); // Adjust as needed

  /** Creates a new PivotTurretModeSubsystem. */
  public PivotTurretModeSubsystem() {
    pivotMotor.setInverted(false);

    // Ensure the encoder is in absolute mode
    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // Set PID setpoint to the current angle initially
    pidController.setSetpoint(pivotEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get the distance from the AprilTag (replace with your own logic)
    String allianceColor = SmartDashboard.getString("Alliance Color", "N/A");
    double distanceFromAprilTag = getDistanceFromAprilTag(allianceColor);

    // Calculate the desired angle based on the distance
    double desiredAngle = calculateDesiredAngle(distanceFromAprilTag);

    // Set the PID setpoint to the desired angle
    pidController.setSetpoint(desiredAngle);

    // Get the PID output and apply it to the motor
    double output = pidController.calculate(pivotEncoder.getPosition());
    pivotMotor.set(output);

    // Update SmartDashboard with relevant information
    SmartDashboard.putNumber("Distance from AprilTag", distanceFromAprilTag);
    SmartDashboard.putNumber("Desired Angle", desiredAngle);
    SmartDashboard.putNumber("PID Output", output);
    SmartDashboard.putNumber("Current Angle", pivotEncoder.getPosition());
  }

  private double getDistanceFromAprilTag(String allianceColor) {

    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    // Get the current best target.
    PhotonTrackedTarget target = result.getBestTarget();
    // Get information from target.
    int targetID = target.getFiducialId();
    if (hasTargets == true) {
      // check if the alliance color is red if so calculate the distance to the
      // apriltag
      // with ID 4 because that is the correct tag id for the red speaker
      if (allianceColor == "red")
        // Calculate distance to AprilTag with ID 4
        if (targetID == 4) {
          double distance = PhotonUtils.calculateDistanceToTargetMeters(0.1, 1.32, 0.0, 0.0);
          return distance;
        }

      // check if the alliance color is blue if so calculate the distance to the
      // apriltag
      // with ID 7 because that is the correct tag id for the blue speaker
      if (allianceColor == "blue")
        // Calculate distance to AprilTag with ID 7
        if (targetID == 7) {
          double distance = PhotonUtils.calculateDistanceToTargetMeters(0.1, 1.32, 0.0, 0.0);
          return distance;
        }
    }
    // Return default distance if AprilTag ID is not 4 or 7
    return 0.0;
  }

  private double calculateDesiredAngle(double distance) {

    // Example: Linear relationship where desired angle increases as distance
    // decreases
    double desiredAngle = 180 - (distance * 10); // Modify the coefficient as needed for tuning

    return desiredAngle;
  }
}
