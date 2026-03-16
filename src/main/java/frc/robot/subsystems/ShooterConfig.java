package frc.robot.subsystems;
/**
 * Data class that matches the structure of shooter.json.
 * This allows the ObjectMapper to automatically fill these variables.
 */
public class ShooterConfig {
    // Motor CAN IDs
    public int shooterMotorLeftID;
    public int shooterMotorRightID;
    public int shooterMotorMidID;
    public int indexerMotorID;
    public int feederMotorID;

    //Actuator IDs
    public int actuatorLeftServo;
    public int actuatorRightServo;

    // Safety Settings
    public int shooterCurrentLimit;
    public double actuatorVelocityTolerance;

    // Operational Speeds
    public double SpinUpSec;

    // PID Constants
    public double shooterP;
    public double shooterI;
    public double shooterD;

    //Speed limits for shooter arm
    public double shooterSpeedMinOutput;
    public double shooterSpeedMaxOutput;
}
