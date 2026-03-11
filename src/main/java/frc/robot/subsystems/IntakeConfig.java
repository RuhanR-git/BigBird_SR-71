package frc.robot.subsystems;

/**
 * Data class that matches the structure of intake.json.
 * This allows the ObjectMapper to automatically fill these variables.
 */
public class IntakeConfig {
    // Motor CAN IDs
    public int intakeArmID;
    public int intakeWheelsID;

    // Safety Settings
    public int intakeArmCurrentLimit;
    public int intakeWheelsCurrentLimit;

    // Operational Speeds
    public double intakeWheelsIntakeSpeed;
    public double intakeWheelsOuttakeSpeed;
    public double intakeArmDeploySpeed;
    public double intakeArmStowSpeed;
}
