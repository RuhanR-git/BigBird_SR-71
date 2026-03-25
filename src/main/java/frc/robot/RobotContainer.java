package frc.robot;

// WPILIB and YAGSL imports
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Note: Removed redundant NamedCommands.registerCommand here to prevent 
    // circular logic since you are calling the full auto file directly.

    configureBindings();
  }

  private void configureBindings() 
  {
    // --- Driver bindings for driving (Driver controller)---

    //Left joystick: Strafing
    //Right joystick: Rotation
    SwerveInputStream driveInputStream = SwerveInputStream.of(
        m_swerveSubsystem.getSwerveDrive(),
        () -> m_driverController.getLeftY() * 1, 
        () -> m_driverController.getLeftX() * 1) 
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) 
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    // --- Extra Driver bindings for driving ---

    // Y -> Zero Gyro
    // B -> Lock Pose, locks robot wheels to only move forward
    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveFieldOriented(driveInputStream));
    m_driverController.y().onTrue(m_swerveSubsystem.zeroGyroCommand());
    m_driverController.b().whileTrue(m_swerveSubsystem.lockPoseCommand());
    // Apply the drive command as the default
    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveFieldOriented(driveInputStream));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_swerveSubsystem.getAutonomousCommand("Test Auto");
  }
}