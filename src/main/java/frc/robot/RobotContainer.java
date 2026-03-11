package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
public class RobotContainer {

  private final SwerveSubsystem driveBase = new SwerveSubsystem();
  
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    // Note: Removed redundant NamedCommands.registerCommand here to prevent 
    // circular logic since you are calling the full auto file directly.

    configureBindings();

    SwerveInputStream driveInputStream = SwerveInputStream.of(
        driveBase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * -1, 
        () -> m_driverController.getLeftX() * 1) 
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) 
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    driveBase.setDefaultCommand(driveBase.driveFieldOriented(driveInputStream));
  }

  private void configureBindings() {
    m_driverController.y().onTrue(driveBase.zeroGyroCommand());
    m_driverController.b().whileTrue(driveBase.lockPoseCommand());
  }

  /**
   * Returns the hard-coded autonomous command.
   */
  public Command getAutonomousCommand() {
    return driveBase.getTestAutoCommand();
  }
}