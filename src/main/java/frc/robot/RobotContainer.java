package frc.robot;

// WPILIB and YAGSL imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// Imports from our code (subsystems)
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//Import from our code (commands)
import frc.robot.commands.ShootSequence;

public class RobotContainer {

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

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

    // --- Operator bindings for intake (Operator controller)---

    // Left Trigger -> Intake Balls
    m_operatorController.leftTrigger()
    .whileTrue(m_intakeSubsystem.intakeCommand()
    .alongWith(Commands.print("Intaking balls...")))
    .onFalse(m_intakeSubsystem.stowCommand());
    
    // Right Trigger -> Jiggle Balls towards shooter
    m_operatorController.rightTrigger()
      .whileTrue(m_intakeSubsystem.shooterJiggleCommand()
      .alongWith(Commands.print("Jiggling intake..."), Commands.runOnce(() -> {
        Pose2d targetPose = new Pose2d(SelectHub.hubPosition(), new Rotation2d());
        SwerveInputStream angleDriveInputStream = driveInputStream.copy().aim(targetPose);
        m_swerveSubsystem.driveFieldOriented(angleDriveInputStream);

        new ShootSequence(m_shooterSubsystem, m_hoodSubsystem, () -> m_swerveSubsystem.getPose());
      })));

    // Apply the drive command as the default
    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveFieldOriented(driveInputStream));
    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.run(() -> m_shooterSubsystem.stop()));
    m_intakeSubsystem.setDefaultCommand(m_intakeSubsystem.stowCommand());
  }

  /**
   * Returns the hard-coded autonomous command.
   */
  public Command getAutonomousCommand() {
    return m_swerveSubsystem.getTestAutoCommand();
  }
}