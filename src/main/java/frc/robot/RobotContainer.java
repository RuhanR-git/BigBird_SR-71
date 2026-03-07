package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootSequence;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final SwerveSubsystem driveBase = new SwerveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    registerNamedCommands();

    configureBindings();

    SwerveInputStream driveInputStream = SwerveInputStream.of(
        driveBase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * 1, 
        () -> m_driverController.getLeftX() * 1) 
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) 
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    m_driverController.rightTrigger(0.5)
        .onTrue(new ShootSequence(shooter));
    // Apply the drive command as the default
    driveBase.setDefaultCommand(driveBase.driveFieldOriented(driveInputStream));
    shooter.setDefaultCommand(shooter.run(() -> shooter.stop()));
  }

  private void registerNamedCommands() {
    // These names must match the "Named Commands" you create in the PathPlanner App
    NamedCommands.registerCommand("Follow Test Path", driveBase.followPathCommand("Test Auto"));
  }

  private void configureBindings() {
    m_driverController.y().onTrue(driveBase.zeroGyroCommand());
    m_driverController.b().whileTrue(driveBase.lockPoseCommand());


  }

  public Command getAutonomousCommand() {
    return driveBase.buildAutoCommand("Test Auto");
  }
}