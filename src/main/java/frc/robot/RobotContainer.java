package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final SwerveSubsystem driveBase = new SwerveSubsystem();
  
  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    registerNamedCommands();

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SwerveInputStream driveInputStream = SwerveInputStream.of(
        driveBase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * 1, 
        () -> m_driverController.getLeftX() * 1) 
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) 
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    driveBase.setDefaultCommand(driveBase.driveFieldOriented(driveInputStream));
  }

  private void registerNamedCommands() {
    // These names must match the "Named Commands" you create in the PathPlanner App
    NamedCommands.registerCommand("Test Auto", Commands.print("All hail MacBeth!"));
  }

  private void configureBindings() {
    m_driverController.y().onTrue(driveBase.zeroGyroCommand());
    m_driverController.b().whileTrue(driveBase.lockPoseCommand());
  }

  public Command getAutonomousCommand() {
    // 6. Return the command selected on the Dashboard
    return autoChooser.getSelected();
  }
}