package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  
  private final SwerveDrive swerveDrive;
  private final Field2d m_field = new Field2d();
  private boolean pathPlannerConfigured = false;

  public SwerveSubsystem() { 
    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    // Publish the field to NetworkTables for Glass visualization
    SmartDashboard.putData("Field", m_field);

    setupPathPlanner();
  }

  public final void setupPathPlanner() {
    try {
      // RobotConfig automatically reads mass/dimensions from settings.json
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose, 
          swerveDrive::resetOdometry, 
          swerveDrive::getRobotVelocity, 
          // 2025 output includes speeds and feedforwards
          (speeds, feedforwards) -> swerveDrive.drive(speeds), 
          new PPHolonomicDriveController(
              new PIDConstants(0.0014645, 0.0, 0.0), // Translation PID
              new PIDConstants(0.0014645, 0.0, 0.0)  // Rotation PID
          ),
          config,
          () -> {
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);

      pathPlannerConfigured = true;
    } catch (IOException | ParseException e) {
      pathPlannerConfigured = false;
      DriverStation.reportError("PathPlanner Config Failed: " + e.getMessage(), e.getStackTrace());
    }
  }

  public void updateVisionOdometry(){
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-first");
    if(limelightMeasurement.tagCount >= 1)
    {
      swerveDrive.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
    }
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  public Command zeroGyroCommand() {
    return runOnce(() -> swerveDrive.zeroGyro());
  }

  public Command lockPoseCommand() {
    return run(() -> swerveDrive.lockPose());
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @Override
  public void periodic() {
    updateVisionOdometry();
    swerveDrive.updateOdometry();
    // Updates the robot icon location in Glass
    m_field.setRobotPose(swerveDrive.getPose());
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
}