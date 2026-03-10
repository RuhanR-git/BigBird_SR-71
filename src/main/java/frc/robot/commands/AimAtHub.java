package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SelectHub;
import frc.robot.subsystems.SwerveSubsystem;

public class AimAtHub extends Command{
    private final SwerveSubsystem swerve;

    public AimAtHub(SwerveSubsystem swerve){
        this.swerve = swerve;
        addRequirements(swerve);
        
    }

     private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = SelectHub.hubPosition();
        // Get the pose of the hub relative to the robot's current pose
        Pose2d targetRelative = hubPosition.relativeTo(swerve.getPose());

        Rotation2d angleToTarget = targetRelative.getRotation();
        return angleToTarget;
    }

}
