package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

public class PrepareShot extends Command{

    ShooterSubsystem shooter;
    HoodSubsystem hood;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PrepareShot(ShooterSubsystem shooter, HoodSubsystem hood, Supplier<Pose2d> robotPoseSupplier) {
        addRequirements(shooter, hood);
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        final Translation2d hubPosition;
        if (alliance.isPresent()&& alliance.get() == Alliance.Blue){
            hubPosition = new Translation2d(Inches.of(182.105), Inches.of(158.845));;
        } else {
            hubPosition = new Translation2d(Inches.of(469.115), Inches.of(158.845));
        }
        
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    private double getLinearActuatorDistance(){
        Distance hubDistance = getDistanceToHub();
        
    }

}
