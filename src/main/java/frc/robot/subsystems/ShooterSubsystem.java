package frc.robot.subsystems;

//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;

//import static edu.wpi.first.units.Units.RPM;

import java.util.List;

//import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;

//import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;


public class ShooterSubsystem extends SubsystemBase {
    // create motors
    private final SparkFlex ShooterMotorLeft, ShooterMotorMid,ShooterMotorRight,IndexerMotor,FeederMotor;
    private final SparkClosedLoopController controllerLeft,controllerRight,controllerMid;

    private final List<SparkFlex> motors;


    private double targetSpeed;
    // @SuppressWarnings("removal")
    public ShooterSubsystem() {

        ShooterMotorLeft = new SparkFlex(FuelConstants.ShooterMotorLeftID, MotorType.kBrushless);
        ShooterMotorMid = new SparkFlex(FuelConstants.ShooterMotorMidID, MotorType.kBrushless);
        ShooterMotorRight = new SparkFlex(FuelConstants.ShooterMotorRightID, MotorType.kBrushless);
        IndexerMotor = new SparkFlex(FuelConstants.IndexerMotorID, MotorType.kBrushless);
        FeederMotor = new SparkFlex(FuelConstants.FeederMotorID, MotorType.kBrushless);

        controllerLeft = ShooterMotorLeft.getClosedLoopController();
        controllerMid = ShooterMotorMid.getClosedLoopController();
        controllerRight = ShooterMotorRight.getClosedLoopController();

        motors = List.of(ShooterMotorLeft, ShooterMotorMid, ShooterMotorRight);

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.inverted(false);
        shooterConfig.smartCurrentLimit(FuelConstants.maxVoltage);
        shooterConfig.closedLoop
        .p(FuelConstants.PIDConstants.ShooterKp)
        .i(FuelConstants.PIDConstants.ShooterKi)
        .d(FuelConstants.PIDConstants.ShooterKd)
        .outputRange(FuelConstants.PIDConstants.ShooterKMinOutput, FuelConstants.PIDConstants.ShooterKMaxOutput);

        SparkFlexConfig ShooterLeftConfig = shooterConfig;
        ShooterLeftConfig.inverted(false);
        SparkFlexConfig ShooterMidConfig = shooterConfig;
        ShooterMidConfig.inverted(false);
        SparkFlexConfig ShooterRightConfig = shooterConfig;
        ShooterRightConfig.inverted(false);
        SparkFlexConfig IndexerConfig = shooterConfig;
        IndexerConfig.inverted(false);
        SparkFlexConfig FeederConfig = shooterConfig;
        FeederConfig.inverted(false);

        ShooterMotorLeft.configure(ShooterLeftConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        ShooterMotorMid.configure(ShooterMidConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        ShooterMotorRight.configure(ShooterRightConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        IndexerMotor.configure(IndexerConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        FeederMotor.configure(IndexerConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

    }

    public void setFeeder(double value) {
        FeederMotor.set(value);
    }

    public void setIndexer(double value) {
        IndexerMotor.set(value);
    }

    public void setShooter(double value) {
        ShooterMotorLeft.set(value);
        ShooterMotorMid.set(value);
        ShooterMotorRight.set(value);
    }

    public void setShooterRPM(double rpm) {
        controllerLeft.setSetpoint(rpm, ControlType.kVelocity);
        controllerMid.setSetpoint(rpm, ControlType.kVelocity);
        controllerRight.setSetpoint(rpm, ControlType.kVelocity);
        targetSpeed = rpm;
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            SparkRelativeEncoder encoder = (SparkRelativeEncoder) motor.getEncoder();
            double rpm = encoder.getVelocity();
            return (Math.abs(rpm - targetSpeed) < FuelConstants.kVelocityTolerance);
        });
    }

    public void stop() {
        ShooterMotorLeft.set(0);
        ShooterMotorMid.set(0);
        ShooterMotorRight.set(0);
        IndexerMotor.set(0);
        FeederMotor.set(0);
    }
}
