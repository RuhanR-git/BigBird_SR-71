package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;


public class ShooterSubsystem extends SubsystemBase {
    // create motors
    private final SparkFlex ShooterMotorLeft, ShooterMotorMid,ShooterMotorRight,IndexerMotor,FeederMotor;
    private final SparkClosedLoopController controllerLeft,controllerRight,controllerMid;

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


        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.inverted(false);
        shooterConfig.smartCurrentLimit(FuelConstants.maxVoltage);

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

    public void setShooterRPM(double value) {
        controllerLeft.setSetpoint(value, ControlType.kVelocity);
        controllerMid.setSetpoint(value, ControlType.kVelocity);
        controllerRight.setSetpoint(value, ControlType.kVelocity);
    }

    public void stop() {
        ShooterMotorLeft.set(0);
        ShooterMotorMid.set(0);
        ShooterMotorRight.set(0);
        IndexerMotor.set(0);
        FeederMotor.set(0);
    }
}
