package frc.robot.subsystems;

//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;


public class ShooterSubsystem extends SubsystemBase {
    // create motors
    private final SparkMax ShooterMotorLeft, ShooterMotorMid,ShooterMotorRight,IndexerMotor,FeederMotor;
    

    // @SuppressWarnings("removal")
    public ShooterSubsystem() {

        ShooterMotorLeft = new SparkMax(FuelConstants.ShooterMotorLeftID, MotorType.kBrushless);
        ShooterMotorMid = new SparkMax(FuelConstants.ShooterMotorMidID, MotorType.kBrushless);
        ShooterMotorRight = new SparkMax(FuelConstants.ShooterMotorRightID, MotorType.kBrushless);
        IndexerMotor = new SparkMax(FuelConstants.IndexerMotorID, MotorType.kBrushless);
        FeederMotor = new SparkMax(FuelConstants.FeederMotorID, MotorType.kBrushless);

        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.inverted(false);
        shooterConfig.smartCurrentLimit(FuelConstants.maxVoltage);

        SparkMaxConfig ShooterLeftConfig = shooterConfig;
        ShooterLeftConfig.inverted(false);
        SparkMaxConfig ShooterMidConfig = shooterConfig;
        ShooterMidConfig.inverted(false);
        SparkMaxConfig ShooterRightConfig = shooterConfig;
        ShooterRightConfig.inverted(false);
        SparkMaxConfig IndexerConfig = shooterConfig;
        IndexerConfig.inverted(false);
        SparkMaxConfig FeederConfig = shooterConfig;
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

    public void stop() {
        ShooterMotorLeft.set(0);
        ShooterMotorMid.set(0);
        ShooterMotorRight.set(0);
        IndexerMotor.set(0);
        FeederMotor.set(0);
    }
}
