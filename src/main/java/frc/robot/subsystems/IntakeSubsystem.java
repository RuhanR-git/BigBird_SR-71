package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  @SuppressWarnings("FieldMayBeFinal")
  private IntakeConfig intakeConfig;
  private final TalonFX rollerMotor;
  private final SparkMax armMotor;

  public IntakeSubsystem() {
    // 1. Load JSON Configuration
    try {
      File configFile = new File(Filesystem.getDeployDirectory(), "intake.json");
      intakeConfig = new ObjectMapper().readValue(configFile, IntakeConfig.class);
    } catch (IOException e) {
      intakeConfig = new IntakeConfig();
    }

    // Initialize Motors (Crucial step: you must instantiate them before configuring)
    rollerMotor = new TalonFX(intakeConfig.intakeWheelsID);
    armMotor = new SparkMax(intakeConfig.intakeArmID, MotorType.kBrushless);

    // Configure Roller (Kraken)
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.StatorCurrentLimit = intakeConfig.intakeWheelsCurrentLimit;
    rollerMotor.getConfigurator().apply(rollerConfig);

    // Configure Arm (NEO)
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.smartCurrentLimit(intakeConfig.intakeArmCurrentLimit);
    armConfig.softLimit.forwardSoftLimit(intakeConfig.intakeArmForwardLimit).forwardSoftLimitEnabled(true);
    armConfig.softLimit.reverseSoftLimit(intakeConfig.intakeArmReverseLimit).reverseSoftLimitEnabled(true);
    armConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    armConfig.apply(armConfig);
  }

  public void runIntakeWheels(double speed) 
  {
    rollerMotor.set(speed); 
  }

  public IntakeConfig getConfig() 
  {
    return intakeConfig;
  }

  public void moveIntakeArm(double speed) 
  { 
    armMotor.set(speed); 
  }
}