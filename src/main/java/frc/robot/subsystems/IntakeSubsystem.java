package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  @SuppressWarnings("FieldMayBeFinal")
  private IntakeConfig intakeConfig;
  private TalonFX rollerMotor;
  private SparkMax armMotor;

  public IntakeSubsystem() {
    // 1. Load JSON Configuration
    try {
      File configFile = new File(Filesystem.getDeployDirectory(), "intake.json");
      intakeConfig = new ObjectMapper().readValue(configFile, IntakeConfig.class);
    } catch (IOException e) {
      System.err.println("Intake JSON Load Failed! Using Hard-coded defaults.");
      intakeConfig = new IntakeConfig();
    }

    // Intake Roller Motors
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.StatorCurrentLimit = intakeConfig.intakeWheelsCurrentLimit;
    rollerMotor.getConfigurator().apply(rollerConfig);

    // Intake Arm Motors 
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.smartCurrentLimit(intakeConfig.intakeArmCurrentLimit);
    armConfig.apply(armConfig);
  }

  // --- Logic Methods ---

  public void runRollers(double speed) { rollerMotor.set(speed); }
  public void movePivot(double speed) { armMotor.set(speed); }

  // --- Command Factories (For Buttons/Autos) ---

  public Command intakeCommand() {
    return runEnd(() -> runRollers(intakeConfig.intakeWheelsIntakeSpeed), () -> runRollers(0));
  }

  public Command outtakeCommand() {
    return runEnd(() -> runRollers(intakeConfig.intakeWheelsOuttakeSpeed), () -> runRollers(0));
  }

  public Command deployCommand() {
    return runEnd(() -> movePivot(intakeConfig.intakeArmDeploySpeed), () -> movePivot(0)).withTimeout(0.75);
  }

  public Command stowCommand() {
    return runEnd(() -> movePivot(intakeConfig.intakeArmStowSpeed), () -> movePivot(0)).withTimeout(0.75);
  }
}