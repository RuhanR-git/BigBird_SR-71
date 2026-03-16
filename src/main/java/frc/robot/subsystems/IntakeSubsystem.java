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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.IntakeConfig;

public class IntakeSubsystem extends SubsystemBase 
{
  @SuppressWarnings("FieldMayBeFinal")
  private IntakeConfig intakeConfig;
  private TalonFX rollerMotor;
  private SparkMax armMotor;

  public IntakeSubsystem() {
    // Load JSON Configuration
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

    //Soft limits
    armConfig.softLimit.forwardSoftLimit(intakeConfig.intakeArmForwardLimit).forwardSoftLimitEnabled(true);
    armConfig.softLimit.reverseSoftLimit(intakeConfig.intakeArmReverseLimit).reverseSoftLimitEnabled(true);

    armConfig.apply(armConfig);
  }

  // --- Logic Methods ---

  public void runIntakeWheels(double speed) { rollerMotor.set(speed); }
  public void moveIntakeArm(double speed) { armMotor.set(speed); }

  // --- Command Factories (For Buttons/Autos) ---

  public Command intakeCommand() {
    return runEnd(() -> runIntakeWheels(intakeConfig.intakeWheelsIntakeSpeed), () -> runIntakeWheels(0));
  }

  public Command outtakeCommand() {
    return runEnd(() -> runIntakeWheels(intakeConfig.intakeWheelsOuttakeSpeed), () -> runIntakeWheels(0));
  }

  public Command deployCommand() {
    return runEnd(() -> moveIntakeArm(intakeConfig.intakeArmDeploySpeed), () -> moveIntakeArm(0)).withTimeout(0.75);
  }

  public Command stowCommand() {
    return runEnd(() -> moveIntakeArm(intakeConfig.intakeArmStowSpeed), () -> moveIntakeArm(0)).withTimeout(0.75);
  }

  // Add these to your IntakeSubsystem class

  /** * Command to lower the arm and spin rollers. 
   * Stops both when the command is interrupted (button released).
   */
  public Command intakeSequence() {
      return runEnd(
          () -> {
              moveIntakeArm(intakeConfig.intakeArmDeploySpeed);
              runIntakeWheels(intakeConfig.intakeWheelsIntakeSpeed);
          },
          () -> {
              moveIntakeArm(0);
              runIntakeWheels(0);
          }
      );
  }

  /**
   * Command to move the arm up and down repeatedly.
   * Returns to stow position when released.
   */
  public Command shooterJiggleCommand() 
  {
    return Commands.sequence(
        // The "Jiggle" loop
        Commands.repeatingSequence(
            run(() -> moveIntakeArm(0.4)).withTimeout(0.2),
            run(() -> moveIntakeArm(-0.4)).withTimeout(0.2)
        )
    ).finallyDo(() -> moveIntakeArm(0)); // Ensure it stops when released
  }
}