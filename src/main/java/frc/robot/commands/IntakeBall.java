package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBall extends Command {
  private final IntakeSubsystem m_intake;

  public IntakeBall(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    m_intake.moveIntakeArm(m_intake.getConfig().intakeArmDeploySpeed);
    m_intake.runIntakeWheels(m_intake.getConfig().intakeWheelsIntakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.moveIntakeArm(0);
    m_intake.runIntakeWheels(0);
  }
}