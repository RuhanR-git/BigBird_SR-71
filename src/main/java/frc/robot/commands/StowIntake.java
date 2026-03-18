package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StowIntake extends Command
{
  private final IntakeSubsystem m_intake;

  public StowIntake(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    // Moves the arm up using the speed from your intake.json
    m_intake.moveIntakeArm(m_intake.getConfig().intakeArmStowSpeed);
    m_intake.runIntakeWheels(0);
  }

  @Override
  public void end(boolean interrupted) 
  {
    m_intake.moveIntakeArm(0);
  }
}