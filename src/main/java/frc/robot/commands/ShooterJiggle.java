package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class ShooterJiggle extends Command {
  private final IntakeSubsystem m_intake;
  private final Command m_jiggleCommand;

  public ShooterJiggle(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);

    // Creates the repeating up/down sequence
    m_jiggleCommand = Commands.repeatingSequence(
        m_intake.run(() -> m_intake.moveIntakeArm(0.4)).withTimeout(0.2),
        m_intake.run(() -> m_intake.moveIntakeArm(-0.4)).withTimeout(0.2)
    );
  }

  @Override
  public void initialize() {
    m_jiggleCommand.initialize();
  }

  @Override
  public void execute() {
    m_jiggleCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_jiggleCommand.end(interrupted);
    m_intake.moveIntakeArm(0);
  }
}