package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ChangeTestPoint extends Command {
  // The subsystem the command runs on
  private final Shooter m_shooter;
  private double amount;

  public ChangeTestPoint(Shooter subsystem, double amount) {
    m_shooter = subsystem;
    this.amount = amount;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.changeTestPoint(amount);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
