package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ShootAtSpeed extends Command {
  // The subsystem the command runs on
  private final Shooter m_shooter;
  private double distance;

  public ShootAtSpeed(Shooter subsystem, double distance) {
    m_shooter = subsystem;
    this.distance = distance;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.shootAtSpeed(distance);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.shooterStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
