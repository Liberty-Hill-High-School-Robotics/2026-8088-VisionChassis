package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class TurretRight extends Command {
  // The subsystem the command runs on
  private final Turret m_turret;

  public TurretRight(Turret subsystem) {
    m_turret = subsystem;
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_turret.turretRight();
    ; // call to method from turret subsystem
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.turretStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
