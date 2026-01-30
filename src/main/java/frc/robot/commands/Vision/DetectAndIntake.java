package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class DetectAndIntake extends ParallelCommandGroup {

  public DetectAndIntake(Vision m_vision, Drive m_drive) {
    PIDController targetPID = new PIDController(.02, 0, 0);
    addCommands(
        DriveCommands.robotDrive(
            m_drive, () -> -0.5, () -> 0.0, () -> targetPID.calculate(0, m_vision.getObjYaw())));
    targetPID.close();
  }

  @Override
  public boolean runsWhenDisabled() {

    return false;
  }
}
