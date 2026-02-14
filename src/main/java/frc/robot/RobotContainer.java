// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
// command imports
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootInHub;
import frc.robot.commands.Shooter.ChangeTestPoint;
import frc.robot.commands.Shooter.TestSetpoint;
import frc.robot.commands.Vision.DetectAndIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
// subsystems imports
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_drive;
  private final Vision m_vision;
  private final Shooter m_shooter;
  private final Turret m_turret;
  // Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        m_vision = new Vision(m_drive::addVisionMeasurement);
        m_shooter = new Shooter();
        m_turret = new Turret();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_vision = new Vision(m_drive::addVisionMeasurement);
        m_shooter = new Shooter();
        m_turret = new Turret();
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_vision = new Vision(m_drive::addVisionMeasurement);
        m_shooter = new Shooter();
        m_turret = new Turret();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX()));

    // add button bindings here
    /*
     * ex:
     * final Trigger CommandName = m_drivercontroller.button();
     * CommandName.toggleontrue(new commandFromCode(m_subsystem(s)));
     */

    // Lock to 0° when A button is held
    final Trigger JoystickDriveAtZero = m_driverController.a();
    JoystickDriveAtZero.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    final Trigger XPattern = m_driverController.x();
    XPattern.onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    // Reset gyro to 0° when B button is pressed
    final Trigger ResetHeading = m_driverController.b();
    ResetHeading.onTrue(
        Commands.runOnce(
                () ->
                    m_drive.setPose(
                        new Pose2d(m_drive.getPose().getTranslation(), Rotation2d.kZero)),
                m_drive)
            .ignoringDisable(true));

    // Example for positioning based on a target
    final Trigger PointAtTarget = m_driverController.y();
    PointAtTarget.whileTrue(new DetectAndIntake(m_vision, m_drive));

    // Shoot at an RPM  based on a given distance
    final Trigger ShootAtTrigger = m_driverController.rightBumper();
    ShootAtTrigger.toggleOnTrue(new ShootInHub(m_turret, m_shooter));

    final Trigger TestSetpoint = m_driverController.leftBumper();
    TestSetpoint.whileTrue(new TestSetpoint(m_shooter));

    final Trigger IncreaseTestPoint = m_driverController.povUp();
    IncreaseTestPoint.whileTrue(new ChangeTestPoint(m_shooter, 100.0));

    final Trigger DecreaseTestPoint = m_driverController.povDown();
    DecreaseTestPoint.whileTrue(new ChangeTestPoint(m_shooter, -100.0));

    final Trigger IncreaseTestPointSmall = m_driverController.start();
    IncreaseTestPointSmall.whileTrue(new ChangeTestPoint(m_shooter, 10.0));

    final Trigger DecreaseTestPointSmall = m_driverController.back();
    DecreaseTestPointSmall.whileTrue(new ChangeTestPoint(m_shooter, -10.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
