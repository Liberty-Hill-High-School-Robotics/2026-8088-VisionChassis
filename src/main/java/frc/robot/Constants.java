// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double kTurretXOffset = Units.inchesToMeters(3.78230505);
  public static final double kTurretYOffset = 0;

  public static final double kTurretMaxAngle =
      270.0; // TODO: get a real number from physical turret
  public static final double kTurretForwardLimit =
      7.5; // TODO: get a real number from physical turret
  public static final double kTurretZeroOffset = (kTurretMaxAngle - 180) / 2;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class CanIDs {
    // can IDs
    public static final int kFLDrivingCAN = 1;
    public static final int kFRDrivingCAN = 2;
    public static final int kBLDrivingCAN = 3;
    public static final int kBRDrivingCAN = 4;

    public static final int kFLTurningCAN = 5;
    public static final int kFRTurningCAN = 6;
    public static final int kBLTurningCAN = 7;
    public static final int kBRTurningCAN = 8;
    public static final int kGyroID = 9;

    // Turret
    public static final int kTurretPivot = 12;

    // Shooter
    public static final int kShooterMotor = 13;
  }

  public static final class MotorSpeeds {

    // Turret gains
    public static final double kTurretP = 0.0;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0.0;
    // https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
    public static final double kTurretS = 0.0; // Static
    public static final double kTurretV = 0.0; // Velocity
    public static final double kTurretA = 0.0; // Acceleration

    public static final double kTurretCruise = 0.0; // Cruise velocity
    public static final double kTurretAccel = 0.0; // Max acceleration
    public static final double kTurretError = 0.0; // Allowed profile error

    public static final double kTurretSpeed = .25;

    // Shooter gains
    public static final double kShooterP = 0.0;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    // https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
    public static final double kShooterS = 0.0; // Static
    public static final double kShooterV = 0.0; // Velocity
    public static final double kShooterA = 0.0; // Acceleration
  }

  public static final class OIConstants {
    // controller ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.2;
  }

  public static final class VisionConstants {
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final Transform3d kFrontRobotToCam =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10.72816001),
                Units.inchesToMeters(9.7311548),
                Units.inchesToMeters(12.4486466)),
            new Rotation3d(0, 0, 0)); // TODO: get real numbers from CAD

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kFrontSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kFrontMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final Transform3d kSideRobotToCam =
        new Transform3d(
            new Translation3d(0, 0.0, 0),
            new Rotation3d(0, 0, 0)); // TODO: get real numbers from CAD

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSideSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kSideMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class FieldConstants {
    public static final Pose2d kBlueHubPose =
        new Pose2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32), new Rotation2d());

    public static final Pose2d kRedHubPose =
        new Pose2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32), new Rotation2d());
  }
}
