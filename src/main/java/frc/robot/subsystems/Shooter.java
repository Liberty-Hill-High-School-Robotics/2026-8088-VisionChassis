package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;

public class Shooter extends SubsystemBase {

  // motors & variables here, define them and create any PIDs needed
  private SparkFlex shooterMotor;
  private SparkClosedLoopController shooterController;

  private double testPoint = 3000.0;

  public Shooter() {
    // config motor settings here
    shooterMotor = new SparkFlex(CanIDs.kShooterMotor, MotorType.kBrushless);
    shooterController = shooterMotor.getClosedLoopController();
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorSpeeds.kShooterP)
        .i(MotorSpeeds.kShooterI)
        .d(MotorSpeeds.kShooterD)
        .feedForward
        .kS(MotorSpeeds.kShooterS)
        .kV(MotorSpeeds.kShooterV)
        .kA(MotorSpeeds.kShooterA);
    shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put smartdashboard stuff, check for limit switches
    SmartDashboard.putNumber("Shooter Actual", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Test Point", testPoint);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    // Mostly used for debug and such
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Should include run/stop/run back, etc.

  // as well as check for limits and reset encoders,
  // return true/false if limit is true, or encoder >= x value

  public void shootAtSpeed(double distance) {
    double setpoint = distance; // TODO: Equation to convert distance to velocity goes here
    SmartDashboard.putNumber("Shooter Setpoint", setpoint);
    shooterController.setSetpoint(setpoint, ControlType.kVelocity);
  }

  public void testSetpoint() {
    shooterController.setSetpoint(testPoint, ControlType.kVelocity);
  }

  public void changeTestPoint(double amount) {
    testPoint += amount;
  }

  public void shooterStop() {
    shooterMotor.set(0);
  }
}
