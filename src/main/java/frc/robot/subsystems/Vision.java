package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html
// READ THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

public class Vision extends SubsystemBase {

  // make sure the name in quotes is EXACTLY the same as it is in PV
  PhotonCamera AprilTagCam = new PhotonCamera("AprilTagCam");
  PhotonCamera OBJCam = new PhotonCamera("OBJCam");
  private final Field2d field = new Field2d();
  private final EstimateConsumer estConsumer;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;
  private double objYaw;

  public Vision(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;
    photonEstimator =
        new PhotonPoseEstimator(
            Constants.VisionConstants.kTagLayout, Constants.VisionConstants.kRobotToCam);
  }

  @Override
  public void periodic() {

    // Cheack every tag to estimate Pose
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (PhotonPipelineResult result : AprilTagCam.getAllUnreadResults()) {
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);

      if (visionEst.isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }

      updateEstimationStdDevs(visionEst, result.getTargets());

      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();
            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
      /*
            if (result.hasTargets()) {
              for (PhotonTrackedTarget tgt : result.getTargets()) {
                if (tgt.getFiducialId() == 3) { // Center on tag ID #3
                  SmartDashboard.putBoolean("TARGET", true);
                  SmartDashboard.putNumber("cameraX", tgt.getBestCameraToTarget().getX());
                  SmartDashboard.putNumber("cameraY", tgt.getBestCameraToTarget().getY());
                  SmartDashboard.putNumber("cameraZ", tgt.getBestCameraToTarget().getZ());
                }
              }
            } else {
              SmartDashboard.putBoolean("TARGET", false);
      }
              */
    }

    for (PhotonPipelineResult result : OBJCam.getAllUnreadResults()) {
      if (result.hasTargets()) {
        PhotonTrackedTarget tgt = result.getBestTarget();
        objYaw = tgt.getYaw();
        SmartDashboard.putBoolean("FUEL", true);
        SmartDashboard.putNumber("objYaw", objYaw);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    // Mostly used for debug and such
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (PhotonTrackedTarget tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          curStdDevs = estStdDevs;
        }
      }
    }
  }

  private Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // Sends Pose to Drive
  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }

  public double getObjYaw() {
    return objYaw;
  }
}
