package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
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
  PhotonCamera FrontTagCam = new PhotonCamera("FrontTagCam");
  PhotonCamera SideTagCam = new PhotonCamera("SideTagCam");
  PhotonCamera OBJCam = new PhotonCamera("OBJCam");

  private final EstimateConsumer estConsumer;
  private final PhotonPoseEstimator frontPhotonEstimator;
  private final PhotonPoseEstimator sidePhotonEstimator;
  private Matrix<N3, N1> curStdDevs;
  private double objYaw;

  public Vision(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;
    frontPhotonEstimator =
        new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kFrontRobotToCam);
    sidePhotonEstimator =
        new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kSideRobotToCam);
  }

  @Override
  public void periodic() {

    // Estimate Pose from front camera
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (PhotonPipelineResult result : FrontTagCam.getAllUnreadResults()) {
      visionEst = frontPhotonEstimator.estimateCoprocMultiTagPose(result);

      if (visionEst.isEmpty()) {
        visionEst = frontPhotonEstimator.estimateLowestAmbiguityPose(result);
      }

      updateEstimationStdDevs(visionEst, result.getTargets(), true);

      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();
            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }

    // Estimate Pose from side camera
    for (PhotonPipelineResult result : SideTagCam.getAllUnreadResults()) {
      visionEst = sidePhotonEstimator.estimateCoprocMultiTagPose(result);

      if (visionEst.isEmpty()) {
        visionEst = sidePhotonEstimator.estimateLowestAmbiguityPose(result);
      }

      updateEstimationStdDevs(visionEst, result.getTargets(), false);

      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();
            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }

    // Object Detection
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
   * @param isFront Whether to caluculate based off the front of side camera
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets,
      Boolean isFront) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs =
          (isFront)
              ? VisionConstants.kFrontSingleTagStdDevs
              : VisionConstants.kSideSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs =
          (isFront)
              ? VisionConstants.kFrontSingleTagStdDevs
              : VisionConstants.kSideSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (PhotonTrackedTarget tgt : targets) {
        var tagPose =
            ((isFront) ? frontPhotonEstimator : sidePhotonEstimator)
                .getFieldTags()
                .getTagPose(tgt.getFiducialId());
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
          curStdDevs =
              (isFront)
                  ? VisionConstants.kFrontSingleTagStdDevs
                  : VisionConstants.kSideSingleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1)
            estStdDevs =
                (isFront)
                    ? VisionConstants.kFrontMultiTagStdDevs
                    : VisionConstants.kSideMultiTagStdDevs;
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
