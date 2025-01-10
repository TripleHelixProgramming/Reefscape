package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;

public class Vision {
  public record EstimatedPoseWithStdevs(EstimatedRobotPose pose, Matrix<N3, N1> stdev) {}

  /*
   * Returns (possibly empty) list of camera pose estimates sorted by
   * ascending timestamp.
   */
  public List<EstimatedPoseWithStdevs> getPoseEstimates() {
    return Arrays.stream(Camera.values())
        .map(cam -> new Pair<>(cam.getEstimatedGlobalPose(), cam.getEstimationStdDevs()))
        .filter(pair -> pair.getFirst().isPresent())
        .map(pair -> new EstimatedPoseWithStdevs(pair.getFirst().get(), pair.getSecond()))
        .sorted(
            (lhs, rhs) -> (int) Math.signum(lhs.pose.timestampSeconds - rhs.pose.timestampSeconds))
        .collect(Collectors.toList());
  }
}
