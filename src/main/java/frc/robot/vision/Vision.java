package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;

public class Vision {
  public record EstimatedPoseWithStdevs(
      EstimatedRobotPose pose, Matrix<N3, N1> stdev, String name) {}

  /*
   * Returns (possibly empty) list of camera pose estimates sorted by
   * ascending timestamp.
   */
  public List<EstimatedPoseWithStdevs> getPoseEstimates() {
    record Tuple<T1, T2, T3>(T1 t1, T2 t2, T3 t3) {}

    return Arrays.stream(Camera.values())
        // return List.of(Camera.FrontRight).stream()
        .map(
            cam ->
                new Tuple<>(
                    cam.getEstimatedGlobalPose(), cam.getEstimationStdDevs(), cam.toString()))
        .filter(tuple -> tuple.t1.isPresent())
        .map(tuple -> new EstimatedPoseWithStdevs(tuple.t1.get(), tuple.t2, tuple.t3))
        .sorted(
            (lhs, rhs) -> (int) Math.signum(lhs.pose.timestampSeconds - rhs.pose.timestampSeconds))
        .collect(Collectors.toList());
  }
}
