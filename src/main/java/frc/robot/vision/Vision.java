package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Vision {

  private Matrix<N3, N1> currentStdevs;

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    for (var camera : Camera.values()) {
      var est = camera.getEstimatedGlobalPose();
      if (!est.isEmpty()) {
        currentStdevs = camera.getEstimationStdDevs();
        return est;
      }
    }
    return Optional.empty();
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return currentStdevs;
  }
}
