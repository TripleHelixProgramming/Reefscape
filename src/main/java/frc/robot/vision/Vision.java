package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Vision {

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    /*
     * TODO: Need some logic here to poll all cameras and select the best one.
     * Look for one with non-empty pose and lowest error.
     */
    return Camera.FrontRight.getEstimatedGlobalPose();
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return Camera.FrontRight.getEstimationStdDevs();
  }
}
