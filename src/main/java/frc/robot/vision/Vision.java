package frc.robot.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision {

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return Camera.FrontRight.getEstimatedGlobalPose();
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return Camera.FrontRight.getEstimationStdDevs();
    }
}
