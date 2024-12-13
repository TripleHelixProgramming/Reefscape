package frc.robot.vision;

import frc.robot.Constants.CameraConstants.CameraName;
import frc.robot.Constants.CameraConstants.TransformRobotToCamera;

public class Vision {

  private final Camera cameraFrontRight =
      new Camera(CameraName.kFrontRight, TransformRobotToCamera.kFrontRight);
  private final Camera cameraFrontLeft =
      new Camera(CameraName.kFrontLeft, TransformRobotToCamera.kFrontLeft);
  private final Camera cameraBackRight =
      new Camera(CameraName.kBackRight, TransformRobotToCamera.kBackRight);
  private final Camera cameraBackLeft =
      new Camera(CameraName.kBackLeft, TransformRobotToCamera.kBackLeft);

  public Vision() {}
}
