package frc.lib;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseLogger extends SubsystemBase {
    private static final PoseLogger defaultPosePublisher = new PoseLogger("Poses");

    private record Info (Supplier<Optional<Pose2d>> supplier, StructPublisher<Pose2d> topic) {}
    private final Map<String, Info> posePublishers = new HashMap<>();
    private final NetworkTable table;


    public PoseLogger(String groupName) {
        table = NetworkTableInstance.getDefault().getTable(groupName);
    }

    public static PoseLogger getDefault() {
        return defaultPosePublisher;
    }

    public void monitor(String name, Supplier<Optional<Pose2d>> poseSupplier) {
        posePublishers.put(name, 
            new Info(poseSupplier, 
                table.getStructTopic(name, Pose2d.struct).publish()));
    }

    public void cancel(String name) {
        posePublishers.remove(name);
    }

    @Override
    public void periodic() {
        posePublishers.forEach( (name, info) -> info.supplier.get().ifPresent(pose -> info.topic.set(pose)));
    }
}
