package frc.robot.Subsystems.Limelight;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Subsystems.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Limelight.LimelightHelpers.RawFiducial;

public interface VisionIO {
    @AutoLog
    public class VisionInput {
        public boolean hasTarget = false;
        public MegaTag2Data megaTag2Data;
    }


    public record MegaTag2Data(
        Pose2d megaTag2Pose,
        double timestamp,
        int tagCount,
        double latency,
        RawFiducial[] megaTag2RawFiducials
    ){}

    default void updateInputs(VisionInput inputs){};


}
