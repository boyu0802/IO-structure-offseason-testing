package frc.robot.Subsystems.Limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lombok.Getter;

public class VisionPoseEstimate {
    private @Getter Pose2d visionEstimatedPose;
    private @Getter double timeStamp = 0.0;
    private @Getter Matrix<N3,N1> stdDev;

    
    public VisionPoseEstimate(Pose2d visionEstimatedPose, double timeStamp, Matrix<N3,N1> stdDev){
        this.stdDev = stdDev;
        this.timeStamp = timeStamp;
        this.visionEstimatedPose = visionEstimatedPose;
    }


}
