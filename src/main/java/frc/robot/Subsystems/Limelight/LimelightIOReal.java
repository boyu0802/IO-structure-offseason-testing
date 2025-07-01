package frc.robot.Subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;

public class LimelightIOReal implements VisionIO {
    private Pose2d lastEstimate;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstants.LimelightName);
    private RobotState state;

    public LimelightIOReal(){
        state = RobotState.getInstance();
    }

    public void LLSettings(){
        LimelightHelpers.setCameraPose_RobotSpace(VisionConstants.LimelightName, 0, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(VisionConstants.LimelightName, state.getRobotYaw().getDegrees(), 0, 0, 0, 0, 0);
    }

    public void updateInputs(VisionIO.VisionInput input){
        input.hasTarget = LimelightHelpers.getTV(VisionConstants.LimelightName);
        if(input.hasTarget){
            var megaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LimelightName);
            input.megaTag2Data = new MegaTag2Data(
                megaTag2Pose.pose,
                megaTag2Pose.timestampSeconds,
                megaTag2Pose.tagCount,
                megaTag2Pose.latency,
                megaTag2Pose.rawFiducials
            );
        }

        LLSettings();
        
    }
}
