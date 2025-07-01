package frc.robot.Subsystems.Limelight;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO io;
    private final RobotState state;
    private final VisionInputAutoLogged inputs = new VisionInputAutoLogged();

    private double lastTimeStamp = 0.0;

    public VisionSubsystem(VisionIO io){
        this.io = io;
        state = RobotState.getInstance();
    }

    public void periodic(){
        double timeStamp = Logger.getTimestamp();

        io.updateInputs(inputs);
        Logger.processInputs("LimelightInputs", inputs);

        if(inputs.hasTarget){
            updateVision(inputs.hasTarget,inputs.megaTag2Data);
        }


        Logger.recordOutput("Vision/latency", Logger.getTimestamp() - timeStamp);
    }

    private void updateVision(boolean hasTarget, VisionIO.MegaTag2Data megaTag2Data){
        if(megaTag2Data.megaTag2Pose() != null){
            var updateTimeStamp = megaTag2Data.timestamp();
            boolean alreadyProcessedTime = updateTimeStamp == lastTimeStamp;

            if(!alreadyProcessedTime && hasTarget){

                if(shouldUseVisionEstimate(megaTag2Data)){
                    Optional<VisionPoseEstimate> megatag2Pose = processMegatag2(megaTag2Data);
                    state.addVision(megatag2Pose.get());

                }
            }

            
        }
    }

    private Optional<VisionPoseEstimate> processMegatag2(VisionIO.MegaTag2Data megaTag2Data){
       return null; 
    }  

    private boolean shouldUseVisionEstimate(VisionIO.MegaTag2Data poseEst){
        return false;
    }

}
