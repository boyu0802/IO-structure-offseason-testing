package frc.robot.Util;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static boolean tuningMode = false;
    public static boolean disableHAL = true;

    public static final RobotMode fakeRobotMode = RobotMode.Sim;

    @AutoLogOutput
    public static RobotMode getRobotType(){
        if(RobotBase.isReal()){
            return RobotMode.Real;
        }else{
            return fakeRobotMode;
        }
    }

    public enum RobotMode{
        Real,
        Sim,
        Replay
    }
}
