package frc.robot.Util;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static boolean tuningMode = false;
    public static boolean disableHAL = false;

    public static final RobotMode robotSimMode = RobotMode.Sim;

    public static RobotMode getRobotType(){
        if(RobotBase.isReal()){
            return RobotMode.Real;
        }else{
            return robotSimMode;
        }
    }

    public enum RobotMode{
        Real,
        Sim,
        Replay
    }
}
