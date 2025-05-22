package frc.robot.Util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

public class LoggedTracer {
    private LoggedTracer(){}

    private static double startTime = -1.0;

    public static void reset(){
        startTime = Timer.getFPGATimestamp();
    }

    public static void record(String name){
        double currentTime = Timer.getFPGATimestamp();
        Logger.recordOutput("LoggedTracer/" + name + "MS", (currentTime - startTime) * 1000);
        startTime = currentTime;
    }
}
