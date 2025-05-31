package frc.robot.Subsystems;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;


//TODO check all constants
public class DrivetrainConstants {
    public static final double kWheelRadius = Units.inchesToMeters(4);
    public static final double kDriveTrainWidth = 12;
    public static final double kDriveTrainLength = 12;//todo: real value
    public static final double kDriveTrainMaxSpeedMPS = 3.0; //TODO: real value
    public static final double kDriveTrainMaxAngularVelocityRadsPerSec = Math.PI; //TODO: real value
    public static final double kLoopTime = 0.02;

    //positive x is front, positive y is left.
    // FL,FR,BL,BR
    public static final Translation2d[] modulePositions = new Translation2d[]{
        new Translation2d(Units.inchesToMeters(kDriveTrainLength / 2), Units.inchesToMeters(kDriveTrainWidth / 2)),
        new Translation2d(Units.inchesToMeters(kDriveTrainLength / 2), Units.inchesToMeters(-kDriveTrainWidth / 2)),
        new Translation2d(Units.inchesToMeters(-kDriveTrainLength / 2), Units.inchesToMeters(kDriveTrainWidth / 2)),
        new Translation2d(Units.inchesToMeters(-kDriveTrainLength / 2), Units.inchesToMeters(-kDriveTrainWidth / 2))
    };









    public static RobotConfig getRobotConfig(){
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return new RobotConfig(0,0, new ModuleConfig(0,0,0,DCMotor.getKrakenX60(1),0,0), 0);
        }
    }





}
