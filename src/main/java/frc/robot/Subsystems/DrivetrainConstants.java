package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


//TODO check all constants
public class DrivetrainConstants {
    public static final double kWheelRadius = Units.inchesToMeters(4);
    public static final double kdriveTrainWidth = 12;
    public static final double kdriveTrainLength = 12;//todo: real value

    //positive x is front, positive y is left.
    // FL,FR,BL,BR
    public static final Translation2d[] modulePositions = new Translation2d[]{
        new Translation2d(Units.inchesToMeters(kdriveTrainLength / 2), Units.inchesToMeters(kdriveTrainWidth / 2)),
        new Translation2d(Units.inchesToMeters(kdriveTrainLength / 2), Units.inchesToMeters(-kdriveTrainWidth / 2)),
        new Translation2d(Units.inchesToMeters(-kdriveTrainLength / 2), Units.inchesToMeters(kdriveTrainWidth / 2)),
        new Translation2d(Units.inchesToMeters(-kdriveTrainLength / 2), Units.inchesToMeters(-kdriveTrainWidth / 2))
    };
}
