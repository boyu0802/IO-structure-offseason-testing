package frc.robot.Subsystems.Drivetrain;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
public interface GyroIO {

    @AutoLog
    public static class GyroInputs{
        public GyroData data = new GyroData(false, Rotation2d.kZero, 0);
        public double[] gyroYawTimestamps = new double[]{};
        public Rotation2d[] gyroYawPositions = new Rotation2d[]{};
    }

    public record GyroData(
        boolean connected,
        Rotation2d yawPosition,
        double yawVelocityRadPerSec
    ){}

    public default void updateInputs(GyroInputs inputs){};
}
