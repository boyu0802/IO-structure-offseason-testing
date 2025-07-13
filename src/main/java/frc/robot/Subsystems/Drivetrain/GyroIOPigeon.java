package frc.robot.Subsystems.Drivetrain;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon implements GyroIO {
    private final Pigeon2 gyro = new Pigeon2(DrivetrainConstants.PigeonID);
    private final StatusSignal<Angle> yawPosition = gyro.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = gyro.getAngularVelocityZWorld();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimeSTampQueue;

    public GyroIOPigeon() {
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);
        yawPosition.setUpdateFrequency(DrivetrainConstants.OdometryFrequency);
        yawVelocity.setUpdateFrequency(50);
        gyro.optimizeBusUtilization();
        yawTimeSTampQueue = OdometryThread.getOdometryThreadInstance().makeTimeStampQueue();
        yawPositionQueue = OdometryThread.getOdometryThreadInstance().registerSignal(yawPosition.clone());

    }

    public void updateInputs(GyroInputsAutoLogged inputs){
        inputs.data = 
            new GyroData(
                BaseStatusSignal.refreshAll(yawPosition, yawVelocity).equals(StatusCode.OK),
                Rotation2d.fromDegrees(yawPosition.getValueAsDouble()),
                Units.degreesToRadians(yawVelocity.getValueAsDouble())
            );
    
        inputs.gyroYawTimestamps = yawTimeSTampQueue.stream().mapToDouble((Double d) -> d).toArray();
        inputs.gyroYawPositions = yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

        yawTimeSTampQueue.clear();
        yawPositionQueue.clear();
    }
}
