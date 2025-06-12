package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon implements GyroIO {
    private final Pigeon2 gyro;

    public GyroIOPigeon(Pigeon2 gyro) {
        this.gyro = gyro;
    }

    public void updateInputs(){
        
    }
}
