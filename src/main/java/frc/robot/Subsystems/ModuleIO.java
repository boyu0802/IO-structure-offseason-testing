package frc.robot.Subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs{
        public ModuleIOData data = new ModuleIOData(
            false,0,0,0,0,0,false,false,Rotation2d.kZero,Rotation2d.kZero,0,0,0,0
        );

        public double[] odometryDrivePositionsRad = new double[]{};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[]{}; 

    }

    public record ModuleIOData(
        boolean driveConnected,
        double drivePositionRad,
        double driveVelRadPerSec,
        double driveAppliedVolts,
        double driveSupplyCurrent,
        double driveTorqueCurrent,
        boolean turnConnected,
        boolean turnEncoderConnected,
        Rotation2d turnAbsolutePosition,
        Rotation2d turnPositionRad,
        double turnVelRadPerSec,
        double turnAppliedVolts,
        double turnSupplyCurrent,
        double turnTorqueCurrent
    ){}


    public default void updateInputs(ModuleIOInputs input){};


    public default void voltageDrive(double voltage){};
    public default void voltageTurn(double voltage){};
    
    public default void velocityDrive(double velocity, double ff){};
    public default void positionTurn(Rotation2d position){};

    public default void setDrivePID(double kP, double kI, double kD){};
    public default void setTurnPID(double kP, double kI, double kD){};

    public default void setBrakeMode(boolean enabled){};

    
}
