package frc.robot.Subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Subsystems.ModuleIOInputsAutoLogged;
import frc.robot.Util.Constants;
import frc.robot.Util.LoggedTracer;
import frc.robot.Util.LoggedTunableNumber;

public class Module {
    private static final LoggedTunableNumber driveKs = new LoggedTunableNumber("Drivetrain/Module/DriveKs");
    private static final LoggedTunableNumber driveKv = new LoggedTunableNumber("Drivetrain/Module/DriveKv");
    private static final LoggedTunableNumber driveKt = new LoggedTunableNumber("Drivetrain/Module/DriveKt");
    private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drivetrain/Module/DriveKp");
    private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drivetrain/Module/DriveKd");
    private static final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drivetrain/Module/TurnKp");
    private static final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drivetrain/Module/TurnKd");

    static{
        switch (Constants.getRobotType()) {
            case Real,Replay: //TODO: test value in amps(current control)
                driveKs.initDefault(5.0);   
                driveKv.initDefault(0);
                driveKt.initDefault(DrivetrainConstants.DriveMotorGearRatio / DCMotor.getKrakenX60Foc(1).KtNMPerAmp);
                driveKp.initDefault(35);
                driveKd.initDefault(0);
                turnKp.initDefault(4000);
                turnKd.initDefault(50.0);
            case Sim://TODO test values in voltage(Sim is in voltage)
                driveKs.initDefault(5.0);   
                driveKv.initDefault(0);
                driveKt.initDefault(0); 
                driveKp.initDefault(35);
                driveKd.initDefault(0);
                turnKp.initDefault(4000);
                turnKd.initDefault(50.0);
                
        } 

    }

    

    public final ModuleIO io;
    public final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    
    private SimpleMotorFeedforward ffModel;

    private final Alert driveDisconnected;
    private final Alert turnDisconnected;
    private final Alert turnEncoderDisconnected;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]{};

    public Module(ModuleIO io, int index){
        this.io = io;
        this.index = index;

        ffModel = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());

        driveDisconnected = new Alert("Drive motor is disconnected on module " + index, Alert.AlertType.kError);
        turnDisconnected = new Alert("Turn motor is disconnected on module " + index, Alert.AlertType.kError);
        turnEncoderDisconnected = new Alert("Turn encoder is disconnected on module " + index, Alert.AlertType.kError);

    }

    public void updateInputs(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
    }
    
    public void periodic(){
        if(driveKs.hasChanged(hashCode()) || driveKv.hasChanged(hashCode())){
            ffModel = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
        }
        if(driveKp.hasChanged(hashCode())||driveKd.hasChanged(hashCode())){
            io.setDrivePID(driveKp.get(), 0, driveKd.get());
        }
        if(turnKp.hasChanged(hashCode())||turnKd.hasChanged(hashCode())){
            io.setTurnPID(turnKp.get(), 0, turnKd.get());
        }

        

        int sampleCount = inputs.odometryDrivePositionsRad.length;
        odometryPositions = new SwerveModulePosition[sampleCount];
        for(int i = 0; i < sampleCount; i++){
            double positionMeters = inputs.odometryDrivePositionsRad[i] * DrivetrainConstants.kWheelRadius; //TODO: change to constants, wheel radius
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }


        driveDisconnected.set(!inputs.data.driveConnected());
        turnDisconnected.set(!inputs.data.turnConnected());
        turnEncoderDisconnected.set(!inputs.data.turnEncoderConnected());

        LoggedTracer.record("DriveTrain/Module" + index);
    }

    public void runSetpoint(SwerveModuleState state){
        double speedRadPerSec = state.speedMetersPerSecond / DrivetrainConstants.kWheelRadius;
        io.velocityDrive(speedRadPerSec,ffModel.calculate(speedRadPerSec));
        io.positionTurn(state.angle);
    }

    public void runSetpoint(SwerveModuleState state, double desiredWheelTorque){
        double speedRadPerSec = state.speedMetersPerSecond / DrivetrainConstants.kWheelRadius;
        double ff = ffModel.calculate(speedRadPerSec) + (driveKt.get() * desiredWheelTorque); //ff is in amps
        io.velocityDrive(speedRadPerSec, ff);
        io.positionTurn(state.angle);
    }

    public void runCharacterization(double voltage){
        io.voltageDrive(voltage);
        io.positionTurn(Rotation2d.kZero);
    }

    public double getPositionMeters(){
        return inputs.data.drivePositionRad() * DrivetrainConstants.kWheelRadius;
    }

    public double getVelocityMetersPerSec(){
        return inputs.data.driveVelRadPerSec() * DrivetrainConstants.kWheelRadius;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public SwerveModulePosition[] getOdometryPositions(){
        return odometryPositions;
    }


    public Rotation2d getAngle(){
        return inputs.data.turnPositionRad();
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }
}
