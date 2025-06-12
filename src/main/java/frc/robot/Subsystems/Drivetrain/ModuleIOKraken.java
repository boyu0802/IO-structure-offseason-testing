package frc.robot.Subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Subsystems.Drivetrain.DrivetrainConstants.ModuleConstants;
import frc.robot.Subsystems.Drivetrain.ModuleIO.ModuleIOData;

public class ModuleIOKraken {
    
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder cancoder;

    private final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    private final Rotation2d encoderOffset;

    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0);

    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveSupplyCurrent;
    private final StatusSignal<Current> driveTorqueCurrent;

    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnSupplyCurrent;
    private final StatusSignal<Current> turnTorqueCurrent;

    public ModuleIOKraken(ModuleConstants constants){
        driveMotor = new TalonFX(constants.driveMotorID());
        turnMotor = new TalonFX(constants.angleMotorID());
        cancoder = new CANcoder(constants.cancoderID());
        encoderOffset = constants.angleOffset();

        driveConfig();
        turnConfig(constants);
        cancoderConfig(constants);

        drivePosition = driveMotor.getPosition();
        drivePositionQueue = OdometryThread.getOdometryThreadInstance().registerSignal(driveMotor.getPosition().clone());
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveTorqueCurrent = driveMotor.getTorqueCurrent();

        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turnMotor.getPosition();
        turnPositionQueue = OdometryThread.getOdometryThreadInstance().registerSignal(turnMotor.getPosition().clone());
        turnVelocity = turnMotor.getVelocity();
        turnAppliedVolts = turnMotor.getMotorVoltage();
        turnSupplyCurrent = turnMotor.getSupplyCurrent();
        turnTorqueCurrent = turnMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50,driveVelocity,driveAppliedVolts,driveSupplyCurrent,driveTorqueCurrent,turnAppliedVolts,turnVelocity,turnSupplyCurrent,turnTorqueCurrent);
        BaseStatusSignal.setUpdateFrequencyForAll(250,drivePosition,turnPosition,turnAbsolutePosition);

        ParentDevice.optimizeBusUtilizationForAll(driveMotor,turnMotor,cancoder);
        

    }

    public void updateInputs(ModuleIO.ModuleIOInputs input){
        input.data = new ModuleIOData(
            BaseStatusSignal.isAllGood(
                drivePosition,
                driveVelocity,
                driveSupplyCurrent,
                driveTorqueCurrent,
                driveAppliedVolts
            ),
            Units.rotationsToRadians(drivePosition.getValueAsDouble()),
            Units.rotationsToRadians(driveVelocity.getValueAsDouble()),
            driveAppliedVolts.getValueAsDouble(),
            driveSupplyCurrent.getValueAsDouble(),
            driveTorqueCurrent.getValueAsDouble(),
            BaseStatusSignal.isAllGood(
                turnPosition,
                turnVelocity,
                turnSupplyCurrent,
                turnTorqueCurrent,
                turnAppliedVolts
            ),
            BaseStatusSignal.isAllGood(turnAbsolutePosition),
            Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()),
            Rotation2d.fromRotations(turnPosition.getValueAsDouble()),
            Units.rotationsToRadians(turnVelocity.getValueAsDouble()),
            turnAppliedVolts.getValueAsDouble(),
            turnSupplyCurrent.getValueAsDouble(),
            turnTorqueCurrent.getValueAsDouble()
        );
        input.odometryDrivePositionsRad = 
            drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
        input.odometryTurnPositions = 
            turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }


    public void driveOpenLoop(double output){
        driveMotor.setControl(torqueCurrentRequest.withOutput(output));
    }

    public void turnOpenLoop(double output){
        turnMotor.setControl(torqueCurrentRequest.withOutput(output));
    }

    public void driveClosedLoop(double velocity, double ff){
        driveMotor.setControl(velocityTorqueCurrentRequest.withVelocity(Units.radiansToRotations(velocity)).withFeedForward(ff));
    }

    public void turnClosedLoop(Rotation2d position){
        turnMotor.setControl(positionTorqueCurrentRequest.withPosition(position.getRotations()));
    }

    private void driveConfig(){
        driveMotorConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.Feedback.SensorToMechanismRatio = DrivetrainConstants.DriveMotorGearRatio ;
        driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DrivetrainConstants.CurrentLimit;
        driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = DrivetrainConstants.CurrentLimit;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = DrivetrainConstants.CurrentLimit;
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(driveMotorConfig);
        driveMotor.setPosition(0, 0.25);
    }

    private void turnConfig(ModuleConstants constants){
        turnMotorConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
        turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnMotorConfig.Feedback.SensorToMechanismRatio = DrivetrainConstants.TurnMotorGearRatio ;
        turnMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DrivetrainConstants.TurnCurrentLimit;
        turnMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = DrivetrainConstants.TurnCurrentLimit;
        turnMotorConfig.CurrentLimits.StatorCurrentLimit = DrivetrainConstants.TurnCurrentLimit;
        turnMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;//changed to fused if pheonix pro
        turnMotorConfig.Feedback.FeedbackRemoteSensorID = constants.cancoderID();
        turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true; 
        turnMotorConfig.MotorOutput.Inverted = constants.angleInvertedValue(); 
        turnMotor.getConfigurator().apply(turnMotorConfig);
    }

    private void cancoderConfig(ModuleConstants constants){
        cancoderConfig.MagnetSensor.SensorDirection = constants.cancoderDirectionValue();
        cancoderConfig.MagnetSensor.MagnetOffset = constants.angleOffset().getRadians();
        cancoder.getConfigurator().apply(cancoderConfig);
    }

}