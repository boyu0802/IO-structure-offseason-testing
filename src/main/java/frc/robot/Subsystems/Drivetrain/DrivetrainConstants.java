package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;


//TODO check all constants
public class DrivetrainConstants {
    public static final double kDeadband = 0.02;

    public static final double kWheelRadius = Units.inchesToMeters(1.96);
    public static final double kDriveTrainWidth = Units.inchesToMeters(24.6);
    public static final double kDriveTrainLength = Units.inchesToMeters(24.6);
    public static final double kDriveTrainRadius = Math.hypot(kDriveTrainLength/2, kDriveTrainWidth/2);
    public static final double kDriveTrainMaxSpeedMPS = 6.95; //TODO: REal VALue
    public static final double kDriveTrainMaxAngularSpeedRadsPerSec = 27.78;
    public static final double kModuleMaxAngularSpeedRadPerSec = 31; //TODO real value
    public static final double kLoopTime = 0.02;

    //TODO: make sure if not on CANFD use 100 or a value that doesn't cause too much CAN utilization
    public static final double OdometryFrequency = 100.0; //Hz, how often the odometry thread runs, this should be higher than the loop time of the robot code to ensure that the odometry is updated more frequently than the robot code runs. 
    
    public static final double ROBOT_MASS_KG = 27.8; 
    public static final double ROBOT_MOI = 10.0; //TODO: real value, moment of inertia of the robot in kg*m^2
    public static final double WHEEL_COF = 0.9; //TODO: real value, coefficient of friction of the wheels, this is used to calculate the maximum acceleration of the robot based on the mass and the coefficient of friction
    
    public static final double DriveMotorGearRatio = 4.5; 
    public static final double TurnMotorGearRatio = 20.0;
    public static final double SlipCurrent = 120; //TODO: real value, current in amps that the motor will slip at, this is used to calculate the maximum speed of the robot based on the motor speed and the gear ratio
   
   public static final double CurrentLimit = 30;
   public static final double TurnCurrentLimit = 30;
   
    //positive x is front, positive y is left.
    // FL,FR,BL,BR
    // public static final Translation2d[] modulePositions = new Translation2d[]{
    //     new Translation2d(Units.inchesToMeters(kDriveTrainLength / 2), Units.inchesToMeters(kDriveTrainWidth / 2)),
    //     new Translation2d(Units.inchesToMeters(kDriveTrainLength / 2), Units.inchesToMeters(-kDriveTrainWidth / 2)),
    //     new Translation2d(Units.inchesToMeters(-kDriveTrainLength / 2), Units.inchesToMeters(kDriveTrainWidth / 2)),
    //     new Translation2d(Units.inchesToMeters(-kDriveTrainLength / 2), Units.inchesToMeters(-kDriveTrainWidth / 2))
    // };
    
    public static final Translation2d[] modulePositions = new Translation2d[]{
        new Translation2d(0.31242, 0.31242),
        new Translation2d(0.31242, -0.31242),
        new Translation2d(-0.31242, 0.31242),
        new Translation2d(-0.31242, -0.31242)
    };

    public static final RobotConfig kRobotConfig = new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        new ModuleConfig(
            kWheelRadius,
            kDriveTrainMaxSpeedMPS,
            WHEEL_COF,
            DCMotor.getKrakenX60Foc(1)
                .withReduction(DriveMotorGearRatio),
            SlipCurrent,
            1),
        modulePositions);

    public static final ModuleConstants FLModuleConstants = new ModuleConstants(
        4,
        3,
        10, 
        InvertedValue.CounterClockwise_Positive,
        InvertedValue.CounterClockwise_Positive,
        SensorDirectionValue.CounterClockwise_Positive,
        Rotation2d.fromRotations(-0.418701171875)
        ); 

    public static final ModuleConstants FRModuleConstants = new ModuleConstants(
        2,
        1,
        9,
        InvertedValue.Clockwise_Positive,
        InvertedValue.CounterClockwise_Positive,
        SensorDirectionValue.CounterClockwise_Positive,
        Rotation2d.fromRotations(0.420654296875)
        ); 


    public static final ModuleConstants BLModuleConstants = new ModuleConstants(
        6,
        5,
        11,
        InvertedValue.CounterClockwise_Positive,
        InvertedValue.CounterClockwise_Positive,
        SensorDirectionValue.CounterClockwise_Positive,
        Rotation2d.fromRotations(-0.433349609375)
        ); 


    public static final ModuleConstants BRModuleConstants = new ModuleConstants(
        8,
        7,
        12,
        InvertedValue.Clockwise_Positive,
        InvertedValue.CounterClockwise_Positive,
        SensorDirectionValue.CounterClockwise_Positive,
        Rotation2d.fromRotations(0.20068359375)
        ); 

    public record ModuleConstants(
        int driveMotorID,
        int angleMotorID,
        int cancoderID,
        InvertedValue driveInvertedValue,
        InvertedValue angleInvertedValue,
        SensorDirectionValue cancoderDirectionValue,
        Rotation2d angleOffset
    ){}

    public static final int PigeonID = 1;
    
    

}

