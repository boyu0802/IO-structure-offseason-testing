package frc.robot;

import java.util.Optional;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Drivetrain.DrivetrainConstants;
import frc.robot.Subsystems.Limelight.VisionPoseEstimate;

public class RobotState {
    public record OdometryRecord(
        double timeStamp,
        SwerveModulePosition[] modulePositions,
        Optional<Rotation2d> gyroYaw
    ){}


    private static RobotState instance = null;

    @AutoLogOutput private static Pose2d OdometryPose = new Pose2d();

    private SwerveModulePosition[] lastModulePosition = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
    private Rotation2d lastGyroYaw = new Rotation2d();
    private double lastGyroAngularVelocity = 0.0;
    private double lastAccelerationX = 0.0;
    private double lastAccelerationY = 0.0;
    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    private Twist2d robotAcceleration = new Twist2d();

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry odometry;

    private boolean hasTarget = false;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(2); //test time

    public RobotState(){
        kinematics = new SwerveDriveKinematics(DrivetrainConstants.modulePositions);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        }, new Pose2d(),
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(99999)), // State standard deviation
            VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(999999)) // Measurement standard deviation
        );
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },new Pose2d());
    }


    public static RobotState getInstance(){
        if(instance == null){
            instance = new RobotState();
        }
        return instance;
    }

    public void addOdometry(OdometryRecord record){
        if(!record.gyroYaw.isPresent()){ //gyro not available in sim
            // Twist2d twist = kinematics.toTwist2d(lastModulePosition,record.modulePositions());
            // Logger.recordOutput("RobotSTate/Odometry/lastModulePosition", lastModulePosition);
            // Logger.recordOutput("RobotSTate/Odometry/record Module Pos", record.modulePositions());
            // Logger.recordOutput("RobotSTate/Odometry/Twist", twist);
            // double angularRate = twist.dtheta;
            // lastGyroYaw = lastGyroYaw .rotateBy(Rotation2d.fromRadians(!Double.isNaN(angularRate) ? angularRate *0.02 : 0));
            
            SwerveModulePosition[] currentModulePositions = record.modulePositions();
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for(int i = 0; i < 4; i++){
                moduleDeltas[i] = new SwerveModulePosition(
                    currentModulePositions[i].distanceMeters - lastModulePosition[i].distanceMeters,
                    currentModulePositions[i].angle.minus(lastGyroYaw)
                );
            }
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            lastGyroYaw = lastGyroYaw.plus(new Rotation2d(twist.dtheta));
            Logger.recordOutput("RobotState/Module 0 angle", currentModulePositions[0].angle.getDegrees()%360);
            Logger.recordOutput("RobotSTate/Odometry/Twist", twist);


        }else{
            lastGyroYaw = record.gyroYaw.get();
        }

        lastModulePosition = record.modulePositions();

        OdometryPose = odometry.update(lastGyroYaw,lastModulePosition);
        Logger.recordOutput("Odometry/pose", OdometryPose);
        Logger.recordOutput("Odometry/gyroYaw", lastGyroYaw.getDegrees());
        // poseBuffer.addSample( record.timeStamp, OdometryPose);
        // poseEstimator.updateWithTime(record.timeStamp,lastGyroYaw, lastModulePosition);

    }

    public void addVision(VisionPoseEstimate visionPose){
        poseEstimator.addVisionMeasurement(visionPose.getVisionEstimatedPose(), visionPose.getTimeStamp(), visionPose.getStdDev());
    }

    @AutoLogOutput
    public Pose2d getRobotPose(){
        return poseEstimator.getEstimatedPosition();
    }
    
    // @AutoLogOutput
    public Rotation2d getRobotYaw(){
        // return poseEstimator.getEstimatedPosition().getRotation();
        return lastGyroYaw;
    }


    

}
