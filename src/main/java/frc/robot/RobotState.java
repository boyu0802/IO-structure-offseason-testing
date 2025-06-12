package frc.robot;

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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Drivetrain.DrivetrainConstants;

public class RobotState {
    public record OdometryRecord(
        double timeStamp,
        SwerveModulePosition[] modulePositions,
        Rotation2d gyroYaw
    ){}

    public record VisionRecord(
        double timeStamp,
        Pose2d visionPose,
        int[] tagId,
        boolean hasTarget,
        double distance
    ){}

    private static RobotState instance = null;

    private static Pose2d OdometryPose = new Pose2d();

    private SwerveModulePosition[] lastModulePosition = new SwerveModulePosition[4];
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
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), new SwerveModulePosition[4], new Pose2d(),
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(99999)), // State standard deviation
            VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(999999)) // Measurement standard deviation
        );
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new SwerveModulePosition[4],new Pose2d());
    }


    public static RobotState getInstance(){
        if(instance == null){
            instance = new RobotState();
        }
        return instance;
    }

    public void addOdometry(OdometryRecord record){
        if(record.gyroYaw == null){ //gyro not available in sim
            Twist2d twist = kinematics.toTwist2d(lastModulePosition,record.modulePositions);
            lastGyroYaw = lastGyroYaw.plus(new Rotation2d(twist.dtheta));

        }else{
            lastGyroYaw = record.gyroYaw;
        }

        lastModulePosition = record.modulePositions;

        OdometryPose = odometry.update(lastGyroYaw,lastModulePosition);
        poseBuffer.addSample( record.timeStamp, OdometryPose);
        poseEstimator.updateWithTime(record.timeStamp,lastGyroYaw, lastModulePosition);

    }

    public void addVision(VisionRecord record){
        hasTarget = record.hasTarget;

        try{
            poseBuffer.getInternalBuffer().firstKey();
        }
    }


    public void addDriveSpeeds(){

    }
    

}
