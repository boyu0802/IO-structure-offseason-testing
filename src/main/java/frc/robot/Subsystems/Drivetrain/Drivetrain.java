package frc.robot.Subsystems.Drivetrain;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Subsystems.GyroInputsAutoLogged;
import frc.robot.Util.Constants;
import frc.robot.Util.LoggedTracer;
import frc.robot.Util.LoggedTunableNumber;



public class Drivetrain extends SubsystemBase{
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyro;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final Alert gyroDisconnectedAlert = new Alert("Gyro is disconnected", Alert.AlertType.kError);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DrivetrainConstants.modulePositions);// FL,FR,BL,BR

    @AutoLogOutput private boolean velocityMode = false;
    @AutoLogOutput private boolean brakeModeOn = false;

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(getChassisSpeeds(),getModuleStates(), DriveFeedforwards.zeros(4));
    private final SwerveSetpointGenerator setpointGenerator;

    public Drivetrain(GyroIO gyro, ModuleIO[] moduleIOs) {
        this.gyro = gyro;
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(moduleIOs[i], i);
        }
        setBrakeMode(true);
        this.setpointGenerator = new SwerveSetpointGenerator(DrivetrainConstants.kRobotConfig,DrivetrainConstants.kDriveTrainMaxAngularVelocityRadsPerSec);
        OdometryThread.getOdometryThreadInstance().start();
    }   


    public void periodic(){
        odometryLock.lock();
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drivetrain/gyroInputs", gyroInputs);
        for(var module : modules){
            module.updateInputs();
        }
        LoggedTracer.record("Drivetrain/moduleInputs");
        odometryLock.unlock();

        for(var module : modules){
            module.periodic();
        }
        

        double[] sampleTimeStamps = 
            Constants.getRobotType() == Constants.RobotMode.Sim 
            ? new double[]{Timer.getFPGATimestamp()} 
            : gyroInputs.odometryYawTimestamps;
        int sampleCount = sampleTimeStamps.length;
        for(int i = 0; i < sampleCount; i++){
            SwerveModulePosition[] wheelPosition = new SwerveModulePosition[4];
            SwerveModuleState[] moduleStates = new SwerveModuleState[4];
            for(int j = 0; j < 4; j++){
                wheelPosition[j] = modules[j].getOdometryPositions()[i];
            }
            RobotState.getInstance().addOdometry(new RobotState.OdometryRecord(sampleTimeStamps[i], wheelPosition, gyroInputs.odometryYawPositions[i]));

        }

        
    }

    public void runVelocity(ChassisSpeeds speeds){
        velocityMode = true;
        
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, DrivetrainConstants.kLoopTime);
        SwerveModuleState[] unoptimizedStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        currentSetpoint = setpointGenerator.generateSetpoint(currentSetpoint, discreteSpeeds, DrivetrainConstants.kLoopTime);
        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        Logger.recordOutput("Drivetrain/SwerveStates/Unoptimized Setpoint", unoptimizedStates );
        Logger.recordOutput("Drivetrain/SwerveStates/Optimized Setpoint", setpointStates);
        Logger.recordOutput("Drivetrain/SwerveChassisSpeeds/setPoint", currentSetpoint.robotRelativeSpeeds());

        for(int i = 0; i<4; i++){
            modules[i].runSetpoint(setpointStates[i]);;
        }
    }


    public void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> moduleForces){
        velocityMode = true;
        
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, DrivetrainConstants.kLoopTime);
        SwerveModuleState[] unoptimizedStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        currentSetpoint = setpointGenerator.generateSetpoint(currentSetpoint, discreteSpeeds, DrivetrainConstants.kLoopTime);
        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        Logger.recordOutput("Drivetrain/SwerveStates/Unoptimized Setpoint", unoptimizedStates );
        Logger.recordOutput("Drivetrain/SwerveStates/Optimized Setpoint", setpointStates);
        Logger.recordOutput("Drivetrain/SwerveChassisSpeeds/setPoint", currentSetpoint.robotRelativeSpeeds());

        SwerveModuleState[] wheelForcesState = new SwerveModuleState[4];
        SwerveModuleState[] currentModuleStates = getModuleStates();

        for(int i = 0; i < 4; i++){
            Rotation2d wheelAngle = currentModuleStates[i].angle;
            setpointStates[i].optimize(wheelAngle);
            setpointStates[i].cosineScale(wheelAngle);

            var wheelForces = moduleForces.get(i);
            Vector<N2> wheelDirection = VecBuilder.fill(wheelAngle.getCos(), wheelAngle.getSin());
            double wheelTorqueNm = wheelForces.dot(wheelDirection) * DrivetrainConstants.kWheelRadius;

            modules[i].runSetpoint(setpointStates[i], wheelTorqueNm);

            wheelForcesState[i] = new SwerveModuleState(wheelTorqueNm,setpointStates[i].angle);
        }

        Logger.recordOutput("Drivetrain/SwerveStates/WheelForces", wheelForcesState);
    }

    public void setBrakeMode(boolean brakeMode){
        if(brakeMode != brakeModeOn){
            for(Module mod: modules){
                mod.setBrakeMode(brakeMode);
            }
            brakeModeOn = brakeMode;
        }
    }

    @AutoLogOutput(key = "Drivetrain/ChassisSpeeds")
    private ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }


    @AutoLogOutput(key = "Drivetrain/ModuleStates")
    private SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public Rotation2d getGyroRotation(){
        return gyroInputs.data.yawPosition();
    }

    public void stop(){
        runVelocity(new ChassisSpeeds());
    }

    public void stopX(){
        Rotation2d[] headings = new Rotation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            headings[i] = DrivetrainConstants.modulePositions[i].getAngle();
            modules[i].io.positionTurn(headings[i]);
        }
        
        // kinematics.resetHeadings(headings); //TODO: see if this one will work
        stop();
    }

    public void runCharacterization(double voltage){
        velocityMode = false;
        for (Module module : modules) {
            module.runCharacterization(voltage);
        }
    }

    public void getWheelRadiusCharacterization(){
        
    }


    

}
