package frc.robot.Subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.Util.LoggedTracer;



public class Drivetrain extends SubsystemBase{
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyro;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final Alert gyroDisconnectedAlert = new Alert("Gyro is disconnected", Alert.AlertType.kError);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        DrivetrainConstants.modulePositions[0],
        DrivetrainConstants.modulePositions[1],
        DrivetrainConstants.modulePositions[2],
        DrivetrainConstants.modulePositions[3]);// FL,FR,BL,BR

    @AutoLogOutput private boolean velocityMode = false;
    @AutoLogOutput private boolean brakeModeOn = false;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint currentSetpoint;

    private final SysIdRoutine driveMotorSysID = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
        (voltage) -> runCharacterization(voltage.in(Volts)),
         null, 
         this));


    // FL,FR,BL,BR
    public Drivetrain(GyroIO gyro, ModuleIO moduleIOFL, ModuleIO moduleIOFR, ModuleIO moduleIOBL, ModuleIO moduleIOBR) {
        this.gyro = gyro;
        
        this.modules[0] = new Module(moduleIOFL,0);
        this.modules[1] = new Module(moduleIOFR,1);
        this.modules[2] = new Module(moduleIOBL,2);
        this.modules[3] = new Module(moduleIOBR,3);
        
        setBrakeMode(true);
        this.setpointGenerator = new SwerveSetpointGenerator(DrivetrainConstants.kRobotConfig,DrivetrainConstants.kModuleMaxAngularSpeedRadPerSec);
        currentSetpoint = new SwerveSetpoint(getChassisSpeeds(),getModuleStates(), DriveFeedforwards.zeros(4));

        OdometryThread.getOdometryThreadInstance().start();
    }   


    public void periodic(){
        odometryLock.lock();
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drivetrain/gyroInputs", gyroInputs);
        for(var module : modules){
            module.periodic();
        }
        LoggedTracer.record("Drivetrain/moduleInputs");
        odometryLock.unlock();
        

        double[] sampleTimeStamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimeStamps.length;
        for(int i = 0; i < sampleCount; i++){
            SwerveModulePosition[] wheelPosition = new SwerveModulePosition[4];
            // SwerveModuleState[] moduleStates = new SwerveModuleState[4];
            for(int j = 0; j < 4; j++){
                wheelPosition[j] = modules[j].getOdometryPositions()[i];
            }
            RobotState.getInstance().addOdometry(new RobotState.OdometryRecord(
                sampleTimeStamps[i], 
                wheelPosition, 
                Optional.ofNullable(gyroInputs.data.connected() ? gyroInputs.gyroYawPositions[i] : null)));

        }

        if(!velocityMode) currentSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
        
    }

    public void runVelocity(ChassisSpeeds speeds){
        velocityMode = true;
        
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, DrivetrainConstants.kLoopTime);
        
        SwerveModuleState[] unoptimizedStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(unoptimizedStates,DrivetrainConstants.kDriveTrainMaxSpeedMPS);
        
        // for(int i = 0; i < 4; i ++){
        //     Rotation2d wheelAngle = getModuleStates()[i].angle;
        //     unoptimizedStates[i].optimize(wheelAngle);
        //     unoptimizedStates[i].cosineScale(wheelAngle);
        // }
        currentSetpoint = setpointGenerator.generateSetpoint(currentSetpoint, speeds, DrivetrainConstants.kLoopTime);
        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();
        Logger.recordOutput("Drivetrain/SwerveStates/Speeds", speeds );
        // Logger.recordOutput("Drivetrain/SwerveStates/currentSetpoint", currentSetpoint );

        // Logger.recordOutput("Drivetrain/SwerveStates/generated setpoint",setpointGenerator.generateSetpoint(currentSetpoint, discreteSpeeds, DrivetrainConstants.kLoopTime));

        // SwerveModuleState[] unoptimizedStates = kinematics.toSwerveModuleStates(speeds);
        // Logger.recordOutput("Drivetrain/SwerveStates/discrete speeds", discreteSpeeds );

        Logger.recordOutput("Drivetrain/SwerveStates/Unoptimized Setpoint", unoptimizedStates );
        // Logger.recordOutput("Drivetrain/SWerveState/Unoptimized SEtpoint/angle", unoptimizedStates[0].angle.getDegrees());
        Logger.recordOutput("Drivetrain/SwerveStates/Optimized Setpoint", setpointStates);
        Logger.recordOutput("Drivetrain/SwerveChassisSpeeds/Optimized States", currentSetpoint.robotRelativeSpeeds());

        for(int i = 0; i<4; i++){
            // modules[i].runSetpoint(unoptimizedStates[i]);
            modules[i].runSetpoint(setpointStates[i]);
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
        for (int i = 0; i < 4; i++) {
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
        // for (int i = 0; i < modules.length; i++) {
        //     headings[i] = DrivetrainConstants.modulePositions[i].getAngle();
        //     modules[i].io.turnVolt(headings[i]);
        // }
        
        kinematics.resetHeadings(headings); //TODO: see if this one will work
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


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return driveMotorSysID.quasistatic(direction);
    }

    
    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return driveMotorSysID.dynamic(direction);
    }

}
