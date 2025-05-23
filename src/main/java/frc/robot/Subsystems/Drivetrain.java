package frc.robot.Subsystems;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.LoggedTunableNumber;


public class Drivetrain extends SubsystemBase{
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyro;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final Alert gyroDisconnectedAlert = new Alert("Gyro is disconnected", Alert.AlertType.kError);

    private static final LoggedTunableNumber coastWaitTime = new LoggedTunableNumber("Drivetrain/CoastWaitTime",0.5);
    private static final LoggedTunableNumber coastMPSThreshold = new LoggedTunableNumber("Drivetrain/CoastMPSThreshold",0.05);

    private final Timer lastMovementTimer = new Timer();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DrivetrainConstants.modulePositions);// FL,FR,BL,BR

    @AutoLogOutput private boolean velocityMode = false;
    @AutoLogOutput private boolean brakeMode = false;

    

}
