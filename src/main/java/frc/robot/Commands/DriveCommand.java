package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Subsystems.Drivetrain.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DrivetrainConstants;

public class DriveCommand extends Command{
    private final Drivetrain drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier thetaSupplier;
    private ChassisSpeeds speeds;

    public DriveCommand(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier){
        this.drivetrain =  drivetrain;
        this.thetaSupplier = thetaSupplier;
        this.ySupplier = ySupplier;
        this.xSupplier = xSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        double xValue =  MathUtil.applyDeadband(xSupplier.getAsDouble(),DrivetrainConstants.kDeadband);
        double yValue =  MathUtil.applyDeadband(ySupplier.getAsDouble(),DrivetrainConstants.kDeadband); 
        double thetaValue =  MathUtil.applyDeadband(thetaSupplier.getAsDouble(),DrivetrainConstants.kDeadband);

        xValue = Math.copySign(Math.pow(xValue, 2), xValue);
        yValue = Math.copySign(Math.pow(yValue, 2), yValue);
        thetaValue = Math.copySign(Math.pow(thetaValue, 2), thetaValue);

        xValue *= DrivetrainConstants.kDriveTrainMaxSpeedMPS;
        yValue *= DrivetrainConstants.kDriveTrainMaxSpeedMPS;
        thetaValue *= DrivetrainConstants.kDriveTrainMaxAngularVelocityRadsPerSec;

        Logger.recordOutput("/DriveCommand/xVal",xValue);
        Logger.recordOutput("/DriveCommand/yVal",yValue);
        Logger.recordOutput("/DriveCommand/thetaVal",thetaValue);

        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(xValue,yValue,thetaValue,  RobotState.getInstance().getRobotYaw());
        // speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xValue, yValue, thetaValue, RobotState.getInstance().getRobotYaw();
        Logger.recordOutput("/DriveCommand/Speeds",speeds);
        Logger.recordOutput("/DriveCommand/RobotYaw", RobotState.getInstance().getRobotYaw().getDegrees());
        drivetrain.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.runVelocity(new ChassisSpeeds());
    }

}
