package frc.robot.Subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
    private static final DCMotor driveMotor = DCMotor.getKrakenX60(1);
    private static final DCMotor turnMotor = DCMotor.getKrakenX60(1);

    private final DCMotorSim driveSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(driveMotor, 1, 1), driveMotor);
    private final DCMotorSim turnSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(turnMotor, 0, 0), turnMotor);

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;

    private PIDController driveController = new PIDController(0, 0, 0);
    private PIDController turnController = new PIDController(0, 0, 0);

    private double driveFFVolts = 0;
    private double driveAppliedVolts = 0;
    private double turnAppliedVolts = 0;

    public ModuleIOSim(){
        turnController.enableContinuousInput(-Math.PI,Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs input){
        if(driveClosedLoop){
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        }else{
            driveController.reset();
        }

        if(turnClosedLoop){
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        }else{
            turnController.reset();
        }


        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts,-12, 12));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts,-12, 12));
        driveSim.update(0.02);
        turnSim.update(0.02);


        input.data = new ModuleIOData(true, driveSim.getAngularPositionRad(), driveSim.getAngularVelocityRadPerSec(), driveAppliedVolts, Math.abs(driveSim.getCurrentDrawAmps()),0.0 , true, true, new Rotation2d(turnSim.getAngularPositionRad()), new Rotation2d(turnSim.getAngularPositionRad()), turnSim.getAngularVelocityRadPerSec(), turnAppliedVolts, Math.abs(turnSim.getCurrentDrawAmps()), 0.0);
        input.odometryDrivePositionsRad = new double[]{input.data.drivePositionRad()};
        input.odometryTurnPositions = new Rotation2d[]{input.data.turnPositionRad()};
    
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD){
        driveController.setPID(kP,kI,kD);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD){
        turnController.setPID(kP,kI,kD);
    }

    @Override
    public void voltageDrive(double voltage){
        driveClosedLoop = false;
        driveAppliedVolts = voltage;
    }

    @Override
    public void voltageTurn(double voltage){
        turnClosedLoop = false;
        turnAppliedVolts = voltage;
    }

    @Override
    public void velocityDrive(double velocity,double ff){
        driveClosedLoop = true;
        driveFFVolts = ff;
        driveController.setSetpoint(velocity);
    }

    @Override
    public void positionTurn(Rotation2d position){
        turnClosedLoop = true;
        turnController.setSetpoint(position.getRadians());
    }



}
