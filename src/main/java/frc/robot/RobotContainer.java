// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.DriveCommand;
import frc.robot.Subsystems.Drivetrain.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DrivetrainConstants;
import frc.robot.Subsystems.Drivetrain.GyroIO;
import frc.robot.Subsystems.Drivetrain.GyroIOPigeon;
import frc.robot.Subsystems.Drivetrain.ModuleIO;
import frc.robot.Subsystems.Drivetrain.ModuleIOKraken;
import frc.robot.Subsystems.Drivetrain.ModuleIOSim;
import frc.robot.Subsystems.Limelight.LimelightIOReal;
import frc.robot.Subsystems.Limelight.VisionIO;
import frc.robot.Subsystems.Limelight.VisionSubsystem;
import frc.robot.Util.Constants;

public class RobotContainer {
  private Drivetrain drive;
  private VisionSubsystem vision;
  private DriveCommand driveCommand;
  private CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {

    
    // FL,FR,BL,BR
    switch (Constants.getRobotType()) {
      case Real->{
      System.out.println("Running in real mode");


        drive = new Drivetrain(
          new GyroIOPigeon(),
          new ModuleIOKraken(DrivetrainConstants.FLModuleConstants),
          new ModuleIOKraken(DrivetrainConstants.FRModuleConstants),
          new ModuleIOKraken(DrivetrainConstants.BLModuleConstants),
          new ModuleIOKraken(DrivetrainConstants.BRModuleConstants)
        );

        vision = new VisionSubsystem(new LimelightIOReal());
      }
      case Sim->{ 
        System.out.println("Running in simulation mode");
        drive = new Drivetrain(
          new GyroIO() {}, 
          new ModuleIOSim(),
          new ModuleIOSim(), 
          new ModuleIOSim(),
          new ModuleIOSim());    

        vision = new VisionSubsystem(new VisionIO(){});
      }
      case Replay->{
        System.out.println("Running in Replay mode");

        drive = new Drivetrain(
          new GyroIO() {}, 
          new ModuleIO() {},
          new ModuleIO() {}, 
          new ModuleIO() {},
          new ModuleIO() {});    

        vision = new VisionSubsystem(new VisionIO(){});
      }
    } 

    
    drive.setDefaultCommand(
      new DriveCommand(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> driverController.getRightX()
      )
    );
    

    configureBindings();
  }

  private void configureBindings() {
    driverController.a().whileTrue(drive.sysIdDynamic(Direction.kForward));
    driverController.b().whileTrue(drive.sysIdDynamic(Direction.kReverse));
    driverController.x().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    driverController.y().whileTrue(drive.sysIdQuasistatic(Direction.kReverse));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
