package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;


import java.io.File;

import javax.print.attribute.standard.PageRanges;

import frc.robot.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.studica.frc.AHRS;

import frc.robot.Subsystems.*;

public class RobotContainer {
  
  //Creating all the subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "JsonConstants"));
  private final armSubsystem arm = new armSubsystem();
  private final intakeSubsystem intake = new intakeSubsystem();
  private final elevatorSubsystem elevator = new elevatorSubsystem();
  final CommandPS5Controller WakakeController = new CommandPS5Controller(0);
  final CommandPS5Controller AmaryanController = new CommandPS5Controller(1);

  public RobotContainer() {

    configureBindings();

     //Default Swerve Command to drive with 3 axis on PS5 Controller
     Command driveSwerve = swerve.driveCommand(
      () -> MathUtil.applyDeadband((-WakakeController.getLeftY() * (((WakakeController.getR2Axis()+ 1)/2) + 2)/3) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.2), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-WakakeController.getLeftX() * (((WakakeController.getR2Axis()+ 1)/2) + 2)/3) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.2), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-WakakeController.getRightX() * (((WakakeController.getR2Axis()+ 1)/2) + 2)/3) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.2), Constants.ControllerDeadband), false, true); //Control heading with right joystick

    //Default Elevator Command to move the elevator with one axis
    Command elevate = new rawElevatorCmd(elevator,
        () -> -MathUtil.applyDeadband(AmaryanController.getRawAxis(5), Constants.ControllerDeadband));

    //Default Arm Command to move the arm with one axis and gravity control
    Command moveArm = new rawArmCmd(arm,
        () -> -MathUtil.applyDeadband(AmaryanController.getRawAxis(1) * 0.35, Constants.ControllerDeadband));

    AHRS navx = (AHRS) swerve.getSwerveDriveConfiguration().imu.getIMU();


    //Default Intake Command ot move the intake with gravity correction as per degree angle of the arm
    Command holdCoral = intake.setMotorSupplier(
        () -> Math.cos(Math.toRadians(arm.getDegrees() - Constants.Intake.angleOffset)) * -Constants.Intake.Kg);

    //Applying all the default commands
    elevator.setDefaultCommand(elevate);
    swerve.setDefaultCommand(driveSwerve);
    arm.setDefaultCommand(moveArm);
    intake.setDefaultCommand(holdCoral);
  }

  /**
   * Configuring all the button bindings for all the controllers
   */
  private void configureBindings() {

    Constants.Arm.poses armPoses = new Constants.Arm.poses();
    Constants.Elevator.poses elevatorPoses = new Constants.Elevator.poses();

    //creating command groups for depositing corals
    SequentialCommandGroup L1 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.L1, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.L1, false));
    SequentialCommandGroup L2 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.L2, Constants.Elevator.MaxSpeed));
    SequentialCommandGroup Lge1 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.algae1, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.algae, false));
    SequentialCommandGroup Lge2 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.algae2, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.algae, false));
    SequentialCommandGroup L3 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.L3, Constants.Elevator.MaxSpeed));
    SequentialCommandGroup L4 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new elevatorPosCmd(elevator, elevatorPoses.L4, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.L4, false));
    SequentialCommandGroup L0 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.algae, false),new elevatorPosCmd(elevator, 0,Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.zero, false));
    SequentialCommandGroup Lauto4 = new SequentialCommandGroup(new ParallelDeadlineGroup(new armPosCmd(arm, armPoses.elevate, false), new intakeCmd(intake, -0.1)), new elevatorPosCmd(elevator, elevatorPoses.L4, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.elevate, false));


    NamedCommands.registerCommand("L0", L0); // registering L0 command group for auto
    NamedCommands.registerCommand("Intake", new intakeCmd(intake, 0.3)); // registering intake command for auto
    NamedCommands.registerCommand("L4", Lauto4);

    WakakeController.triangle().onTrue(Commands.runOnce(swerve::zeroGyro));
    WakakeController.povRight().whileTrue(swerve.driveToPose(getTargetPose(false, swerve.getHeading().getDegrees()), 0));
    WakakeController.povLeft().whileTrue(swerve.driveToPose(getTargetPose(true, swerve.getHeading().getDegrees()), 0));
    WakakeController.povUp().whileTrue(new reefAlign(swerve, false, Constants.CV.middleAngle));
    WakakeController.R1().whileTrue(new reefAlign(swerve, false, Constants.CV.rightAngle));
    WakakeController.L1().whileTrue(new reefAlign(swerve, false, Constants.CV.leftAngle));

    // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
    //   WakakeController.povUp().whileTrue(swerve.driveToPose(Constants.stationPoses.Blue, 0));
    // } else{
    //   WakakeController.povUp().whileTrue(swerve.driveToPose(Constants.stationPoses.Red, 0));

    // }

    AmaryanController.R2().whileTrue(new intakeCmd(intake, 0.3));
    AmaryanController.L2().whileTrue(new intakeCmd(intake, 0.6));
    AmaryanController.R1().whileTrue(new intakeCmd(intake, -0.3));
    AmaryanController.L1().whileTrue(new intakeCmd(intake, -0.1));
    AmaryanController.povUp().onTrue(Lge2);
    AmaryanController.povLeft().onTrue(Lge1);
    AmaryanController.cross().onTrue(L2);
    AmaryanController.circle().onTrue(L3);
    AmaryanController.triangle().onTrue(L4);
    AmaryanController.square().onTrue(L1);
    AmaryanController.povDown().onTrue(L0);
    AmaryanController.touchpad().whileTrue(new SequentialCommandGroup(new rawArmCmd(arm, () -> 0), new rawElevatorCmd(elevator, () -> 0)));
  }

  /**
   * Command for the robot to run during autonomous
   * @return Autonomous Command of the robot for the command scheduler
   */
  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("New Auto");
  }

  /**
   * A small function to return the reef pose for alignment
   * 
   * @param left true if the alignment pose is on the left, false if on the right
   * @param heading The current heading of the robot
   * @return the Pose2d for alignment
   */
  public Pose2d getTargetPose(boolean left, double heading){
    
    boolean allianceBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue; //get the alliance colour
    Pose2d pose = new Pose2d();

    if(left){
    if (allianceBlue){
      if (Math.abs(heading - 0) < 30 ){
        pose = Constants.reefPosesBlue.reef1;
        SmartDashboard.putBoolean("DrivingToPose", true);
      } else if(Math.abs(heading - -60) < 30 ){
        pose = Constants.reefPosesBlue.reef3;
      } else if(Math.abs(heading - -120) < 30 ){
        pose = Constants.reefPosesBlue.reef5;
      } else if(Math.abs(heading - -180) < 30 ){
        pose = Constants.reefPosesBlue.reef7;
      } else if(Math.abs(heading - 180) < 30 ){
        pose = Constants.reefPosesBlue.reef7;
      } else if(Math.abs(heading - 120) < 30 ){
        pose = Constants.reefPosesBlue.reef9;
      } else if(Math.abs(heading - 60) < 30 ){
        pose = Constants.reefPosesBlue.reef11;
      }
    } else{
      if (Math.abs(heading - 0) < 30 ){
        pose = Constants.reefPosesRed.reef1;
      } else if(Math.abs(heading - -60) < 30 ){
        pose = Constants.reefPosesRed.reef3;
      } else if(Math.abs(heading - -120) < 30 ){
        pose = Constants.reefPosesRed.reef5;
      } else if(Math.abs(heading - -180) < 30 ){
        pose = Constants.reefPosesRed.reef7;
      } else if(Math.abs(heading - 180) < 30 ){
        pose = Constants.reefPosesRed.reef7;
      } else if(Math.abs(heading - 120) < 30 ){
        pose = Constants.reefPosesRed.reef9;
      } else if(Math.abs(heading - 60) < 30 ){
        pose = Constants.reefPosesRed.reef11;
      }
    }
  } else{
    if (allianceBlue){
      if (Math.abs(heading - 0) < 30 ){
        pose = Constants.reefPosesBlue.reef2;
        SmartDashboard.putBoolean("DrivingToPose", true);
      } else if(Math.abs(heading - -60) < 30 ){
        pose = Constants.reefPosesBlue.reef4;
      } else if(Math.abs(heading - -120) < 30 ){
        pose = Constants.reefPosesBlue.reef6;
      } else if(Math.abs(heading - -180) < 30 ){
        pose = Constants.reefPosesBlue.reef8;
      } else if(Math.abs(heading - 180) < 30 ){
        pose = Constants.reefPosesBlue.reef8;
      } else if(Math.abs(heading - 120) < 30 ){
        pose = Constants.reefPosesBlue.reef10;
      } else if(Math.abs(heading - 60) < 30 ){
        pose = Constants.reefPosesBlue.reef12;
      }
    } else{
      if (Math.abs(heading - 0) < 30 ){
        pose = Constants.reefPosesRed.reef2;
      } else if(Math.abs(heading - -60) < 30 ){
        pose = Constants.reefPosesRed.reef4;
      } else if(Math.abs(heading - -120) < 30 ){
        pose = Constants.reefPosesRed.reef6;
      } else if(Math.abs(heading - -180) < 30 ){
        pose = Constants.reefPosesRed.reef8;
      } else if(Math.abs(heading - 180) < 30 ){
        pose = Constants.reefPosesRed.reef8;
      } else if(Math.abs(heading - 120) < 30 ){
        pose = Constants.reefPosesRed.reef10;
      } else if(Math.abs(heading - 60) < 30 ){
        pose = Constants.reefPosesRed.reef12;
      }
    }
  }
    return pose;
  }
}

