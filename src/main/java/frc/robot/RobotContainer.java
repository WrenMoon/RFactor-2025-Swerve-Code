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
      () -> MathUtil.applyDeadband((-WakakeController.getLeftY() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-WakakeController.getLeftX() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-WakakeController.getRightX()) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband), false, true); //Control heading with right joystick

    //Default Elevator Command to move the elevator with one axis
    Command elevate = new rawElevatorCmd(elevator,
        () -> -MathUtil.applyDeadband(AmaryanController.getRawAxis(5), Constants.ControllerDeadband));

    //Default Arm Command to move the arm with one axis and gravity control
    Command moveArm = new rawArmCmd(arm,
        () -> -MathUtil.applyDeadband(AmaryanController.getRawAxis(1) * 0.35, Constants.ControllerDeadband));

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
    SequentialCommandGroup L1 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, elevatorPoses.L1, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.elevate, true)), new armPosCmd(arm, armPoses.L1, true));
    SequentialCommandGroup L2 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.L2, 0.1), new armPosCmd(arm, armPoses.elevate, true)));
    SequentialCommandGroup Lge1 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.algae, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.algae1, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.algae, true)));
    SequentialCommandGroup Lge2 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, elevatorPoses.algae2, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.elevate, true)), new armPosCmd(arm, armPoses.algae, true));
    SequentialCommandGroup L3 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.L3, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.elevate, true)));
    SequentialCommandGroup L4 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, elevatorPoses.L4a, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.elevate, true)), new armPosCmd(arm, armPoses.L4, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, elevatorPoses.L4b, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.L4, true)), new armPosCmd(arm, armPoses.L4, true));
    SequentialCommandGroup L0 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.algae, false),new elevatorPosCmd(elevator, 5,Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.zero, false));


    NamedCommands.registerCommand("L0", L0); // registering L0 command group for auto
    NamedCommands.registerCommand("ElevatorMid", new elevatorPosCmd(elevator, elevatorPoses.L4a, Constants.Elevator.MaxSpeed));
    NamedCommands.registerCommand("Intake", new intakeCmd(intake, 0.3, false)); // registering intake command for auto
    NamedCommands.registerCommand("L4", L4);
    NamedCommands.registerCommand("Right Align", new reefAlign(swerve, false, Constants.CV.rightAngle));
    NamedCommands.registerCommand("Left Align", new reefAlign(swerve, false, Constants.CV.leftAngle));
    NamedCommands.registerCommand("Slow Forward", swerve.driveCommand(
      () -> 0.3,
      () -> 0,
      () -> 0, false, false));
    

    WakakeController.touchpad().onTrue(Commands.runOnce(swerve::zeroGyro));
    WakakeController.povUp().whileTrue(new reefAlign(swerve, false, Constants.CV.middleAngle));
    WakakeController.R1().whileTrue(new reefAlign(swerve, false, Constants.CV.rightAngle));
    WakakeController.L1().whileTrue(new reefAlign(swerve, false, Constants.CV.leftAngle));

    AmaryanController.R2().whileTrue(new intakeCmd(intake, 0.3, true));
    AmaryanController.L2().whileTrue(new intakeCmd(intake, 0.6, true));
    AmaryanController.R1().whileTrue(new intakeCmd(intake, -0.3, true));
    AmaryanController.L1().whileTrue(new intakeCmd(intake, -0.1, true));
    AmaryanController.povUp().onTrue(Lge2);
    AmaryanController.povLeft().onTrue(Lge1);
    AmaryanController.cross().onTrue(L2);
    AmaryanController.circle().onTrue(L3);
    AmaryanController.triangle().onTrue(L4);
    AmaryanController.square().onTrue(L1);
    AmaryanController.povDown().onTrue(L0);
    AmaryanController.touchpad().whileTrue(new SequentialCommandGroup(new rawArmCmd(arm, () -> 0), new rawElevatorCmd(elevator, () -> 0)));
    
    // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
      WakakeController.cross().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef1, 0));
      WakakeController.circle().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef6, 0));
      WakakeController.triangle().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef5, 0));
      WakakeController.povDown().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef2, 0));
      WakakeController.povLeft().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef3, 0));
      // WakakeController.povUp().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef4, 0));
      WakakeController.povRight().whileTrue(swerve.driveToPose(Constants.PosesBlue.stationLeft, 0));
      WakakeController.square().whileTrue(swerve.driveToPose(Constants.PosesBlue.stationRight, 0));
    // } else{ 
    //   WakakeController.cross().whileTrue(swerve.driveToPose(Constants.PosesRed.reef1, 0));
    //   WakakeController.circle().whileTrue(swerve.driveToPose(Constants.PosesRed.reef6, 0));
    //   WakakeController.triangle().whileTrue(swerve.driveToPose(Constants.PosesRed.reef5, 0));
    //   WakakeController.povDown().whileTrue(swerve.driveToPose(Constants.PosesRed.reef2, 0));
    //   WakakeController.povLeft().whileTrue(swerve.driveToPose(Constants.PosesRed.reef3, 0));
    //   // WakakeController.povUp().whileTrue(swerve.driveToPose(Constants.PosesRed.reef4, 0));
    //   WakakeController.povRight().whileTrue(swerve.driveToPose(Constants.PosesRed.stationLeft, 0));
    //   WakakeController.square().whileTrue(swerve.driveToPose(Constants.PosesRed.stationRight, 0));
    // // }

  }


  /**
   * Command for the robot to run during autonomous
   * @return Autonomous Command of the robot for the command scheduler
   */
  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("Low Auto");
  }
}