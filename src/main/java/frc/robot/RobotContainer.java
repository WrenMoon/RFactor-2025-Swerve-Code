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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
    //  Command driveSwerve = swerve.driveCommand(
    //   () -> MathUtil.applyDeadband((-WakakeController.getLeftY() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
    //   () -> MathUtil.applyDeadband((-WakakeController.getLeftX() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
    //   () -> MathUtil.applyDeadband((-WakakeController.getRightX()) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband), false, true); //Control heading with right joystick

    Command driveSwerve = swerve.driveCommand(
      () -> MathUtil.applyDeadband((-WakakeController.getLeftY() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-WakakeController.getLeftX() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
      () -> getHeadingAngleX(),
      () -> getHeadingAngleY()
    );
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
    SequentialCommandGroup L1 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, elevatorPoses.L1, Constants.Elevator.MaxSpeed, false), new armPosCmd(arm, armPoses.elevate, true)), new ParallelCommandGroup(new armPosCmd(arm, armPoses.L1, true), new elevatorPosCmd(elevator, elevatorPoses.L1, Constants.Elevator.MaxSpeed, true)));
    SequentialCommandGroup L2 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.L2, 0.6, true), new armPosCmd(arm, armPoses.elevate, true)));
    SequentialCommandGroup Lge1 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.algae, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.algae1, Constants.Elevator.MaxSpeed, true), new armPosCmd(arm, armPoses.algae, true)));
    SequentialCommandGroup Lge2 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, elevatorPoses.algae2, Constants.Elevator.MaxSpeed, false), new armPosCmd(arm, armPoses.elevate, true)), new ParallelCommandGroup(new armPosCmd(arm, armPoses.algae, true), new elevatorPosCmd(elevator, elevatorPoses.algae2, Constants.Elevator.MaxSpeed, true)));
    SequentialCommandGroup L3 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.L3, Constants.Elevator.MaxSpeed, true), new armPosCmd(arm, armPoses.elevate, true)));
    // SequentialCommandGroup L4 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, elevatorPoses.L4a, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.elevate, true)), new armPosCmd(arm, armPoses.L4, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, elevatorPoses.L4b, Constants.Elevator.MaxSpeed), new armPosCmd(arm, armPoses.L4, true)), new armPosCmd(arm, armPoses.L4, true));
    SequentialCommandGroup L4 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, elevatorPoses.L4b, Constants.Elevator.MaxSpeed, false), new armPosCmd(arm, armPoses.elevate, true)), new ParallelCommandGroup(new elevatorPosCmd(elevator, elevatorPoses.L4b, Constants.Elevator.MaxSpeed, true), new armPosCmd(arm, armPoses.L4, true)));
    SequentialCommandGroup L0 = new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, 2.5, Constants.Elevator.MaxSpeed, false), new armPosCmd(arm, armPoses.elevate, true)), new armPosCmd(arm, armPoses.zero, false));


    NamedCommands.registerCommand("L0", new SequentialCommandGroup(new armPosCmd(arm, armPoses.elevate, false), new ParallelDeadlineGroup(new elevatorPosCmd(elevator, 0, Constants.Elevator.MaxSpeed, false), new armPosCmd(arm, armPoses.elevate, true)), new armPosCmd(arm, armPoses.zero, false))); // registering L0 command group for auto
    NamedCommands.registerCommand("ElevatorMid", new elevatorPosCmd(elevator, elevatorPoses.L4a, Constants.Elevator.MaxSpeed, false));
    NamedCommands.registerCommand("Intake", new intakeCmd(intake, 0.3, false)); // registering intake command for auto
    NamedCommands.registerCommand("IntakeCoral", new intakeCmd(intake, 0.3, true)); // registering intake command for auto
    NamedCommands.registerCommand("L4", L4);
    NamedCommands.registerCommand("Right Align", new reefAlign(swerve, false, Constants.CV.rightAngle));
    NamedCommands.registerCommand("Left Align", new reefAlign(swerve, false, Constants.CV.leftAngle));
    NamedCommands.registerCommand("Slow Forward", swerve.driveCommand(
      () -> 0.2,
      () -> 0,
      () -> 0, false, false));
    

    WakakeController.touchpad().onTrue(Commands.runOnce(swerve::zeroGyro));
    WakakeController.touchpad().onTrue(Commands.runOnce(swerve::setHeadingCorrection));
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

    // RobotModeTriggers.autonomous().onTrue(swerve.setHeadingCorrection(false));
    // WakakeController.L3().onTrue(swerve.setHeadingCorrection(true));
    // RobotModeTriggers.teleop().onTrue(swerve.setHeadingCorrection(false));


    // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
      // WakakeController.cross().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef1, 0));
      // WakakeController.circle().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef6, 0));
      // WakakeController.triangle().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef5, 0));
      // WakakeController.povDown().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef2, 0));
      // WakakeController.povLeft().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef3, 0));
      // // WakakeController.povUp().whileTrue(swerve.driveToPose(Constants.PosesBlue.reef4, 0));
      // WakakeController.povRight().whileTrue(swerve.driveToPose(Constants.PosesBlue.stationLeft, 0));
      // WakakeController.square().whileTrue(swerve.driveToPose(Constants.PosesBlue.stationRight, 0));
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


  /**%
   * Command for the robot to run during autonomous
   * @return Autonomous Command of the robot for the command scheduler
   */
  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("Middle Auto");
  }

  public double getHeadingAngleX(){
    
    double headingX = 0;
    if(WakakeController.triangle().getAsBoolean()){
      headingX = 0;
    } else if (WakakeController.square().getAsBoolean()){
      headingX = -1;
    } else if (WakakeController.circle().getAsBoolean()){
      headingX = 1;
    } else if (WakakeController.cross().getAsBoolean()){
      headingX = 0;
    } else if (WakakeController.R3().getAsBoolean()){

      headingX = -WakakeController.getRightX();

    } else if (Math.abs(WakakeController.getRightX()) > 0.7 || Math.abs(WakakeController.getRightY()) > 0.7){
      
      double heading = JoystickHeading();

      if (Math.abs(heading - 0) < 30){

        headingX = 0;

      } else if  (Math.abs(heading - 60) < 30){

        headingX = -1;

      } else if  (Math.abs(heading - 120) < 30){

        headingX = -1;

      } else if  (Math.abs(heading - 180) < 30){

        headingX = 0;

      } else if  (Math.abs(heading - 240) < 30){

        headingX = 1;

      } else if  (Math.abs(heading - 300) < 30){

        headingX = 1;

      } else if  (Math.abs(heading - 360) < 30){

        headingX = 0;

      }
    }

    SmartDashboard.putNumber("HeadingX", headingX);
    return headingX;
  }

  public double getHeadingAngleY(){
    
    double headingY = 0;
    
    if(WakakeController.triangle().getAsBoolean()){
      headingY = 1;
    } else if (WakakeController.square().getAsBoolean()){
      headingY= 0.7;
    } else if (WakakeController.circle().getAsBoolean()){
      headingY = 0.7;
    } else if (WakakeController.cross().getAsBoolean()){
      headingY = -1;

    } else if (WakakeController.R3().getAsBoolean()){

      headingY = -WakakeController.getRightY();

    } else if (Math.abs(WakakeController.getRightX()) > 0.7 || Math.abs(WakakeController.getRightY()) > 0.7){

      double heading = JoystickHeading();

      if (Math.abs(heading - 0) <= 30){

        headingY = 1;

      } else if  (Math.abs(heading - 60) <= 30){

        headingY = 0.5773502691896258;

      } else if  (Math.abs(heading - 120) <= 30){

        headingY = -0.5773502691896258;

      } else if  (Math.abs(heading - 180) <= 30){

        headingY = -1;

      } else if  (Math.abs(heading - 240) <= 30){

        headingY = -0.5773502691896258;

      } else if  (Math.abs(heading - 300) <= 30){

        headingY = 0.5773502691896258;

      } else if  (Math.abs(heading - 360) <= 30){

        headingY = 1;

      }

    SmartDashboard.putNumber("HeadingY", headingY);

    }
    return headingY;
  }

  public double JoystickHeading(){
      double X = -WakakeController.getRightX();  // X component (leftward, so -X)
      double Y = -WakakeController.getRightY();  // Y component (upward)

      double resultX = -X;
      double resultY = Y;

      double angleRad = Math.atan2(resultX, resultY);

      double angleDeg = Math.toDegrees(angleRad);

      if (angleDeg < 0) {
          angleDeg += 360;
      }

      return angleDeg;
  }
} 