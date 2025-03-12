package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.elevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class elevatorPosCmd extends Command {
  private final elevatorSubsystem elevator;
  private final double targetPose;
  private final PIDController PIDelevator;
  private boolean endLoop = false;

  public elevatorPosCmd(elevatorSubsystem elevator, double targetPose) {
    this.elevator = elevator;
    this.targetPose = targetPose;
    PIDelevator = new PIDController(Constants.Elevator.kp, 0, Constants.Elevator.kd);

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    endLoop = false;
    PIDelevator.setSetpoint(targetPose);
  }

  @Override
  public void execute() {

    double speed = PIDelevator.calculate(elevator.getEncoder());
    elevator.setMotor(speed);

    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("elevatorPosCmd", true);
      SmartDashboard.putNumber("Elevator encoder", elevator.getEncoder());
      SmartDashboard.putNumber("Elevator Target Pose", targetPose);
      SmartDashboard.putNumber("ELevator speed", speed);
    }

    if (speed < 0.07) {
      endLoop = true;
    }

  }

  @Override
  public void end(boolean interrupted) {
    elevator.setMotor(0);
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("elevatorPosCmd", false);
    }
  }

  @Override
  public boolean isFinished() {
    return endLoop;
  }
}
