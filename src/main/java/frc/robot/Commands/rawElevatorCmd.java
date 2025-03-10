package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.elevatorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class rawElevatorCmd extends Command {
  private final elevatorSubsystem elevator;
  private final DoubleSupplier leftSpeed;
  private final DoubleSupplier rightSpeed;

  public rawElevatorCmd(elevatorSubsystem elevator, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    this.elevator = elevator;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // if (speed.getAsDouble() > 0) {
    //   if (climber.getLeftEncoder() < Constants.Climber.maxPose) {
    //     leftSpeed = speed.getAsDouble();
    //   } else {
    //     leftSpeed = 0;
    //   }
    //   if (climber.getRightEncoder() < Constants.Climber.maxPose) {
    //     rightSpeed = speed.getAsDouble();
    //   } else {
    //     rightSpeed = 0;
    //   }
    // } else if (speed.getAsDouble() < 0) {
    //   if (climber.getLeftEncoder() > Constants.Climber.minPose) {
    //     leftSpeed = speed.getAsDouble();
    //   } else {
    //     leftSpeed = 0;
    //   }
    //   if (climber.getRightEncoder() > Constants.Climber.minPose) {
    //     rightSpeed = speed.getAsDouble();
    //   } else {
    //     rightSpeed = 0;
    //   }
    // } else {
    //   leftSpeed = 0;
    //   rightSpeed = 0;
    // }

    elevator.setMotor(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("elevatorCmd", true);
      SmartDashboard.putNumber("Elevator Left encoder", elevator.getLeftEncoder());
      SmartDashboard.putNumber("Elevator Right encoder", elevator.getRightEncoder());

    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setMotor(0, 0);
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("elevatorCmd", false);

    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
