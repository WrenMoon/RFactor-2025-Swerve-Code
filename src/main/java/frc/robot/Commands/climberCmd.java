package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.climberSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class climberCmd extends Command {
  private final climberSubsystem climber;
  // private final DoubleSupplier speed;
  DoubleSupplier leftSpeed;
  DoubleSupplier rightSpeed;

  public climberCmd(climberSubsystem climber, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed){//DoubleSupplier speed) {
    this.climber = climber;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    // this.speed = speed;

    addRequirements(climber);
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

    climber.setMotor(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("climberCmd", true);
      SmartDashboard.putNumber("Left encoder", climber.getLeftEncoder());
      SmartDashboard.putNumber("Right encoder", climber.getRightEncoder());

    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMotor(0, 0);
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("climberCmd", false);

    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
