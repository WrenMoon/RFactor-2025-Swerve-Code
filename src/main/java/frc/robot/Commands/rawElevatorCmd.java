package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.elevatorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class rawElevatorCmd extends Command {
  private final elevatorSubsystem elevator;
  private final DoubleSupplier speed;

  public rawElevatorCmd(elevatorSubsystem elevator, DoubleSupplier speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    elevator.setMotor(speed.getAsDouble());
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("rawElevatorCmd", true);
      SmartDashboard.putNumber("Elevator encoder", elevator.getEncoder());
      SmartDashboard.putNumber("Elevator speed", speed.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setMotor(0);
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("rawElevatorCmd", false);

    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
