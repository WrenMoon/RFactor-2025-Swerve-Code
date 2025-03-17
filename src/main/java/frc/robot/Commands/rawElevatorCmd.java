package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.elevatorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class rawElevatorCmd extends Command {
  private final elevatorSubsystem elevator;
  private final DoubleSupplier speed;

  /**
   * A command to move the elevator with at a certain power
   * 
   * @param elevator the elevator subsystem to move
   * @param speed    the speed to move the elevator at
   */
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

    if ((speed.getAsDouble() > 0 && elevator.getEncoder() < Constants.Elevator.poses.maxPose)
        || (speed.getAsDouble() < 0 && elevator.getEncoder() > Constants.Elevator.poses.minPose)) { // Check if the elevator is goind out of bounds
      elevator.setMotor(speed.getAsDouble() + Constants.Elevator.Kg); // Apply the speed to the motor with feedforward gravity correction
    } else{
      elevator.setMotor(Constants.Elevator.Kg); // Stop the motor from going out of bounds
    }

    // Smardashboard for debugging
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("rawElevatorCmd", true);
      SmartDashboard.putNumber("Elevator encoder", elevator.getEncoder());
      SmartDashboard.putNumber("Elevator speed", speed.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setMotor(0);  // Stop the motor when the command is stopped

    // Smartdashboard for debugging
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("rawElevatorCmd", false);

    }
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interupted by the command scheduler
  }
}
