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
  private final boolean holdPID;
  private boolean endLoop = false;
  private double maxSpeed;
  
  /**
   * A command to move the elevator to an encoder setpoint using PID Feedback
   * 
   * @param elevator The elevator subsystem to move
   * @param targetPose the target setpoint to move to in encoder ticks
   * @param holdPID whether or not to hold the PID loop after acceptable error
   *                   is achieved
   */
  public elevatorPosCmd(elevatorSubsystem elevator, double targetPose, double maxSpeed, boolean holdPID){
    this.elevator = elevator;
    this.targetPose = targetPose;
    this.holdPID = holdPID;
    PIDelevator = new PIDController(Constants.Elevator.kp, 0, Constants.Elevator.kd);
    this.maxSpeed = maxSpeed;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    endLoop = false;
    PIDelevator.setSetpoint(targetPose); // PID setpoint
  }
  

  @Override
  public void execute() {

    double speed = PIDelevator.calculate(elevator.getEncoder()); //PID Correction value
    speed = speed + ((speed > 0)? Constants.Elevator.MinSpeed : -Constants.Elevator.MinSpeed) + Constants.Elevator.Kg; //Add the minimum speed to counteract friction, matching the sign of the PID speed and add the gravity correction speed, always positive
    speed = Math.min(Math.max(speed, -maxSpeed), maxSpeed); //Applying Speed Limits


    elevator.setMotor(speed); //applies the speed to the motor

    //Smartdashboard for debugging
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("elevatorPosCmd", true);
      SmartDashboard.putNumber("Elevator encoder", elevator.getEncoder());
      SmartDashboard.putNumber("Elevator Target Pose", targetPose);
      SmartDashboard.putNumber("ELevator PID speed", speed);
      SmartDashboard.putNumber("Elevator Ks", ((speed > 0)? Constants.Elevator.MinSpeed : -Constants.Elevator.MinSpeed));
    }

    if (Math.abs(elevator.getEncoder() - targetPose) < 1 && !holdPID) { //endcase when setpoint achieved.
      endLoop = true;
    }

  }

  @Override
  public void end(boolean interrupted) {
    elevator.setMotor(0);
    
    //Smartdashboard for debugging
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("elevatorPosCmd", false);
    }
  }

  @Override
  public boolean isFinished() {
    return endLoop;
  }
}
