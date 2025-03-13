package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.intakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class intakeCmd extends Command {
  private final intakeSubsystem Intake;
  private final double intakeSpeed;
  private boolean endLoop = false;
  private double loopCounter = 0;

  /**
   * A Command to move the intake at the input speed.
   * 
   * @param Intake the intake subsystem to move 
   * @param intakeSpeed the speed to move the intake at
   */
  public intakeCmd(intakeSubsystem Intake, double intakeSpeed) {
    this.Intake = Intake;
    this.intakeSpeed = intakeSpeed;
    addRequirements(Intake);
  }

  @Override
  public void initialize() {
    //Initialise endLoop and loopCounter
    endLoop = false;
    loopCounter = 0;
  }

  @Override
  public void execute() {

    Intake.setMotor(intakeSpeed); //apply the motor speed

    //Smartdashboard for debugging
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("Intake", true);
    }

    //start the delay counter if the switch is pressed
    //If the counter has already been started, no need to check the limit switch
    if (Intake.getLimitSwitch() || loopCounter > 0){
      loopCounter+=1; //increment the counter by 1
    }

    //if the required time delay(counter)is met, set endLoop to true
    if (loopCounter == Constants.Intake.waitCount){ 
      endLoop = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    Intake.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return endLoop; //runs until endLoop becomes true
  }
}