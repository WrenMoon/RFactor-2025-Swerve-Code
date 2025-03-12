package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.intakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//A Command to move the intake at the input speed.

public class intakeCmd extends Command {
  private final intakeSubsystem Intake;
  private final double intakeSpeed;

  public intakeCmd(intakeSubsystem Intake, double intakeSpeed) {
    this.Intake = Intake;
    this.intakeSpeed = intakeSpeed;
    addRequirements(Intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    Intake.setMotor(intakeSpeed); //apply the motor speed

    //Smartdashboard for debugging
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("Intake", true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Intake.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return Intake.getLimitSwitch(); //runs until the limit switch is pressed
  }
}