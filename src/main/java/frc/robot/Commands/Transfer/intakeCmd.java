package frc.robot.Commands.Transfer;

import frc.robot.Constants;
import frc.robot.Subsystems.intakeSubsystem;
import frc.robot.Subsystems.loaderSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class intakeCmd extends Command {
  private final intakeSubsystem Intake;
  private final loaderSubsystem loader;
  private final double intakeSpeed;
  private final double loaderSpeed;
  private boolean everTrue = false;
  private boolean endLoop = false;

  public intakeCmd(intakeSubsystem Intake, loaderSubsystem loader, double intakeSpeed, double loaderSpeed) {
    this.Intake = Intake;
    this.loader = loader;
    this.intakeSpeed = intakeSpeed;
    this.loaderSpeed = loaderSpeed;
    addRequirements(Intake);
  }

  @Override
  public void initialize() {
    endLoop = false;
    everTrue = false;
    SmartDashboard.putBoolean("Limit Switch", endLoop);

  }
        
  @Override
  public void execute() {

    Intake.setMotors(intakeSpeed, intakeSpeed);
    loader.setMotors(loaderSpeed);

    if (loader.limitSwitch()) {
      everTrue = true;
    } else if (everTrue && !loader.limitSwitch()) {
      endLoop = true;
    }

    SmartDashboard.putBoolean("Limit Switch", endLoop);
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("Intake", true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Intake.setMotors(0, 0);
    loader.setMotors(0);
    SmartDashboard.putBoolean("Limit Switch", endLoop);
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("Intake", false);
    }
  }

  @Override
  public boolean isFinished() {
    return endLoop;
  }
}