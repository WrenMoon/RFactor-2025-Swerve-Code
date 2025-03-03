package frc.robot.Commands.Transfer;

import frc.robot.Constants;
import frc.robot.Subsystems.loaderSubsystem;
import frc.robot.Subsystems.shooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class shootCmd extends Command {
  private final shooterSubsystem shooter;
  private final loaderSubsystem loader;
  double speed = 0;
  boolean autoShoot;
  boolean endLoop = false;
  double timer = 0;
  double maxSpeed;

  public shootCmd(shooterSubsystem shooter, loaderSubsystem loader, boolean autoShoot, double maxSpeed) {
    this.shooter = shooter;
    this.autoShoot = autoShoot;
    this.maxSpeed = maxSpeed;
    this.loader = loader;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    speed = 0;
    endLoop = false;
    timer = 0;
    SmartDashboard.putNumber("Shooter Speed", speed);
  }

  @Override
  public void execute() {
    speed = speed + (maxSpeed - speed) / 40;

    if (speed > maxSpeed - 0.05) {
      speed = maxSpeed;
      timer++;
    }

    if (timer > 22 && autoShoot) {
      loader.setMotors(1);
    }

    if (timer > 50 && autoShoot) {
      endLoop = true;
    }

    SmartDashboard.putNumber("Shooter Speed", speed);
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("ShootCmd", true);
    }

    shooter.setMotors(speed, speed);
  }

  @Override
  public void end(boolean interrupted) {
    speed = 0;
    SmartDashboard.putNumber("Shooter Speed", speed);
    shooter.setMotors(0, 0);
    loader.setMotors(0);
    if (Constants.smartEnable) {
      SmartDashboard.putBoolean("shootCmd", false);

    }
  }

  @Override
  public boolean isFinished() {
    return endLoop;
  }
}