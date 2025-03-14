package frc.robot.Commands.pathfindAlign;

import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;



public class rightAlign extends Command {
  private final SwerveSubsystem swerve;

  /**
   * A Command to align the robot to the right position of the reef.
   * 
   * @param swerve the swerve subsystem to move
   */
  public rightAlign(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double heading = swerve.getHeading().getDegrees(); //get the heading of the robot
    boolean allianceBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue; //get the alliance colour

    if (allianceBlue){
      if (Math.abs(heading - 0) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef1, 0);
        SmartDashboard.putBoolean("DrivingToPose", true);
      } else if(Math.abs(heading - -60) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef3, 0);
      } else if(Math.abs(heading - -120) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef5, 0);
      } else if(Math.abs(heading - -180) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef7, 0);
      } else if(Math.abs(heading - 180) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef7, 0);
      } else if(Math.abs(heading - 120) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef9, 0);
      } else if(Math.abs(heading - 60) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef11, 0);
      }
    } else{
      if (Math.abs(heading - 0) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef1, 0);
      } else if(Math.abs(heading - -60) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef3, 0);
      } else if(Math.abs(heading - -120) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef5, 0);
      } else if(Math.abs(heading - -180) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef7, 0);
      } else if(Math.abs(heading - 180) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef7, 0);
      } else if(Math.abs(heading - 120) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef9, 0);
      } else if(Math.abs(heading - 60) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef11, 0);
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0,0), 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}