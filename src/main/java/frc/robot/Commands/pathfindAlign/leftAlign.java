package frc.robot.Commands.pathfindAlign;

import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class leftAlign extends Command {
  private final SwerveSubsystem swerve;

  /**
   *A Command to align the robot to the left position of the reef.
   * 
   * @param swerve the swerve subsystem to move
   */
  public leftAlign(SwerveSubsystem swerve) {
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
        swerve.driveToPose(Constants.reefPosesBlue.reef2, 0);
      } else if(Math.abs(heading - -60) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef4, 0).schedule();
      } else if(Math.abs(heading - -120) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef6, 0).schedule();
      } else if(Math.abs(heading - -180) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef8, 0).schedule();
      } else if(Math.abs(heading - 180) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef8, 0).schedule();
      } else if(Math.abs(heading - 120) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef10, 0).schedule();
      } else if(Math.abs(heading - 60) < 30 ){
        swerve.driveToPose(Constants.reefPosesBlue.reef12, 0).schedule();
      }
    } else{
      if (Math.abs(heading - 0) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef2, 0);
      } else if(Math.abs(heading - -60) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef4, 0);
      } else if(Math.abs(heading - -120) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef6, 0);
      } else if(Math.abs(heading - -180) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef8, 0);
      } else if(Math.abs(heading - 180) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef8, 0);
      } else if(Math.abs(heading - 120) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef10, 0);
      } else if(Math.abs(heading - 60) < 30 ){
        swerve.driveToPose(Constants.reefPosesRed.reef12, 0);
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