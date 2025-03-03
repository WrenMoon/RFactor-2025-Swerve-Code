package frc.robot.Commands.CV;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.SwerveSubsystem;

public class NoteAlign extends Command {

    private final SwerveSubsystem swerve;
    private boolean endLoop = false;

    public NoteAlign(SwerveSubsystem swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        endLoop = false;
    }

    @Override
    public void execute() {
        
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
        
        if (llresults != null) {
            
            double TX = Math.toRadians(LimelightHelpers.getTX(""));
            
            if (Math.abs(TX) < 0.05) {
                endLoop = true;
            }
            
            swerve.drive(new Translation2d(0,0), -TX, false);


                if (Constants.smartEnable) {
                    SmartDashboard.putNumber("Note Align Correction", TX);
                    SmartDashboard.putBoolean("NoteAlign", true);
            }
        } else {
            endLoop = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        if (Constants.smartEnable) {
            SmartDashboard.putBoolean("NoteAlign", false);
        }

        swerve.drive(new Translation2d(0,0), 0,     false);

    }

    @Override
    public boolean isFinished() {
        return endLoop;
    }

}
