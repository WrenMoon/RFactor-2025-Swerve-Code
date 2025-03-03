package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class intakeSubsystem extends SubsystemBase {
    
    final SparkMax leftMotor = new SparkMax(Constants.Intake.intakeLeftID, MotorType.kBrushless);
    final SparkMax rightMotor = new SparkMax(Constants.Intake.intakeRightID, MotorType.kBrushless);
    final SparkMaxConfig leftConfig = new SparkMaxConfig();
   final SparkMaxConfig rightConfig = new SparkMaxConfig();
    
    public intakeSubsystem() {
      leftConfig.idleMode(IdleMode.kCoast);
    leftConfig.inverted(Constants.Intake.leftInvert);
    rightConfig.inverted(Constants.Intake.rightInvert);
    rightConfig.idleMode(IdleMode.kCoast);
    leftMotor.configure(leftConfig, null, null);
    rightMotor.configure(rightConfig, null, null);
    
  }
  
  @Override
  public void periodic() {
  }

  public void setMotors(double speedLeft, double speedRight) {
    leftMotor.set(speedLeft);
    rightMotor.set(speedRight);
  }

  public double getLeftEncoder(){
    return leftMotor.getEncoder().getPosition();
  }

  public double getRightEncoder(){
    return rightMotor.getEncoder().getPosition();
  }

}
