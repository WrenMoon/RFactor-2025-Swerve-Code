package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class climberSubsystem extends SubsystemBase {
    
    final SparkMax motorLeft = new SparkMax(Constants.Climber.climberLeft, MotorType.kBrushless);
    final SparkMax motorRight = new SparkMax(Constants.Climber.climberRight, MotorType.kBrushless);
    final SparkMaxConfig leftConfig = new SparkMaxConfig();
    final SparkMaxConfig rightConfig = new SparkMaxConfig();
    
    public climberSubsystem() {

    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(Constants.Climber.leftInvert);
    rightConfig.inverted(Constants.Climber.rightInvert);
    rightConfig.idleMode(IdleMode.kBrake);
    motorLeft.configure(leftConfig, null, null);
    motorRight.configure(rightConfig, null, null);
    }
  
  @Override
  public void periodic() {
  }

  public void setMotor(double speed_left, double speed_right) {
    motorLeft.set(speed_left);
    motorRight.set(speed_right);
  }

  public double getLeftEncoder(){
    return motorLeft.getEncoder().getPosition();
  }

  public double getRightEncoder(){
    return motorRight.getEncoder().getPosition();
  }
}
