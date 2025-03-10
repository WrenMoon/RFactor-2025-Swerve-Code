package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class elevatorSubsystem extends SubsystemBase {
    
    final SparkMax motorLeft = new SparkMax(Constants.Elevator.elevatorLeft, MotorType.kBrushless);
    final SparkMax motorRight = new SparkMax(Constants.Elevator.elevatorRight, MotorType.kBrushless);
    final SparkMaxConfig leftConfig = new SparkMaxConfig();
    final SparkMaxConfig rightConfig = new SparkMaxConfig();
    
    public elevatorSubsystem() {

    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(Constants.Elevator.leftInvert);
    rightConfig.inverted(Constants.Elevator.rightInvert);
    rightConfig.idleMode(IdleMode.kBrake);
    motorLeft.configure(leftConfig, null, null);
    motorRight.configure(rightConfig, null, null);
    
    // leftConfig.follow(motorRight, (Constants.Elevator.leftInvert == Constants.Elevator.rightInvert) ? false : true);
    }
  
  @Override
  public void periodic() {
  }

  public void setMotor(double speed) {
    motorRight.set(speed);
    motorLeft.set(speed);
  }

  public double getLeftEncoder(){
    return motorLeft.getEncoder().getPosition();
  }

  public double getRightEncoder(){
    return motorRight.getEncoder().getPosition();
  }
}
