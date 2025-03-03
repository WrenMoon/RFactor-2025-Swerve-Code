package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants;
public class armSubystem extends SubsystemBase {

  final SparkMax leftMotor = new SparkMax(Constants.Arm.armLeftID, MotorType.kBrushless);
  final SparkMax rightMotor = new SparkMax(Constants.Arm.armRightID, MotorType.kBrushless);
  final SparkMaxConfig leftConfig = new SparkMaxConfig();
  final SparkMaxConfig rightConfig = new SparkMaxConfig();
  final DutyCycleEncoder ThroughBoreEncoder = new DutyCycleEncoder(0);

  final PIDController PIDarm = new PIDController(Constants.Arm.kp, 0, Constants.Arm.kd);

  public armSubystem() {

    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(Constants.Arm.leftInvert);
    rightConfig.inverted(Constants.Arm.rightInvert);
    rightConfig.idleMode(IdleMode.kBrake);
    leftMotor.configure(leftConfig, null, null);
    rightMotor.configure(rightConfig, null, null);
  }

  @Override
  public void periodic() {
  }

  public void setMotors(double Speed) {
    if (Speed >= 0) {
      if (getAbsoluteEncoder() <= Constants.Arm.MaxPose) {
        leftMotor.set(Speed);
        rightMotor.set(Speed);
      } else {
        leftMotor.set(0);
        rightMotor.set(0);
      }

    } else if (Speed <= 0) {
      if (Constants.Arm.MinPose <= getAbsoluteEncoder()) {
        leftMotor.set(Speed);
        rightMotor.set(Speed);
      } else {
        leftMotor.set(0);
        rightMotor.set(0);
      }
    } else {
      leftMotor.set(0);
      rightMotor.set(0);
    }
  }

  public void setRawMotors(double leftSpeed, double rightSpeed){
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  public double getAbsoluteEncoder() {
    return (ThroughBoreEncoder.get() - Constants.Arm.absoluteOffset) * Constants.Arm.absoluteInvert;
  }

  public double getRawEncoder() {
    return ThroughBoreEncoder.get();
  }

}
