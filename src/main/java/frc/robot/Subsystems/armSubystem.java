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

  final SparkMax motor = new SparkMax(Constants.Arm.ArmID, MotorType.kBrushless);
  final SparkMaxConfig config = new SparkMaxConfig();

  // final PIDController PIDarm = new PIDController(Constants.Arm.kp, 0, Constants.Arm.kd);

  public armSubystem() {

    config.idleMode(IdleMode.kBrake);
    config.inverted(Constants.Arm.armInvert);
    motor.configure(config, null, null);
  }

  @Override
  public void periodic() {
  }

  // public void setMotors(double Speed) {
  //   if (Speed >= 0) {
  //     if (getAbsoluteEncoder() <= Constants.Arm.MaxPose) {
  //       leftMotor.set(Speed);
  //       rightMotor.set(Speed);
  //     } else {
  //       leftMotor.set(0);
  //       rightMotor.set(0);
  //     }

  //   } else if (Speed <= 0) {
  //     if (Constants.Arm.MinPose <= getAbsoluteEncoder()) {
  //       leftMotor.set(Speed);
  //       rightMotor.set(Speed);
  //     } else {
  //       leftMotor.set(0);
  //       rightMotor.set(0);
  //     }
  //   } else {
  //     leftMotor.set(0);
  //     rightMotor.set(0);
  //   }
  // }

  public void setRawMotors(double speed){
    motor.set(speed);
  }

}
