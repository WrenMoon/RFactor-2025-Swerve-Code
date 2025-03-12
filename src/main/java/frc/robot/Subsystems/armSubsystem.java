package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class armSubsystem extends SubsystemBase {

  final SparkMax motor = new SparkMax(Constants.Arm.ArmID, MotorType.kBrushless);
  final SparkMaxConfig config = new SparkMaxConfig();


  public armSubsystem() {

    config.idleMode(IdleMode.kBrake);
    config.inverted(Constants.Arm.armInvert);
    motor.configure(config, null, null);
  }

  @Override
  public void periodic() {
  }

  public void setMotor(double speed) {
    motor.set(speed);
  }

  public double getEncoder() {
    return motor.getEncoder().getPosition();
  }

  public double getDegrees() {
    double encoder = motor.getEncoder().getPosition();
    double zeroedEncoder = encoder - Constants.Arm.zeroEncoder;
    return zeroedEncoder * Constants.Arm.degreesPerEncoder;
  }

}
