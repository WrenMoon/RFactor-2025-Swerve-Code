package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class intakeSubsystem extends SubsystemBase {

  final SparkMax motor = new SparkMax(Constants.Intake.intakeID, MotorType.kBrushless);
  final SparkMaxConfig config = new SparkMaxConfig();

  public intakeSubsystem() {

    config.idleMode(IdleMode.kCoast);
    config.inverted(Constants.Intake.Invert);
    motor.configure(config, null, null);

  }

  @Override
  public void periodic() {
  }

  public void setMotor(double speed) {
    motor.set(speed);
  }

  public Command setMotorSupplier(DoubleSupplier speed){
    return run(() -> {
      motor.set(speed.getAsDouble());
    });
  }

  public double getEncoder() {
    return motor.getEncoder().getPosition();
  }

}
