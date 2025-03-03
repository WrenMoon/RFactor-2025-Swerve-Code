package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class loaderSubsystem extends SubsystemBase {
    
    final SparkMax loaderMotor = new SparkMax(Constants.Loader.loaderID, MotorType.kBrushless);
    final DigitalInput limitSwitch = new DigitalInput(Constants.Loader.limitSwitch);
        final SparkMaxConfig loaderConfig = new SparkMaxConfig();


  public loaderSubsystem() {
    
    loaderConfig.inverted(Constants.Loader.motorInvert);
    loaderConfig.idleMode(IdleMode.kCoast);
    loaderMotor.configure(loaderConfig, null, null);
  }


  
  @Override
  public void periodic() {
  }

  public void setMotors(double speed){
    loaderMotor.set(speed * Constants.Shooter.loaderSpeed);
  }

  public boolean limitSwitch(){
    return !limitSwitch.get();
  }
}
