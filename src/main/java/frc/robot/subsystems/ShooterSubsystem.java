package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstans;



public class ShooterSubsystem extends SubsystemBase {
  private final PWMVictorSPX RightShooterMotor;
  private final PWMVictorSPX LeftShooterMotor;

  
  public ShooterSubsystem() {
    RightShooterMotor = new PWMVictorSPX(6);
    LeftShooterMotor = new PWMVictorSPX(5);
  }
 
  //! This function  set the power of the upper shooter motor
  public void setMotors(double speed) {
   RightShooterMotor.set(speed*ShooterConstans.ShootPower);
   LeftShooterMotor.set(-speed*ShooterConstans.ShootPower);
  }
  
  //! This funtion stop the shooter motors
  public void stopMotors() {
   RightShooterMotor.set(0);
   LeftShooterMotor.set(0);
  }

@Override
public void periodic() {
  RealTime();
  SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());

}
public double RealTime(){
  double x= Timer.getFPGATimestamp();
  return x;
  }
}
