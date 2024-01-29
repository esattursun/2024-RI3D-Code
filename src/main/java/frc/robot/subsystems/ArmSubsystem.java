// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  DutyCycleEncoder encoder = new DutyCycleEncoder(0);
  private final PWMVictorSPX arml;
  private final PWMVictorSPX armr;
  public ArmSubsystem() {
    arml = new PWMVictorSPX(8);
    armr = new PWMVictorSPX(7);
  }

  /*public void arm_to_pos(double pos){

      double Kp = -15.0;
      double error = encoder.getDistance();
      double power = Kp * error;
     move_arm(power);
  }
  */
public double GetEncodervalue(){
  double  encodervalue =encoder.getDistance();
  return encodervalue;
}

  public void move_arm(double power)
  {
       double max_power = 0.5;

      // Stop from making too much torque
      if (power > max_power) {
          power = max_power;
      } else if (power < -max_power) {
          power = -max_power;
      }

      arml.set(-power);
      armr.set(-power);  // -ve, as motors are pointing in opposite directions
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    GetEncodervalue();
  }
  
   public boolean isAtClose(){
    double error = GetEncodervalue() - 8.76;
    return (Math.abs(error) < 0.05);
  }

  public boolean isAtCloseShoot(){
    double error = GetEncodervalue() - 7.95;
    return (Math.abs(error) < 0.05);
  }

  public boolean amplifier(){
    double error = GetEncodervalue() - 4.875;
    return (Math.abs(error) < 0.05);
  }

  public boolean speaker(){
    double error = GetEncodervalue() - 2;
    return (Math.abs(error) < 0.05);
  }


}
