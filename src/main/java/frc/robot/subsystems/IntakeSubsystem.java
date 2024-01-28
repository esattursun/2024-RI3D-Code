// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
   private final PWMVictorSPX IntakeMotor;


  public IntakeSubsystem() {
    IntakeMotor = new PWMVictorSPX(0);


  }
public void SetIntakeMotor(double Intakespeed){
  IntakeMotor.set(Intakespeed);
}
public double RealTime(){
  double x= Timer.getFPGATimestamp();
  return x;
  }
  public void stopMotor(){
    IntakeMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
