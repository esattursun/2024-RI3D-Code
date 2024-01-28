// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


public final class Constants {
  public static class JoystickConstants {
    //! Joysticks Ports
    public static final int m_gamepadPort = 0;
    
  }
  
  public static class DriveConstans {
    //! Drive Motors Id
    public static final int rightfrontPwmId = 3;
    public static final int rightbackPwmId = 2;
    public static final int leftfrontPwmId = 0;
    public static final int leftbackPwmId = 1;
    //! Control Axis
    public static final int xAxis=1;
    public static final int yAxis=4; 

  }

  public static class ShooterConstans{
    //! Shooter Motors Id
    public static final int ShooterUpPwmId=7;
    public static final int ShooterDownPwmId=8;

    //! Oto shoot Charge Power
    public static final double OtoShootChargePower=1;

    //! Oto shoot Powers
    public static final double OtoShootUpPower=1;
    public static final double OtoShootDownPower=1;

    //! Oto intake Powers
    public static final double OtoIntakeUpPower=-1;
    public static final double OtoIntakeDownPower=-1;

    //! For example,if your shooter shoot with -1, set the value to -1
    public static final double ShootPower=1;
  }
  
}