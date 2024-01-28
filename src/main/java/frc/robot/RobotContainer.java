// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.DriveConstans;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.AutoArm;
import frc.robot.commands.DriveJoystickCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.AutoArm.armPosition;
import frc.robot.commands.Autonomous.AutoCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
 //subsystems 
 private final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
 private final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
 private final static DriveSubsystem driveSubsystem = new DriveSubsystem();
 private final static ArmSubsystem armSubsystem = new ArmSubsystem();

 public final ShuffleboardTab Auto_Event_Map = Shuffleboard.getTab("Auto");
 //Joysticks
 private final Joystick m_gamepad = new Joystick (JoystickConstants.m_gamepadPort);

  public RobotContainer() {

   driveSubsystem.setDefaultCommand(new DriveJoystickCommand(
    driveSubsystem,
    ()-> -m_gamepad.getRawAxis(DriveConstans.xAxis)*0.5,
    ()-> m_gamepad.getRawAxis(DriveConstans.yAxis)*0.5
   ));
    
    configureButtonBindings();
  }

  
  
  private void configureButtonBindings() {
/*//                     Shooter manual
 * // Shooter up
 * new JoystickButton(m_gamepad, 9).whileTrue(new ShooterManualCommand(shooterSubsystem, 1, 0));//shoot
 * new JoystickButton(m_gamepad,8).whileTrue(new ShooterManualCommand(shooterSubsystem, -1,0));//intake
 * // Shooter Down
 * new JoystickButton(m_gamepad,7).whileTrue(new ShooterManualCommand(shooterSubsystem, 0,1));//shoot
 * new JoystickButton(m_gamepad,6).whileTrue(new ShooterManualCommand(shooterSubsystem, 0,-1));//intake
*/
  
  
  //! Shooter autonomous
  new JoystickButton(m_gamepad, 1).toggleOnTrue(new ShootCommand(
    shooterSubsystem,intakeSubsystem,
     3,
     1
     )); 

  //! Intake autonomous
  new JoystickButton(m_gamepad, 4).toggleOnTrue(new IntakeCommand(
    intakeSubsystem,
     0.5
     ));
     
     //climbing button
    new JoystickButton(m_gamepad, 0).toggleOnTrue(new SequentialCommandGroup(
      new AutoArm(armSubsystem, 1, armPosition.closedd),
     new ShootCommand(shooterSubsystem,intakeSubsystem,3,1 )
    ));

    //amplifier buton
    new JoystickButton(m_gamepad, 5).toggleOnTrue(new SequentialCommandGroup(
      new AutoArm(armSubsystem, 1, armPosition.amplifierr),
      new ShootCommand(shooterSubsystem, intakeSubsystem, 3, 1)
    ));
    //speaker buton
    new JoystickButton(m_gamepad, 5).toggleOnTrue(new SequentialCommandGroup(
      new AutoArm(armSubsystem, 2, armPosition.speakerr),
      new ShootCommand(shooterSubsystem, intakeSubsystem, 3, 1)
    ));
     

  }

 


  public Command getAutonomousCommand() {
    //return new AutoSICommand(shooterSubsystem, 2, 1, 1.1);
 
 
    return new AutoCommand();



 /*return new ColorIntakeCommand(
    shooterSubsystem,
     0.5, 
     4,
     1
  );
*/
 
  //return driveSubsystem.getAuton();
  }

  
  public static ShooterSubsystem getShooterSubsystem(){
    return shooterSubsystem;
  }
  public static DriveSubsystem getDriveSubsystem(){
     return driveSubsystem;
  }
}
