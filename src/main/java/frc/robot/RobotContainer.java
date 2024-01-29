// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.AutoArm;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.autonomous.FRCPathPlanner;
import frc.robot.commands.AutoArm.armPosition;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {
 //subsystems 
 public static final PoseEstimation poseEstimation = new PoseEstimation();
 public static final Drivetrain drivetrain = new Drivetrain();

   public static Field2d field = new Field2d();
  public static Field2d nodeSelector = new Field2d();

 
   

 private final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
 private final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
 private final static ArmSubsystem armSubsystem = new ArmSubsystem();

 public final ShuffleboardTab Auto_Event_Map = Shuffleboard.getTab("Auto");
 //Joysticks
 private final Joystick m_gamepad = new Joystick (JoystickConstants.m_gamepadPort);
   private DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, poseEstimation, m_gamepad);

  public RobotContainer() {

    FRCPathPlanner.setSmartDashboard();
    FRCPathPlanner.CommandNameEntry();
    FRCPathPlanner.addAutoOptions();
    FRCPathPlanner.addPathOptions();
    FRCPathPlanner.FindPath();

    drivetrain.setDefaultCommand(driveCommand);
    configureButtonBindings();
  }

  
  
  private void configureButtonBindings() {

  
  
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
      new AutoArm(armSubsystem, armPosition.closedd),
     new ShootCommand(shooterSubsystem,intakeSubsystem,3,1 )
    ));

    //amplifier buton
    new JoystickButton(m_gamepad, 5).toggleOnTrue(new SequentialCommandGroup(
      new AutoArm(armSubsystem,armPosition.amplifierr),
      new ShootCommand(shooterSubsystem, intakeSubsystem, 3, 1)
    ));
    //speaker buton
    new JoystickButton(m_gamepad, 5).toggleOnTrue(new SequentialCommandGroup(
      new AutoArm(armSubsystem,  armPosition.speakerr),
      new ShootCommand(shooterSubsystem, intakeSubsystem, 3, 1)
    ));
     

  }

 


  public Command getAutonomousCommand() {
    
 
 
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
  public static IntakeSubsystem getIntakeSubsystem(){
     return intakeSubsystem;
  }
  public static PoseEstimation poseEstimation(){
    return poseEstimation;
  }
  
  public static Drivetrain getSwerveSubsystem() {
    return drivetrain;
  }
}
