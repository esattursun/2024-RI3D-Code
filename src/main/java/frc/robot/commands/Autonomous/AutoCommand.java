// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveForward;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;

public class AutoCommand extends SequentialCommandGroup {
    
  

  public AutoCommand() {
    addCommands(
     new IntakeCommand(RobotContainer.getShooterSubsystem(), 0.5),
     new WaitCommand(0.5),
     new DriveForward(RobotContainer.getDriveSubsystem(),0.5,0.5),
     new ShootCommand(RobotContainer.getShooterSubsystem(), 4, 1)
    );
    
  }


}