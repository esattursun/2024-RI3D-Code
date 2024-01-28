// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class AutoArm extends Command {
  /** Creates a new AutoArm. */
    private final ArmSubsystem armSubsystem;
  private final PIDController pidController;  
  private final armPosition armPosition;

  public static enum armPosition{
    closedd,
    amplifierr,
    speakerr
  }

  /** Creates a new AutoIntake. */
  public AutoArm(ArmSubsystem armSubsystem, double setPoint, armPosition armPositiond) {
    this.armSubsystem = armSubsystem;
    this.armPosition = armPositiond;
    this.pidController = new PIDController(1.2, .3, .1);
    pidController.setSetpoint(setPoint);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    System.out.println("IntakePIDCmd started!");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(armSubsystem.GetEncodervalue());
    armSubsystem.move_arm(-speed * .5);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.move_arm(0);
    System.out.println("IntakePIDCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armPosition == armPosition.amplifierr){
      return this.armSubsystem.amplifier();

    }else if(armPosition == armPosition.closedd){
      return this.armSubsystem.isAtClose();

    }else if(armPosition == armPosition.speakerr){
      return this.armSubsystem.speaker();
    
    
    }else{
      return false;
    }
  }
}
