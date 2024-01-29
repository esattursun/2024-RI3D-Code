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
    closeshoot,
    amplifierr,
    speakerr
  }

  /** Creates a new AutoIntake. */
  public AutoArm(ArmSubsystem armSubsystem,armPosition armPositiond) {
    this.armSubsystem = armSubsystem;
    this.armPosition = armPositiond;
    this.pidController = new PIDController(1.2, .3, .1);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    System.out.println("IntakePIDCmd started!");

    switch(armPosition){
      case closedd:
        pidController.setSetpoint(8.76);
      case closeshoot:
        pidController.setSetpoint(7.95);
      case amplifierr:
        pidController.setSetpoint(4.875);
      case speakerr:
        pidController.setSetpoint(2);
      default:
        break;
      
    }
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

    switch (armPosition) {
      case closedd:
          return this.armSubsystem.isAtClose();
      case closeshoot:
          return this.armSubsystem.isAtCloseShoot();
      case amplifierr:
          return this.armSubsystem.amplifier();
      case speakerr:
          return this.armSubsystem.speaker();
      default:
          return false;
  }
  }
}
