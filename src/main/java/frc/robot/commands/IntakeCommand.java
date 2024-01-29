package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstans;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.TimerUtils;



public class IntakeCommand extends Command {
  
  private final IntakeSubsystem IntakeSubsystem;
 
  private double StartTime;
  private double KeepTime;
  private double PassingTime;
  


  public IntakeCommand(IntakeSubsystem IntakeSubsystem,double KeepTime) {
    this.IntakeSubsystem = IntakeSubsystem; 
    this.KeepTime = KeepTime;

   addRequirements(IntakeSubsystem);
  }

 
  @Override
  public void initialize() {
    System.out.println("Intake started");
    //! Time the command start
    StartTime=Timer.getFPGATimestamp();
  }

  
  @Override
  public void execute() {
    double PassingTime = TimerUtils.PassingTimeCalculator(StartTime);

    if (PassingTime <= KeepTime){
      //! Until the passing time equals the keep time.
        IntakeSubsystem.SetIntakeMotor(ShooterConstans.OtoIntakeUpPower);
      
    }

   
  }
  

  @Override
  public void end(boolean interrupted) {
   IntakeSubsystem.stopMotor();
   System.out.println("intake finished");
  }

  
  @Override
  public boolean isFinished() {
    if(PassingTime > KeepTime+0.1){
      return true;
    }else{
      return false;
    }
  }
}

