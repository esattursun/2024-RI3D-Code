package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstans;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootCommand extends Command {
  
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
 
  private double StartTime;
  private double ChargeTime;
  private double WorkingTimeAfterChargeTime;

  public ShootCommand(ShooterSubsystem shooterSubsystem,IntakeSubsystem intakeSubsystem,double ChargeTime,double WorkingTimeAfterChargeTime) {
    this.shooterSubsystem = shooterSubsystem; 
    this.intakeSubsystem=intakeSubsystem;
    this.ChargeTime = ChargeTime;
    this.WorkingTimeAfterChargeTime = WorkingTimeAfterChargeTime;


   addRequirements(shooterSubsystem);
  }

 
  @Override
  public void initialize() {
    System.out.println("shoot started");
    //! Time the command start
    StartTime = Timer.getFPGATimestamp();
    
  
  }
 
  @Override
  public void execute() {
  
  if (PassingTime() <= ChargeTime) {
    //! Until the passing time equals the Charging time.
      shooterSubsystem.setMotors(ShooterConstans.OtoShootChargePower);

  } else if (PassingTime() >= ChargeTime && PassingTime() <= ChargeTime+WorkingTimeAfterChargeTime) {
    //! From the end of the charging time to the shots end time.
      intakeSubsystem.SetIntakeMotor(-1);
      shooterSubsystem.setMotors(1);
   } 

PassingTime();
 }

  public double PassingTime(){
  double PassingTime=shooterSubsystem.RealTime()-StartTime;
  return PassingTime;
 }
  

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotors();
    System.out.println("shoot finished");
  }

  @Override
  public boolean isFinished() {
    if(PassingTime()>=  ChargeTime+WorkingTimeAfterChargeTime+0.1){
      return true;
    }else{
      return false;
    }
   
  }
  
}