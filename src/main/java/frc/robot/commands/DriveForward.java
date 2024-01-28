package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForward extends Command {
    
    private double StartTime;
    private double drivetime;
    private double drivespeed;

    private final DriveSubsystem driveSubsystem;
    

    public DriveForward(DriveSubsystem driveSubsystem,double drivetime,double drivespeed) {

        this.drivetime=drivetime;
        this.drivespeed=drivespeed;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ArcadeDriveCmd started!");
        StartTime = Timer.getFPGATimestamp();

        if(drivespeed<-1){
            drivespeed=-1;
        }else if(drivespeed>1){
            drivespeed=1;
        }

    }

    @Override
    public void execute() {
    PassingTime();
    driveSubsystem.setMotors(drivespeed, drivespeed);
    
    }

    public double PassingTime(){
    double PassingTime=driveSubsystem.RealTime()-StartTime;

    return PassingTime;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
        driveSubsystem.stop();
    }

 
    @Override
    public boolean isFinished() {
        if(PassingTime() <= drivetime){
            return false;
        }else{
            return true;
        }
    }
    
}