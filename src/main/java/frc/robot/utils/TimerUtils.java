package frc.robot.utils;
import edu.wpi.first.wpilibj.Timer;

public class TimerUtils {
    
public static double RealTime(){
  double RealTime= Timer.getFPGATimestamp();
  return RealTime;
  }

public static double PassingTimeCalculator(double InitializeTime){
   double PassingTime = RealTime()-InitializeTime;
   return PassingTime;

}

}
