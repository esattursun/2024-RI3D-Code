package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;


public class ColorSensor extends SubsystemBase {
    public String colorString;
    
    private final I2C.Port i2cPort;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
    public final ColorSensorV3 m_colorSensor ;
  
    /**
     * A Rev Color Match object is used to register and detect known colors. This can 
     * be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    public  final ColorMatch m_colorMatcher ;
  
    //color targets
    public  final Color kBlueTarget;
    public  final Color kGreenTarget;
    public  final Color kRedTarget;
    public  final Color kYellowTarget;
    public  final Color kOrangeTarget;

  public ColorSensor() {
    i2cPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorMatcher = new ColorMatch();

    kBlueTarget = new Color(0.143, 0.427, 
    0.429);
    kGreenTarget = new Color(0.197, 0.561, 0.240);
    kRedTarget = new Color(0.561, 0.232, 0.114);
    kYellowTarget = new Color(0.361, 0.524, 0.113);
    kOrangeTarget = new Color(0.262, 0.456, 0.227);
  }
 

@Override
public void periodic() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
    if (match.color == kOrangeTarget) {
        colorString = "Orange";
      } else {
        colorString = "Unknown";
      }
  
  SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
}
 
}
