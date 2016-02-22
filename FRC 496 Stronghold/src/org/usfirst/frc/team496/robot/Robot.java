package org.usfirst.frc.team496.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.AnalogGyro;//necessary?
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Ultrasonic;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
    RobotDrive myRobot;
    Joystick driveStick, opStick;
    Talon portMotor, starMotor, armPort, armStar;
    
    Compressor compressor;
    DoubleSolenoid portSole, starSole;
    DigitalInput armLimit;
    
    //Gyro gyro;//What kind of gyro?
    
    //Ultrasonic fUltra, sUltra;
    
    
    PowerDistributionPanel pdp;
    
    public static final int portMotorPWM = 		0;
    public static final int starMotorPWM = 		1;
    public static final int armPortMotorPWM = 	2;
    public static final int armStarMotorPWM = 	3;
    
    public static final int portSoleChanIn = 	4;
    public static final int portSoleChanOut = 	5;
    public static final int starSoleChanIn = 	6;
    public static final int starSoleChanOut = 	7;
    
    //public static final int armLimitChan = 4;
    boolean val = false;
    
    public Robot() {
        myRobot = new RobotDrive(0, 1);
        myRobot.setExpiration(0.1);
        driveStick = new Joystick(0);
        opStick = new Joystick(1);
        
        portMotor = new Talon(portMotorPWM);
        starMotor = new Talon(starMotorPWM);
        armPort = new Talon(armPortMotorPWM);
        armStar = new Talon(armStarMotorPWM);
        
        compressor = new Compressor();
        
        portSole = new DoubleSolenoid(portSoleChanIn, portSoleChanOut);
        starSole = new DoubleSolenoid(starSoleChanIn, starSoleChanOut);
        //armLimit = new DigitalInput(armLimitChan);
        
        //gyro = new AnalogGyro(1);
        
        
        pdp = new PowerDistributionPanel();
        
    }
    
    public void robotInit() {
      
    }

    public void autonomous() {
    	
    }

    public void operatorControl() {
        myRobot.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
        	SmartDashboard.putNumber("Voltage", pdp.getVoltage());
			SmartDashboard.putNumber("Current", pdp.getTotalCurrent());
			//SmartDashboard.putNumber("Arm Motor P", pdp.getCurrent(14));
			//SmartDashboard.putNumber("Arm Motor S", pdp.getCurrent(15));
			SmartDashboard.putBoolean("Arm Up?", armLimit.get());
			SmartDashboard.putBoolean("Arm extended?", val);
			
        	myRobot.arcadeDrive(driveStick.getY(), driveStick.getTwist()); // drive with arcade style (use right driveStick)
            
        	armPort.set(-opStick.getY());
        	armPort.set(opStick.getY());
        	
        	if(opStick.getTrigger())
        	{
        		if(!val)
        		{
            		portSole.set(DoubleSolenoid.Value.kReverse);
            		starSole.set(DoubleSolenoid.Value.kForward);
            		val = !val;
        		}
        		else
        		{
            		portSole.set(DoubleSolenoid.Value.kForward);
            		starSole.set(DoubleSolenoid.Value.kReverse);
            		val = !val;
        		}

        	}
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    public void test() {
    }
}