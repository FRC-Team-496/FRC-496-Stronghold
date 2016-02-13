package org.usfirst.frc.team496.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
    RobotDrive myRobot;
    Joystick stick;
    Talon portMotor, starMotor, armPortMotor, armStarMotor;
    
    //Compresssor Compressor;
    DoubleSolenoid portSole, starSole;
    DigitalInput armLimit;
    
    PowerDistributionPanel pdp;
    
    public static final int portMotorPWM = 		0;
    public static final int starMotorPWM = 		1;
    public static final int armPortMotorPWM = 	2;
    public static final int armStarMotorPWM = 	3;
    
    public static final int portSoleChanIn = 	0;
    public static final int portSoleChanOut = 	1;
    public static final int starSoleChanIn = 	2;
    public static final int starSoleChanOut = 	3;
    
    public static final int armLimitChan = 4;
    boolean val = false;
    
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    SendableChooser chooser;

    public Robot() {
        myRobot = new RobotDrive(0, 1);
        myRobot.setExpiration(0.1);
        stick = new Joystick(0);
        
        portMotor = new Talon(portMotorPWM);
        starMotor = new Talon(starMotorPWM);
        armPortMotor = new Talon(armPortMotorPWM);
        armStarMotor = new Talon(armStarMotorPWM);
        
        portSole = new DoubleSolenoid(portSoleChanIn, portSoleChanOut);
        starSole = new DoubleSolenoid(starSoleChanIn, starSoleChanOut);
        armLimit = new DigitalInput(armLimitChan);
        
        
        pdp = new PowerDistributionPanel();
        
    }
    
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto modes", chooser);
    }

    public void autonomous() {
    	
    	String autoSelected = (String) chooser.getSelected();
//		String autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
    	
    	switch(autoSelected) {
    	case customAuto:
            myRobot.setSafetyEnabled(false);
            myRobot.drive(-0.5, 1.0);	// spin at half speed
            Timer.delay(2.0);		//    for 2 seconds
            myRobot.drive(0.0, 0.0);	// stop robot
            break;
    	case defaultAuto:
    	default:
            myRobot.setSafetyEnabled(false);
            myRobot.drive(-0.5, 0.0);	// drive forwards half speed
            Timer.delay(2.0);		//    for 2 seconds
            myRobot.drive(0.0, 0.0);	// stop robot
            break;
    	}
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
			
        	myRobot.arcadeDrive(stick); // drive with arcade style (use right stick)
            
        	if(stick.getTrigger())
        	{
        		armPortMotor.set(stick.getThrottle());
        		armPortMotor.set(stick.getThrottle());
        	}
        	
        	if(stick.getRawButton(2))
        	{
        		if(!val)
        		{
            		portSole.set(DoubleSolenoid.Value.kForward);
            		starSole.set(DoubleSolenoid.Value.kForward);
            		val = !val;
        		}
        		else
        		{
            		portSole.set(DoubleSolenoid.Value.kReverse);
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






