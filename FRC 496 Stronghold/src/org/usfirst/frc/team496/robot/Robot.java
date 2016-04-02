package org.usfirst.frc.team496.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
//import edu.wpi.first.wpilibj.AnalogGyro;//necessary?
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
//import edu.wpi.first.wpilibj.Ultrasonic;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
    RobotDrive myRobot;
    Joystick driveStick, opStick;
    Talon portMotor, starMotor, armPort, armStar;
    Victor shooter;
    Compressor compressor;
    //DigitalInput armLimit;
    BuiltInAccelerometer accel;
    PowerDistributionPanel pdp;
    
    public static final int portMotorPWM = 		0;
    public static final int starMotorPWM = 		1;
    public static final int armPortMotorPWM = 	2;
    public static final int armStarMotorPWM = 	3;
    
    
    public Robot() {
        myRobot = new RobotDrive(0, 1);
        myRobot.setExpiration(0.1);
        driveStick = new Joystick(0);
        opStick = new Joystick(1);
        
        portMotor = new Talon(portMotorPWM);
        starMotor = new Talon(starMotorPWM);
        armPort = new Talon(armPortMotorPWM);
        armStar = new Talon(armStarMotorPWM);
        
        shooter = new Victor(4);
        
        accel = new BuiltInAccelerometer();
        
        compressor = new Compressor();
        
        pdp = new PowerDistributionPanel();
        
    }
    
    public void robotInit() {
      
    }

    public void autonomous() {
    	compressor.stop();
    	myRobot.drive(-.5, 0);
    	Timer.delay(4.0);
    	myRobot.drive(0, 0);
    }

    public void operatorControl() {
        myRobot.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
        	compressor.stop();
        	//SmartDashboard.putNumber("Voltage", pdp.getVoltage());
			SmartDashboard.putNumber("Current", pdp.getTotalCurrent());
			SmartDashboard.putNumber("Accel X: ", accel.getX());
			SmartDashboard.putNumber("Accel Y: ", accel.getY());
			SmartDashboard.putNumber("Accel Z: ", accel.getZ());
			myRobot.arcadeDrive(driveStick.getY(), driveStick.getTwist()); // drive with arcade style (use right driveStick)
            
        	armPort.set(-opStick.getY());
        	armPort.set(opStick.getY());
        	
        	armPort.set(opStick.getY()/3);
			armStar.set(opStick.getY()/3);
			
        	if(opStick.getTrigger())
        	{
        		shooter.set(1.0);
        		shooter.set(-1.0);

        	}
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    public void test() {
    }
}