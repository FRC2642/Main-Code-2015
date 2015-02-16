package org.usfirst.frc.team2642.robot;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends IterativeRobot {
	
	Joystick driveStick; Joystick auxStick; Joystick auxCard; 
	RobotDrive drive;
	Gyro gyro;
	DigitalInput liftUpperLimit;
	DigitalInput liftLowerLimit;
	Talon rightPicker;
	Talon leftPicker;
	Talon lift;
	Encoder liftEncoder;
	DigitalInput toteIn;

	
	Compressor compressor;
	Solenoid dogs;
	Solenoid pusher;
	Solenoid flipper;
	
	
	//DigitalInput toteIn; 
	//DigitalInput fiveTotesIn; DigitalInput liftUpperLimit;DigitalInput liftLowerLimit;

	int autoLoopCounter;
	double Kp = 0.04;
	double angle;
    int crabStraightCounter;
    double crabStraightSet;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	driveStick = new Joystick(0);
    	auxStick = new Joystick(1);
    	auxCard = new Joystick(2);
    	gyro = new Gyro(0);
    	drive = new RobotDrive(2,0,1,3); 
    	drive.setInvertedMotor(MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(MotorType.kRearRight, true);
    	
    	rightPicker = new Talon(6);
    	leftPicker = new Talon(5);
    	lift = new Talon(4);
    	liftEncoder = new Encoder(6,7);
    	liftUpperLimit = new DigitalInput(8);
    	liftLowerLimit = new DigitalInput(9);
    	
    	compressor = new Compressor(0);
    	dogs = new Solenoid(0);
    	pusher = new Solenoid(1);
    	flipper = new Solenoid(2);
    	
    	
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	gyro.reset();
    	crabStraightCounter = 0;
    	compressor.start();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        angle = gyro.getAngle();
        
    	if(auxCard.getRawButton(5)){ //reset gyro
       	gyro.reset();

    	}
    	
        if (driveStick.getRawButton(2) && crabStraightCounter < 1){ //turn on crab straight
        	crabStraightSet = gyro.getAngle();
        	crabStraightCounter++;
        
        }else if (driveStick.getRawButton(2) && crabStraightCounter >= 1){ //crab straight
        	drive.mecanumDrive_Cartesian(driveStick.getX(), driveStick.getY(), (angle - crabStraightSet)*-Kp, 0);
        	crabStraightCounter++;

        }else if( !driveStick.getRawButton(2) && crabStraightCounter >= 1){ //reset counter
        	crabStraightCounter = 0;
        }else if(driveStick.getRawButton(1)){ //full speed
        	drive.mecanumDrive_Cartesian(driveStick.getX(), driveStick.getY(), driveStick.getTwist(), gyro.getAngle());

        }else if(auxCard.getRawButton(6)){ //turn of field oriented control 
        	drive.mecanumDrive_Cartesian(driveStick.getX(), driveStick.getY(), driveStick.getTwist(), 0);

        }else{
        	drive.mecanumDrive_Cartesian(driveStick.getX()/2, driveStick.getY()/2, driveStick.getTwist()/2, gyro.getAngle());
        }
        
        if(driveStick.getRawButton(3)){ //flipper 
        	flipper.set(true);
        }else{
        	flipper.set(false);
        }
 //==============================================================================================
        
        
        //lift
        if(auxStick.getRawButton(3) && !liftUpperLimit.get()){  //up
        	lift.set(1);
        }else if(auxStick.getRawButton(2) && !liftUpperLimit.get()){  //down
        	lift.set(-0.9);
        }else{
        	lift.set(0);                            
        }
        
//===================================================================================================
        //picker
        if(auxCard.getRawButton(9)) { //in
        	rightPicker.set(0.5);
        	leftPicker.set(-0.5); //left is opposite
        }else if(auxCard.getRawButton(7)) { //out
        	rightPicker.set(-0.5);
        	leftPicker.set(0.5);
        }else{
        	leftPicker.set(0);
        	rightPicker.set(0);		
        }
        
        if(auxStick.getRawButton(6)){
        	dogs.set(true);
        }else{
        	dogs.set(false);
        }
        
        if(auxStick.getRawButton(7)){
        	pusher.set(true);
        }else{
        	pusher.set(false);
        }
        
        //Timer.delay(0.005);
        System.out.println(liftEncoder.getDistance());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
