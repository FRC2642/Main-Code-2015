package org.usfirst.frc.team2642.robot;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;


public class Robot extends IterativeRobot {
	
	Joystick stick; Joystick stickAux;
	RobotDrive drive;
	Gyro gyro;
	DigitalInput tooHigh;
	DigitalInput tooLow;
	Talon rightPicker;
	Talon leftPicker;
	Talon lift;
	//Talon lift;
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
    	stick = new Joystick(0);
    	gyro = new Gyro(0);
    	drive = new RobotDrive(0,2,1,3); 
    	drive.setInvertedMotor(MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(MotorType.kRearRight, true);
    	rightPicker = new Talon(6);
    	leftPicker = new Talon(5);
    	stick = new Joystick(0);
    	lift = new Talon(4);
    	tooHigh = new DigitalInput(8);
    	tooLow = new DigitalInput(9);
    	
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
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        angle = gyro.getAngle();
        
    	if(stick.getRawButton(12)){ //reset gyro
       	gyro.reset();

    	}
    	
        if (stick.getRawButton(2) && crabStraightCounter < 1){ //turn on crab straight
        	crabStraightSet = gyro.getAngle();
        	crabStraightCounter++;
        
        }else if (stick.getRawButton(2) && crabStraightCounter >= 1){
        	drive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), (angle - crabStraightSet)*-Kp, 0);
        	crabStraightCounter++;

        }else if( !stick.getRawButton(2) && crabStraightCounter >= 1){
        	crabStraightCounter = 0;
        }else if(stick.getRawButton(1)){
        	drive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(), gyro.getAngle());

        }else if(stick.getRawButton(11)){ 
        	drive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(), 0);

        }else{
        	drive.mecanumDrive_Cartesian(stick.getX()/2, stick.getY()/2, stick.getTwist()/2, gyro.getAngle());

        }
        
        //auxilar functions test
        if(stick.getRawButton(5) && !tooHigh.get()){  //Button 3 makes the lift go up unless it hits the upper limit switch.
        	lift.set(1);
        }else if(stick.getRawButton(3) && !tooLow.get()){  //Button 5 makes the lift go down unless it hits the lower limit switch.
        	lift.set(-0.9);
        }else{
        	lift.set(0);                            //If neither of the buttons are pressed, the lift stays where it is.
        }
        
        if(stick.getRawButton(6)) { //in
        	rightPicker.set(0.5);
        	leftPicker.set(-0.5); //left is opposite
        }else if(stick.getRawButton(4)) { //out
        	rightPicker.set(-0.5);
        	leftPicker.set(0.5);
        }else{
        	leftPicker.set(0);
        	rightPicker.set(0);		
        }
        
        //System.out.println(gyro.getAngle());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
