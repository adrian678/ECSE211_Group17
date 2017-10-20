package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ZiplineLab {
	
	private static EV3UltrasonicSensor usSensor = 		new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));		//TODO was S4	light sensor
	public static EV3LargeRegulatedMotor leftMotor = 	new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));	//left motor
	public static EV3LargeRegulatedMotor rightMotor =	new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));	//right motor
	public static TextLCD t = LocalEV3.get().getTextLCD(); 	//explanation: the LCD has been made static so that
															//it could be included in the method below
															//however, because we are considering instantiating the main class,
															//this may need to change later
	
	//measurements of robot 
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 12.0;
	public static final int FORWARD_SPEED = 180;
	public static final int ROTATE_SPEED = 120;
	
	public static int yPosZip;
	public static int xPosZip;
	public static int xPosOneAway;
	public static int yPosOneAway;
	
	//declarations of other necessary classes and threads
	final static Odometer odometer = new Odometer(leftMotor, rightMotor);
	final static OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);			//TODO decide on protection modifier for these two, or move them to main
	private static UltrasonicPoller usPoller;
	private static float[] usData = new float[] {0};
//	SensorModes usSensor = new EV3UltrasonicSensor(usSensor);
//	private SampleProvider usSample = new usSensor.getMode("Distance");
//	final static Odometer odometer = new Odometer(leftMotor, rightMotor);
//	OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
//	private static UltrasonicPoller usPoller;
	
	public static void main(String[] args){
		
		//get coordinates of zipline from user
		getZipCoordinates();		//TODO test this
		
		//TODO instantiate and run odometer				- make sure this does not start until after zipline coords have been added or LCD screen will be messy
		t.clear();
		odometer.start();
		odometryDisplay.start();
		
		UltrasonicPoller USPoller = null; //TODO is the poller necessary to keep here, and should it persist? do we only use US localization once, do we even use it at all?
		UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, usSensor);
		Distance distanceController = new Distance(ultrasonicLocalizer);
		ultrasonicLocalizer.fallingEdge(odometer);			//if possible, we should separate the light localization from the ultrasonic localization
	//	usPoller = new UltrasonicPoller(us, usData, ultrasonicLocalizer);
	//	usPoller.start();
		//localize first
		travelTo(xPosOneAway, yPosOneAway);
		//instantiate light localizer
		LightLocalizer lightLoc = new LightLocalizer(leftMotor, rightMotor, odometer);
		
		//actually use light localizer
		//TODO finish testing changes for light localization to make it go in right direction
		lightLoc.drive(WHEEL_RADIUS, WHEEL_RADIUS, TRACK);
			//TODO review the necessity of an UltrasonicController interface
			//TODO decide whether other classes should have access to the motors, or whether they should call it from zipLineLab
			//TODO decide which class should contain a map of the grid. Maybe navigation? maybe odometer.
		//once localization is completed, begin navigation towards zip coords
		
		//TODO instantiate and run light localization	- takes light sensor as input
		
		//instantiate and run localization class
	}
	
	private static void getZipCoordinates(){
		int buttonChoice;														//buttonChoice will hold value of buttons pressed
		do{
		t.drawString("You must enter the coordinates of the zipline to begin", 0, 0);
		t.drawString("coordinates of the zipline", 0, 1);
		t.drawString("currently, the robot starts off facing Y direction", 0, 2);
		t.drawString("so enter the coordinates in the order (y,x)", 0, 3);
		t.drawString("coordinates in the order (y,x)", 0, 4);
		t.drawString("(y,x)", 0, 5);
		t.drawString("press right button to continue and enter your coords", 0, 6);
		t.drawString("to continue", 0, 7);
		buttonChoice = Button.waitForAnyPress();
		} while(buttonChoice != Button.ID_RIGHT);
		yPosZip = 0;
		do{
			t.drawString("Assuming a 4 by 4 grid, choose y pos", 0, 0);
			t.drawString("Press right to increase", 0, 1);
			t.drawString("Press left to decrease", 0, 2);
			t.drawString("Current pos" + yPosZip, 0, 3);
			t.drawString("Current enter to continue", 0, 4);
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_RIGHT){
				yPosZip++;
			} else if(buttonChoice == Button.ID_LEFT){
				yPosZip--;
			}
			} while(buttonChoice != Button.ID_ENTER);
		xPosZip = 0;
		do{
			t.clear();
			t.drawString("Assuming a 4 by 4 grid, choose x pos", 0, 0);
			t.drawString("Press right to increase", 0, 1);
			t.drawString("Press left to decrease", 0, 2);
			t.drawString("Current pos" + xPosZip, 0, 3);
			t.drawString("Current enter to continue", 0, 4);
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_RIGHT){
				xPosZip++;
			} else if(buttonChoice == Button.ID_LEFT){
				xPosZip--;
			}
			} while(buttonChoice != Button.ID_ENTER);
		int xPosOneAway = 0;
		do{
			t.clear();
			t.drawString("Now enter the x position one tile away, choose x pos", 0, 0);
			t.drawString("Press right to increase", 0, 1);
			t.drawString("Press left to decrease", 0, 2);
			t.drawString("Current pos" + xPosOneAway, 0, 3);
			t.drawString("Current enter to continue", 0, 4);
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_RIGHT){
				xPosOneAway++;
			} else if(buttonChoice == Button.ID_LEFT){
				xPosOneAway--;
			}
			} while(buttonChoice != Button.ID_ENTER);
		int yPosOneAway = 0;
		do{
			t.drawString("Now enter the x position one tile away, choose x pos", 0, 0);
			t.drawString("Press right to increase", 0, 1);
			t.drawString("Press left to decrease", 0, 2);
			t.drawString("Current pos" + yPosOneAway, 0, 3);
			t.drawString("Current enter to continue", 0, 4);
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_RIGHT){
				yPosOneAway++;
			} else if(buttonChoice == Button.ID_LEFT){
				yPosOneAway--;
			}
			} while(buttonChoice != Button.ID_ENTER);
		
	}
	public static void travelTo(double x, double y){
		x = x *30.48;
		y = y*30.48;	
		//get current x and y from odometer 
		double currentX = odometer.getX();
		double currentY = odometer.getY();
		double changeInX = x - currentX;
		double changeInY = y - currentY;
		//calculate minimal angle to turn to 
		double ThetaInRadians = Math.atan2(changeInX, changeInY);		
		double newTheta = ThetaInRadians * 180/Math.PI;
		double changeInTheta = newTheta - odometer.getTheta();		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		//pass that angle into the turnTo method
		if(changeInTheta < 180.0 && changeInTheta > -180){
			//rotate to newTheta
			turnTo(changeInTheta);
		} else if(changeInTheta < -180){
			//rotate to changeInTheta + 360
			turnTo(changeInTheta + 360);		//
		}
		else if(changeInTheta > 180){
			//rotate to changeInTheta - 360
			turnTo(changeInTheta - 360);
		}
		double hypotenuseDist = Math.sqrt(Math.pow(changeInX, 2) + Math.pow(changeInY, 2));
		int degreeDist = convertDistance(WHEEL_RADIUS, hypotenuseDist);					
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(degreeDist, true);
	    rightMotor.rotate( degreeDist, false);	
	    
}

private static void turnTo(double theta){
	leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);			//TODO test if this is covered
    rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), false);
}
public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }
public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
