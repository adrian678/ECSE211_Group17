package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class UltrasonicLocalizer extends Thread implements UltrasonicController {

	public static final double WHEEL_RADIUS = 2.1;		//TODO decide whether to keep these, or call them from main class
	public static final double TRACK = 12.0;
	public static double alpha;
	public static double beta;
	public static double theta;
	private static final int ROTATE_SPEED = 50;
	private static final int correction = 30;			//TODO add explanation of correction, and probably remove it(probably was to correct left motor power)
															
	
	private static final int DISTWall = 25;
	private static final int WALLGap = 5;
	
	private Odometer odometer;
	private static int distance;
	private EV3UltrasonicSensor USSensor;
	private static final int MAX_DISTANCE = 255;
	private float[] usData = new float[] {0};
	private int count;
	
	//motors. //TODO decide whether to keep or change way they are accessed. Might cause Thread issue
			//initially, we had called these through LocalizationLab.motor, so if we were to repeat this, we would need an association to ZiplineLab, and call that
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	public UltrasonicLocalizer (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer, EV3UltrasonicSensor USSensor) {
		leftMotor = leftMotor;
		rightMotor = rightMotor;
		this.odometer = odometer;
		this.USSensor = USSensor;
	}

	public void fallingEdge(Odometer odometer) {		
		//for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {LocalizationLab.leftMotor, LocalizationLab.rightMotor}) {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			
							
			// rotate the robot until it sees no wall
			while(getData() < DISTWall + WALLGap){
				leftMotor.forward();
				rightMotor.backward();
			}
				Sound.beep();
				// keep rotating until the robot sees a back wall and get angle alpha
			while(getData() > DISTWall){
				leftMotor.forward();
				rightMotor.backward();
			}
				Sound.beep();
					//get the angle from the odometer
				rightMotor.stop(true);
				leftMotor.stop(true);
				alpha = odometer.getTheta();
					
				// switch direction and wait until it sees no wall
			while(getData() < DISTWall + WALLGap){
				leftMotor.backward();
				rightMotor.forward();
			}
				Sound.beep();
			while(getData() > DISTWall){
				leftMotor.backward();
				rightMotor.forward();
			}
				// keep rotating until the robot sees left wall and get beta angle
			
			rightMotor.stop(true);		//TODO why are these both true? test for improvement when leftMotor boolean is false
			leftMotor.stop(true);		//this should be false, otherwise, theta might increase (slightly) between wall detection and angle querying
			
			beta = odometer.getTheta();	
				//get the angle from the odometer
					//if our angle A is larger than B, it means we passed the 0 point, and that angle A is "negative".
					
				//if our alpha is bigger than beta, subtract 360.
				if(alpha> beta){
					alpha = alpha - 360;
				}
				//calculate the average angle and the zero point (zero is x axis)
				double averageAngle = (alpha + beta)/2;
			
					//double zero = (theta + odometer.getTheta());
					
					// correction constant helps correct consistent error in final position
					double zero =  beta-averageAngle -correction;

					
					leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, zero ), true);
					rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, zero), false);
				
					//added this to bring it past the line
				this.odometer.setTheta(0);
				
				while(Button.waitForAnyPress() != Button.ID_ESCAPE);		//a waiting loop in between the ultrasonic and light localization sections
				
				leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, 15), true);
				rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, 15), false);
				leftMotor.rotate(100, true);
				rightMotor.rotate(100, false);
				leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, 35), true);
				rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, 35), false);
				

				LightLocalizer lightLoc = new LightLocalizer(leftMotor, rightMotor, odometer);

				lightLoc.drive(WHEEL_RADIUS, WHEEL_RADIUS, TRACK);
				while (Button.waitForAnyPress() != Button.ID_ESCAPE);
				System.exit(0);

		}
		
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	void setUSDistance(int idk) {
		distance = idk;
		//TODO decide if above statement needs to be synchronized
	}


	private int getData(){
		int dist;
		try { Thread.sleep(50); } catch (InterruptedException e) {}

		// there will be a delay here
		USSensor.getDistanceMode().fetchSample(usData, 0);
		dist = (int) (usData[0]*100);

		if(dist>MAX_DISTANCE && count<=3){ //filter for false positives or negatives
			count++; return distance;
		}
		else if(dist>MAX_DISTANCE && count>3){
			return MAX_DISTANCE;
		}
		else{
			count=0;
			distance=dist;
			return dist;
		}
	}

	@Override
	public void processUSData(int distance) {
	}

	@Override
	public int readUSDistance() {
		return 0;
	}
}


