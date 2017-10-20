package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class LightLocalizer {
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	protected Odometer odom;
	int tachCounter = 0;
	double[] thetaAtLines = new double[8];		//there are 4 lines to cross; however there are 8
												//spaces used for the rare case when a line is detected twice
												//or the light sensor stops on a line
												//TODO if this is no longer a problem, change it back to 4 or 5
	double degreeFrom0;
	final int linesToCross = 4;
	final double WHEEL_RADIUS = 2.1;
	final double TRACK = 12.45;		//TODO was 11.5
	final int FORWARD_SPEED = 150;
	final double distanceFromCenter = 15.5; //TODO was 14
	
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer ode){
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odom = ode;
	}
	
	public  void drive(double leftRadius, double rightRadius, double width) {
			

			LightCollection collector = new LightCollection();
			collector.addLight(this);
			collector.start();
			while(tachCounter < linesToCross){
				leftMotor.forward();
				rightMotor.backward();
			}
			///now to make changes for Nathan's bot
			//it will scan the lines such that [0] = [3]			x
										//		[1] = [0]			y
										//		[2] = [1]			x	dif between new[0] and new[2] / 2 * cos = -y
										//		[3] = [2]			y	dif between [3] and [1] / 2 * cos = -x
			collector.setSamplesCollectedToTrue();
			/*
			if(thetaAtLines[3] - thetaAtLines[1] < 0) 
				thetaAtLines[3] = thetaAtLines[3] + 360;
			double yTachDist = thetaAtLines[3] - thetaAtLines[1];
			if(thetaAtLines[2] - thetaAtLines[0] < 0) 
				thetaAtLines[2] = thetaAtLines[2] + 360;
			double xTachDist = thetaAtLines[2] - thetaAtLines[0];
			*/
//			if(thetaAtLines[1] - thetaAtLines[3] < 0) 
//				thetaAtLines[1] = thetaAtLines[1] + 360;
			double yTachDist = thetaAtLines[3] - thetaAtLines[1];
//			if(thetaAtLines[0] - thetaAtLines[2] < 0) 
//				thetaAtLines[0] = thetaAtLines[0] + 360;
			double xTachDist = thetaAtLines[2] - thetaAtLines[0];
			
			double yPos = -distanceFromCenter * Math.cos(Math.toRadians(yTachDist/2));		//one of these had a minus	
			double xPos = distanceFromCenter * Math.cos(Math.toRadians(xTachDist/2));			
		
			
		//	leftMotor.rotate((int) ((360 - xTachDist) * 2.1/2), true); 		
		//	rightMotor.rotate(-(int) ((l) * 2.1/2), false);
			double lastAng = 90 + 180-thetaAtLines[2] - xTachDist/2;  
			leftMotor.rotate((int) ((lastAng) * 2.1/2), true); 		
			rightMotor.rotate(-(int) ((lastAng) * 2.1/2), false);
			
		//	turnTo(lastAng);
			odom.setX(xPos);
			odom.setY(yPos);
			odom.setTheta(0);
			travelTo(0,0);
			turnTo(degreeFrom0);
			
			
	  }
	
	public void getTach(){
		thetaAtLines[tachCounter] = odom.getTheta();
		tachCounter++;
	}
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }

	  private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }
	  public void travelTo(double x, double y){
		  	x = x *30.48;
			y = y*30.48;
			//get current x and y from odometer 
			double currentX = odom.getX();
			double currentY = odom.getY();
			double changeInX = x - currentX;
			double changeInY = y - currentY;

			//calculate minimal angle to turn to 
			double ThetaInRadians = Math.atan2(changeInX, changeInY);		
			double newTheta = ThetaInRadians * 180/Math.PI;
			double changeInTheta = newTheta - odom.getTheta();	
			degreeFrom0 = -changeInTheta;
			leftMotor.setSpeed(100);										//TODO replace magic number with a rotate speed
			rightMotor.setSpeed(100);		
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
	  private void turnTo(double theta){
		  	//this method will turn the robot theta degrees clockwise
		  	leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);			
		 	rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), false);
		}
}

