package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	//motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	//TODO add tertiary motor here
	// robot position
	private double x;
	private double y;
	private int xTilePos;
	private int yTilePos;
	private double theta;
	// variables for determining distances traveled
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	//robot measurements
	private double wheelradius = 2.1;
	private double width = 12.0; // was 11.5 before
	
	private static final long ODOMETER_PERIOD = 25; // odometer update period, in ms
	
	//TODO consider including a direction enumeration again - discuss with group
	// public Direction direction = Direction.NORTH; //enumeration used to determine approximate direction
	private Object lock; /* lock object for mutual exclusion */

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		//TODO add tertiary motor to constructor
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			// TODO put (some of) your odometer code here

			int nowTachoL = leftMotor.getTachoCount(); // Most recent left
														// tachometer reading
			int nowTachoR = rightMotor.getTachoCount(); // Most recent right
														// tachometer reading
			double distL = ((Math.PI * wheelradius * (nowTachoL - leftMotorTachoCount)) / 180); // distance travelled by left wheel
			double distR = ((Math.PI * wheelradius * (nowTachoR - rightMotorTachoCount)) / 180); // distance travelled by right wheel
																									 
			setLeftMotorTachoCount(nowTachoL); // updates left tacho count
			setRightMotorTachoCount(nowTachoR); // updates right tacho count

			double deltaD = (0.5 * (distL + distR)); // the distance traveled by the two bototm wheels are averaged together
			double deltaT = ((distL - distR) / width); // in radians

			synchronized (lock) {
				//the variables within this block may be written to by mutliple threads; thus they have been placed in a syncrhonized block
				double degreesDeltaT = Math.toDegrees(deltaT);
				double currentT = getTheta() + degreesDeltaT;
				if (currentT > 360) { // wraps angle back to 0 when angle > 360
					currentT = currentT - 360;
				}
				if (currentT < 0) { // wraps angle to 360 when angle < 0
					currentT = 360 + currentT;
				}
				// sets value of theta
				setTheta(currentT);

				double dX = deltaD * Math.sin(Math.toRadians(getTheta())); // change of distance traveled in X
				double dY = deltaD * Math.cos(Math.toRadians(getTheta())); // change of distance traveled in Y															

				setX(getX() + dX); // updates X position
				setY(getY() + dY); // updates Y position


			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}
	
	public void setXTilePos(int xTile) {
		synchronized (lock) {
			this.xTilePos = xTile;
		}
	}
	public void setYTilePos(int yTile) {
		synchronized (lock) {
			this.yTilePos = yTile;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount
	 *            the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount
	 *            the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;
		}
	}
	/**
	 * @param bearing
	 *            will change robot's direction among NORTH, EAST, SOUTH, WEST
	 *            if the angle( falls within thresholds
	 */

}
