package ca.mcgill.ecse211.lab4;

public class Distance implements UltrasonicController{



	//unlike the pController, the bang bang controller does not adjust based on error. 
	//Instead, the BB controller has thresholds, over and under which, it will correct error at fixed values

	private int distance;
	private int filterControl;
	private UltrasonicLocalizer usLocalizer;
	private static final int FILTER_OUT = 20;

	public Distance(UltrasonicLocalizer usLocalizer) {
		this.usLocalizer = usLocalizer;
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;

		if (distance >= 255 && filterControl < FILTER_OUT) { // Used a filter in order not to use the bad coordinates
			// This is the same filter that was given for PController.java
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
		usLocalizer.setUSDistance(this.distance);
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}

