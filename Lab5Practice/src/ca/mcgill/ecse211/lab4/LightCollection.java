package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightCollection extends Thread{
	private static final long LIGHT_SAMPLE_PERIOD = 100;		//TODO was 100
	private static Port portColor = LocalEV3.get().getPort("S2"); //TODO was S1
	private static SensorModes myColor = new EV3ColorSensor(portColor);
	private LightLocalizer lightLocal = null;
	private static SampleProvider myColorSample = myColor.getMode("Red"); // sampleProvider will get values from "Red" mode colorSensor		
	private float[] sampleColor = new float[myColor.sampleSize()];
	
	
	public static int numSamples;
	public static int currentSample;
	public boolean areSamplesCollected;			//TODO decide on public or private for this
	// constructor
	public LightCollection() {

	}

	public void run() {

		long correctionStart, correctionEnd;
		while (true) {
			correctionStart = System.currentTimeMillis();
			myColorSample.fetchSample(sampleColor, 0);
			currentSample = (int) (sampleColor[0] * 100.0);
			if (currentSample < 50) {	//was 50
				if(lightLocal != null && !areSamplesCollected){
					lightLocal.getTach();
					Sound.beep();
				}
			}

			// this ensures that light sampling only occurs once per period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < LIGHT_SAMPLE_PERIOD) {
				try {
					Thread.sleep(LIGHT_SAMPLE_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					//nothing is expected to interrupt this thread
				}
			}
		}
	}
	
	public void addLight(LightLocalizer light){
		  this.lightLocal = light;
	  }
	public void setSamplesCollectedToTrue(){
		areSamplesCollected = true;
	}


}

