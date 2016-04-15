package main;

import lejos.robotics.SampleProvider;
import lejos.robotics.filter.AbstractFilter;

/**
 * Ultrasonic Sensor class, used to return the distance between the sensor and any objects in front of it.
 * @author Oliver Palmer
 *
 */
public class UltrasonicSensor extends AbstractFilter {
	
	private float[] sample;
	
	public UltrasonicSensor(SampleProvider source) {
		super(source);
		sample = new float[sampleSize];
	}
	
	/**
	 * Measures the distance to an object in front of the Ultrasonic sensor
	 * @return Float - Distance to object detected
	 */
	public float distance() {
		super.fetchSample(sample, 0);
		return sample[0];
	}

}
