package main;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

/**
 * The controller class for the project. Contains the flow of logic for the program. Along with
 * Sensor and movement configuration
 * @author Oliver Palmer
 *
 */
public class V2Control {

	TouchSensor touch;
	UltrasonicSensor ultrasonic;
	Boolean trig = false;
	
	public static void main(String[] args) {
		new V2Control();
	}
	
	public V2Control(){
		MovePilot pilot = getPilot();
		Brick brick = BrickFinder.getDefault();
		
		// Touch sensor config
		Port s1 = brick.getPort("S1");
		EV3TouchSensor tSensor = new EV3TouchSensor(s1);
		touch = new TouchSensor(tSensor);
		
		// Ultrasonic sensor config
		Port s2 = brick.getPort("S2");
		NXTUltrasonicSensor uSensor = new NXTUltrasonicSensor(s2);
		ultrasonic = new UltrasonicSensor(uSensor.getMode("Distance"));
		
		pilot.forward();
		while(Button.ESCAPE.isUp()){
			Delay.msDelay(100);
			// Enter trig approach mode
			if (Button.ENTER.isDown()){
				if (trig){
					trig = false;
				} else {
					trig = true;
				}
			}
			// Turns 90 degrees if an object is detected less than 30cm away from the sensor
			if (ultrasonic.distance() < 0.3){
				pilot.stop();
				while (ultrasonic.distance() < 0.3){
					pilot.rotate(-90);
					Delay.msDelay(2000);
				}
				pilot.forward();
			}
			if (trig && ultrasonic.distance() < 0.4){
				approachWall(pilot);
			}
			// Reverses and turns the robot when the touch sensor is pressed
			if (touch.pressed()){
				pilot.stop();
				pilot.travel(-50);
				pilot.rotate(-90);
				pilot.forward();
			}
		}
		pilot.stop();
		tSensor.close();
		uSensor.close();
		System.exit(0);
	}
	
	/**
	 * Creates a Move Pilot object to control the movement of the EV3
	 * @return new MovePilot object
	 */
	public MovePilot getPilot(){
		Wheel wheelL = WheeledChassis.modelWheel(Motor.A, 43.2).offset(68.4);
		Wheel wheelR = WheeledChassis.modelWheel(Motor.B, 43.2).offset(-68.4);
		Chassis chassis = new WheeledChassis(new Wheel[]{wheelL, wheelR}, WheeledChassis.TYPE_DIFFERENTIAL);
		return pilotConfig(new MovePilot(chassis));
	}
	
	/**
	 * Configuration for the Move Pilot. Sets angular and linear speed and acceleration.
	 * @param pilot the Move Pilot to be configured
	 * @return The configured Move Pilot object
	 */
	public MovePilot pilotConfig(MovePilot pilot) {
		pilot.setLinearSpeed(100);
		pilot.setLinearAcceleration(100);
		pilot.setAngularSpeed(100);
		pilot.setAngularAcceleration(100);
		return pilot;
	}
	
	/**
	 * Failed method  that uses trigonometry to approach a wall
	 * @param pilot the Move Pilot
	 */
	public void approachWall(MovePilot pilot){
		pilot.stop();
		Delay.msDelay(1000);
		double l1 = ultrasonic.distance();
		LCD.clear();
		LCD.drawString("l1 = " + l1, 0, 2);
		Delay.msDelay(1000);
		pilot.rotate(-90);
		Delay.msDelay(1000);
		double l2 = ultrasonic.distance();
		LCD.drawString("l2 = " + l2, 0, 3);
		double theta = Math.toDegrees(Math.atan(l1/l2));
		double diff = 180 - (90 + theta);
		LCD.drawString("turn = " + diff, 0, 4);
		pilot.rotate(diff);
		pilot.forward();
		
	}

}
