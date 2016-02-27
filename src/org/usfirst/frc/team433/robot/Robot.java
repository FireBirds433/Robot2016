//SPOCK - includes camera toggle, ball mechanism, hanging mechanism. Gyro reset and calibrate are in robotinit so autonomous can only be run once
//(if attempt is made to run autonomous again without restarting robot, initial gyro value will not be set to zero)

package org.usfirst.frc.team433.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Set;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	Joystick xbox;
	CANTalon LF = new CANTalon(1);
	CANTalon LR = new CANTalon(2);
	CANTalon RF = new CANTalon(3);
	CANTalon RR = new CANTalon(4);
	CANTalon shooter = new CANTalon(5); // positive: intake, negative: outtake
	CANTalon Rope6 = new CANTalon(6);// negative: unwind, positive: pull in
	CANTalon Rope7 = new CANTalon(7);// negative: unwind, positive: pull in
	CANTalon Tape8 = new CANTalon(8);// negative: pull in, positive: unwind
	CANTalon Tape9 = new CANTalon(9);// negative: pull in, positive: unwind
	CameraServer server;
	Compressor compressor = new Compressor(0);
	Solenoid solenoid0 = new Solenoid(0, 0); // speed shifting
	Solenoid solenoid1 = new Solenoid(1, 1);
	Solenoid solenoid2 = new Solenoid(2, 2);
	int autoLoopCounter;
	int NORMSPEED = 100; // forward and backward
	int Timer;
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	// CAMERA SWITCHING
	int currSession; // which camera?
	int sessionfront;// front camera
	int sessionback; // back camera
	Image frame;

	// ANALOG SWITCHES
	AnalogInput theSwitch1 = new AnalogInput(0);
	AnalogInput theSwitch2 = new AnalogInput(1);
	AnalogInput theSwitch3 = new AnalogInput(2);

	// BALL INTAKE/SHOOTER
	DigitalInput limitswitch = new DigitalInput(4); // limit switch true when
													// not hit, false when hit

	// COMPRESSOR SWITCHING
	int currCompressor;
	int compressoron;
	int compressoroff;

	//Break Mode
	/*Rope6.
	Rope7.enableBrakeMode(true);
	*/
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	public void robotInit() {

		// Robot Drive
		myRobot = new RobotDrive(LF, LR, RF, RR); // Left front, left rear,
													// right front, right rear
		stick = new Joystick(0); // joystick
		xbox = new Joystick(1); // xbox controller

		// Live Window
		LiveWindow.addActuator("Drive Talon", "Right Front", RF);
		LiveWindow.addActuator("Drive Talon", "Right Rear", RR);
		LiveWindow.addActuator("Drive Talon", "Left Front", LF);
		LiveWindow.addActuator("Drive Talon", "Left Rear", LR);
		LiveWindow.addActuator("Rope Talon", "Rope Talon 6", Rope6);
		LiveWindow.addActuator("Rope Talon", "Rope Talon 7", Rope7);
		LiveWindow.addActuator("Tape Talon", "Tape Talon 8", Tape8);
		LiveWindow.addActuator("Tape Talon", "Tape Talon 9", Tape9);
		LiveWindow.addActuator("Shifting", "Solenoid", solenoid0);
		LiveWindow.addActuator("Intake", "Talon 5", shooter);
		LiveWindow.addActuator("Compressor", "Compressor", compressor);

		// CAMERA VARIABLES
		/*
		 * frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		 * sessionfront = NIVision.IMAQdxOpenCamera("cam0",
		 * NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		 * sessionback = NIVision.IMAQdxOpenCamera("cam1",0.
		 * 
		 * NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		 * currSession = sessionfront;
		 * NIVision.IMAQdxConfigureGrab(currSession);
		 */
		// AUTONOMOUS
		 gyro.reset();
		 gyro.calibrate();

		// COMPRESSOR
		currCompressor = compressoron;

		// brakes

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */

	public void autonomousInit() {
		autoLoopCounter = 0;
		gyro.calibrate();
	}

	/**
	 * This function is called periodically during autonomous
	 */

	public void autonomousPeriodic() {
		// SmartDashboard.putNumber("Gyro", gyro.getAngle());
		SmartDashboard.putNumber("AutoLoopCounter", autoLoopCounter);
		
		// Switches gathering
		double switRaw1 = theSwitch1.pidGet();// analog switch 1
		double switRaw2 = theSwitch2.pidGet();// analog switch 2
		double switRaw3 = theSwitch3.pidGet();// analog switch 3
		int switNum1;
		int switNum2;
		int switNum3;
		// Raw analog to binary (0 and 1)
		if (switRaw1 > .1) {
			switNum1 = 1;
		} else {
			switNum1 = 0;
		}
		if (switRaw2 > .1) {
			switNum2 = 1;
		} else {
			switNum2 = 0;
		}
		if (switRaw3 > .1) {
			switNum3 = 1;
		} else {
			switNum3 = 0;
		}
		// Converting binary numbers to a single integer for case statements
		int switBinFin = ((switNum1 * 4) + (switNum2 * 2) + (switNum3));
		// Uses integer to run autonomous program 0-7
		int noSwitches = 1;
		switch (noSwitches) {
		case 0:// Is case 0 possible?
			DoNothing();// do nothing
			break;
		case 1:
			LowBarShooter();// cross low bar and shoot low goal
			break;
		case 2:
			Rampart(); // cross defense backwards and turn
			break;
		case 3:
			StraightDefense(); // drive over defense
			break;
		case 4:
			CrossLine(); // breach but dont cross a defense
			break;
		case 5:
			DoNothing(); // do nothing
			break;
		case 6:
			LowBarShooterNoGyro(); // low bar and low goal with no gyro
			break;
		case 7:
			SpybotScore(); // shoot low goal from spybot
			break;
		default:
			DoNothing(); // do nothing
			break;
		}
	}

	// Autonomous programs
	public void DoNothing() {// down down down (0/5/default)
		// dont move
		LF.set(0);
		RF.set(0);
		LR.set(0);
		RR.set(0);
	}

	public void LowBarShooter() { // down down up (1)
		// Use gyro to cross low bar and shoot low goal
		double distanceN = 227; // loops until turn
		double distanceP = 313; // loops until batter
		double shootinTime = 335; // loops for shooting
		double angle = gyro.getAngle();
		double heynow = 30; // The angle we want divided by 2
		double speedyspeed = .8; // motor speed
		double Kp = .03; // How fast are we turning?	
		solenoid0.set(true);
		
		if (autoLoopCounter < 3) {// Drive straight
			myRobot.arcadeDrive(-.4, -angle * Kp);
			shooter.set(0);
			// Is the motor inverted?
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter < distanceN && autoLoopCounter >= 3) {// Drive straight
			myRobot.arcadeDrive(-speedyspeed, -angle * Kp);
			shooter.set(0);
			// Is the motor inverted?
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter == distanceN && Math.abs(angle) < heynow) {// Time
																		// to
																		// start
																		// turning
			myRobot.arcadeDrive(-speedyspeed, -angle * Kp);// should he be going
															// straight? :P
			LF.set(-.8);// Test for inversion
			LR.set(-.8);
			RF.set(.3);
			RR.set(.3);
			shooter.set(0);
			return;
		}
		if (Math.abs(angle) >= heynow && autoLoopCounter == distanceN) {// We
																		// found
																		// the
																		// angle
			autoLoopCounter++;
			shooter.set(0);
			return;
		}
		if (autoLoopCounter > distanceN && autoLoopCounter < distanceP) { // Go
																			// straight
																			// after
																			// turning
			myRobot.arcadeDrive(-speedyspeed, -(angle - 2 * heynow) * Kp);// stay
																			// straight
																			// relative
																			// to
																			// the
																			// new
																			// angle
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter == distanceP) { // We're there stop
			LF.set(0);
			RF.set(0);
			LR.set(0);
			RR.set(0);
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter > distanceP && autoLoopCounter < shootinTime) { // Shoot
			shooter.set(1);
			autoLoopCounter++;
			return;// dont need this but it looks cool
		}
		if (autoLoopCounter >= shootinTime) { // Shoot
			shooter.set(0);
			autoLoopCounter++;
			return;// dont need this but it looks cool
		}
	}

	public void Rampart() { // Down up down (2)
		// cross a defense backwards and turn 180
		solenoid0.set(true);
		double distanceN = 79; // loops until defense crossed
		// double angle = gyro.getAngle();
		double angle = 0;
		double speedyspeed = .9; // motor speed
		double Kp = .03; // How fast are we turning?

		if (autoLoopCounter < distanceN) {// Drive straight
			myRobot.arcadeDrive(speedyspeed, -angle * Kp); // Make sure
															// speedyspeed is
															// heading backwards
			// Is the motor inverted?
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter == distanceN && Math.abs(angle) < 180) {// Time to
																	// start
																	// turning
			LF.set(-.5);// Test for inversion
			LR.set(-.5);
			RF.set(.5);
			RR.set(.5);
			return;
		}
		if (Math.abs(angle) >= 180 && autoLoopCounter == distanceN) {// We found
																		// the
																		// angle
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter > distanceN) {// stop
			LF.set(0);
			RF.set(0);
			LR.set(0);
			RR.set(0);
		}

	}

	public void StraightDefense() {// Down up up (3)
		// drive straight over a defense
		solenoid0.set(true);
		double distanceN = 79; // loops until turn
		// double angle = gyro.getAngle();
		double angle = 0;
		double speedyspeed = .9; // motor speed
		double Kp = .03; // How fast are we turning?

		if (autoLoopCounter < distanceN) {// Drive straight
			myRobot.arcadeDrive(-speedyspeed, -angle * Kp);
			// Is the motor inverted?
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter >= distanceN) {// Cleared defense, stop
			LF.set(0);
			RF.set(0);
			LR.set(0);
			RR.set(0);
		}
	}

	public void CrossLine() { // Up down down (4)
		// breaches a defense but doesn't cross
		double distanceN = 58; // loops until turn (make it smaller)
		// double angle = gyro.getAngle();
		double angle = 0;
		double speedyspeed = .9; // motor speed
		double Kp = .03; // How fast are we turning?

		if (autoLoopCounter < distanceN) {// Drive straight
			myRobot.arcadeDrive(-speedyspeed, -angle * Kp);
			// Is the motor inverted?
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter >= distanceN) {// Cleared defense, stop
			LF.set(0);
			RF.set(0);
			LR.set(0);
			RR.set(0);
		}
	}

	public void LowBarShooterNoGyro() {// up up down (6)
		// cross low bar and shoot low goal dead reckoning
		solenoid0.set(true);
		double distanceN = 114; // loops until turn
		double distanceT = 120; // loops we turn for
		double distanceP = 151; // loops until batter
		double shootinTime = 231; // loops for shooting
		double speedyspeed = .9; // motor speed

		if (autoLoopCounter < distanceN) {// Drive Straight
			myRobot.arcadeDrive(-speedyspeed, 0);
			autoLoopCounter++;
			return;
		}

		if (autoLoopCounter >= distanceN && autoLoopCounter < distanceT) {// turning
			LF.set(.2);
			RF.set(-.5);
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter >= distanceT && autoLoopCounter < distanceP) {// drive
																			// to
																			// batter
			LF.set(speedyspeed);
			RF.set(speedyspeed);
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter >= distanceP && autoLoopCounter <= shootinTime) {// Shooting
																				// in
																				// low
																				// goal
			LF.set(0);
			RF.set(0);
			shooter.set(-1);
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter >= shootinTime) {// done shooting
			shooter.set(0);
			return;
		} else {
			LF.set(0);
			RF.set(0);
			return;
		}
	}

	public void SpybotScore() { // Up up up (7)
		// move from corner as spybot and score in low goal
		solenoid0.set(true);
		double distanceN = 14; // loops until turn
		double distanceP = 46; // loops until batter
		double shootinTime = 126; // loops for shooting
		// double angle = gyro.getAngle();
		double angle = 0;
		double heynow = 45; // The angle we want divided by 2
		double speedyspeed = .9; // motor speed
		double Kp = .03; // How fast are we turning?
		
		if (autoLoopCounter < distanceN) {// Drive straight
			myRobot.arcadeDrive(-speedyspeed, -angle * Kp);
			// Is the motor inverted?
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter == distanceN && Math.abs(angle) < heynow) {// Turn
																		// right
			myRobot.arcadeDrive(-speedyspeed, -angle * Kp);// should he be going
			// straight? :P
			LF.set(-.5);// Test for inversion
			LR.set(-.5);
			RF.set(.2);
			RR.set(.2);
			return;
		}
		if (Math.abs(angle) >= heynow && autoLoopCounter == distanceN) {// Found
																		// angle
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter > distanceN && autoLoopCounter < distanceP) { // Go
																			// straight
																			// after
																			// turning
			myRobot.arcadeDrive(-speedyspeed, -(angle - 2 * heynow) * Kp);// stay
																			// straight
																			// relative
																			// to
																			// the
																			// new
																			// angle
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter == distanceP) { // We're there stop
			LF.set(0);
			RF.set(0);
			LR.set(0);
			RR.set(0);
			autoLoopCounter++;
			return;
		}
		if (autoLoopCounter > distanceP && autoLoopCounter < shootinTime) { // Shoot
			shooter.set(-1);
			autoLoopCounter++;
			return;// dont need this but it looks cool
		}
	}

	public void GyroTest() {// For testing purposes only
		solenoid0.set(true);
		double distanceN = 10; // LOOPS UNTIL TURN
		// UNCOMMENT FOR GYRO
		// double angle = gyro.getAngle();
		double angle = 0; // REMOVE
		double speedyspeed = .9; // MOTOR POWER
		double Kp = .03;

		if (autoLoopCounter < distanceN) {// DRIVE STRAIGT
			myRobot.arcadeDrive(-speedyspeed, -angle * Kp);
			autoLoopCounter++;
			return;
		} else {
			LF.set(0);
			RF.set(0);
		}
	}

	public void teleopInit() {
		/*
		 * frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		 * sessionfront = NIVision.IMAQdxOpenCamera("cam1",
		 * NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		 * sessionback = NIVision.IMAQdxOpenCamera("cam2",
		 * NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		 * currSession = sessionfront;
		 * NIVision.IMAQdxConfigureGrab(currSession);
		 */
		currCompressor = compressoron;
	}

	/**
	 * []\ This function is called periodically during operator control
	 */

	public void teleopPeriodic() {

		// BASIC DRIVE CONTROL - JOYSTICK
		double stickZ = stick.getRawAxis(2);
		double stickY = stick.getRawAxis(1); // 1 = y-axis
		double Z2norm = stickZ * (NORMSPEED / 100.0);
		double y2norm = stickY * (NORMSPEED / 100.0) + Math.signum(stickY) * 0.05;

		// TURNING SENSITIVITY IN HIGH SPEED
		if (stick.getRawButton(1)) {
			Z2norm = stickZ * (NORMSPEED / 125.0);
		} else {
			Z2norm = stickZ * (NORMSPEED / 100.0);
		}
		myRobot.arcadeDrive(y2norm, Z2norm, true);

		// BASIC DRIVE CONTROL - XBOX

		// HANGING MECHANISM
		double lefttrigger = xbox.getRawAxis(2); // left trigger
		boolean hangingextend = lefttrigger != 0; // boolean left trigger
		double righttrigger = xbox.getRawAxis(3);// right trigger
		boolean hangingpullin = righttrigger != 0;// boolean right trigger

		// Hanging extend and retract
		if (hangingextend || hangingpullin) {
			if (hangingextend) {// extend (left trigger)
				// medium speed
				Rope6.set(.5);
				Rope7.set(.5);
				Tape8.set(1);
				Tape9.set(1);
			}
			if (hangingpullin) {// pull in (right trigger)
				Rope6.set(-.5);
				Rope7.set(-.5);
				Tape8.set(-1);
				Tape9.set(-1);

			}
		} else { // no buttons, dont move
			Rope6.set(0);
			Rope7.set(0);
			Tape8.set(0);
			Tape9.set(0);
		}
		// Taughtening rope
		if (!hangingextend && !hangingpullin) { // pull in just rope
			if (xbox.getRawButton(6)) { // tighten rope (right bumper)
				Rope6.set(.75);
				Rope7.set(.75);
			}
			if (xbox.getRawButton(5)) { // loosen rope
										// (left bumper)
				Rope6.set(-.75);
				Rope7.set(-.75);
			}
		}

		// BALL INTAKE
		if (xbox.getRawButton(3) || xbox.getRawButton(4))

		{

			if (xbox.getRawButton(3)) {// intake
				shooter.set(1);
			}
			if (xbox.getRawButton(4)) {// outake
				shooter.set(-1);
			}
		} else

		{ // no button, stop motor
			shooter.set(0);
		}
		if (!limitswitch.get() && !xbox.getRawButton(3))

		{ // limit
			// switch
			// stops
			// intakes
			shooter.set(0);
		}

		// COMPRESSOR AND SOLENOID CONTROL
		if (currCompressor == compressoron)

		{ // compressor will automatically
			// begin filling up
			// when driver station is
			// enabled; should be done
			// BEFORE robot goes out on
			// field
			if (stick.getRawButton(1)) {
				solenoid0.set(false); // solenoid set "true" will
										// push piston in
				solenoid1.set(false); // solenoid set "true" will push piston in
			}

			else {
				solenoid0.set(true);// solenoid set "true" will retract piston
				solenoid1.set(true);// solenoid set "true" will retract piston
			}
		}

		// COMPRESSOR TOGGLE
		/*
		 * boolean compressoroffif = xbox.getRawButton(7) && currCompressor ==
		 * compressoron; boolean compressoronif = xbox.getRawButton(8) &&
		 * currCompressor == compressoroff; String compressorStat =
		 * "compressoron";
		 * 
		 * if (compressoroffif || currCompressor == compressoroff) {
		 * compressor.stop(); currCompressor = compressoroff; compressorStat =
		 * "compressoroff";
		 * 
		 * } if (compressoronif || currCompressor == compressoron) {
		 * compressor.start(); currCompressor = compressoron; compressorStat =
		 * "compressoron"; }
		 */

		// CAMERA TOGGLE
		if (stick.getRawButton(2))

		{
			/*
			 * if (currSession == sessionfront) {
			 * NIVision.IMAQdxStopAcquisition(currSession); currSession =
			 * sessionback; NIVision.IMAQdxConfigureGrab(currSession); }
			 * 
			 * else if (currSession == sessionback) {
			 * NIVision.IMAQdxStopAcquisition(currSession); currSession =
			 * sessionfront; NIVision.IMAQdxConfigureGrab(currSession); }
			 */
		}
		
		boolean hangingbackward = xbox.getRawButton(8);
		
		if (hangingbackward) {
			solenoid2.set(true);
		}
		
		/*
		 * NIVision.IMAQdxGrab(currSession, frame, 1);
		 * CameraServer.getInstance().setImage(frame);
		 */

		// SWITCHES
		double switRaw1 = theSwitch1.pidGet();
		double switRaw2 = theSwitch2.pidGet();
		double switRaw3 = theSwitch3.pidGet();

		int switNum1;
		int switNum2;
		int switNum3;

		if (switRaw1 > .006)

		{
			switNum1 = 1;
		} else

		{
			switNum1 = 0;
		}
		if (switRaw2 > .006)

		{
			switNum2 = 1;
		} else

		{
			switNum2 = 0;
		}
		if (switRaw3 > .006)

		{
			switNum3 = 1;
		} else

		{
			switNum3 = 0;
		}

		int switBinFin = ((switNum1 * 4) + (switNum2 * 2) + (switNum3));

		// LIMITSWITCH BALL DRIVER STATION DISPLAY

		/*
		 * NIVision.Rect oval = new NIVision.Rect(10, 10, 100, 100); if
		 * (limitswitch.get()){ NIVision.imaqDrawShapeOnImage(frame, frame,
		 * oval, DrawMode.PAINT_VALUE, ShapeMode.SHAPE_OVAL, 32767.0f); }
		 */

		// SMART DASHBOARD
		SmartDashboard.putNumber("Switch1 Binary", switNum1);
		SmartDashboard.putNumber("Switch2 Binary", switNum2);
		SmartDashboard.putNumber("Switch3 Binary", switNum3);
		SmartDashboard.putNumber("Switch1 Raw", switRaw1);
		SmartDashboard.putNumber("Switch2 Raw", switRaw2);
		SmartDashboard.putNumber("Switch3 Raw", switRaw3);
		SmartDashboard.putNumber("Binary Readout", switBinFin);
		SmartDashboard.putBoolean("Limit Switch", limitswitch.get());
		SmartDashboard.putBoolean("Compressor On", compressor.enabled());
		SmartDashboard.putNumber("On Compressor", compressoron);
		SmartDashboard.putNumber("Off Compressor", compressoroff);
		SmartDashboard.putNumber("Axes 2", lefttrigger);
		SmartDashboard.putNumber("Axes 3", righttrigger);
		SmartDashboard.putBoolean("hanging extend", hangingextend);
		SmartDashboard.putBoolean("hanging pull in", hangingpullin);
		SmartDashboard.putNumber("autoLoopCOunter", autoLoopCounter);
		SmartDashboard.putNumber("gyro", gyro.getAngle());
		SmartDashboard.putBoolean("solenoid 1", solenoid1.get());
		SmartDashboard.putBoolean("solenoid 0", solenoid0.get());
		

	}

	public Robot() {

		// CAMERA CONTROL
		server = CameraServer.getInstance();
		server.setQuality(50);
		server.startAutomaticCapture("cam1");
	}

	public void testPeriodic() {
	}
}