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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.DriverStation;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	Joystick xbox;
	CANTalon LF = new CANTalon(1); // THIS IS INVERTED ON BROBOT OK HAVE A NICE
									// DAY :D
	CANTalon LR = new CANTalon(2); // Comment put for practice
	CANTalon RF = new CANTalon(3);
	CANTalon RR = new CANTalon(4); // Comment out for practice
	CANTalon Tape = new CANTalon(5);
	CANTalon Pizzaz = new CANTalon(1, 3);
	CameraServer server;
	Button hanging = new JoystickButton(xbox, 7);
	Button hanging1 = new JoystickButton(xbox, 8);
	Compressor compressor = new Compressor(0);
	Solenoid solenoid0 = new Solenoid(0, 0);
	Solenoid solenoid1 = new Solenoid(1, 1);
	int autoLoopCounter;
	int NORMSPEED = 100; // forward and backward
	int Timer;
	public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	PIDController pidgyro = new PIDController(1, 0, 5, gyro, Pizzaz);

	// !!SWITCHES!!
	int autoProgram = 1;
	int whichbot = 1;
	// Brobot:1 Spock:2

	// CAMERA SWITCHING

	int currSession;
	int sessionfront;
	int sessionback;
	Image frame;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	public void robotInit() {
		// Robot Drive
		switch (whichbot) {
		case 1:// Brobot
			myRobot = new RobotDrive(LF, RF);
			gyro.reset();
			gyro.calibrate();
			break;
		case 2:// Spock
			myRobot = new RobotDrive(LF, LR, RF, RR);
			break;
		}
		stick = new Joystick(0); // joystick
		xbox = new Joystick(1); // xbox controller

		// Live Window
		switch (whichbot) {
		case 1:// Brobot
			LiveWindow.addActuator("Talon", "Talon Right Front", RF);
			LiveWindow.addActuator("Talooon", "Talon Left Front", LF);
			LiveWindow.addActuator("Shifting", "Solenoid0", solenoid0);
			LiveWindow.addActuator("Shifting", "Solenoid1", solenoid1);
			LiveWindow.addSensor("Heyo", "Hi", gyro);
			break;

		case 2:// Spock
			LiveWindow.addActuator("Talon", "Talon Right Front", RF);
			LiveWindow.addActuator("Talon", "Talon Right Rear", RR);
			LiveWindow.addActuator("Talon", "Talon Left Front", LF);
			LiveWindow.addActuator("Talon", "Talon Left Rear", LR);
			LiveWindow.addActuator("Talon", "Tape Talon AKA Talon 5", Tape);
			LiveWindow.addActuator("Shifting", "Solenoid", solenoid0);
			// LiveWindow.addActuator("Shifting", "Solenoid",solenoid);
			// LiveWindow.addActuator("Shifting", "Compressor",compressor);
			break;
		}
		LiveWindow.addSensor("Gyro", 1, gyro);
		// LiveWindow.addActuator("Joystick", "HEYO", (LiveWindowSendable) new

		/*
		 * XBOX PORTS FOR JOYSTICKS LeftXaxis - 1 LeftYaxis - 2 RightXaxis - 4
		 * RightYaxis - 5 DPad Left/Right - 6
		 * 
		 * XBOX PORTS FOR BUTTONS X - 1 A - 2 B - 3 Y - 4 LB - z-axis
		 */

		// CAMERA CONTROLS
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		sessionfront = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		sessionback = NIVision.IMAQdxOpenCamera("cam1", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		currSession = sessionfront;
		NIVision.IMAQdxConfigureGrab(currSession);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */

	public void autonomousInit() {
		autoLoopCounter = 0;
		pidgyro.enable();
		pidgyro.setContinuous(true);
		pidgyro.setPercentTolerance(3);
		pidgyro.setOutputRange(-.8, .8);
		pidgyro.reset();
		double currentangle = gyro.getAngle();
		gyro.reset();
		gyro.calibrate();
	}

	/**
	 * This function is called periodically during autonomous
	 */

	public void autonomousPeriodic() {// PROGRAMS WILL NEED TO BE MODIFIED FOR
										// SPOCK
		/*
		 * NIVision.IMAQdxGrab(currSession, frame, 1);
		 * CameraServer.getInstance().setImage(frame);
		 */
		switch (autoProgram) {
		case 1:// Testing for gyroscope
				// CURRENTLY, THE ROBOT WILL JUST CONTINUE STRAIGHT WITHOUT
				// STOPPING
				// AND SEEMS TO WAVER A BIT
			double n = 350;
		
			if (autoLoopCounter < n) {// Drive forward
				double Kp = 0.03;
				double angle = gyro.getAngle();
				myRobot.drive(-0.5, -angle * Kp);
				autoLoopCounter++;
			}
			
			double startAngle = gyro.getAngle();
			double test = gyro.getAngle();

			if (Math.abs(test) < 90 && autoLoopCounter >= n) {// TURN
				test = gyro.getAngle();
				myRobot.arcadeDrive(0, -.5);
			}
			if (Math.abs(test) >= 90 && autoLoopCounter >= n) {// STOP TURNING
				myRobot.arcadeDrive(0, 0);
				double precise = gyro.getAngle();
				autoLoopCounter++;
			}
			if (autoLoopCounter < 900 && autoLoopCounter > n) {// Drive forward
				double Kp = 0.03;
				double angle = gyro.getAngle() + 90;
				myRobot.drive(-0.5, -angle * Kp);
				autoLoopCounter++;
			}

			/*
			 * if (gyro.getAngle() < 90 && autoLoopCounter < 100) {
			 * myRobot.arcadeDrive(0, -1);// JOANIE AND MRSOLEY WERE HERE :P
			 * autoLoopCounter++; } else { RF.set(0); LF.set(0);
			 */
	
			/*
			 * myRobot.drive (-0.5, -90.0 + gyro.getAngle()); double setpoint =
			 * -90.0; pidgyro.setSetpoint(setpoint);
			 */

			/* } */
			
			/*
			 * else if (autoLoopCounter >= 50 && pidgyro.onTarget()) {// angle
			 * found LF.set(0); RF.set(0);
			 * 
			 * }
			 * 
			 * else if (autoLoopCounter >= 50 && pidgyro.onTarget() == false){//
			 * angle not found RF.set(.5); LF.set(-.5);
			 */
			/*
			 * try { Thread.sleep(2000); } catch (InterruptedException ex) {
			 * Thread.currentThread().interrupt(); } }
			 */

			break;
		case 2:// Ideal Autonomous (still need to implement distances and
				// shooting)
			autoLoopCounter = 0;
			double Kp = 0.03;
			double angle = gyro.getAngle();

			if (gyro.getAngle() < 90 && autoLoopCounter < 100) {
				RF.set(.5);
				LF.set(.5);
				autoLoopCounter++;
			} else {
				RF.set(0);
				LF.set(0);
			}
			// Still need to stop and shoot
			break;
		default:// Ideal Autonomous (redundancy in case of an error)
			if (autoLoopCounter < 100) {// Drive over low bar (distance not
				// implemented)
				myRobot.drive(-0.5, 0.0);
				autoLoopCounter++;
			}
			if (gyro.getAngle() != 120) {// turn
				LF.set(.5);
				RF.set(-.5);
			}
			if (gyro.getAngle() >= 120) {// stop turning
				RF.set(1);
				LF.set(1);
				autoLoopCounter++;
			}
			if (autoLoopCounter > 100) {// Drive to batter
				RF.set(1);
				LF.set(1);
				// Still need to stop and shoot
			}
			break;
		}

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */

	public void teleopInit() {
		Tape.set(0);
	}

	/**
	 * []\ This function is called periodically during operator control
	 */

	public void teleopPeriodic() {
		// BASIC DRIVE CONTROL - JOYSTICK
		double stickZ = 0;
		double stickY = stick.getRawAxis(1); // 1 = y-axis
		double Z2norm = 0;
		double y2norm = stickY * (NORMSPEED / 100.0) + Math.signum(stickY) * 0.05;

		// VARIABLES FOR DRIFTING AND TURNING RANGE RESTRICTIONS
		boolean zaxis = stick.getRawAxis(2) > 0 || stick.getRawAxis(2) < 0;
		boolean speedcontrol = stick.getRawButton(1);
		boolean ydrift = stick.getRawAxis(1) < .5 && stick.getRawAxis(1) > -.5;
		// sets range in which robot will not move

		// DRIVER CONTROLS (CHANGED IN SWITCHES AT TOP)

		// TURNING CONTROL
		stickZ = stick.getRawAxis(2);

		// TURNING SENSITIVITY IN HIGH SPEED
		if (stick.getRawButton(1)) {
			Z2norm = stickZ * (NORMSPEED / 125.0);
		} else {
			Z2norm = stickZ * (NORMSPEED / 100.0);
		}
		// DRIFTING CONTROL
		/*
		 * if (ydrift) { LF.set(0); RF.set(0); }
		 */
		myRobot.arcadeDrive(y2norm, Z2norm, true);

		// BASIC DRIVE CONTROL - XBOX CONTROLLER
		double xboxX = xbox.getRawAxis(1); // xboxX equals the value of the
		// x-axis
		double xboxY = xbox.getRawAxis(2); // xboxY equals the value of the
		// y-axis

		// insert motor name for ball intake.set(xboxY); //ball intake will
		// respond to the left xbox joystick, but only to values on the
		// x-axis, any side to side movement will not be read

		/*
		 * if (xbox.getRawButton(7) && xbox.getRawButton(8)) { //hanging
		 * mechanism will only work if the back AND //start buttons are pressed
		 * 
		 * insert motor name for hanging mechanism.set(1); }
		 */

		// COMPRESSOR AND SOLENOID CONTROL
		compressor.start(); // compressor will automatically begin filling up
		// when driver station
		// is enabled; should be done BEFORE robot goes out on field

		switch (whichbot) {
		case 1:// Brobot
			if (speedcontrol) { // when trigger is pulled, solenoid will
								// activate
				solenoid0.set(true); // solenoid set "true" will push piston in
										// (high speed)
			} else {
				solenoid0.set(false);// solenoid set "true" will retract piston
										// (low
				// speed)
			}
			break;
		case 2:// Spock
			if (speedcontrol) {
				solenoid0.set(false); // solenoid set "true" will push piston in
										// (high
				// speed)
				solenoid1.set(false); // solenoid set "true" will push piston in
										// (high
				// speed)
			} else {
				solenoid0.set(true);// solenoid set "true" will retract piston
									// (low
				// speed)
				solenoid1.set(true);// solenoid set "true" will retract piston
									// (low
				// speed)
			}
			break;
		}
		/*
		 * if (stick.getRawButton(3)){ LF.set(0); LR.set(0); } if
		 * (stick.getRawButton(4)){ RF.set(0); RR.set(0); }
		 */
		// HANGING
		/*
		 * if (stick.getRawButton(6)) { Tape.set(.5); } else if
		 * (stick.getRawButton(4)) { Tape.set(-.5); } else if
		 * (stick.getRawButton(5)) { Tape.set(.25); } else if
		 * (stick.getRawButton(3)) { Tape.set(-.25); } else { Tape.set(0); }
		 */

		SmartDashboard.putNumber("Joystick", stick.getRawAxis(3));
		// CAMERA CONTROL
		if (stick.getRawButton(2)) {
			if (currSession == sessionfront) {
				NIVision.IMAQdxStopAcquisition(currSession);
				currSession = sessionback;
				NIVision.IMAQdxConfigureGrab(currSession);
			} else if (currSession == sessionback) {
				NIVision.IMAQdxStopAcquisition(currSession);
				currSession = sessionfront;
				NIVision.IMAQdxConfigureGrab(currSession);
			}
		}
		NIVision.IMAQdxGrab(currSession, frame, 1);
		CameraServer.getInstance().setImage(frame);
	}

	public Robot() {
		// CAMERA CONTROL
		server = CameraServer.getInstance();
		server.setQuality(50); // the
		// camera name (ex "cam0") can be found through the roborio web //
		// interface server.startAutomaticCapture("cam0");

	}

	/**
	 * 
	 * This function is called periodically during test mode
	 * 
	 */
	public void testPeriodic() {
		LiveWindow.run();
		SmartDashboard.putData("PID tune", pidgyro);
		SmartDashboard.putNumber("Gyro angle", gyro.getAngle());

	}
}
