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
	CANTalon LF = new CANTalon(1);
	CANTalon LR = new CANTalon(2);
	CANTalon RF = new CANTalon(3);
	CANTalon RR = new CANTalon(4);
	CANTalon shooter = new CANTalon(5);
	CANTalon Tape1 = new CANTalon(6);
	CANTalon Tape2 = new CANTalon(7);
	CameraServer server;
	Compressor compressor = new Compressor(0);
	Solenoid solenoid0 = new Solenoid(0, 0);
	Solenoid solenoid1 = new Solenoid(1, 1);
	int autoLoopCounter;
	int NORMSPEED = 100; // forward and backward
	int Timer;
	// ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	// TEST

	// CAMERA SWITCHING
	int currSession;
	int sessionfront;
	int sessionback;
	Image frame;

	// HANGING
	int lowspeedextend;
	int highspeedextend;
	int lowspeedpullin;
	int highspeedpullin;
	int currspeed;

	AnalogInput theSwitch1 = new AnalogInput(0);
	AnalogInput theSwitch2 = new AnalogInput(1);
	AnalogInput theSwitch3 = new AnalogInput(2);

	int[] switArray = new int[4];

	// BALL INTAKE/SHOOTER
	DigitalInput limitswitch = new DigitalInput(4);

	// COMPRESSOR
	int currCompressor;
	int compressoron;
	int compressoroff;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	public void robotInit() {

		// Robot Drive
		myRobot = new RobotDrive(LF, LR, RF, RR);
		stick = new Joystick(0); // joystick
		xbox = new Joystick(1); // xbox controller

		// Live Window
		LiveWindow.addActuator("Talon", "Talon Right Front", RF);
		LiveWindow.addActuator("Talon", "Talon Right Rear", RR);
		LiveWindow.addActuator("Talon", "Talon Left Front", LF);
		LiveWindow.addActuator("Talon", "Talon Left Rear", LR);
		LiveWindow.addActuator("Talon", "Tape Talon AKA Talon 6", Tape1);
		LiveWindow.addActuator("Shifting", "Solenoid", solenoid0);
		LiveWindow.addActuator("Intake", "Talon 5", shooter);
		LiveWindow.addActuator("Things that compress", "Compressor", compressor);

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
		/*
		 * gyro.reset(); gyro.calibrate();
		 */

		// COMPRESSOR
		currCompressor = compressoron;

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */

	public void autonomousInit() {
		autoLoopCounter = 0;

	}

	/**
	 * This function is called periodically during autonomous
	 */

	public void autonomousPeriodic() {
		// BINARY SWITCH READING
		double switRaw1 = theSwitch1.pidGet();
		double switRaw2 = theSwitch2.pidGet();
		double switRaw3 = theSwitch3.pidGet();
		int switNum1;
		int switNum2;
		int switNum3;
		if (switRaw1 > .006) {
			switNum1 = 1;
		} else {
			switNum1 = 0;
		}
		if (switRaw2 > .006) {
			switNum2 = 1;
		} else {
			switNum2 = 0;
		}
		if (switRaw3 > .006) {
			switNum3 = 1;
		} else {
			switNum3 = 0;
		}
		int switBinFin = ((switNum1 * 4) + (switNum2 * 2) + (switNum3));
		/*
		 * NIVision.IMAQdxGrab(currSession, frame, 1);
		 * CameraServer.getInstance().setImage(frame);
		 */
		autoLoopCounter = 0;
		double Kp = 0.03;

		/* double angle = gyro.getAngle(); */

		if (autoLoopCounter < 350) {
			RF.set(.5);
			RR.set(.5);
			LF.set(.5);
			LR.set(.5);
			autoLoopCounter++;
		}

		/*
		 * if (Math.abs(angle) < 90 && autoLoopCounter >= 350); { LF.set(-1);
		 * LR.set(-1); RF.set(1); RR.set(1); }
		 * 
		 * if (Math.abs(angle)>= 90) { RF.set(0); RR.set(0); LF.set(0);
		 * LR.set(0); }
		 * 
		 * if (Math.abs(angle) >= 90 && autoLoopCounter >= 350); { RF.set(.5);
		 * RR.set(.5); LF.set(.5); LR.set(.5); }
		 */

		if (autoLoopCounter == 600)
			RF.set(0);
		RR.set(0);
		LF.set(0);
		LR.set(0);
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */

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
		}

		else {
			Z2norm = stickZ * (NORMSPEED / 100.0);
		}

		myRobot.arcadeDrive(y2norm, Z2norm, true);

		// BASIC DRIVE CONTROL - XBOX

		// HANGING MECHANISM
		double lefttrigger = xbox.getRawAxis(2);
		boolean hangingextend = lefttrigger != 0;
		double righttrigger = xbox.getRawAxis(3);
		boolean hangingspeed = righttrigger != 0;

		/*
		 * if (hangingextend) { Tape1.set(.5); Tape2.set(.5); } if (hangingspeed
		 * && hangingextend) ; { Tape1.set(1); Tape2.set(1); }
		 * 
		 * if (xbox.getRawButton(1)) { Tape1.set(-.25); Tape2.set(-.25);
		 * currspeed = lowspeedpullin; } if (xbox.getRawButton(1) &&
		 * hangingspeed) ; { Tape1.set(-.5); Tape2.set(-.5); } if
		 * (xbox.getRawButton(2)) { Tape1.set(0); Tape2.set(0); }
		 */

		// BALL INTAKE
		/*
		 * double shooteraxis = xbox.getRawAxis(1); boolean ballintake =
		 * shooteraxis > 0 || shooteraxis < 0; if (ballintake) {
		 * shooter.set(shooteraxis); }
		 */

		/*
		 * boolean shootOnFor = xbox.getRawButton(7); boolean shootOnBa =
		 * xbox.getRawButton(8);
		 */
		if (xbox.getRawButton(3) || xbox.getRawButton(4)) {

			if (xbox.getRawButton(3)) {
				shooter.set(1);
			}
			if (xbox.getRawButton(4)) {
				shooter.set(-1);
			}
		} else {
			shooter.set(0);
		}
		// COMPRESSOR AND SOLENOID CONTROL
		if (currCompressor == compressoron) {
			// compressor will automatically begin filling
			// up
			// when driver station is enabled; should be
			// done
			// BEFORE robot goes out on field
			if (stick.getRawButton(1)) {
				solenoid0.set(false); // solenoid set "true" will push piston in
				solenoid1.set(false); // solenoid set "true" will push piston in
			}

			else {
				solenoid0.set(true);// solenoid set "true" will retract piston
				solenoid1.set(true);// solenoid set "true" will retract piston
			}
		}

		boolean compressoroffif = xbox.getRawButton(7) && currCompressor == compressoron;
		boolean compressoronif = xbox.getRawButton(8) && currCompressor == compressoroff;
		String compressorStat = "compressoron";

		if (compressoroffif || currCompressor == compressoroff) {
			compressor.stop();
			currCompressor = compressoroff;
			compressorStat = "compressoroff";
			

		} 
		if (compressoronif || currCompressor == compressoron) {
			compressor.start();
			currCompressor = compressoron;
			compressorStat = "compressoron";
		}

		// CAMERA CONTROL
		if (stick.getRawButton(2)) {
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
		/*
		 * NIVision.IMAQdxGrab(currSession, frame, 1);
		 * CameraServer.getInstance().setImage(frame);
		 */
		double switRaw1 = theSwitch1.pidGet();
		double switRaw2 = theSwitch2.pidGet();
		double switRaw3 = theSwitch3.pidGet();

		int switNum1;
		int switNum2;
		int switNum3;

		if (switRaw1 > .006) {
			switNum1 = 1;
		} else {
			switNum1 = 0;
		}
		if (switRaw2 > .006) {
			switNum2 = 1;
		} else {
			switNum2 = 0;
		}
		if (switRaw3 > .006) {
			switNum3 = 1;
		} else {
			switNum3 = 0;
		}
		int switBinFin = ((switNum1 * 4) + (switNum2 * 2) + (switNum3));

		if (!limitswitch.get() && !xbox.getRawButton(3)) {
			shooter.set(0);
		}

		boolean limitBool = limitswitch.get();

		/*
		 * SmartDashboard.putNumber("Switch1", switRaw1);
		 * SmartDashboard.putNumber("Switch2", switRaw2);
		 * SmartDashboard.putNumber("Switch3", switRaw3);
		 */
		SmartDashboard.putNumber("Switch1 Binary", switNum1);
		SmartDashboard.putNumber("Switch2 Binary", switNum2);
		SmartDashboard.putNumber("Switch3 Binary", switNum3);
		SmartDashboard.putNumber("Binary Readout", switBinFin);
		SmartDashboard.putBoolean("Limit Switch", limitBool);
		SmartDashboard.putBoolean("Compressor On", compressor.enabled());
		SmartDashboard.putString("Current Compressor", compressorStat);
		SmartDashboard.putNumber("On Compressor", compressoron);
		SmartDashboard.putNumber("Off Compressor", compressoroff);

		/*
		 * NIVision.Rect rect = new NIVision.Rect(10,10,100,100); if
		 * (!limitswitch.get()) { NIVision.imaqDrawShapeOnImage(frame, frame,
		 * rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f); } else {
		 * NIVision.imaqDrawShapeOnImage(frame, frame, rect,
		 * DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 1.1f); }
		 */
	}

	public Robot() {

		// CAMERA CONTROL
		/*
		 * server = CameraServer.getInstance(); server.setQuality(50); // the
		 */ // camera name (ex "cam0") can be found through the roborio web
			// interface server.startAutomaticCapture("cam0");
	}

	/**
	 * This function is called periodically during test mode
	 */

	public void testPeriodic() {
	}
}