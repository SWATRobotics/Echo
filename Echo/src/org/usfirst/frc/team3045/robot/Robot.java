/**
 * @author FRC Team 3045
 * @version 1.0
 */

package org.usfirst.frc.team3045.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.*;;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
Notes for Enson, 9/30/2018

Talons 6 and 7 are the LEFT DRIVE TALONS
Talons 3 and 5 are the RIGHT DRIVE TALONS

There's a method specifically named TankDrive that you can change to add more talons if need be

*/

public class Robot extends IterativeRobot {
	
	// Talons are set to specific IDs that can be found on the roborio web browser interface (roborio-3045-frc.local) in Internet Explorer (Microsoft Edge, View in Internet Explorer)
	// The drive talons are 5-6 / 7-3
	// Practice bot is 4-1 / 3-5
	// intake: 0-1
	// winch: 4-2
	public WPI_TalonSRX talon0 = new WPI_TalonSRX(0);
	public WPI_TalonSRX talon1 = new WPI_TalonSRX(1);
	public WPI_TalonSRX talon2 = new WPI_TalonSRX(2);
	public WPI_TalonSRX talon3 = new WPI_TalonSRX(3);
	public WPI_TalonSRX talon4 = new WPI_TalonSRX(4);
	public WPI_TalonSRX talon5 = new WPI_TalonSRX(5);
	public WPI_TalonSRX talon6 = new WPI_TalonSRX(6);
	public WPI_TalonSRX talon7 = new WPI_TalonSRX(7);
	
	// This is only for driving the robot with the control sticks. Auton and other stuff just writes to the talons directly
	public MecanumDrive driveTrain = new MecanumDrive(talon5, talon4, talon1, talon0);
	
	// Auton values
	public String gameData = new String();
	public char closeSide;
	public char scale;
	public char farSide;
	
	//Control stuff
	public String stickIndicator = new String();
	
	//Cameras
	public UsbCamera cam1;
	//public UsbCamera cam2 = new UsbCamera("Climb Camera", 1);
	
	public CameraServer camServer = CameraServer.getInstance();	
	
	//Timer to deal with waiting stuff
	public Timer timer = new Timer();
	
	// Joysticks are done similarly to talons, with IDs being based off of the Driver Station instead
	public Joystick gamepad1 = new Joystick(0);
	public Joystick stick2 = new Joystick(1);
	public Compressor cp = new Compressor();
	//public DoubleSolenoid ClawSolenoid = new DoubleSolenoid(0, 0, 1);
	public Solenoid ScaleSolenoid = new Solenoid(0,0);
	public Solenoid IntakeSolenoid = new Solenoid(0,1);
	public DoubleSolenoid FlipperIn = new DoubleSolenoid(0,4,5);
	public DoubleSolenoid FlipperOut = new DoubleSolenoid(0,6,7);
	public DriverStation driverStation = DriverStation.getInstance();
	public SmartDashboard shuffleBoard = new SmartDashboard();
	public RobotController robotControl;
	
	public Solenoid PushSolenoid = new Solenoid(0,3);
	
	//
	public String side = "Left"; // USE THIS FOR AUTON SIDE!
	public boolean isShooting = false; // USE THIS IF WE ARE SHOOTING OR DRIVING FORWARD
	//

	//random value by enson
	
	public void robotInit() {
		
		StopRobot();
		FlipperOut.set(Value.kForward);
		FlipperIn.set(Value.kReverse);
		
		cam1 = new UsbCamera("CAMERA", 0);
		
		camServer.addCamera(cam1);
		camServer.startAutomaticCapture(cam1);
		//camServer.addCamera(cam2);
		
		// Limits the output current of each talon
		/*
		talon0.configPeakCurrentLimit(20, 5);
		talon0.enableCurrentLimit(true);
		talon1.configPeakCurrentLimit(20, 5);
		talon1.enableCurrentLimit(true);
		talon6.configPeakCurrentLimit(20, 5);
		talon6.enableCurrentLimit(true);
		talon7.configPeakCurrentLimit(20, 5);
		talon7.enableCurrentLimit(true);
		*/
		
		talon0.overrideLimitSwitchesEnable(false);
		talon1.overrideLimitSwitchesEnable(false);
		talon2.overrideLimitSwitchesEnable(false);
		talon3.overrideLimitSwitchesEnable(false);
		talon4.overrideLimitSwitchesEnable(false);
		talon5.overrideLimitSwitchesEnable(false);
		talon6.overrideLimitSwitchesEnable(false);
		talon7.overrideLimitSwitchesEnable(false);
		
		
	}
	/*
	public void robotPeriodic() {
		talon0.valueUpdated();
		talon1.valueUpdated();
		talon2.valueUpdated();
		talon3.valueUpdated();
		talon4.valueUpdated();
		talon5.valueUpdated();
		talon6.valueUpdated();
		talon7.valueUpdated();
	}
	*/
	
	public void StopRobot() {
		// Stops each robot
		talon6.set(ControlMode.PercentOutput, 0);
		talon7.set(ControlMode.PercentOutput, 0);
		talon3.set(ControlMode.PercentOutput, 0);
		talon5.set(ControlMode.PercentOutput, 0);
	}
	
	public void autonomousInit() {
		cp.start();
		gameData = driverStation.getGameSpecificMessage();
		closeSide = gameData.charAt(0);
		scale = gameData.charAt(1);
		farSide = gameData.charAt(2);		
	}
	
	public void disabledPeriodic() {
		StopRobot();
	}
	
	public void TankDrive() {
		double leftInput = 0.70 * gamepad1.getY();
		double rightInput = 0.70 * stick2.getY();
		talon6.set(ControlMode.PercentOutput, leftInput); // Left motor
		talon7.set(ControlMode.PercentOutput, leftInput); // Left motor
		talon3.set(ControlMode.PercentOutput, rightInput); // Right motor
		talon5.set(ControlMode.PercentOutput, rightInput); // Right motor
	}

	////////////////////////////////
	
	public void autonomousPeriodic() {
		
		//
		timer.reset();
		timer.start();
		while (timer.get() < 5) {
			StopRobot();
		}
		//
		timer.reset();
		timer.start();
		while (timer.get() < 3) {
			System.out.println("Time:" + timer.get());
			talon6.set(ControlMode.PercentOutput, -.3);
			talon7.set(ControlMode.PercentOutput, -.3);
			talon3.set(ControlMode.PercentOutput, -.3);
			talon5.set(ControlMode.PercentOutput, -.3);		}
		StopRobot();
		timer.reset();
		timer.start();
		while (timer.get() < 13) {
			StopRobot();
		}
		 
		 
	}
	
	///////////////////////////////
	
	public void teleopInit() {
		cp.start();
		
		camServer.startAutomaticCapture(cam1);
	}
	
	public void DriveTest() {
		int dpadInput = gamepad1.getPOV(0);
		if (!gamepad1.getRawButton(1)) {
		switch (dpadInput) {
		case 0:
			talon0.set(ControlMode.PercentOutput,.2);
			System.out.println("Moving Talon 0");
			break;
		case 45:
			talon1.set(ControlMode.PercentOutput,.2); // BACK RIGHT -- THIS ONE IS BACKWARDS
			System.out.println("Moving Talon 1");
			break;
		case 90:
			talon2.set(ControlMode.PercentOutput,.2); // BACK LEFT
			System.out.println("Moving Talon 2");
			break;
		case 135:
			talon3.set(ControlMode.PercentOutput,.2); // FRONT RIGHT
			System.out.println("Moving Talon 3");
			break;
		case 180:
			talon4.set(ControlMode.PercentOutput,.2); // 
			System.out.println("Moving Talon 4");
			break;
		case 225:
			talon5.set(ControlMode.PercentOutput,.2); // FRONT LEFT
			System.out.println("Moving Talon 5");
			break;
		case 270:
			talon6.set(ControlMode.PercentOutput,.2); //
			System.out.println("Moving Talon 6");
			break;
		case 315:
			talon7.set(ControlMode.PercentOutput,.2);
			System.out.println("Moving Talon 7");
			break;
		}
		} else {
			StopRobot();	
		}
	}
	
	public void Drive() {
		//changed talon6 and talon3 to talon1 and talon0
		// Gets the amount that the triggers on the controller are being pressed down
		double lTrigger = Math.pow(gamepad1.getRawAxis(2),2); // Squaring these values will allow for more fine movements at low speeds while still retaining max speed
		double rTrigger = Math.pow(gamepad1.getRawAxis(3),2);
		
		// Input of the D-Pad as an ANGLE, in degrees, which 0 being straight forward
		int dpadInput = gamepad1.getPOV(0);
		//double throttleStick = gamepad1.getRawAxis(5);
		if (dpadInput == 180 ) {
			// BACKWARD
			talon1.set(ControlMode.PercentOutput,1);
			talon0.set(ControlMode.PercentOutput,1);
			talon5.set(ControlMode.PercentOutput,-1);
			talon4.set(ControlMode.PercentOutput,-1);	
		} else if (dpadInput == 0) {
			// FORWARD
			talon1.set(ControlMode.PercentOutput, -1);
			talon0.set(ControlMode.PercentOutput, -1);
			talon5.set(ControlMode.PercentOutput, 1);
			talon4.set(ControlMode.PercentOutput, 1);
		} else if (dpadInput == 90) {
			// Bottom right and top left wheel go full
			talon1.set(ControlMode.PercentOutput, 1);
			talon0.set(ControlMode.PercentOutput, -1);
			talon5.set(ControlMode.PercentOutput, 1);
			talon4.set(ControlMode.PercentOutput, -1);
		} else if (dpadInput == 270) {
			// Bottom right and top left wheel go full
			talon1.set(ControlMode.PercentOutput, -1);
			talon0.set(ControlMode.PercentOutput, 1);
			talon5.set(ControlMode.PercentOutput, -1);
			talon4.set(ControlMode.PercentOutput, 1);
		} else if (dpadInput == 45) {
			// Bottom right and top left wheel go full, rest 0
			talon1.set(ControlMode.PercentOutput, 0);
			talon0.set(ControlMode.PercentOutput, -1);
			talon5.set(ControlMode.PercentOutput, 1);
			talon4.set(ControlMode.PercentOutput, 0);
		} else if (dpadInput == 315) {
			// Bottom left and top right wheel go full, rest 0
			talon1.set(ControlMode.PercentOutput, -1);
			talon0.set(ControlMode.PercentOutput, 0);
			talon5.set(ControlMode.PercentOutput, 0);
			talon4.set(ControlMode.PercentOutput, 1);
		} else if (dpadInput == 225) {
			// Bottom right and top left wheel go full -, rest 0
			talon1.set(ControlMode.PercentOutput, 0);
			talon0.set(ControlMode.PercentOutput, 1);
			talon5.set(ControlMode.PercentOutput, -1);
			talon4.set(ControlMode.PercentOutput, 0);
		} else if (dpadInput == 135) {
			// Bottom left and top right wheel go full -, rest 0
			talon1.set(ControlMode.PercentOutput, 1);
			talon0.set(ControlMode.PercentOutput, 0);
			talon5.set(ControlMode.PercentOutput, 0);
			talon4.set(ControlMode.PercentOutput, -1);
		} else if (lTrigger > 0) { // Turning
			talon1.set(ControlMode.PercentOutput, -lTrigger);
			talon0.set(ControlMode.PercentOutput, -lTrigger);
			talon5.set(ControlMode.PercentOutput, -lTrigger);
			talon4.set(ControlMode.PercentOutput, -lTrigger);
		} else if (rTrigger > 0) {
			talon1.set(ControlMode.PercentOutput, rTrigger);
			talon0.set(ControlMode.PercentOutput, rTrigger);
			talon5.set(ControlMode.PercentOutput, rTrigger);
			talon4.set(ControlMode.PercentOutput, rTrigger);
		} else {
			StopRobot();
		}
		
		
	}
	
	public void ClawControl() {
		boolean aButton = gamepad1.getRawButton(1);
		//boolean lBumper = gamepad1.getRawButton(5); // Left bumper is dead on one of the controllers, and I'd prefer not to use it
		if (aButton) {
			IntakeSolenoid.set(true);
		} else {
			IntakeSolenoid.set(false);
		}
	}
	
	public void IntakeMotors() {
		
		//talon0.configPeakCurrentLimit(50, 5);
		//talon0.enableCurrentLimit(true);
		//talon1.configPeakCurrentLimit(50, 5);
		//talon1.enableCurrentLimit(true);
		
		boolean bButton = gamepad1.getRawButton(2);
		boolean yButton = gamepad1.getRawButton(4);
		if (yButton && !bButton) {
			talon2.set(ControlMode.PercentOutput, -.5);
			talon6.set(ControlMode.PercentOutput, -.5);
		} else if (!yButton && bButton) {
			talon2.set(ControlMode.PercentOutput, 1);
			talon6.set(ControlMode.PercentOutput, 1);
		} else {
			talon2.set(ControlMode.PercentOutput, 0);
			talon6.set(ControlMode.PercentOutput, 0);
		}
	}
	
	public void Flipper() {
		/*
		boolean bButton = gamepad1.getRawButton(2);
		boolean xButton = gamepad1.getRawButton(3);
		boolean yButton = gamepad1.getRawButton(4);
		*/
		boolean backButton = gamepad1.getRawButton(3);
		if (backButton) {
			FlipperOut.set(Value.kForward);
			FlipperIn.set(Value.kForward);
		} else {
			FlipperOut.set(Value.kReverse);
			FlipperIn.set(Value.kReverse);
		}
	}
	
	public void Winch() {
		//boolean xButton = gamepad1.getRawButton(2);
		//boolean yButton = gamepad1.getRawButton(3);
		//boolean rBumper = gamepad1.getRawButton(6);
		//boolean lBumper = gamepad1.getRawButton(5);
		//double add = 0.1;
		//double rTrigger = Math.pow(gamepad1.getRawAxis(3),1);
		double throttle = -1 * gamepad1.getRawAxis(5);
		//double throttleFunc = Math.pow((.5-throttle)+.5, 3);
		talon3.set(ControlMode.PercentOutput, throttle);	
	}
	
	public void ScaleClaw() {
		boolean fButton = gamepad1.getRawButton(8);
		if (fButton) {
			ScaleSolenoid.set(true);
		} else {
			ScaleSolenoid.set(false);
		}
	}
	
	public void PushThing() {
		boolean pushButton = gamepad1.getRawButton(3); // MAKE THIS THE BUTTON YOU WANT TO PUSH THE THINGY
		if (pushButton) {
			PushSolenoid.set(true);
		} else {
			PushSolenoid.set(false);
		}
	}
	
	public void StickControl() {
		// Gets the input of each stick axis individually. At rest, both sticks sit at (0.5,0.5)
		//talon0.setInverted(true);
		
		
				double LstickX = gamepad1.getRawAxis(0);
				double LstickY = -1 * gamepad1.getRawAxis(1);
				double RstickX = gamepad1.getRawAxis(4);
				double RstickY = gamepad1.getRawAxis(5);
				
				// The angle and quadrant of the left stick position is determined. This is used to determine which wheels to run in which direction to allow direct circular movement
				String quadrant = new String();
				double xInput = -2*(LstickX-.5);
				double yInput = -2*(LstickY-.5);
				double rotInput = 2*(RstickX)-.5;
				double phi = Math.atan2(yInput, xInput);
				phi = phi + (Math.PI);
				phi = Math.toDegrees(phi);
				double mag = Math.sqrt(Math.pow(LstickX, 2) + Math.pow(LstickY, 2));
				double rho = Math.pow(mag,1);
				
				//System.out.println("Drive speed: " + rho);
				//System.out.println("Vector magnitude: " + mag);
				System.out.println("X-axis: " + LstickX);
				System.out.println("Y-axis: " + LstickY);
				//System.out.println("Angle: " + phi);
				
				// Phi is the angle the stick input makes with the immediate right vector, from pi to -pi
				// Rho is the magnitude of the vector created by the stick input, and governs how fast the robot will move.
				
				
				
				//driveTrain.drivePolar(rho, phi, 0); //2*(RstickX-.5)
				//driveTrain.driveCartesian(LstickX, LstickY, 0);
				
	}
	
	// This is a function that gets data from the robot and sends it to the Shuffleboard layout.
	public void dataCentre() {
		String switchData = driverStation.getGameSpecificMessage();
		SmartDashboard.putString("Field Layout", switchData);
		boolean browningOut = RobotController.isBrownedOut();
		SmartDashboard.putBoolean("Brownout Indicator", browningOut);
		double[] inputArray = {talon6.getMotorOutputPercent(), talon7.getMotorOutputPercent(), talon3.getMotorOutputPercent(), talon5.getMotorOutputPercent()};
		SmartDashboard.putNumberArray("Drive Train Input", inputArray);
		double batteryVolts = RobotController.getBatteryVoltage();
		SmartDashboard.putNumber("Battery Voltage", batteryVolts);
	}

	public void teleopPeriodic() {
		//Drive();
		//Flipper();
		//Winch();
		//StickControl();
		//PushThing();
		//ClawControl();
		//dataCentre();
		//ScaleClaw();
		//IntakeMotors();
		//TankDrive();
	}

	public void testInit() {
		cp.start();
	}
	
	// Test lets you run each individual talon using the D-pad.
	public void testPeriodic() {
		//DriveTest();
		dataCentre();
		//IntakeMotors();
		StickControl();
		//Drive();
	}
}

