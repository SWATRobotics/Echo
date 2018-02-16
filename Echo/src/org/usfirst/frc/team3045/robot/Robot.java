/**
 * @author FRC Team 3045
 * @version 1.0
 */

package org.usfirst.frc.team3045.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.*;;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Robot extends IterativeRobot {
	
	// Talons are set to specific IDs that can be found on the roborio web browser interface (roborio-3045-frc.local) in Internet Explorer (Microsoft Edge, View in Internet Explorer)
	// The drive talons are 6-7 / 3-5
	public TalonSRX talon0 = new TalonSRX(0);
	public TalonSRX talon1 = new TalonSRX(1);
	public TalonSRX talon2 = new TalonSRX(2);
	public TalonSRX talon3 = new TalonSRX(3);
	public TalonSRX talon4 = new TalonSRX(4);
	public TalonSRX talon5 = new TalonSRX(5);
	public TalonSRX talon6 = new TalonSRX(6);
	public TalonSRX talon7 = new TalonSRX(7);
	
	// Joysticks are done similarly to talons, with IDs being based off of the Driver Station instead
	public Joystick gamepad1 = new Joystick(0);
	public Compressor cp = new Compressor();
	public DoubleSolenoid ClawSolenoid = new DoubleSolenoid(0, 0, 1);
	public DoubleSolenoid flipper = new DoubleSolenoid(0, 2, 3);
	public DriverStation driverStation = DriverStation.getInstance();
	
	// gameData is a string that contains the correct alliance sides for the close switch, scale, and far switch, in that order
	public String gameData;
	//
	public String side = "Left"; // USE THIS FOR AUTON SIDE!
	public boolean isShooting = true; // USE THIS IF WE ARE SHOOTING OR DRIVING FORWARD OR NOT
	//

	public void robotInit() {
		
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
		
		/* Auton stuff
		//gameData = driverStation.getGameSpecificMessage();
		// SmartDashboard GUI stuff for telling the driver which switch side is which color
		
		SmartDashboard.putString("closeSwitch", "Close Switch: " + gameData.charAt(0));
		SmartDashboard.putString("scale","Scale: " + gameData.charAt(1));
		SmartDashboard.putString("farSwitch",String.format("Far Switch: %c" , gameData.charAt(2)));
		
		
		
		if (isShooting) {
			if (gameData.charAt(0) == 'L') { // Left side is ours
				if (side.equals("Left") {
					//Go forward
				} else if (side.equals("Right")) {
					// Go diagonally up-right
				} else if (sided.equals("Middle"))
					// Go straight-left/straight right
				}
			} else if (gameData.charAt(0) == 'R') { // Right side is ours
				if (isLeft) {
					//Go diagonally up-right
				} else {
					// Go forward
				}
			}
			Timer.delay(3); // Amount of seconds it takes to reach the switch (speed of wheel in revs/minute // circumference of wheel)?
			StopRobot();
			// Drop cube off in switch
		} else {
			// Drive forward
			Timer.delay(3); // Amount of seconds it takes to reach the switch (speed of wheel in revs/minute // circumference of wheel)?
			StopRobot();
		}
		*/
	}
	
	public void disabledPeriodic() {
		StopRobot();
	}

	public void autonomousPeriodic() {
		
	}
	
	public void teleopInit() {
		cp.start();
	}
	
	public void DriveTest() {
		int dpadInput = gamepad1.getPOV(0);
		if (!gamepad1.getRawButton(1)) {
		switch (dpadInput) {
		case 0:
			talon0.set(ControlMode.PercentOutput,.2);
			break;
		case 45:
			talon1.set(ControlMode.PercentOutput,.2); // BACK RIGHT -- THIS ONE IS BACKWARDS
			break;
		case 90:
			talon2.set(ControlMode.PercentOutput,.2); // BACK LEFT
			break;
		case 135:
			talon3.set(ControlMode.PercentOutput,.2); // FRONT RIGHT
			break;
		case 180:
			talon4.set(ControlMode.PercentOutput,.2); // 
			break;
		case 225:
			talon5.set(ControlMode.PercentOutput,.2); // FRONT LEFT
			break;
		case 270:
			talon6.set(ControlMode.PercentOutput,.2); // 
			break;
		case 315:
			talon7.set(ControlMode.PercentOutput,.2);
			break;
		}
		} else {
			StopRobot();	
		}
	}
	
	public void Drive() {
		
		// Gets the amount that the triggers on the controller are being pressed down
		double lTrigger = Math.pow(gamepad1.getRawAxis(2),2); // Squaring these values will allow for more fine movements at low speeds while still retaining max speed
		double rTrigger = Math.pow(gamepad1.getRawAxis(3),2);
		
		// Input of the D-Pad as an ANGLE, in degrees, which 0 being straight right
		int dpadInput = gamepad1.getPOV(0);
		if (dpadInput == 180 ) {
			// BACKWARD
			talon6.set(ControlMode.PercentOutput,-1);
			talon7.set(ControlMode.PercentOutput,-1);
			talon5.set(ControlMode.PercentOutput,-1);
			talon3.set(ControlMode.PercentOutput,-1);	
		} else if (dpadInput == 0) {
			// FORWARD
			talon6.set(ControlMode.PercentOutput, 1);
			talon7.set(ControlMode.PercentOutput, 1);
			talon5.set(ControlMode.PercentOutput, 1);
			talon3.set(ControlMode.PercentOutput, 1);
		} else if (dpadInput == 90) {
			// Bottom right and top left wheel go full
			talon6.set(ControlMode.PercentOutput, 1);
			talon7.set(ControlMode.PercentOutput, -1);
			talon5.set(ControlMode.PercentOutput, 1);
			talon3.set(ControlMode.PercentOutput, -1);
		} else if (dpadInput == 270) {
			// Bottom right and top left wheel go full
			talon6.set(ControlMode.PercentOutput, -1);
			talon7.set(ControlMode.PercentOutput, 1);
			talon5.set(ControlMode.PercentOutput, -1);
			talon3.set(ControlMode.PercentOutput, 1);
		} else if (dpadInput == 45) {
			// Bottom right and top left wheel go full, rest 0
			talon6.set(ControlMode.PercentOutput, 1);
			talon7.set(ControlMode.PercentOutput, 0);
			talon5.set(ControlMode.PercentOutput, 1);
			talon3.set(ControlMode.PercentOutput, 0);
		} else if (dpadInput == 315) {
			// Bottom left and top right wheel go full, rest 0
			talon6.set(ControlMode.PercentOutput, 0);
			talon7.set(ControlMode.PercentOutput, 1);
			talon5.set(ControlMode.PercentOutput, 0);
			talon3.set(ControlMode.PercentOutput, 1);
		} else if (dpadInput == 225) {
			// Bottom right and top left wheel go full -, rest 0
			talon6.set(ControlMode.PercentOutput, -1);
			talon7.set(ControlMode.PercentOutput, 0);
			talon5.set(ControlMode.PercentOutput, -1);
			talon3.set(ControlMode.PercentOutput, 0);
		} else if (dpadInput == 135) {
			// Bottom left and top right wheel go full -, rest 0
			talon6.set(ControlMode.PercentOutput, 0);
			talon7.set(ControlMode.PercentOutput, -1);
			talon5.set(ControlMode.PercentOutput, 0);
			talon3.set(ControlMode.PercentOutput, -1);
		} else if (lTrigger > 0) { // Turning
			talon6.set(ControlMode.PercentOutput, -lTrigger);
			talon7.set(ControlMode.PercentOutput, lTrigger);
			talon3.set(ControlMode.PercentOutput, -lTrigger);
			talon5.set(ControlMode.PercentOutput, lTrigger);
		} else if (rTrigger > 0) {
			talon6.set(ControlMode.PercentOutput, rTrigger);
			talon7.set(ControlMode.PercentOutput, -rTrigger);
			talon3.set(ControlMode.PercentOutput, rTrigger);
			talon5.set(ControlMode.PercentOutput, -rTrigger);
		} else {
			StopRobot();
		}
		
		
	}
	
	public void ClawControl() {
		boolean aButton = gamepad1.getRawButton(1);
		//boolean lBumper = gamepad1.getRawButton(5); // Left bumper is dead on one of the controllers, and I'd prefer not to use it
		if (aButton) {
			ClawSolenoid.set(DoubleSolenoid.Value.kForward);
		} else {
			ClawSolenoid.set(DoubleSolenoid.Value.kReverse);
		}
	}
	
	public void Flipper() {
		/*
		boolean bButton = gamepad1.getRawButton(2);
		boolean xButton = gamepad1.getRawButton(3);
		boolean yButton = gamepad1.getRawButton(4);
		*/
		boolean rBumper = gamepad1.getRawButton(6);
		if (rBumper) {
			flipper.set(DoubleSolenoid.Value.kForward);
		} else {
			flipper.set(DoubleSolenoid.Value.kReverse);
		}
	}
	
	public void Winch() {
		boolean xButton = gamepad1.getRawButton(3);
		if (xButton) {
			talon0.set(ControlMode.PercentOutput, 1);
			talon1.set(ControlMode.PercentOutput, 1);
		} else {
			talon0.set(ControlMode.PercentOutput, 0);
			talon1.set(ControlMode.PercentOutput, 0);
		}
	}
	
	public void StickControl() {
		// Gets the input of each stick axis individually. At rest, both sticks sit at (0.5,0.5)
				double LstickX = gamepad1.getRawAxis(0);
				double LstickY = gamepad1.getRawAxis(1);
				double RstickX = gamepad1.getRawAxis(4);
				double RstickY = gamepad1.getRawAxis(5);
				
				// The angle and quadrant of the left stick position is determined. This is used to determine which wheels to run in which direction to allow direct circular movement
				String quadrant = new String();
				double xInput = 2*(LstickX-.5);
				double yInput = -2*(LstickY-.5);
				double phi = Math.atan2(yInput, xInput);
				double rho = Math.pow((Math.hypot(xInput, yInput)),2);
				
				// Phi is the angle the stick input makes with the immediate right vector, from pi to -pi
				// Rho is the magnitude of the vector created by the stick input, and governs how fast the robot will move.
				
				// Multidimensional array to hold the values that will run each wheel
				double[][] drivePoints = new double[2][2];
				
				if (xInput != 0 && yInput != 0) {
					if (phi > 0 && phi < Math.PI/2) {
						quadrant = "S1";
					} else if (phi >= Math.PI/2 && phi < Math.PI) {
						quadrant = "S2";
					} else if (phi <= -Math.PI/2 && phi > -Math.PI) {
						quadrant = "S3";
						phi = (2*Math.PI) + phi;
					} else if (phi <= 0 && phi > -Math.PI/2) {
						quadrant = "S4";
						phi = (2*Math.PI) + phi;
					}
				} else if (xInput == 0 && yInput != 0) {
					quadrant = "NaN";
					drivePoints[0][0] = -1*yInput;
					drivePoints[0][1] = 1*yInput;
					drivePoints[1][0] = drivePoints[0][1];
					drivePoints[1][1] = drivePoints[0][0];
				} else if (xInput != 0 && yInput == 0) {
					quadrant = "NaN";
					drivePoints[0][0] = 1*xInput;
					drivePoints[0][1] = -1*xInput;
					drivePoints[1][0] = drivePoints[0][1];
					drivePoints[1][1] = drivePoints[0][0];
				} else if (xInput == 0 && yInput == 0) {
					quadrant = "NaN";
					StopRobot();
				}
				
				
				switch (quadrant) {
				case "S1":
					drivePoints[0][0] = rho;
					drivePoints[0][1] = rho* (4/Math.PI)*(phi-(Math.PI/4));
					drivePoints[1][0] = drivePoints[0][1];
					drivePoints[1][1] = drivePoints[0][0];
					break;
				case "S2":
					drivePoints[0][0] = rho* -(4/Math.PI)*(phi-(3*Math.PI/4));
					drivePoints[0][1] = rho;
					drivePoints[1][0] = drivePoints[0][1];
					drivePoints[1][1] = drivePoints[0][0];
					break;
				case "S3":
					drivePoints[0][0] = -rho;
					drivePoints[0][1] = rho* -(4/Math.PI)*(phi-(5*Math.PI/4));
					drivePoints[1][0] = drivePoints[0][1];
					drivePoints[1][1] = drivePoints[0][0];
					break;
				case "S4":
					drivePoints[0][1] = rho* (4/Math.PI)*(phi-(7*Math.PI/4));
					drivePoints[0][0] = -rho;
					drivePoints[1][0] = drivePoints[0][1];
					drivePoints[1][1] = drivePoints[0][0];
					break;
				case "NaN":
					break;
				default:
					System.out.println("Stopping robot 'cause no input");
					StopRobot();
					break;
				}
				
				// Top left motor, same as Bottom Right
				talon6.set(ControlMode.PercentOutput, drivePoints[0][0]);
				// Top right motor, same as Bottom Left
				talon7.set(ControlMode.PercentOutput, drivePoints[0][1]);
				// Bottom left motor, same as Top Right
				talon3.set(ControlMode.PercentOutput, drivePoints[1][0]);
				// Bottom right motor, same as Top Left
				talon5.set(ControlMode.PercentOutput, drivePoints[1][1]);
	}

	public void teleopPeriodic() {
		Drive();
		Flipper();
		Winch();
		//StickControl();
		//ClawControl();
		//DriveTest();
	}

	public void testInit() {
	
		
	}
	
	public void testPeriodic() {
		talon0.set(ControlMode.PercentOutput, 1);
		talon1.set(ControlMode.PercentOutput, 1);
		talon2.set(ControlMode.PercentOutput, 1);
		talon3.set(ControlMode.PercentOutput, 1);
		talon4.set(ControlMode.PercentOutput, 1);
		talon5.set(ControlMode.PercentOutput, 1);
		talon6.set(ControlMode.PercentOutput, 1);
		talon7.set(ControlMode.PercentOutput, 1);
	}
}

