package org.usfirst.frc.team3045.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon;


public class Robot extends IterativeRobot {
	
	public CANTalon talon0 = new CANTalon(0);
	public CANTalon talon1 = new CANTalon(1);
	public CANTalon talon6 = new CANTalon(6);
	public CANTalon talon7 = new CANTalon(7);
	
	public Joystick gamepad1 = new Joystick(0);
	public Compressor cp = new Compressor();
	public DoubleSolenoid ClawSolenoid = new DoubleSolenoid(0, 0, 1);
	public DoubleSolenoid flipper = new DoubleSolenoid(0, 2, 3);
	public DriverStation driverStation = DriverStation.getInstance();
	
	public String gameData;
	//
	public boolean isLeft = false; // USE THIS FOR AUTON SIDE!
	public boolean isShooting = true; // USE THIS IF WE ARE SHOOTING OR DRIVING FORWARD OR NOT
	//

	public void robotInit() {
		talon0.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		talon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		talon6.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		talon7.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		
		talon0.setCurrentLimit(20);
		talon0.EnableCurrentLimit(true);
		talon1.setCurrentLimit(20);
		talon1.EnableCurrentLimit(true);
		talon6.setCurrentLimit(20);
		talon6.EnableCurrentLimit(true);
		talon7.setCurrentLimit(20);
		talon7.EnableCurrentLimit(true);
	}
	
	public void StopRobot() {
		talon6.set(0);
		talon1.set(0);
		talon0.set(0);
		talon7.set(0);
	}

	
	public void autonomousInit() {
		cp.start();
		
		
		//gameData = driverStation.getGameSpecificMessage();
		// SmartDashboard GUI stuff for telling the driver which switch side is which color
		SmartDashboard.putString("closeSwitch", "Close Switch: " + gameData.charAt(0));
		SmartDashboard.putString("scale","Scale: " + gameData.charAt(1));
		SmartDashboard.putString("farSwitch",String.format("Far Switch: %c" , gameData.charAt(2)));
		
		
		if (isShooting) {
			if (gameData.charAt(0) == 'L') { // Left side is ours
				if (isLeft) {
					//Go forward
				} else {
					// Go diagonally up-right
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
	}

	public void autonomousPeriodic() {
		
	}
	
	public void teleopInit() {
		cp.start();
		talon0.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		talon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		talon6.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		talon7.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}
	
	public void Drive() {
		
		double lTrigger = Math.pow(gamepad1.getRawAxis(2),2); // Squaring these values will allow for more fine movements at low speeds while still retaining max speed
		double rTrigger = Math.pow(gamepad1.getRawAxis(3),2);
		
		
		int dpadInput = gamepad1.getPOV(0);
		if (dpadInput == 270 ) {
			// Top right and bottom left wheel go full
			talon6.set(1);
			talon1.set(-1);
			talon0.set(-1);
			talon7.set(1);	
		} else if (dpadInput == 90) {
			// Bottom right and top left wheel go full
			talon6.set(-1);
			talon1.set(1);
			talon0.set(1);
			talon7.set(-1);
		} else if (dpadInput == 0) {
			// Bottom right and top left wheel go full
			talon6.set(1);
			talon1.set(-1);
			talon0.set(1);
			talon7.set(-1);
		} else if (dpadInput == 180) {
			// Bottom right and top left wheel go full
			talon6.set(-1);
			talon1.set(1);
			talon0.set(-1);
			talon7.set(-1);
		} else if (lTrigger > 0) {
			talon6.set(lTrigger);
			talon1.set(lTrigger);
			talon0.set(lTrigger);
			talon7.set(lTrigger);
		} else if (rTrigger > 0) {
			talon6.set(-rTrigger);
			talon1.set(-rTrigger);
			talon0.set(-rTrigger);
			talon7.set(-rTrigger);
		} else {
			StopRobot();
		}
		
		/*
		double LaxisX = gamepad1.getRawAxis(0);
		double LaxisY = gamepad1.getRawAxis(1);
		double RaxisX = gamepad1.getRawAxis(4);
		double RaxisY = gamepad1.getRawAxis(5);
		*/
		
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

	public void teleopPeriodic() {
		Drive();
		Flipper();
		ClawControl();
	}

	public void testInit() {
		
	}
	public void testPeriodic() {
		talon6.set(1);
		Timer.delay(2);
		StopRobot();
		talon7.set(1);
		Timer.delay(2);
		StopRobot();
		talon1.set(1);
		Timer.delay(2);
		StopRobot();
		talon0.set(2);
		Timer.delay(2);
		StopRobot();
	}
}

