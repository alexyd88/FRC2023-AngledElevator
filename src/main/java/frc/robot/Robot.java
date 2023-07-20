// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;

// Systems
import frc.robot.systems.ElevatorArmFSM;
import frc.robot.HardwareMap;
import frc.robot.systems.SpinningIntakeFSM;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	private ElevatorArmFSM fsmSystem;
	private SpinningIntakeFSM spinningIntake;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		// Instantiate all systems here
		if (!HardwareMap.isElevatorArmDisabled()) {
			fsmSystem = new ElevatorArmFSM();
		}
		if (!HardwareMap.isSpinningIntakeDisabled()) {
			spinningIntake = new SpinningIntakeFSM();
		}
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		if (!HardwareMap.isElevatorArmDisabled()) {
			fsmSystem.reset();
		}
		if (!HardwareMap.isSpinningIntakeDisabled()) {
			spinningIntake.reset();
		}
	}

	@Override
	public void autonomousPeriodic() {
		if (!HardwareMap.isElevatorArmDisabled()) {
			fsmSystem.update(null);
		}
		if (!HardwareMap.isSpinningIntakeDisabled()) {
			spinningIntake.update(null);
		}
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		if (!HardwareMap.isElevatorArmDisabled()) {
			fsmSystem.reset();
		}
		if (!HardwareMap.isSpinningIntakeDisabled()) {
			spinningIntake.reset();
		}
	}

	@Override
	public void teleopPeriodic() {
		if (!HardwareMap.isElevatorArmDisabled()) {
			fsmSystem.update(input);
		}
		if (!HardwareMap.isSpinningIntakeDisabled()) {
			spinningIntake.update(input);
		}
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}
