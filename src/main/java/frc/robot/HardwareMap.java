package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_DRIVE_FRONT_RIGHT = 1;
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 2;
	public static final int CAN_ID_SPARK_DRIVE_FRONT_LEFT = 3;
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 4;
	public static final int CAN_ID_ARM = 5;
	public static final int CAN_ID_SPINNER_MOTOR = 40;

	// Pneumatics channel numbers
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_FORWARD = 1;
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_REVERSE = 2;

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_SPINNING_INTAKE_CHANNEL = 9;
	private static final int DIO_ELEVATOR_ARM_CHANNEL = 8;
	private static final int DIO_WRIST_CHANNEL = 7;

	private static DigitalInput testBoardPin = new DigitalInput(HardwareMap.DIO_SPINNING_INTAKE_CHANNEL);
	private static DigitalInput elevatorArmPin = new DigitalInput(HardwareMap.DIO_ELEVATOR_ARM_CHANNEL);
	private static DigitalInput wristPin = new DigitalInput(HardwareMap.DIO_WRIST_CHANNEL);

	/**
	 * Check if the current RoboRIO has spinning intake disabled
	 * @return true if the spinning intake is disabled
	 */
	public static boolean isSpinningIntakeDisabled() {
		return !HardwareMap.testBoardPin.get();
	}

	/**
	 * Check if the current RoboRIO has elevator arm disabled
	 * @return true if the elevtor arm is disabled
	 */
	public static boolean isElevatorArmDisabled() {
		return !HardwareMap.elevatorArmPin.get();
	}

	/**
	 * Check if the current RoboRIO has wrist disabled
	 * @return true if the wrist is disabled
	 */
	public static boolean isWristDisabled() {
		return !HardwareMap.wristPin.get();
	}
}
