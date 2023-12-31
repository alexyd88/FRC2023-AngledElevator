package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int LEFT_JOYSTICK_PORT = 0;
	private static final int RIGHT_JOYSTICK_PORT = 1;
	private static final int ELEVATOR_HIGH = 7;
	private static final int ELEVATOR_MID = 9;
	private static final int ELEVATOR_LOW = 11;
	private static final int ELEVATOR_UP = 3;
	private static final int ELEVATOR_DOWN = 5;
	private static final int WRIST_UP = 6;
	private static final int WRIST_DOWN = 4;
	private static final int FINE_TUNING_BUTTON = 2;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private Joystick rightJoystick;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);

		rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickX() {
		return leftJoystick.getX();
	}

	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickY() {
		return leftJoystick.getY();
	}

	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(10);
	}

	/**
	 * Get the value of the high button.
	 * @return True if button is pressed
	 */
	public boolean isHighButtonPressed() {
		return leftJoystick.getRawButton(ELEVATOR_HIGH);
	}

	/**
	 * Get the value of the mid button.
	 * @return True if button is pressed
	 */
	public boolean isMidButtonPressed() {
		return leftJoystick.getRawButton(ELEVATOR_MID);
	}

	/**
	 * Get the value of the low button.
	 * @return True if button is pressed
	 */
	public boolean isLowButtonPressed() {
		return leftJoystick.getRawButton(ELEVATOR_LOW);
	}

	/**
	 * Get the value of the elevator up button.
	 * @return True if button is pressed
	 */
	public boolean isElevatorUpButtonPressed() {
		return leftJoystick.getRawButton(ELEVATOR_UP);
	}

	/**
	 * Get the value of the elevator down button.
	 * @return True if button is pressed
	 */
	public boolean isElevatorDownButtonPressed() {
		return leftJoystick.getRawButton(ELEVATOR_DOWN);
	}

	/**
	 * Get the value of the release button.
	 * @return True if button is pressed
	 */
	public boolean isReleaseButtonPressed() {
		return leftJoystick.getTriggerPressed();
	}

	/**
	 * Get the value of the wrist down button.
	 * @return True if button is pressed
	 */
	public boolean isWristDownButtonPressed() {
		return leftJoystick.getRawButton(WRIST_DOWN);
	}

	/**
	 * Get the value of the wrist up button.
	 * @return True if button is pressed
	 */
	public boolean isWristUpButtonPressed() {
		return leftJoystick.getRawButton(WRIST_UP);
	}

	/**
	 * Get the value of the fine tuning button.
	 * @return True if button is pressed
	 */
	public boolean isFineTuningButtonPressed() {
		return leftJoystick.getRawButton(FINE_TUNING_BUTTON);
	}
	
	/**
	 * Get the value of the throttle.
	 * @return True throttle is forward
	 */
	public boolean isThrottleForward() {
		return leftJoystick.getThrottle() <= 0;
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return rightJoystick.getX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}

	/* ======================== Private methods ======================== */

}
