package frc.robot.systems;
// WPILib Imports
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Robot;


// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class SpinningIntakeFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum SpinningIntakeFSMState {
		START_STATE,
		IDLE_SPINNING,
		IDLE_STOP,
		RELEASE
	}
	// Distance definitions
	public enum ItemType {
		CUBE,
		CONE,
		EMPTY
	}
	//FIX VALUES
	private static final double KEEP_SPEED = 0.07;
	private static final double INTAKE_SPEED = 0.5;
	private static final double RELEASE_SPEED = -1; //DONT FORGET -
	private static final double RELEASE_SPEED_LOW = -0.3;
	private static final double CURRENT_THRESHOLD = 20;
	private static final double TIME_RESET_CURRENT = 0.5;
	private static final int MIN_RELEASE_DISTANCE = 800;
	private static final int AVERAGE_SIZE = 10;
	private static final double OVERRUN_THRESHOLD = 0.007;
	//variable for armFSM, 0 means no object, 1 means cone, 2 means cube
	private static ItemType itemType = ItemType.EMPTY;
	private boolean isMotorAllowed = false;
	private boolean toggleUpdate = true;
	private boolean needsReset = true;
	private int tick = 0;
	private boolean hasTimerStarted = false;
	private double[] currLogs = new double[AVERAGE_SIZE];


	/* ======================== Private variables ======================== */
	private SpinningIntakeFSMState currentState;
	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax spinnerMotor;
	private Timer timer;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public SpinningIntakeFSM() {
		timer = new Timer();
		spinnerMotor = new CANSparkMax(HardwareMap.CAN_ID_SPINNER_MOTOR,
				CANSparkMax.MotorType.kBrushless);
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public SpinningIntakeFSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = SpinningIntakeFSMState.START_STATE;
		hasTimerStarted = false;
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			return;
		}
		//Robot.getStringLog().append("spinning intake start in " + currentState.toString());
		double begin = Timer.getFPGATimestamp();
		/*if (input.isDisableUpdatedPressed()) {
			toggleUpdate = !toggleUpdate;
			SmartDashboard.putBoolean("Is update enabled", toggleUpdate);
		}*/
		if (input.isThrottleForward()) {
			itemType = ItemType.CUBE;
		} else {
			itemType = ItemType.CONE;
		}
		if (toggleUpdate) {
			SmartDashboard.putNumber("output current", spinnerMotor.getOutputCurrent());
			SmartDashboard.putString("spinning intake state", currentState.toString());
			SmartDashboard.putNumber("velocity", spinnerMotor.getEncoder().getVelocity());
			SmartDashboard.putString("item type", itemType.toString());
			SmartDashboard.putNumber("spinner power", spinnerMotor.get());
			if (input.isIntakeButtonPressed()) {
				isMotorAllowed = !isMotorAllowed;
				spinnerMotor.set(0);
			}
			switch (currentState) {
				case START_STATE:
					handleStartState();
					break;
				case IDLE_SPINNING:
					handleIdleSpinningState();
					break;
				case IDLE_STOP:
					handleIdleStopState();
					break;
				case RELEASE:
					handleReleaseState(input);
					break;
				default:
					throw new IllegalStateException("Invalid state: " + currentState.toString());
			}
			SpinningIntakeFSMState previousState = currentState;

			double currentTime = Timer.getFPGATimestamp();
			double timeTaken = currentTime - begin;
			if (timeTaken > OVERRUN_THRESHOLD) {
				System.out.println("ALERT ALERT SPINNING INTAKE HANDLER " + timeTaken);
				System.out.println("intake state" + currentState);
			}
			begin = Timer.getFPGATimestamp();

			currentState = nextState(input);

			currentTime = Timer.getFPGATimestamp();
			timeTaken = currentTime - begin;
			if (timeTaken > OVERRUN_THRESHOLD) {
				System.out.println("ALERT ALERT SPINNING INTAKE nextState " + timeTaken);
				System.out.println("intake state" + currentState);
			}
		} else {
			SmartDashboard.putBoolean("disabled", true);
		}
		double currentTime = Timer.getFPGATimestamp();
		double timeTaken = currentTime - begin;
		//Robot.getStringLog().append("spinning intake ending");
		//Robot.getStringLog().append("Time taken for loop: " + timeTaken);
	}
	/**
	 * Run given state and return if state is complete.
	 * @param state SpinningIntakeFSMState state gives the state that the intakefsm is in
	 * @return Boolean that returns if given state is complete
	 */
	public boolean updateAutonomous(SpinningIntakeFSMState state) {
		isMotorAllowed = true;
		switch (state) {
			case START_STATE:
				handleStartState();
				break;
			case IDLE_SPINNING:
				handleIdleSpinningState();
				break;
			case IDLE_STOP:
				handleIdleStopState();
				break;
			case RELEASE:
				handleReleaseState(null);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + state.toString());
		}
		switch (state) {
			case START_STATE:
				return true;
			case IDLE_SPINNING:
				return true;
			case IDLE_STOP:
				return true;
			case RELEASE:
				return timer.hasElapsed(1);
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/*-------------------------NON HANDLER METHODS ------------------------- */
	/**
	 * Returns the type of object currently in the grabber.
	 * @return int 0 1 or 2 for nothing, cone, cube
	 */
	public static ItemType getObjectType() {
		return itemType;
	}
	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private SpinningIntakeFSMState nextState(TeleopInput input) {
		if (input == null) {
			return SpinningIntakeFSMState.START_STATE;
		}
		switch (currentState) {
			case START_STATE:
				return SpinningIntakeFSMState.IDLE_SPINNING;
			case IDLE_SPINNING:
				if (input.isReleaseButtonPressed()) {
					return SpinningIntakeFSMState.RELEASE;
				}
				if (needsReset && isMotorAllowed && toggleUpdate) {
					timer.reset();
					timer.start();
					needsReset = false;
				}
				if (timer.hasElapsed(TIME_RESET_CURRENT)) {
					currLogs[tick % AVERAGE_SIZE] = spinnerMotor.getOutputCurrent();
					tick++;

					double avg = 0;
					for (int i = 0; i < AVERAGE_SIZE; i++) {
						avg += currLogs[i];
					}
					avg /= AVERAGE_SIZE;

					if (avg > CURRENT_THRESHOLD) {
						return SpinningIntakeFSMState.IDLE_STOP;
					}
				}
				return SpinningIntakeFSMState.IDLE_SPINNING;
			case IDLE_STOP:
				if (input.isReleaseButtonPressed()) {
					return SpinningIntakeFSMState.RELEASE;
				}
				return SpinningIntakeFSMState.IDLE_STOP;
			case RELEASE:
				if (!input.isReleaseButtonPressed()) {
					needsReset = true;
					return SpinningIntakeFSMState.IDLE_SPINNING;
				}
				return SpinningIntakeFSMState.RELEASE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 */
	private void handleStartState() {

	}
	private void handleIdleSpinningState() {
		if (isMotorAllowed) {
			spinnerMotor.set(INTAKE_SPEED);
		}
	}
	private void handleIdleStopState() {
		spinnerMotor.set(KEEP_SPEED);
	}
	private void handleReleaseState(TeleopInput input) {
		if (input == null) {
			if (!hasTimerStarted) {
				timer.reset();
				timer.start();
				hasTimerStarted = true;
			}
		}
		for (int i = 0; i < AVERAGE_SIZE; i++) {
			currLogs[i] = 0;
		}
		if (input != null) {
			if (itemType == ItemType.CUBE) {
				spinnerMotor.set(RELEASE_SPEED);
			} else {
				spinnerMotor.set(RELEASE_SPEED_LOW);
			}
		} else {
			/*if (AutoPathChooser.getSelectedNode() == 0) {
				spinnerMotor.set(RELEASE_SPEED_LOW);
			} else {
				spinnerMotor.set(RELEASE_SPEED);
			}*/
		}
		itemType = ItemType.EMPTY;
		isMotorAllowed = true;
	}
}