package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import net.thefletcher.revrobotics.enums.SparkMaxLimitSwitchType;
import frc.robot.HardwareMap;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorArmFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		HIGH,
		MIDDLE,
		LOW,
		UP,
		DOWN
	}

	private static final float UP_POWER = 0.1f;
	private static final float DOWN_POWER = 0.1f;
	private static final double PID_CONSTANT_ARM_P = 0.00000001;
	private static final double PID_CONSTANT_ARM_I = 0.00000001;
	private static final double PID_CONSTANT_ARM_D = 0.00000001;
	private static final float MAX_UP_POWER = 0.2f;
	private static final float MAX_DOWN_POWER = -0.2f;
	// arbitrary encoder amounts
	private static final float LOW_ENCODER_ROTATIONS = -5;
	private static final float MID_ENCODER_ROTATIONS = 50;
	private static final float HIGH_ENCODER_ROTATIONS = 100;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax armMotor;
	private SparkMaxPIDController pidControllerArm;
	private SparkMaxLimitSwitch limitSwitchHigh;
	private SparkMaxLimitSwitch limitSwitchLow;
	private double currentEncoder;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ElevatorArmFSM() {
		// Perform hardware init
		armMotor = new CANSparkMax(HardwareMap.CAN_ID_ARM,
										CANSparkMax.MotorType.kBrushless);
		armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		limitSwitchHigh = armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
		limitSwitchLow = armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
		limitSwitchHigh.enableLimitSwitch(true);
		limitSwitchLow.enableLimitSwitch(true);
		pidControllerArm = armMotor.getPIDController();
		pidControllerArm.setP(PID_CONSTANT_ARM_P);
		pidControllerArm.setI(PID_CONSTANT_ARM_I);
		pidControllerArm.setD(PID_CONSTANT_ARM_D);
		pidControllerArm.setOutputRange(MAX_DOWN_POWER, MAX_UP_POWER);
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
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
		currentState = FSMState.IDLE;

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
		if (currentState != FSMState.IDLE) {
			currentEncoder = armMotor.getEncoder().getPosition();
		}
		switch (currentState) {
			case IDLE:
				handleIdleState(input);
				break;
			case HIGH:
				handleHighState(input);
				break;
			case MIDDLE:
				handleMiddleState(input);
				break;
			case LOW:
				handleLowState(input);
				break;
			case UP:
				handleUpState(input);
				break;
			case DOWN:
				handleDownState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
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
	private FSMState nextState(TeleopInput input) {
		if (input == null) {
			return FSMState.IDLE;
		}
		switch (currentState) {
			case IDLE:
				if (input.isHighButtonPressed()) {
					return FSMState.HIGH;
				} else if (input.isMidButtonPressed()) {
					return FSMState.MIDDLE;
				} else if (input.isLowButtonPressed()) {
					return FSMState.LOW;
				} else if (input.isUpButtonPressed()) {
					return FSMState.UP;
				} else if (input.isDownButtonPressed()) {
					return FSMState.DOWN;
				}
			case HIGH:
				if (input.isHighButtonPressed()) {
					return FSMState.HIGH;
				} else {
					return FSMState.IDLE;
				}
			case MIDDLE:
				if (input.isMidButtonPressed()) {
					return FSMState.MIDDLE;
				} else {
					return FSMState.IDLE;
				}
			case LOW:
				if (input.isLowButtonPressed()) {
					return FSMState.LOW;
				} else {
					return FSMState.IDLE;
				}
			case UP:
				if (input.isUpButtonPressed()) {
					return FSMState.UP;
				} else {
					return FSMState.IDLE;
				}
			case DOWN:
				if (input.isDownButtonPressed()) {
					return FSMState.DOWN;
				} else {
					return FSMState.IDLE;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		pidControllerArm.setReference(currentEncoder, CANSparkMax.ControlType.kPosition);
	}
	private void handleHighState(TeleopInput input) {
		pidControllerArm.setReference(HIGH_ENCODER_ROTATIONS, CANSparkMax.ControlType.kPosition);
	}
	private void handleMiddleState(TeleopInput input) {
		pidControllerArm.setReference(MID_ENCODER_ROTATIONS, CANSparkMax.ControlType.kPosition);
	}
	private void handleLowState(TeleopInput input) {
		pidControllerArm.setReference(LOW_ENCODER_ROTATIONS, CANSparkMax.ControlType.kPosition);
	}
	private void handleUpState(TeleopInput input) {
		pidControllerArm.setReference(UP_POWER, CANSparkMax.ControlType.kDutyCycle);
	}
	private void handleDownState(TeleopInput input) {
		pidControllerArm.setReference(DOWN_POWER, CANSparkMax.ControlType.kDutyCycle);
	}
}
