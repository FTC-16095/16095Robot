package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Elevator {
    public enum State {
        IDLE,
        TOP,
        ELEVATE,
        REPOSITION
    }

    public static State elevatorState = State.IDLE;

    private Robot robot;
    private Sensors sensors;
    private HardwareMap hardwareMap;
    private Vision vision;

    private final Motor leftLiftMotor;
    private final Motor rightLiftMotor;

    private Motor.Encoder encoder;

    private final MotorGroup motors;

    private static final double DEFAULT_POSITION_TOLERANCE = 13.6;
    private static final double DEFAULT_POWER = 999;

    public Elevator(Robot robot, Sensors sensors, HardwareMap hardwareMap, Vision vision) {
        this.robot = robot;
        this.sensors = sensors;
        this.hardwareMap = hardwareMap;
        this.vision = vision;

        leftLiftMotor = new Motor(hardwareMap, "leftLiftMotor", Motor.GoBILDA.RPM_312);
        rightLiftMotor = new Motor(hardwareMap, "rightLiftMotor", Motor.GoBILDA.RPM_312);

        leftLiftMotor.setInverted(false);

        motors = new MotorGroup(leftLiftMotor, rightLiftMotor);
        motors.setRunMode(Motor.RunMode.PositionControl);

        setElevatorPositionCoefficient(Constants.elevatorP);

        motors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        encoder = leftLiftMotor.encoder;
        encoder.setDistancePerPulse(18.0);

        motors.stopAndResetEncoder();
        encoder.reset();
        encoder.setDirection(Motor.Direction.FORWARD);
        motors.set(0);

        elevatorState = State.IDLE;

    }

    public void setElevatorPositionCoefficient(double kP) {
        motors.setPositionCoefficient(kP);
    }

    public int getPos() {
        return motors.getCurrentPosition();
    }

    public void update() {
        switch (elevatorState) {
            case ELEVATE:
                toTargetPosition(20,2000);
            case REPOSITION:
                toTargetPosition(0,2000);
        }
    }

    public double getVelocity() {
        return motors.getVelocity();
    }

    public void toTargetPosition(int targetPosition,long timeoutMillis) {
        motors.setTargetPosition(targetPosition);
        motors.setPositionTolerance(DEFAULT_POSITION_TOLERANCE);

        long startTime = System.currentTimeMillis();

        while (!motors.atTargetPosition() && (System.currentTimeMillis() - startTime) < timeoutMillis) {
            motors.set(DEFAULT_POWER);
        }

        motors.stopMotor();
        elevatorState = State.TOP;
    }

}
