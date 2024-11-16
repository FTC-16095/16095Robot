package org.firstinspires.ftc.team16095.subsystems.elevator;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team16095.Constants;
import org.firstinspires.ftc.team16095.Robot;
import org.firstinspires.ftc.team16095.sensors.Sensors;
import org.firstinspires.ftc.team16095.vision.Vision;

public class Elevator {
    public enum State {
        TOP,
        BOTTOM,
        ELEVATING,
        REPOSITIONING
    }

    private Robot robot;
    private Sensors sensors;
    private HardwareMap hardwareMap;
    private Vision vision;

    private final Motor leftMotor;
    private final Motor rightMotor;

    private Motor.Encoder encoder;

    private final MotorGroup motors;

    private static final double DEFAULT_POSITION_TOLERANCE = 13.6;
    private static final double DEFAULT_POWER = 0.75;

    public Elevator(Robot robot, Sensors sensors, HardwareMap hardwareMap, Vision vision) {
        this.robot = robot;
        this.sensors = sensors;
        this.hardwareMap = hardwareMap;
        this.vision = vision;

        leftMotor = new Motor(hardwareMap, "elevator_left", Motor.GoBILDA.RPM_312);
        rightMotor = new Motor(hardwareMap, "elevator_right", Motor.GoBILDA.RPM_312);

        rightMotor.setInverted(true);

        motors = new MotorGroup(leftMotor, rightMotor);
        motors.setRunMode(Motor.RunMode.PositionControl);
        motors.setPositionCoefficient(Constants.elevatorP);

        motors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        encoder = leftMotor.encoder;
        encoder.setDistancePerPulse(18.0);

        motors.resetEncoder();
        encoder.reset();
        encoder.setDirection(Motor.Direction.FORWARD);
        motors.set(0);
    }

    public void setElevatorPositionCoefficient(double kP) {
        motors.setPositionCoefficient(kP);
    }

    public int getPos() {
        return motors.getCurrentPosition();
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
    }

}
