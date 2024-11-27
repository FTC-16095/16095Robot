package org.firstinspires.ftc.teamcode.subsystems.claw;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Claw {
    public enum State {
        IDLE,
        CAUGHT,
        CATCHING,
        STRETCHING,
        RESET,
        RELEASED,
        RELEASING
    }

    public static State clawState = State.IDLE;
    public static State stretchState = State.IDLE;

    private Robot robot;
    private Sensors sensors;
    private Vision vision;
    private HardwareMap hardwareMap;

    private Motor slideMotor;
    private Motor.Encoder encoder;

    private ServoEx clawServo;
    private ServoEx clawTurnServo;

    public Claw(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        this.hardwareMap = hardwareMap;

        slideMotor = new Motor(hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312);

        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.setPositionCoefficient(Constants.stretchP);

        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        encoder = slideMotor.encoder;
        slideMotor.stopAndResetEncoder();
        encoder.setDistancePerPulse(18.0);

        clawServo = new SimpleServo(
                hardwareMap, "clawServo", Constants.clawServoMinAngle, Constants.clawServoMaxAngle, AngleUnit.DEGREES
        );

        clawTurnServo = new SimpleServo(
                hardwareMap, "clawTurnServo", Constants.angleServoMinAngle, Constants.angleServoMaxAngle, AngleUnit.DEGREES
        );

//        clawServo.setPosition(0);

    }

    public void clawUpdate() {
        switch (clawState) {

        }
    }

    public void stretchUpdate() {
        switch (stretchState) {

        }
    }

    public void setStretchDirection(boolean isForward) {
        slideMotor.setInverted(!isForward);
    }

    public void setClawDirection(boolean isForward) {
        clawServo.setInverted(!isForward);
    }

    public void catchObject(int stretchPosition) {
        clawRelease();
        clawStretch(stretchPosition);
        clawCatch();
    }

    public double getStretchPosition() {
        return slideMotor.getCurrentPosition();
    }

    public double getStretchVelocity() {
        return slideMotor.getCorrectedVelocity();
    }

    public void clawStretch(int positionTicks) {

        slideMotor.setTargetPosition(positionTicks);
        slideMotor.setPositionTolerance(13.6);   // allowed maximum error

        while (!slideMotor.atTargetPosition()) {
            slideMotor.set(0.75);
        }
        slideMotor.stopMotor();
    }

    public void clawRepositioning() {

        slideMotor.setTargetPosition(0);
        slideMotor.setPositionTolerance(13.6);

        while (!slideMotor.atTargetPosition()) {
            slideMotor.set(0.75);
        }
        slideMotor.stopMotor();
    }

    private void clawCatch() {
        clawServo.rotateByAngle(90, AngleUnit.DEGREES);
        // to be added: if sensors detected
        clawState = State.CAUGHT;
    }

    public void clawRelease() {
        if (clawState == State.CAUGHT){
            clawServo.rotateByAngle(-90, AngleUnit.DEGREES);
            clawState = State.RELEASED;
        }

    }

    public void clawRotate(double angle) {
        clawTurnServo.rotateByAngle(angle, AngleUnit.DEGREES);
    }

    public void clawTwist() {
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + 1);
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - 1);
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + 1);
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - 1);

    }

    public boolean encoderTest() {
        return encoder.getPosition() == slideMotor.getCurrentPosition();
    }

}
