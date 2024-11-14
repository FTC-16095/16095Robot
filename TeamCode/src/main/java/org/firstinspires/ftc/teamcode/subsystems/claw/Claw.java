package org.firstinspires.ftc.teamcode.subsystems.claw;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Claw {
    public enum State {
        CAUGHT,
        CATCHING,
        RELEASE,
        RELEASING,
        ERR
    }

    private Robot robot;
    private Sensors sensors;
    private Vision vision;
    private HardwareMap hardwareMap;

    private Motor stretchMotor;
    private Motor.Encoder encoder;

    private ServoEx clawServo;

    public Claw(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        this.hardwareMap = hardwareMap;

        stretchMotor = new Motor(hardwareMap, "stretchMotor", Motor.GoBILDA.RPM_312);
        stretchMotor.setRunMode(Motor.RunMode.PositionControl);
        stretchMotor.setPositionCoefficient(Constants.stretchP);

        encoder = stretchMotor.encoder;
        stretchMotor.resetEncoder();
        encoder.setDistancePerPulse(18.0);

        clawServo = new SimpleServo(
                hardwareMap, "clawServo", Constants.clawServoMinAngle, Constants.clawServoMaxAngle, AngleUnit.DEGREES
        );
        clawServo.setPosition(0);

        telemetry.addData("Claw Servo Position", clawServo.getPosition());
        telemetry.addData("Claw Servo Angle(Degrees)", clawServo.getAngle(AngleUnit.DEGREES));

    }

    private void setStretchDirection(boolean isForward) {
        stretchMotor.setInverted(!isForward);
    }

    private void setClawDirection(boolean isForward) {
        clawServo.setInverted(!isForward);
    }

    private void catchObject(int stretchPosition) {
        clawStretch(stretchPosition);
        clawCatch();
    }

    private double getStretchPosition() {
        return stretchMotor.getCurrentPosition();
    }

    private double getStretchVelocity() {
        return stretchMotor.getCorrectedVelocity();
    }

    private void clawStretch(int positionTicks) {
        stretchMotor.setTargetPosition(positionTicks);

        stretchMotor.setPositionTolerance(13.6);   // allowed maximum error

        while (!stretchMotor.atTargetPosition()) {
            stretchMotor.set(0.75);
        }
        stretchMotor.stopMotor();
    }

    private void clawCatch() {
        clawServo.turnToAngle(90);
        // to be added: if sensors detected
        clawServo.turnToAngle(0);
    }

}
