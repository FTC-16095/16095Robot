package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Drivetrain {

    public State state;

    public enum State {
        FOLLOW_SPLINE,
        GO_TO_POINT,
        DRIVE,
        FINAL_ADJUSTMENT,
        BRAKE,
        WAIT_AT_POINT,
        IDLE
    }

    private Robot robot;
    private Sensors sensors;
    private HardwareMap hardwareMap;
    private Vision vision;
    private final DcMotorEx rightBackMotor;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx leftBackMotor;

    public Drivetrain(HardwareMap hardwareMap, Robot robot, Sensors sensors, Vision vision) {
        this.robot = robot;
        this.sensors = sensors;
        this.hardwareMap = hardwareMap;
        this.vision = vision;

        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class,"rightFrontMotor");
        leftFrontMotor = hardwareMap.get(DcMotorEx.class,"leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBackMotor");
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double LBP, double LFP,double RBP,double RFP) {
        rightBackMotor.setPower(Range.clip(RBP,-1,1));
        rightFrontMotor.setPower(Range.clip(RFP,-1,1));
        leftFrontMotor.setPower(Range.clip(LFP,-1,1));
        leftBackMotor.setPower(Range.clip(LBP,-1,1));
    }

    public void setPowerWithGamepad(double leftY, double leftX,double RightY, double RightX) {
        setPower(- leftY - leftX + RightX,
                - leftY + leftX + RightX,
                - leftY + leftX - RightX,
                - leftY - leftX - RightX);
    }

    public void enablePIDFControl(PIDFCoefficients pidfCoef) {
        rightBackMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        rightFrontMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        leftBackMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        leftFrontMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
    }

    public void setVelocity(double LBV, double LFV,double RBV,double RFV) {
        rightBackMotor.setVelocity(Range.clip(RBV, -Constants.maxMotorAngularVelocity, Constants.maxMotorAngularVelocity));
        rightFrontMotor.setVelocity(Range.clip(RFV, -Constants.maxMotorAngularVelocity, Constants.maxMotorAngularVelocity));
        leftFrontMotor.setVelocity(Range.clip(LFV, -Constants.maxMotorAngularVelocity, Constants.maxMotorAngularVelocity));
        leftBackMotor.setVelocity(Range.clip(LBV, -Constants.maxMotorAngularVelocity, Constants.maxMotorAngularVelocity));
    }

    public void setVelocityWithGamepad(double leftY, double leftX,double RightY, double RightX) {
        setVelocity(- leftY - leftX + RightX,
                - leftY + leftX + RightX,
                - leftY + leftX - RightX,
                - leftY - leftX - RightX);
    }

    /**
     * Call this function to set BRAKE state when no power for EVERY motor in MECANUM Drivetrain.
     */
    public void setZeroPowerBrake() {
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setRobotDirection(boolean isHeadingFront) {
        if (isHeadingFront) {
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else {
            rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }


}