package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.utils16095.Globals;
import org.firstinspires.ftc.teamcode.utils16095.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utils16095.RunMode;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {

    Robot robot;

    private double yawOffset;


    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBackMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBackMotor");

        robot = new Robot(hardwareMap, null);

        Trigger isAPressed = new Trigger(() -> gamepad1.a);
        Trigger isBPressed = new Trigger(() -> gamepad1.b);
        Trigger isXPressed = new Trigger(() -> gamepad1.x);
        Trigger isYPressed = new Trigger(() -> gamepad1.y);

        Trigger isLeftBumperPressed = new Trigger(() -> gamepad1.left_bumper);
        Trigger isRightBumperPressed = new Trigger(() -> gamepad1.right_bumper);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"od");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setOffsets(155,-25);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.update();
            telemetry.addData("Elevator Position", robot.elevator.getPos());
            telemetry.addData("Stretch Position", robot.claw.getStretchPosition());

            if (isAPressed.get()) {
                robot.elevator.toTargetPosition(20,5000);
                // to be determined
            }

            if (isBPressed.get()) {
                robot.claw.catchObject(10);
                // to be determined
            }

            if (isXPressed.get()) {
                robot.elevator.toTargetPosition(0,5000);
            }

            if (isYPressed.get()) {
                robot.claw.clawRelease();
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double botHeading = odo.getHeading() - yawOffset;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.dpad_left) {
                yawOffset = odo.getHeading() - yawOffset;
            }

//            double botHeading = odo.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

        }
    }
}