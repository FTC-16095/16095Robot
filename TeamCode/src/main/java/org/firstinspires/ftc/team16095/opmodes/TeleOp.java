package org.firstinspires.ftc.team16095.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16095.Constants;
import org.firstinspires.ftc.team16095.Robot;
import org.firstinspires.ftc.team16095.sensors.Sensors;
import org.firstinspires.ftc.team16095.utils.Globals;
import org.firstinspires.ftc.team16095.utils.RunMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="mecanum", group="Linear OpMode")
public class TeleOp extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Robot robot;
    Sensors sensors = new Sensors();

    // Grab Constants and give them to drivetrain
    private final PIDFCoefficients pidfTeleopCoefficients =
            new PIDFCoefficients(Constants.teleP, Constants.teleI, Constants.teleD, Constants.teleF);

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new Robot(hardwareMap, null);

        robot.drivetrain.setZeroPowerBrake();
        robot.drivetrain.enablePIDFControl(pidfTeleopCoefficients);
        robot.drivetrain.setRobotDirection(true);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            robot.drivetrain.setVelocityWithGamepad(
                    gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_x);
            //robot.drivetrain.setPowerWithGamepad(
            //      gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_x);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        }

    }
}