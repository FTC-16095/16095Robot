package org.firstinspires.ftc.team16095.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16095.Constants;
import org.firstinspires.ftc.team16095.Robot;
import org.firstinspires.ftc.team16095.sensors.Sensors;
import org.firstinspires.ftc.team16095.utils16095.Globals;
import org.firstinspires.ftc.team16095.utils16095.RunMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="mecanum", group="Linear OpMode")
public class TeleOp extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Robot robot;
    Sensors sensors = new Sensors();
    private int stretchPosition = 20;
    private int targetPosition = 20;
    private int rotate = 0;

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

            // or set power, depend on operator
            robot.drivetrain.setVelocityWithGamepad(
                    gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_x);

            // elevate
            if (gamepad1.a) {
                robot.elevator.toTargetPosition(targetPosition, 10000);
            }

            // stretch, catch
            if (gamepad1.b) {
                robot.claw.catchObject(stretchPosition);
            }

//            if (sensors.someSensor.detected()) {
//                Claw.state = Claw.State.CAUGHT;
//            }

            if (gamepad1.x) {
                robot.claw.clawRelease();
            }

//            if (sensors.someSensor.detected()) {
//                Claw.state = Claw.State.RELEASE;
//            }

            if (gamepad1.y) {
                robot.elevator.toTargetPosition(0, 10000);
            }

            if (gamepad1.left_bumper) {
                rotate = 1;
            }

            if (gamepad1.right_bumper) {
                rotate = -1;
            }

            robot.claw.clawRotate(2 * rotate);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        }

    }
}