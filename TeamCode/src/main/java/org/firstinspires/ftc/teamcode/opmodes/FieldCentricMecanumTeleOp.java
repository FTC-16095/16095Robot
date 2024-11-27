package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {

    Robot robot;
    private GamepadEx gamepadEx1;
    private double yawOffset;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, null);

        // DRIVER 1
        ButtonToggle rightBumper_1 = new ButtonToggle();
        ButtonToggle leftBumper_1 = new ButtonToggle();
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();
        ButtonToggle y_1 = new ButtonToggle();
        ButtonToggle a_1 = new ButtonToggle();
        ButtonToggle leftTrigger_1 = new ButtonToggle();
        ButtonToggle rightTrigger_1 = new ButtonToggle();
        ButtonToggle leftTrigger_1_double = new ButtonToggle();
        ButtonToggle left_dpad_1 = new ButtonToggle();
        ButtonToggle down_dpad_1 = new ButtonToggle();

        // DRIVER 2
        ButtonToggle dpadUp_2 = new ButtonToggle();
        ButtonToggle dpadDown_2 = new ButtonToggle();
        ButtonToggle dpadLeft_2 = new ButtonToggle();
        ButtonToggle dpadRight_2 = new ButtonToggle();
        ButtonToggle leftTrigger_2 = new ButtonToggle();
        ButtonToggle rightTrigger_2 = new ButtonToggle();
        ButtonToggle leftBumper_2 = new ButtonToggle();
        ButtonToggle rightBumper_2 = new ButtonToggle();
        ButtonToggle a_2 = new ButtonToggle();
        ButtonToggle x_2 = new ButtonToggle();
        ButtonToggle y_2 = new ButtonToggle();

        Globals.RUNMODE = RunMode.TELEOP;
        telemetry.addData("Status", "Initialized");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (a_1.isClicked(gamepad1.a)) {
                robot.elevator.toTargetPosition(5000, 5000);
            }

            if (b_1.isClicked(gamepad1.b)) {
                robot.claw.catchObject(10);
            }

            if (leftBumper_1.isClicked(gamepad1.left_bumper)) {
                robot.claw.clawRotate(-10);
            }

            if (rightBumper_1.isClicked(gamepad1.right_bumper)) {
                robot.claw.clawRotate(10);
            }

            if (x_1.isClicked(gamepad1.x)) {
                robot.elevator.toTargetPosition(0,5000);
            }

            if (y_1.isClicked(gamepad1.y)) {
                robot.claw.clawRepositioning();
            }

            robot.drivetrain.setDefaultCommand(
                    new TeleopDriveCommand(
                            robot.drivetrain, () -> -gamepad1.left_stick_y,
                    () -> -gamepad1.left_stick_x, () -> gamepad1.right_stick_x,
                    () -> gamepad1.left_stick_button
                    )
            );

            telemetry.update();

        }
    }
}