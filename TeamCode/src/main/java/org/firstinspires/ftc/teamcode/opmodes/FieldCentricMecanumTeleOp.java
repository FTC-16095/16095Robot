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

        gamepadEx1 = new GamepadEx(gamepad1);

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

            telemetry.addData("Elevator Position", robot.elevator.getPos());
            telemetry.addData("Stretch Position", robot.claw.getStretchPosition());

            if (a_1.isClicked(gamepad1.a)) {
                if (Elevator.elevatorState == Elevator.State.IDLE) {
                    robot.elevator.toTargetPosition(20, 5000);
                }
                else if (Elevator.elevatorState == Elevator.State.TOP) {
                    robot.elevator.toTargetPosition(0,5000);
                }
                else {
                    continue;
                }
                // to be determined
            }

            if (b_1.isClicked(gamepad1.b)) {
                if (Claw.clawState == Claw.State.IDLE && Claw.stretchState == Claw.State.IDLE) {
                    robot.claw.catchObject(10);
                }
                else if (Claw.clawState == Claw.State.CAUGHT && Claw.stretchState == Claw.State.CAUGHT) {
                    robot.claw.clawRepositioning();
                }
                else {
                    continue;
                }
                // to be determined
            }

            if (leftBumper_1.isClicked(gamepad1.left_bumper) && rightBumper_1.isReleased(gamepad1.right_bumper)) {
                robot.claw.clawRotate(10);
            }

            if (leftBumper_1.isReleased(gamepad1.left_bumper) && rightBumper_1.isClicked(gamepad1.right_bumper)) {
                robot.claw.clawRotate(10);
            }

            if (x_1.isClicked(gamepad1.x)) {
                robot.claw.clawTwist();
            }

            if (y_1.isClicked(gamepad1.y)) {

            }

            robot.drivetrain.setDefaultCommand(
                    new TeleopDriveCommand(
                            robot.drivetrain, () -> -gamepadEx1.getLeftY(),
                    () -> -gamepadEx1.getLeftX(), () -> gamepadEx1.getRightX(),
                    () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                    )
            );

            telemetry.update();

        }
    }
}