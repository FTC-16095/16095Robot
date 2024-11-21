package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.utils16095.Globals;
import org.firstinspires.ftc.teamcode.utils16095.RunMode;

@Disabled
@TeleOp
@Config
public class SlideTest extends LinearOpMode {
    public static double distance = 10;
    private Claw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TEST;
        Robot robot = new Robot(hardwareMap, null);
        double stretchPos = claw.getStretchPosition();
        double stretchVel = claw.getStretchVelocity();
    }
}
