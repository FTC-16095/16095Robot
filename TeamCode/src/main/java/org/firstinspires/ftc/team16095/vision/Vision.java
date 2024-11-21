package org.firstinspires.ftc.team16095.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team16095.Robot;

public class Vision extends LinearOpMode {
    private Limelight3A limelight;
    private LLResult lastResult;
    private Robot robot;

    public Vision(HardwareMap hardwareMap, Robot robot) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.robot = robot;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();
    }
    
    public LLResult getLastResult(){
        return limelight.getLatestResult();
    }
}
