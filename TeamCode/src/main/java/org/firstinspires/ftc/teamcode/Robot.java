package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {

    public final Drivetrain drivetrain;
    public final Sensors sensors;

    public HardwareMap hardwareMap;
    //public final Deposit deposit;
    public final Vision vision;
    public final Elevator elevator;
    public final Claw claw;

    public Robot(HardwareMap hardwareMap, Vision vision) {
        this.vision = vision;

        elevator = new Elevator();
        claw = new Claw(hardwareMap, this);
        sensors = new Sensors();

        drivetrain = new Drivetrain(hardwareMap, this, sensors, vision);

        Dashboard.setup();
    }

    private void updateTelemetry() {
        // to be done
        Dashboard.packet.put("Loop Time", GET_LOOP_TIME());
        Dashboard.sendTelemetry();
    }

}