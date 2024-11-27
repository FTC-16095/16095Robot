package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {

    private final MecanumDrive drivetrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier rotate;
    private final DoubleSupplier fun;
    private final BooleanSupplier shouldReset;

    public TeleopDriveCommand(MecanumDrive drivetrain,
                              DoubleSupplier forward,
                              DoubleSupplier fun,
                              DoubleSupplier rotate,
                              BooleanSupplier shouldReset) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.rotate = rotate;
        this.fun = fun;
        this.shouldReset = shouldReset;

        addRequirements((Subsystem) drivetrain);
    }

    @Override
    public void execute() {
        if (shouldReset.getAsBoolean()) {
            drivetrain.reset();
        }
        drivetrain.moveRobot(forward.getAsDouble(), fun.getAsDouble(), rotate.getAsDouble());
    }

}