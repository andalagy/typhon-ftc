package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Decode Auto Center Left", group = "Decode")
public class DecodeAuto_CenterLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        DecodeAutoRoutine routine = new DecodeAutoRoutine(this, StartPos.CENTER_LEFT);
        routine.init();

        while (!isStarted() && !isStopRequested()) {
            routine.initLoop();
            idle();
        }

        waitForStart();
        if (isStopRequested()) {
            routine.stop();
            return;
        }

        routine.start();
        while (opModeIsActive()) {
            routine.update();
            idle();
        }

        routine.stop();
    }
}
