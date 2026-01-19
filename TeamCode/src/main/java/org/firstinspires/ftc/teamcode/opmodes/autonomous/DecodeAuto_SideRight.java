package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Decode Auto Side Right", group = "Decode")
public class DecodeAuto_SideRight extends LinearOpMode {
    @Override
    public void runOpMode() {
        DecodeAutoRoutine routine = new DecodeAutoRoutine(this, StartPos.SIDE_RIGHT);
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
