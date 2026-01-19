package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Decode Auto Side Right", group = "Decode")
public class DecodeAuto_SideRight extends LinearOpMode {
    @Override
    public void runOpMode() {
        Alliance alliance = Alliance.BLUE;
        DecodeAutoRoutine routine = new DecodeAutoRoutine(this, alliance, StartPos.SIDE_RIGHT);
        routine.init();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                alliance = Alliance.BLUE;
            } else if (gamepad1.b) {
                alliance = Alliance.RED;
            }
            routine.setAlliance(alliance);
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
