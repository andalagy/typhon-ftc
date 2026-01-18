package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Mirrors the right-side auto so the same routine can run from the opposite alliance wall.
 */
@Autonomous(name = "Decode Auto Right (Mirrored)", group = "Main")
public class DecodeAuto_RightMirrored extends DecodeAuto_Right {
    @Override
    protected boolean isMirrored() {
        return true;
    }
}
