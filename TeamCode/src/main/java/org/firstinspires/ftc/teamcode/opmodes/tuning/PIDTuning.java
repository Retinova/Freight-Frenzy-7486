package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.supers.ConfigVars;
import org.firstinspires.ftc.teamcode.supers.Mode;
import org.firstinspires.ftc.teamcode.supers.Robot;

@TeleOp(name="PID Tuning", group="tuning")
public class PIDTuning extends LinearOpMode {
    Robot r;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(this, Mode.AUTO);

        r.initCheck();

        waitForStart();

        while(isStarted() && !isStopRequested()){
            if(gamepad1.a){
                r.odo.turn(90);
            }
            else if(gamepad1.b){
                r.odo.turn(-90);
            }
            else if(gamepad1.x){
                r.odo.turn(120);
            }

            r.odo.updateCoeffs(ConfigVars.turnP, ConfigVars.turnI, ConfigVars.turnD);
            r.dashT.addData("PID Constants", "P: %.4f | I: %.4f | D: %.4f", ConfigVars.turnP, ConfigVars.turnI, ConfigVars.turnD);
            r.dashT.update();
        }
    }
}
