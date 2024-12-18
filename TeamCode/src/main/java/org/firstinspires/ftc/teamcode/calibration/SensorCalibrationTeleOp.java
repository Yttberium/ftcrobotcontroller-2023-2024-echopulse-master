package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;

@TeleOp(group = "calibration")
public class SensorCalibrationTeleOp extends LinearOpMode {
    private final CommandScheduler scheduler= new CommandScheduler();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DepositSubsystem depositSubsystem = new DepositSubsystem(this);
        waitForStart();
        scheduler.schedule(depositSubsystem.telemetry());
        while(opModeIsActive())
        {
            telemetry.update();
            scheduler.update();
        }
    }
}
