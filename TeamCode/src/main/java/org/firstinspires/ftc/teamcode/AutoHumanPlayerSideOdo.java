package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Autonomous for Red Alliance - Start Near Human Player
@Autonomous(name = "Red Alliance Auto - Human Player Side", group = "Autonomous")
public class AutoHumanPlayerSideOdo extends LinearOpMode {
    /*
    private void MecanumDriveCircuit(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x) drive;
    private ArmController armController;
    private DR4BController dr4bController;
    private Intake intake;
    private Claw claw;
    */

    @Override
    public void runOpMode() {
        /*
        // Initialize hardware
        drive = new MecanumDrive(hardwareMap);
        armController = new ArmController(hardwareMap);
        dr4bController = new DR4BController(hardwareMap);
        intake = new Intake(hardwareMap);
        claw = new Claw(hardwareMap);

        // Wait for start
        waitForStart();

        if (opModeIsActive()) {
            // 1. Score preloaded sample on high rung
            driveToHighRung();
            dr4bController.raiseToHighRung();
            claw.releaseSample();

            // 2. Push a blue block into the human player area
            driveToBlueBlock();
            drive.moveForward(10); // Slight move in (calculate exact distance needed)
            drive.moveBackward(10); // Slight move out (calculate exact distance needed)

            // 3. Score blue sample
            driveToHighRung();
            dr4bController.raiseToHighRung();
            claw.releaseSample();
        }
         */

        /*
        // Helper functions for positioning
    private void driveToHighRung() {
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Pose2d(0, 0)) // Replace with calculated coordinates for the high rung
                .build());
    }

    private void driveToBlueBlock() {
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Pose2d(0, 0)) // Replace with calculated coordinates for the blue block
                .build());
    }
         */

    }
}
