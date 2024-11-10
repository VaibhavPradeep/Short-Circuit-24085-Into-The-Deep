
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends OpMode {
    MecanumDrive drive = new MecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    IntakeLiftCamera ILC = new IntakeLiftCamera();

    @Override
    public void init() {
        drive.initDrive(hardwareMap);
        ILC.initILC(hardwareMap);
    }

    @Override
    public void loop() {
        // mecanum drive
        drive.driveMecanum();

        // moive dr4b motors, gampad 2, dpad up is up, down is down
        ILC.moveDR4BMotors(gamepad2.dpad_up, gamepad2.dpad_down);

        // move intake servo, x is stop, a is intake, b is outake
        ILC.moveIntakeServo(gamepad1.a, gamepad1.x, gamepad1.b);

        // move wrist servo, left bumper moves left, right bumper moves it right
        ILC.moveWristServo(gamepad2.left_bumper, gamepad2.right_bumper);

        // specimen servo, left bumper opens it, right closes it
        ILC.moveSpecimenServo(gamepad1.left_bumper, gamepad1.right_bumper);

        // moves arm servo, left is intake position, right is depositing into basket
        ILC.moveArmServo(gamepad2.dpad_left, gamepad2.dpad_right);

        // updates telemetry
        ILC.addTelemetry(telemetry);

    }
}
