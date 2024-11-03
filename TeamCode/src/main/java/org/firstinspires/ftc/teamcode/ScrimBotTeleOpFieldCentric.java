
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "ScrimBotTeleOpFieldCentric")
public class ScrimBotTeleOpFieldCentric extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, armMotor;

    private CRServo intakeServo;
    private BNO055IMU imu;
    private Orientation angles;

    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;
    int[] maxPositions = {2100}; // potentially something else

    int[] minPositions = {0};
    double slideSpeed = 0.6;

    public void moveArmMotor () {
        int armPos = armMotor.getCurrentPosition();

        if(gamepad2.dpad_up) {
            armPos += 50;
        }
        else if(gamepad2.dpad_down) {
            armPos -= 50;
        }

        armMotor.setTargetPosition(armPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(slideSpeed);

        if (armMotor.getCurrentPosition() == armPos) {
            armMotor.setPower(0);
        }

    }

    public void moveIntakeServo () {
        if (gamepad2.a) {
            intakeServo.setPower(INTAKE_COLLECT);
            telemetry.addLine("yuh");
        }
        else if (gamepad2.x) {
            intakeServo.setPower(INTAKE_OFF);
        }
        else if (gamepad2.b) {
            intakeServo.setPower(INTAKE_DEPOSIT);
        }
    }

    @Override
    public void init() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //armSlide = hardwareMap.get(DcMotor.class, "armSlide");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);  // Front left stays forward
        frontRight.setDirection(DcMotor.Direction.REVERSE); // Reverse direction for front right
        backLeft.setDirection(DcMotor.Direction.REVERSE);   // Reverse direction for back left
        backRight.setDirection(DcMotor.Direction.REVERSE);  // Reverse direction for back right

        // Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// jk
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeServo.setPower(INTAKE_OFF);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get joystick values
        double y = -gamepad1.left_stick_y; // Remember, y is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Adjust for strafing imbalance
        double rotate = gamepad1.right_stick_x;

        // Get the robot's heading in radians
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double robotHeading = angles.firstAngle;

        // Calculate field-centric x and y values
        double xField = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
        double yField = x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

        // Calculate motor power for field-centric control
        double frontLeftPower = yField + xField + rotate;
        double frontRightPower = yField - xField - rotate;
        double backLeftPower = yField - xField + rotate;
        double backRightPower = yField + xField - rotate;

        // Normalize the values so no value exceeds 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        moveIntakeServo();
        moveArmMotor();
        //moveArmSlide();

        telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        //telemetry.addData("Target Position", armSlide.getTargetPosition());
        //telemetry.addData("Current Position", armSlide.getCurrentPosition());
        telemetry.update();
        telemetry.update();

    }
}
