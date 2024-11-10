
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "wristServoZero")
public class ZeroServos extends OpMode {
    Servo Servo1;
    Servo Servo2;
    Servo Servo3;

    @Override
    public void init() {

        Servo1 = hardwareMap.get(Servo.class, "armServo");
        Servo2 = hardwareMap.get(Servo.class, "wristServo");
        Servo3 = hardwareMap.get(Servo.class, "specimenServo");

        Servo1.setPosition(0);
        Servo2.setPosition(0);
        Servo3.setPosition(0);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            Servo1.setPosition(0);
            Servo2.setPosition(0);
            Servo3.setPosition(0);
        }
    }
}

