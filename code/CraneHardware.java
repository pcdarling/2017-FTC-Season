package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CraneHardware {

    public DcMotor Shoulder = null;
    public DcMotor Extender = null;
    public Servo Nabber = null;

    HardwareMap hwMap = null;

    public CraneHardware() { // Constructor
    }

    public void init(HardwareMap ahwMap){
        //save reference to hwMap
        hwMap = ahwMap;

        Shoulder = hwMap.dcMotor.get("shoulder");
        Extender = hwMap.dcMotor.get("extender");
        Nabber = hwMap.servo.get("Nabber");

        Shoulder.setDirection(DcMotor.Direction.FORWARD);
        Extender.setDirection(DcMotor.Direction.FORWARD);

        Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shoulder.setPower(0);
        Extender.setPower(0);
    }
}
