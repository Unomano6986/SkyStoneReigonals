package org.firstinspires.ftc.teamcode; /**
 *
 * Created by Danny, FTC Team 13108
 * version 1.0 Aug 29, 2019
 *
 */



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
/*

          X FRONT X
        X           X
      X  FL       FR  X
              X
             XXX
              X
      X  BL       BR  X
        X           X
          X       X
*/
@TeleOp(name = "MecanumWheels")

public class MecanumDrive extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor Elevator;
    DcMotor Pivot;
    //Servos
    Servo ServL;
    Servo ServR;
    Servo ArmServo;

    Servo RightPivot;
    Servo LeftPivot;
    Servo RightPinch;
    Servo LeftPinch;
    Servo CapServo;




    boolean changed = false; //Outside of loop()



    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position


    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;



    @Override
    public void init() {

        //motors
        motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
        motorBackRight = hardwareMap.dcMotor.get("BackRight");
        Elevator = hardwareMap.dcMotor.get("Elevator");
        Pivot = hardwareMap.dcMotor.get("Pivot");

        //servos
        ServL = hardwareMap.servo.get("ServLeft");
        ServR = hardwareMap.servo.get("ServRight");
        ArmServo = hardwareMap.servo.get("ArmServ");
        CapServo = hardwareMap.servo.get("Capstone");



        RightPivot = hardwareMap.servo.get("RightPivot");
        LeftPivot = hardwareMap.servo.get("LeftPivot");

        RightPinch = hardwareMap.servo.get("RightPinch");
        LeftPinch = hardwareMap.servo.get("LeftPinch");


        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);








        ArmServo.setPosition(0);

        LeftPivot.setPosition(0.8);
        RightPivot.setPosition(0.2);

        LeftPinch.setPosition(0.0);
        RightPinch.setPosition(1.0);










    }


    @Override
    public void loop() {



        Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Drive Train Code
        //Tank Drive Left Stick controls left side Right stick controls right

        motorFrontLeft.setPower(-gamepad1.left_stick_y);
        motorBackLeft.setPower(-gamepad1.left_stick_y);

        motorFrontRight.setPower(-gamepad1.right_stick_y);
        motorBackRight.setPower(-gamepad1.right_stick_y);


        //slide left
        if (gamepad1.dpad_left) {
            motorFrontLeft.setPower(-0.8);
            motorBackLeft.setPower(0.8);

            motorFrontRight.setPower(0.8);
            motorBackRight.setPower(-0.8);
        }
        //slide right
        if (gamepad1.dpad_right) {
            motorFrontLeft.setPower(0.8);
            motorBackLeft.setPower(-0.8);

            motorFrontRight.setPower(-0.8);
            motorBackRight.setPower(0.8);
        }
        //move forward
        if (gamepad1.dpad_up) {
            motorFrontLeft.setPower(1);
            motorBackLeft.setPower(1);
            motorFrontRight.setPower(1);
            motorBackRight.setPower(1);

        }
        //move back
        if (gamepad1.dpad_down) {
            motorFrontLeft.setPower(-1);
            motorBackLeft.setPower(-1);
            motorFrontRight.setPower(-1);
            motorBackRight.setPower(-1);
        }

        //XRail Elevator
        Elevator.setPower(gamepad2.left_stick_y);

        //Foundation Servos
        if (gamepad1.a) {
            ServR.setPosition(0.7);
            ServL.setPosition(0.1);
        }
        if (gamepad1.y) {
            ServR.setPosition(0.15);
            ServL.setPosition(0.7);
        }





        //Skystone Servos Down



        //Gripper for the cube

        if (gamepad2.b) {
            ArmServo.setPosition(0.6);
        } else {
            ArmServo.setPosition(0.1);
        }

        if (gamepad2.a) {
            CapServo.setPosition(1);
        } else {
            CapServo.setPosition(0.5);
        }





        //Pivot Motor for X rail
        Pivot.setPower(0.4 * gamepad2.right_stick_y);


        // Show joystick information as some other illustrative data
        telemetry.addData("Servo Position Left",ServL.getPosition());
        telemetry.addData("Servo Position Right",ServR.getPosition());
        telemetry.addData("Pivot",RightPivot.getPosition());
        telemetry.addData("PinchLeft", RightPinch.getPosition());
        telemetry.addData(">", "Done");
        telemetry.update();
    }


    @Override
    public void stop() {

    }




}
