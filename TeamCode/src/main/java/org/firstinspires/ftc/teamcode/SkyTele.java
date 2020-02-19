/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@TeleOp(name="Sky Tele")
@Disabled

public class SkyTele extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor IntakeL = null;
    private DcMotor IntakeR = null;
    private DcMotor LiftL = null;
    private DcMotor LiftR = null;
    private Servo PushServ = null;
    private Servo Pinch = null;
    private Servo Extend = null;
    private Servo Twist = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FL  = hardwareMap.get(DcMotor.class, "frontright");
        FR = hardwareMap.get(DcMotor.class, "frontleft");
        BL  = hardwareMap.get(DcMotor.class, "backleft");
        BR = hardwareMap.get(DcMotor.class, "backright");
        IntakeL = hardwareMap.get(DcMotor.class, "Intakeleft");
        LiftL = hardwareMap.get(DcMotor.class, "Liftright");
        IntakeR = hardwareMap.get(DcMotor.class, "Intakeright");
        LiftR = hardwareMap.get(DcMotor.class, "Liftleft");
        PushServ = hardwareMap.get(Servo.class, "Push");
        Pinch = hardwareMap.get(Servo.class, "Pinch");
        Extend = hardwareMap.get(Servo.class, "Extend");
        Twist = hardwareMap.get(Servo.class, "Twist");





        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        LiftR.setDirection(DcMotor.Direction.REVERSE);
        IntakeR.setDirection(DcMotor.Direction.REVERSE);

        PushServ.setPosition(0.48);
        Twist.setPosition(1.0);
        Pinch.setPosition(0.6);
        Extend.setPosition(0.78);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status:", "Good to Go!");
    }


    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        BL.setPower(gamepad1.left_stick_y);
        FL.setPower(-gamepad1.left_stick_y);
        FR.setPower(-gamepad1.left_stick_y);
        BR.setPower(gamepad1.left_stick_y);

        BL.setPower(-gamepad1.right_stick_x);
        FL.setPower(-gamepad1.right_stick_x);
        FR.setPower(gamepad1.right_stick_x);
        BR.setPower(gamepad1.right_stick_x);

        LiftL.setPower(gamepad2.left_stick_y);
        LiftR.setPower(gamepad2.left_stick_y);





        //slide left
        if (gamepad1.dpad_right) {
            FL.setPower(-0.6);
            BL.setPower(0.6);

            FR.setPower(0.6);
            BR.setPower(-0.6);
        }
        //slide right
        if (gamepad1.dpad_left) {
            FL.setPower(0.6);
            BL.setPower(-0.6);

            FR.setPower(-0.6);
            BR.setPower(0.6);
        }
        //move forward
        if (gamepad1.dpad_up) {
            FL.setPower(0.3);
            BL.setPower(-0.3);
            FR.setPower(0.3);
            BR.setPower(-0.3);

        }
        //move back
        if (gamepad1.dpad_down) {
            FL.setPower(-0.3);
            BL.setPower(0.3);
            FR.setPower(-0.3);
            BR.setPower(0.3);
        }
        if (gamepad1.y){
            IntakeL.setPower(0.7);
            IntakeR.setPower(0.7);
        }
        if(gamepad1.x){
            IntakeR.setPower(0);
            IntakeL.setPower(0);
        }

        if (gamepad1.a){
            IntakeL.setPower(-0.7);
            IntakeR.setPower(-0.7);
        }

        if(gamepad1.b){
            PushServ.setPosition(0.68);
        }
        else{
            PushServ.setPosition(0.41);
        }
        if(gamepad1.right_bumper){
            Pinch.setPosition(0.0);
        }
       else{
            Pinch.setPosition(0.6);
        }
        if(gamepad2.right_bumper){
            Extend.setPosition(0.78);
        }
        if(gamepad2.left_bumper){
            Extend.setPosition(0.36);
        }

        









        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Servos",Extend.getPosition());
    }


    @Override
    public void stop() {
    }

}
