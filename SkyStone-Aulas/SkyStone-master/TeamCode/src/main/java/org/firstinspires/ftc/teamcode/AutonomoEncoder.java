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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Autonomo Encoder", group="Pushbot")
public class AutonomoEncoder extends LinearOpMode {

    HardwareAutonomo         robot   = new HardwareAutonomo();   // O Hardware do autonômo
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // Sempre confira qual o CPR do motor.
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     //A redução
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // Diâmetro da roda
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motorEsquerda.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorDireita.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorEsquerdaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorDireitaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorEsquerdaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorDireitaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Manda a mensagem se o encoder estiver resetado
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.motorEsquerda.getCurrentPosition(),
                          robot.motorDireita.getCurrentPosition());
        telemetry.update();

        // Espera o start
        waitForStart();

        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);
        encoderDrive(TURN_SPEED,   12, -12, 4.0);
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    //Método de processamento para a posição setada.
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            //Determina a próxima posição
            newLeftTarget = robot.motorEsquerda.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorDireita.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            //Seta a posição alvo
            robot.motorEsquerda.setTargetPosition(newLeftTarget);
            robot.motorDireita.setTargetPosition(newRightTarget);
            robot.motorEsquerdaTras.setTargetPosition(newLeftTarget);
            robot.motorDireitaTras.setTargetPosition(newRightTarget);

            //Coloca no modo de ir até a psoição
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorDireita.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorEsquerdaTras.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorDireitaTras.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Reseta o tempo e começa a movimentação
            runtime.reset();
            robot.motorEsquerda.setPower(Math.abs(speed));
            robot.motorDireita.setPower(Math.abs(speed));
            robot.motorEsquerdaTras.setPower(Math.abs(speed));
            robot.motorDireitaTras.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.motorEsquerda.isBusy() && robot.motorDireita.isBusy())) {

                telemetry.addData("Path1",  "Rodar até %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Posição atual %7d :%7d",
                                            robot.motorEsquerda.getCurrentPosition(),
                                            robot.motorDireita.getCurrentPosition());
                telemetry.update();
            }

            // Para os motores a cada chamada do método
            robot.motorEsquerda.setPower(0);
            robot.motorDireita.setPower(0);
            robot.motorEsquerdaTras.setPower(0);
            robot.motorDireitaTras.setPower(0);

            // Desliga o run to position.
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorEsquerdaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorDireitaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
