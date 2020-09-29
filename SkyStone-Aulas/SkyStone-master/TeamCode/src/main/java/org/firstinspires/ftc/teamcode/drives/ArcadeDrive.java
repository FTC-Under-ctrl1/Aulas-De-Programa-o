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

package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name=" TeleOp ArcadeDrive", group="Linear Opmode")
public class ArcadeDrive extends LinearOpMode {
    //Membros do código
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorEsquerda = null;
    private DcMotor motorDireita = null;
    private DcMotor motorDireitoBack = null;
    private DcMotor motorEsquerdoBack = null;

    @Override
    public void runOpMode() {
        double poderEsquerda;
        double poderDireita;
        double max;

        motorEsquerda = hardwareMap.get(DcMotor.class, "motor_Esquerda");
        motorDireita = hardwareMap.get(DcMotor.class, "motor_Direita");
        motorEsquerdoBack = hardwareMap.get(DcMotor.class, "motor_EsquerdoBack");
        motorDireitoBack = hardwareMap.get(DcMotor.class, "motor_DireitoBack");

        motorDireita.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEsquerdoBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorEsquerda.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //Declaração variáveis
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            //Atribuindo o valor
            poderEsquerda = drive + turn;
            poderDireita = drive - turn;

            //Escolhendo o maior valor
            max = Math.max(Math.abs(poderEsquerda), Math.abs(poderDireita));

            //Proporcionalidade para não ultrapassar +/- 1
            if (max > 1.0) {
                poderEsquerda /= max;
                poderDireita /= max;
            }

            //Mandando a força para os motores
            motorDireita.setPower(poderDireita);
            motorEsquerda.setPower(poderEsquerda);
            motorDireitoBack.setPower(poderDireita);
            motorEsquerdoBack.setPower(poderEsquerda);

        }
    }
}
