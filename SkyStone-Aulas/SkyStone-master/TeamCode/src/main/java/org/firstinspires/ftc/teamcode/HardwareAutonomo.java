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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareAutonomo
{
    /* Public OpMode members. */
    public DcMotor  motorEsquerda   = null;
    public DcMotor  motorDireita    = null;
    public DcMotor  motorEsquerdaTras = null;
    public DcMotor  motorDireitaTras = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareAutonomo(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        //Referência ao hwMap
        hwMap = ahwMap;

        //Vinculação a Ds
        motorEsquerda  = hwMap.get(DcMotor.class, "motor_Esquerda");
        motorEsquerda = hwMap.get(DcMotor.class, "motor_EsquerdaTras");
        motorDireita = hwMap.get(DcMotor.class, "motor_Direita");
        motorDireitaTras = hwMap.get(DcMotor.class, "motor_DireitaTras");

        motorEsquerda.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDireita.setDirection(DcMotor.Direction.REVERSE);
        motorDireitaTras.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        motorEsquerda.setPower(0);
        motorEsquerdaTras.setPower(0);
        motorDireita.setPower(0);
        motorDireitaTras.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorEsquerda.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorEsquerdaTras.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDireitaTras.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
 }

