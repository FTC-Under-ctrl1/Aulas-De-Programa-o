package org.firstinspires.ftc.teamcode.servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name=" Servo toggle", group="Linear Opmode")
public class servo extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Declaração dos Servos
    Servo servoDireita = null;
    Servo servoEsquerda = null;

    @Override
    public void runOpMode() {
        //Variáveis gamepad
        boolean buttonA;

        //variável para posição do servo
        double servoPos = 0;

        //Vinculação ao Rc
        servoDireita = hardwareMap.get(Servo.class, "servo_Direita");
        servoEsquerda = hardwareMap.get(Servo.class, "servo_Esquerda");

        //Direção dos servos
        servoDireita.setDirection(Servo.Direction.FORWARD);
        servoEsquerda.setDirection(Servo.Direction.REVERSE);

        //Movimenta o servo quando inicia o robô para 0
        servoDireita.setPosition(servoPos);
        servoEsquerda.setPosition(servoPos);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Referenciando a variável buttonA ao botão a do gamepad
            buttonA = gamepad1.a;

            //Manda o valor 1 apenas quando botão segurado
            if(buttonA) {
                servoPos = 1;
            } else servoPos = 0;

            //Manda o valor para os servos
            servoDireita.setPosition(servoPos);
            servoEsquerda.setPosition(servoPos);
        }

    }

}
