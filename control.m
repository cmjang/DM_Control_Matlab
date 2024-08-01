addpath('.\DM_CAN\');
clear;
motor1=Motor(DM_Motor_Type.DM4310, 0x01, 0x11);%设置电机id
motor2=Motor(DM_Motor_Type.DM4310, 0x05, 0x15);
MotorControl1 = MotorControl('COM8',921600);%串口号和波特率
MotorControl1.addMotor(motor1);
MotorControl1.addMotor(motor2);
MotorControl1.enable(motor1);
MotorControl1.enable(motor2);
pause(1);
MotorControl1.switchControlMode(motor1,Control_Type.VEL);
pause(0.1);
MotorControl1.switchControlMode(motor2,Control_Type.MIT);
pause(0.1);
MotorControl1.save_motor_param(motor1);
MotorControl1.save_motor_param(motor2);
pause(1);
index=1;
for index =1: 10000
    y=sin(index/100);
    MotorControl1.control_Vel(motor1, y*10);
    % MotorControl1.control_Pos_Vel(motor2,y*10,5);
    MotorControl1.controlMIT(motor2,30,0.4,y*5,0,0);
    disp(['Motor1--vel:',num2str(motor1.getVelocity()),'pos:',num2str(motor1.getPosition()),'tau:',num2str(motor1.getTorque())]);
    disp(['Motor2--vel:',num2str(motor2.getVelocity()),'pos:',num2str(motor2.getPosition()),'tau:',num2str(motor2.getTorque())]);
end
disp("end");
delete(MotorControl1.serial_);