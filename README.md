# 达妙Matlab库

Matlab支持的地方都能用，爽用！

感谢苗总的大力支持！

**欢迎加入QQ群：677900232 进行达妙电机技术交流。欢迎进入达妙店铺进行点击选购**[首页-达妙智能控制企业店-淘宝网 (taobao.com)](https://shop290016675.taobao.com/?spm=pc_detail.29232929/evo365560b447259.shop_block.dshopinfo.59f47dd6w4Z4dX)

### 1.引用达妙库

达妙的相关我已经放在DM_CAN文件夹下，如果要使用的话建议加上下面这句话在代码开头

```matlab
addpath('.\DM_CAN\');
clear;
```

此外如果出现串口报错串口被占用等问题，极有可能是上一次代码运行因为手动或者报错而中断，解决方法是推荐在代码开头加上clear;清除相关对象。或者重新插拔串口

### 2.定义控制对象

定义电机对象，有几个电机就定义几个每个电机的masterid和canid都要不一样。重要的事情：不要把masterid设为0x00

```matlab
motor1=Motor(DM_Motor_Type.DM4310, 0x01, 0x11);%设置电机id
motor2=Motor(DM_Motor_Type.DM4310, 0x05, 0x15);
```

第一个参数为电机类型，第二个是SlaveID即电机的CANID（电机的ID）,第三个参数是MasterID是主机ID，建议MasterID设置的都不一样，比SlaveID整体高一个。例如Motor1的SlaveID是0x01，MasterID是0x11.这样是最好。

初始化电机控制对象。传入参数是串口参数，波特率是921600，串口进行选择。demo是windows所以是'COM8'

```matlab
MotorControl1 = MotorControl('COM8',921600);%串口号和波特率
```

### 3.电机控制

**推荐在每帧控制完后延迟2ms或者1ms，usb转can默认有缓冲器没有延迟也可使用，但是推荐加上延迟。**

**目前库函数中每个控制指令执行完已经加上1ms的延迟了，如果控制3个电机就不需要加延迟了，如果控制6个电机用一个usb转can，建议在控制指令后再加1ms延迟**

添加电机是addMotor，然后使能电机是enable。使能完后建议加上1s的延迟

```python
MotorControl1.addMotor(motor1);
MotorControl1.addMotor(motor2);
MotorControl1.enable(motor1);
MotorControl1.enable(motor2);
pause(1);
```

#### 3.1MIT模式

使能电机后可以使用MIT模式控制，推荐用MIT模式控制。

第一个参数是电机对象，第二个参数是kp，第三个是kd，第四个是位置，第五个是速度，第六个是力矩大小。具体请参考达妙手册关于mit协议。

```matlab
MotorControl1.controlMIT(motor2,30,0.4,y*5,0,0);
```

#### 3.2 位置速度模式

位置速度模式，第一个参数是电机对象，第二个是位置，第三个是转动速度。具体的参数介绍已经写了函数文档。

例子如下

```matlab
y=sin(index/100);
MotorControl1.control_Pos_Vel(motor2,y*10,5);
```

#### 3.3 速度模式

例子如下，第一个是电机对象，第二个是电机速度

```matlab
y=sin(index/100);
MotorControl1.control_Vel(motor1, y*10);
```

目前达妙的新固件支持切换

#### 3.4力位混合模式

第一个是电机对象，第二个是电机位置，第三个是电机速度范围是0-10000，第四个是电机电流范围为0-10000。具体详细请查看达妙文档

例子如下

```python
MotorControl1.control_pos_force(motor1, 10, 1000,100)
```

### 4.电机模式更改

达妙电机新固件支持使用can进行电机模式修改，以及修改其他参数等操作。具体请咨询达妙客服。

通过下面的函数可以对电机的控制模式进行修改。支持MIT,POS_VEL,VEL,Torque_Pos。四种控制模式在线修改，建议相邻两个之间修改加点延迟。下面是修改的demo。

```python
MotorControl1.switchControlMode(motor1,Control_Type.VEL);
pause(0.1);
MotorControl1.switchControlMode(motor2,Control_Type.MIT);
```

#### 4.1**保存参数**

默认电机修改模式等操作后参数不会保存到flash中，需要使用命令如下进行保存至电机的flash中。保存完延迟一下

```python
MotorControl1.save_motor_param(motor1);
MotorControl1.save_motor_param(motor2);
pause(1);
```

#### 4.2电机控制类参数更改

这个更改的是电机上位机类中的参数不是电机内部的参数。可以修改PMAX，VMAX，TMAX。如果电机中这些参数修改了，需要把对应的控制参数的PMAX也更改了。**注意这没有更改电机内部的参数**

```matlab
MotorControl1.change_control_PMAX(motor2,DM_Motor_Type.DM4310,12.5);
MotorControl1.change_control_VMAX(motor2,DM_Motor_Type.DM4310,30);
MotorControl1.change_control_TMAX(motor2,DM_Motor_Type.DM4310,10);
```

