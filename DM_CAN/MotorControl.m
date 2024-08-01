classdef MotorControl < handle
    properties
        serial_
        motors_map
        send_data_frame
        Limit_param
    end
    
    methods
        function obj = MotorControl(COM, baudrate)

            obj.motors_map = containers.Map('KeyType','uint32', 'ValueType','any');
            %限制参数     PMAX VMAX TMAX  行为所代表的不同电机
            obj.Limit_param=[12.5,30,10; %DM4310
                         12.5,50,10; %DM4310_48V
                         12.5,8,28;  %DM4340
                         12.5,10,28; %DM4340_48V
                         12.5,45,20; %DM6006
                         12.5,45,40; %DM8006
                         12.5,45,54; %DM8009
                         ];
            obj.serial_ = serialport(COM,baudrate,"Timeout",0.01);
            obj.send_data_frame = uint8([0x55, 0xAA, 0x1e, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00]);
            flush(obj.serial_);
        end
        
        function delete(obj)
            delete(obj.serial_)
        end 

        function controlMIT(obj, DM_Motor, kp, kd, q, dq, tau)
            % MIT Control Mode Function
            if ~isKey(obj.motors_map, DM_Motor.SlaveID)
                disp('Motor ID not found');
                return;
            end
            kp_uint = obj.float_to_uint(kp, 0, 500, 12);
            kd_uint = obj.float_to_uint(kd, 0, 5, 12);
            MotorType = DM_Motor.MotorType;
            q_uint = obj.float_to_uint(q, -obj.Limit_param(MotorType,1), obj.Limit_param(MotorType,1), 16);
            dq_uint = obj.float_to_uint(dq, -obj.Limit_param(MotorType,2), obj.Limit_param(MotorType,2), 12);
            tau_uint = obj.float_to_uint(tau, -obj.Limit_param(MotorType,3), obj.Limit_param(MotorType,3), 12);
            data_buf = uint8([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);

            data_buf(1) = bitand(bitshift(q_uint, -8),255);
            data_buf(2) = bitand(q_uint,255);
            data_buf(3) = bitshift(dq_uint, -4);
            data_buf(4) = bitor(bitshift(bitand(dq_uint, 15), 4),bitand(bitshift(kp_uint, -8), 15));
            data_buf(5) = bitand(kp_uint,255);
            data_buf(6) = bitshift(kd_uint, -4);
            data_buf(7) = bitor(bitshift(bitand(kd_uint, 15), 4), bitand(bitshift(tau_uint, -8), 15));
            data_buf(8) = bitand(tau_uint,255);
            obj.send_data_frame(15) = 0;
            obj.send_data_frame(14) = DM_Motor.SlaveID;
            obj.send_data_frame(22:29) = data_buf;
            write(obj.serial_, obj.send_data_frame,"uint8");
            pause(0.001);
            obj.recv();
        end
        
        
        function control_Pos_Vel(obj, Motor, P_desired, V_desired)
            % Control the motor in position and velocity control mode
            if ~isKey(obj.motors_map, Motor.SlaveID)
                disp('Motor ID not found');
                return;
            end
            obj.send_data_frame(14) = Motor.SlaveID;
            obj.send_data_frame(15) = 0x01; % vel pos control need 0x100+canid
            data_buf = uint8(zeros(1, 8));
            P_desired_uint8s = obj.float_to_uint8s(P_desired);
            V_desired_uint8s = obj.float_to_uint8s(V_desired);
            data_buf(1:4) = P_desired_uint8s;
            data_buf(5:8) = V_desired_uint8s;
            obj.send_data_frame(22:29) = data_buf;
            write(obj.serial_, obj.send_data_frame,"uint8");
            pause(0.001);
            obj.recv();
        end

        function control_Vel(obj, Motor, Vel_desired)
            % control the motor in velocity control mode 电机速度控制模式
            % :param Motor: Motor object 电机对象
            % :param Vel_desired: desired velocity 期望速度
            if ~isKey(obj.motors_map, Motor.SlaveID)
                disp('Motor ID not found');
                return;
            end
            obj.send_data_frame(14) = Motor.SlaveID;
            obj.send_data_frame(15) = 2; % vel control need 0x200+canid
            data_buf = uint8([0, 0, 0, 0]);
            Vel_desired_uint8s = obj.float_to_uint8s(Vel_desired);
            data_buf(1:4) = Vel_desired_uint8s;
            obj.send_data_frame(22:25) = data_buf;
            write(obj.serial_, obj.send_data_frame,"uint8");
            pause(0.001);
            obj.recv(); % receive the data from serial port
        end
        
        function control_pos_force(obj, Motor, Pos_des, Vel_des, i_des)
            % control the motor in EMIT control mode 电机力位混合模式
            % :param Pos_des: desired position rad 期望位置 单位为rad
            % :param Vel_des: desired velocity rad/s 期望速度 为放大100倍
            % :param i_des: desired current rang 0-10000 期望电流标幺值放大10000倍
            % 电流标幺值：实际电流值除以最大电流值，最大电流见上电打印
            if ~isKey(obj.motors_map, Motor.SlaveID)
                disp('Motor ID not found');
                return;
            end
            obj.send_data_frame(14) = Motor.SlaveID;
            obj.send_data_frame(15) = 3; % vel control need 0x200+canid
            data_buf = uint8(zeros(1, 8));
            Pos_desired_uint8s = float_to_uint8s(Pos_des);
            data_buf(1:4) = Pos_desired_uint8s;
            Vel_uint = uint16(Vel_des);
            ides_uint = uint16(i_des);
            data_buf(5) = bitand(Vel_uint, 255);
            data_buf(6) = bitshift(Vel_uint, -8);
            data_buf(7) = bitand(ides_uint, 255);
            data_buf(8) = bitshift(ides_uint, -8);
            obj.send_data_frame(22:29) = data_buf;
            write(obj.serial_, obj.send_data_frame,"uint8");
            pause(0.001);
            obj.recv(); % receive the data from serial port
        end
        
        function enable(obj, Motor)
            % enable motor 使能电机
            % 最好在上电后几秒后再使能电机
            % :param Motor: Motor object 电机对象
            obj.control_cmd(Motor, uint8(0xFC));
        end
        
        function disable(obj, Motor)
            % disable motor 失能电机
            % :param Motor: Motor object 电机对象
            obj.control_cmd(Motor, uint8(0xFD));
        end
        
        function zero_position(obj, Motor)
            % set the zero position of the motor 设置电机0位
            % :param Motor: Motor object 电机对象
            obj.control_cmd(Motor, uint8(0xFE));
        end
        
        function recv(obj)
            read_num=obj.serial_.NumBytesAvailable;
            if read_num ~=0
                data_recv = read(obj.serial_, read_num ,"uint8");
                packets = obj.extract_packets(data_recv);
                for i = 1:length(packets)
                    packet = packets{i};
                    if length(packet) == 16
                        data = packet(8:15);
                        CANID = bitor(bitshift(uint32(packet(7)), 24), bitor(bitshift(uint32(packet(6)), 16), bitor(bitshift(uint32(packet(5)), 8), uint32(packet(4)))));
                        CMD = packet(2);
                        obj.process_packet(data, CANID, CMD);
                    end
                end
            end
        end

      function process_packet(obj, data, CANID, CMD)
            if CMD == 17
                if isKey(obj.motors_map, CANID)
                    q_uint = bitor(bitshift(uint16(data(2)), 8), uint16(data(3)));
                    dq_uint = bitor(bitshift(uint16(data(4)), 4), bitshift(uint16(data(5)), -4));
                    tau_uint = bitor(bitshift(bitand(uint16(data(5)), 15), 8), uint16(data(6)));
                    MotorType_recv = obj.motors_map(CANID).MotorType;
                    recv_q = obj.uint_to_float(q_uint, -obj.Limit_param(MotorType_recv,1), obj.Limit_param(MotorType_recv,1), 16);
                    recv_dq = obj.uint_to_float(dq_uint, -obj.Limit_param(MotorType_recv,2), obj.Limit_param(MotorType_recv,2), 12);
                    recv_tau = obj.uint_to_float(tau_uint, -obj.Limit_param(MotorType_recv,3), obj.Limit_param(MotorType_recv,3), 12);
                    obj.motors_map(CANID).recv_data(recv_q, recv_dq, recv_tau);
                end
            end
        end
        
        function addMotor(obj, Motor)
            % add motor to the motor control object 添加电机到电机控制对象
            % :param Motor: Motor object 电机对象
            obj.motors_map(Motor.SlaveID) = Motor;
            if Motor.MasterID ~= 0
                obj.motors_map(Motor.MasterID) = Motor;
            end
        end
        
        function control_cmd(obj, Motor, cmd)
            data_buf = uint8([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd]);
            obj.send_data_frame(14) = uint8(Motor.SlaveID);
            obj.send_data_frame(22:29) = data_buf;
            write(obj.serial_, obj.send_data_frame,"uint8");
        end
        
        function read_motor_param(obj, Motor, RID)
            data_buf = uint8([uint8(Motor.SlaveID), 0, 51, uint8(RID), 0, 0, 0, 0]);
            obj.send_data_frame(14) = 255;
            obj.send_data_frame(15) = 7;
            obj.send_data_frame(22:29) = data_buf;
            write(obj.serial_, obj.send_data_frame,"uint8");
        end
        
        function write_motor_param(obj, Motor, RID, data)
            data_buf = uint8([uint8(Motor.SlaveID), 0, 85, uint8(RID), 0, 0, 0, 0]);
            data_buf(5:8) = data;
            obj.send_data_frame(14) = 0xFF;
            obj.send_data_frame(15) = 0x07;
            obj.send_data_frame(22:29) = data_buf;
            write(obj.serial_, obj.send_data_frame,"uint8");
        end
        
        function switchControlMode(obj, Motor, ControlMode)
            % switch the control mode of the motor 切换电机控制模式
            % :param Motor: Motor object 电机对象
            % :param ControlMode: Control_Type 电机控制模式 example:MIT:Control_Type.MIT MIT模式
            write_data = uint8([uint8(ControlMode), 0, 0, 0]);
            obj.write_motor_param(Motor, 10, write_data);
        end
        
        function changeMasterID(obj, Motor, MasterID)
            % change the MasterID of the motor 改变电机主机ID
            % :param Motor: Motor object 电机对象
            % :param MasterID: MasterID 主机ID
            write_data = uint8([uint8(MasterID), 0, 0, 0]);
            obj.write_motor_param(Motor, 7, write_data);
        end
        
        function save_motor_param(obj, Motor)
            data_buf = uint8([uint8(Motor.SlaveID), 0, 0xAA, 0, 0, 0, 0, 0]);
            obj.send_data_frame(14) = 0xFF;
            obj.send_data_frame(15) = 0x07;
            obj.send_data_frame(22:29) = data_buf;
            write(obj.serial_, obj.send_data_frame,"uint8");
            
        end
        
        function change_control_PMAX(obj,Motor_Type,new_PMAX)
            %这是修改MotorControl类中电机对应的PMAX，不修改电机内部的
            obj.Limit_param(uint32(Motor_Type),1)=new_PMAX;
        end

        function change_control_VMAX(obj,Motor_Type,new_VMAX)
            %这是修改MotorControl类中电机对应的VMAX，不修改电机内部的
            obj.Limit_param(uint32(Motor_Type),2)=new_VMAX;
        end

        function change_control_TMAX(obj,Motor_Type,new_TMAX)
            %这是修改MotorControl类中电机对应的TMAX，不修改电机内部的
            obj.Limit_param(uint32(Motor_Type),3)=new_TMAX;
        end
        
        function x = LIMIT_MIN_MAX(obj,x, min_val, max_val)
            if x <= min_val
                x = min_val;
            elseif x > max_val
                x = max_val;
            end
        end

        function uint_val = float_to_uint(obj,x, x_min, x_max, bits)
            x = obj.LIMIT_MIN_MAX(x, x_min, x_max);
            span = x_max - x_min;
            data_norm = (x - x_min) / span;
            uint_val = uint16(data_norm * ((2^bits) - 1));
        end
        
        function float_val = uint_to_float(obj,x, min_val, max_val, bits)
            span = max_val - min_val;
            data_norm = double(x) / ((2^bits) - 1);
            float_val = single(data_norm * span + min_val);
        end
    
        function uint8s = float_to_uint8s(obj,value)
            % Pack the float into 4 bytes
            packed = typecast(single(value), 'uint8');
            % Return the bytes as a row vector
            uint8s = reshape(packed, 1, []);
        end
    
        function packets = extract_packets(obj,data)
            % Extract packets from the serial data
            packets = {};
            header = hex2dec('AA');
            tail = hex2dec('55');
            i = 1;
            while i <= length(data) - 1
                % Find the start of a packet
                if data(i) == header
                    start_index = i;
                    % Look for the end of the packet
                    i = i + 1;
                    while i <= length(data) && data(i) ~= tail
                        i = i + 1;
                    end
                    % If a tail is found, extract the packet
                    if i <= length(data) && data(i) == tail
                        end_index = i;
                        packets{end + 1} = data(start_index:end_index);
                    end
                end
                i = i + 1;
            end
        end
    end
end