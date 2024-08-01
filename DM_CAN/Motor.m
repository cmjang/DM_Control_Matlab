classdef Motor < handle
    % Class to define a motor object
    properties
        Pd
        Vd
        cmd_q
        cmd_dq
        cmd_tau
        state_q
        state_dq
        state_tau
        cmd_kp
        cmd_kd
        SlaveID
        MasterID
        MotorType
    end
    methods
        function obj = Motor(MotorType, SlaveID, MasterID)
            % Constructor to initialize motor object
            % MotorType: Motor type
            % SlaveID: CANID motor ID
            % MasterID: Master ID, recommended not to set to 0
            obj.Pd = single(0);
            obj.Vd = single(0);
            obj.cmd_q = single(0);
            obj.cmd_dq = single(0);
            obj.cmd_tau = single(0);
            obj.state_q = single(0);
            obj.state_dq = single(0);
            obj.state_tau = single(0);
            obj.cmd_kp = single(0);
            obj.cmd_kd = single(0);
            obj.SlaveID = uint32(SlaveID);
            obj.MasterID = uint32(MasterID);
            obj.MotorType = uint32(MotorType);
        end
        
        function obj = recv_data(obj, q, dq, tau)
            % Method to receive data and update motor state
            obj.state_q = q;
            obj.state_dq = dq;
            obj.state_tau = tau;
        end
        
        function obj = save_cmd(obj, cmd_kp, cmd_kd, q, dq, tau)
            % Method to save command data
            obj.cmd_q = q;
            obj.cmd_dq = dq;
            obj.cmd_tau = tau;
            obj.cmd_kp = cmd_kp;
            obj.cmd_kd = cmd_kd;
        end
        
        function q = getPosition(obj)
            % Method to get the position of the motor
            q = obj.state_q;
        end
        
        function dq = getVelocity(obj)
            % Method to get the velocity of the motor
            dq = obj.state_dq;
        end
        
        function tau = getTorque(obj)
            % Method to get the torque of the motor
            tau = obj.state_tau;
        end
    end
end