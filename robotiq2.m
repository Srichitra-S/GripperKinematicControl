classdef robotiq2 < handle
    % This class includes all robot grippers from Robotiq that communicate
    % through Modbus RTU or TCP.
    
    properties
        ID = '09'
        COM
        Connected = 0
        Connection
        ConnectionMode
        OperationMode
        Buffer
    end
    
    methods
        
        % Object constructor
        function obj = robotiq
            disp('Welcome to Robotiq!');
        end
        
        % Gripper connection
        function obj = Connect(obj)
            serialinfo = instrhwinfo('serial');
            if ~isempty(serialinfo.AvailableSerialPorts)
                for i=1:length(serialinfo.AvailableSerialPorts)
                    obj.COM = char(serialinfo.AvailableSerialPorts(i));
                    break
                end
                obj.Connection = serial(obj.COM,'BaudRate',115200,'DataBits',8,'Parity','none','StopBits',1,'Timeout',1);
                fopen(obj.Connection);
                activate = calculoCRC(hex2dec([ obj.ID ;'06';'03';'E8';'01';'00']));
                fwrite(obj.Connection,activate);
                resp = fread(obj.Connection,8); % empty buffer
                status = calculoCRC(hex2dec([ obj.ID ;'03';'07';'D0';'00';'01'])); % asks for the status of the gripper
                try
                    fwrite(obj.Connection,status);
                    resp = fread(obj.Connection,7);
                    while ~strcmp(dec2hex(resp(4)),'31')
                        fwrite(obj.Connection,status);
                        resp = fread(obj.Connection,7);
                    end
                catch
                    disp('Connection not established for some unknown reason.');
                end
                obj.Connected = 1;
                obj.OperationMode = 'Basic';
                disp('Connection established.');
            else
                disp('There are no available ports.');
            end
        end
        
        % Change grasping mode
        function obj = ChangeMode(obj,mode)
            error_flag = 0;
            switch mode
                case 'Basic'
                    m = '0';
                    send = calculoCRC(hex2dec([obj.ID;'06';'03';'E8';'09';'00']));
                case 'Pinch'
                    m = '1';
                    send = calculoCRC(hex2dec([obj.ID;'06';'03';'E8';'0B';'00']));
                case 'Wide'
                    m = '2';
                    send = calculoCRC(hex2dec([obj.ID;'06';'03';'E8';'0D';'00']));
                case 'Scissor'
                    m = '3';
                    send = calculoCRC(hex2dec([obj.ID;'06';'03';'E8';'0F';'00']));
                case 'Individual'
                    % m = '2'; % -------------> pensar nesse caso
                    send = calculoCRC(hex2dec([obj.ID;'06';'03';'E8';'09';'0C']));
                otherwise
                    error_flag = 1;
                    disp('Invalid entry.');
            end
            if error_flag == 0
                obj.OperationMode = mode;
                fwrite(obj.Connection,send);
                resp = fread(obj.Connection,8); % empty buffer
                status = calculoCRC(hex2dec([obj.ID ;'03';'07';'D0';'00';'01'])); % asks for the status of the gripper
                try
                    fwrite(obj.Connection,status);
                    resp = fread(obj.Connection,7);
                    bin = de2bi(resp(4));
                    bin = dec2hex(bi2de(bin(2:3)));
                    if exist('m','var')
                        while ~strcmp(bin,m)
                            fwrite(obj.Connection,status);
                            resp = fread(obj.Connection,7);
                            bin = de2bi(resp(4));
                            bin = dec2hex(bi2de(bin(2:3)));
                        end
                    end
                    sprintf('%s mode activated.',mode)
                    if ~strcmp(mode,'Individual')
                        obj.Send([0 255 255]);
                    end
                catch ME
                    disp('Mode change was unsuccessful.');
                    ME
                end
            else
                disp('Invalid entry.');
            end
        end
        
        % Send position, velocity and force to gripper
        function resp = Send(obj,pose)
            switch obj.OperationMode
                case 'Basic'
                    send = calculoCRC([ hex2dec([obj.ID;'10';'03';'E8';'00';'03';'06';'09';'00';'00'])' pose(1,1) pose(1,2) pose(1,3)]);
                case 'Pinch'
                    send = calculoCRC([ hex2dec([obj.ID;'10';'03';'E8';'00';'03';'06';'0B';'00';'00'])' pose(1,1) pose(1,2) pose(1,3)]);
                case 'Wide'
                    send = calculoCRC([ hex2dec([obj.ID;'10';'03';'E8';'00';'03';'06';'0D';'00';'00'])' pose(1,1) pose(1,2) pose(1,3)]);
                case 'Scissor'
                    send = calculoCRC([ hex2dec([obj.ID;'10';'03';'E8';'00';'03';'06';'0F';'00';'00'])' pose(1,1) pose(1,2) pose(1,3)]);
                case 'Individual'
                    send = calculoCRC([ hex2dec([obj.ID;'10';'03';'E8';'00';'08';'10';'09';'0C'; '00' ])' pose(1,1) pose(1,2) pose(1,3) ...
                                        pose(2,1) pose(2,2) pose(2,3) pose(3,1) pose(3,2) pose(3,3) pose(4,1) pose(4,2) pose(4,3) 00 ]);
            end
            error_flag = 0;
            for i=1:length(send)
                if (send(i)<0)||(send(i)>255)
                    error_flag = 1;
                    break
                end
            end
            if error_flag == 0
                fwrite(obj.Connection,send);
                resp = fread(obj.Connection,8);
            else
                disp('Invalid entry.');
            end
        end
        
        % Receive data from gripper
        function state = Receive(obj)
            % [ 09 03 07 210 00 06 101 205 ]
            if (obj.Connection.BytesAvailable ~= 0)
                fread(obj.Connection,obj.Connection.BytesAvailable);
            end
            send = calculoCRC(hex2dec([obj.ID ;'03';'07';'D2';'00';'06']));
            fwrite(obj.Connection,send);
            try
                resp = fread(obj.Connection,17);
                posA = resp(4);
                currentA = resp(5);
                posB = resp(7);
                currentB = resp(8);
                posC = resp(10);
                currentC = resp(11);
                posS = resp(13);
                currentS = resp(14);
                state = [posA currentA ; posB currentB ; posC currentC ; posS currentS ];
            catch
                disp('Cannot read gripper status.');
                ME
            end
        end
        
        % Algoritmo para fechar a garra e detectar objeto. GP(X,Y) é uma
        % matrix 3x3 cujas linhas representam os dedos e as colunas
        % representam a coordenadas dos vetores de grasping, no frame do
        % objeto.
        function [ GP , DELTA ] = Detect(obj,scissor)     % MODIFICAR PARA PERMITIR GRASPING COM QUALQUER ANGULO DE SCISSOR
            id = str2num(obj.ID);
            % Garra fecha em sua vel. mínima.
            
            % Prepara a garra para a captura do objeto
            pose = [ 0 255 0 ; 0 255 0 ; 0 255 0 ; scissor 255 0 ];
            obj.Send(pose);
            pause(3);
            
            old_state = obj.Receive()
            i = old_state(1,1);
            % Incrementa a posição até o fechamento
            i = i + 2;
            pose = [ i 0 0 ; i 0 0 ; i 0 0 ; scissor 0 0 ];
            obj.Send(pose);
            pause(1);
            % Captura o state após o movimento
            state = obj.Receive()
            
            while state ~= old_state
                
                old_state = obj.Receive()
%                 i = old_state(1,1);
                % Incrementa a posição até o fechamento
                i = i + 2;
                pose = [ i 0 0 ; i 0 0 ; i 0 0 ; scissor 0 0 ];
                obj.Send(pose);
                pause(1);
                % Captura o state após o movimento
                state = obj.Receive()
                
            end
            
            state
            old_state
            
            pose = [ state(1,1) 255 0 ; state(2,1) 255 0 ; state(3,1) 255 0 ; state(4,1) 255 0 ];
            obj.Send(pose);
            
            'objeto detectado!'
            
%             move = [ id 16 03 232 00 06 12 11 00 00 255 0 0 255 0 0 255 0 0 ];
%             move = calculoCRC(move);
%             fwrite(obj.Connection,move);
%             resp = fread(obj.Connection,8);

            % Recupera o status da garra continuamente, até que um objeto seja
            % detectado.
%             grip_stopped = [ id 03 07 208 00 01 133 207];
%             fwrite(obj.Connection,grip_stopped);
%             resp = fread(obj.Connection,7);
%             gripper_status = resp(4);
%             object_status = resp(5);
% 
%             while (gripper_status~=123)&&(gripper_status~=187)
%                 grip_stopped = [ id 03 07 208 00 01 133 207];
%                 fwrite(obj.Connection,grip_stopped);
%                 resp = fread(obj.Connection,7);
%               %  pause(0.01);
%                 gripper_status = resp(4);
%                 obj_status = resp(5);
%             end

            % Quando o objeto for detectado, mantém a garra na posição atual.
%             fwrite(obj.Connection,[ id 03 07 210 00 06 101 205 ]);
%             resp = fread(obj.Connection,17);
%             posA = resp(4);
%             posB = resp(7);
%             posC = resp(10);
%             pos_scissor = resp(13);
%             move = [ id 16 03 232 00 06 12 11 00 00 posA 0 0 posB 0 0 posC 0 0 ];
%             move = calculoCRC(move);
%             fwrite(obj.Connection,move);
%             response = fread(obj.Connection,8);
            % 'objeto detectado!'

            % Usa a cinemática direta para descobrir a largura do objeto:
            % Converte os inteiros em ângulos e concatena num vetor para calcular a
            % cin. direta dos dedos.
%             thetaHand = byte2o([ posA posB posC pos_scissor ]);
%             
%             % Cinemática direta dos dedos
%             [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics(thetaHand);
%             
%             obj_dim = abs(p2(2)-p1(2)); % largura do objeto em y (mm)
% 
%             % Calcula os parâmetros de grasping.
%             GP = [   0   -obj_dim/2  0 ;...
%                    p2(1)  obj_dim/2  0 ;...
%                    p3(1)  obj_dim/2  0 ];
%             
%             % Calcula a separação dos dedos
%             DELTA = [ (p1-p2)' ;...
%                       (p2-p3)' ;...
%                       (p3-p1)' ];
            
        end
        
        % Close gripper
        function Close(obj)
            try
                obj.ChangeMode('Basic');
                pose = [ 255 255 0 ];
                obj.Send(pose);
            catch
                disp('Cannot close the gripper.');
                ME
            end
        end
        
        function Release( obj )
            % RELEASE! (solta objeto)
            id = str2num(obj.ID);
            op = obj.OperationMode;
            switch op
                case 'Basic'
                    move = [ id 16 03 232 00 08 16 09 12 00 0 255 0 0 255 0 0 255 0  0  255 0 00];
                case 'Pinch'
                    move = [ id 16 03 232 00 08 16 09 12 00 0 255 0 0 255 0 0 255 0 255 255 0 00];
                case 'Wide'
                    move = [ id 16 03 232 00 08 16 09 12 00 0 255 0 0 255 0 0 255 0 140 255 0 00];
                otherwise
                    move = [ id 16 03 232 00 08 16 09 12 00 0 255 0 0 255 0 0 255 0  0  255 0 00];
            end
            move = calculoCRC(move);
            fwrite(obj.Connection,move);
            resp = fread(obj.Connection,8); % pointless (empties buffer)
        end
        
        % Disconnect gripper
        function obj = Disconnect(obj)
            clear obj.Connection;
            delete(instrfind);
        end
        %
    end
    
end

