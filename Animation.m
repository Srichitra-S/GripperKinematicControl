%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Matheus Ferreira dos Reis - Robotiq gripper class %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Implements high-level methods for the animation of a 12 DoF robot gripper.

classdef Animation < handle
    
    properties
        l1; l2; l3; l4a; l4b; l4c;
        xa; ya; za; xb; yb; zb; xc; yc; zc;
        x; y; z;                x1; y1; z1;
        fig; esfera; xarrow; yarrow; zarrow;
        f4A; f1A; f2A; f3A; pA;
        f4B; f1B; f2B; f3B; pB;
        f4C; f1C; f2C; f3C; pC;
    end
    
    methods
        % Object constructor
        function obj = Animation( varargin )
            
            nVar = length(varargin);
            
            % Comprimentos das falanges
            obj.l1 = 5.7; obj.l2 = 3.8; obj.l3 = 2.2;
            obj.l4a = 2.2; obj.l4b = 2.2; obj.l4c = 2.2;

            % Posições iniciais das bases dos dedos
            obj.xa = -4.45; obj.ya = 0; obj.za = -3.2;      % dedo A
            obj.xb = 4.45; obj.yb = -3.65; obj.zb = -3.2;   % dedo B 
            obj.xc = 4.45; obj.yc = 3.65; obj.zc = -3.2;    % dedo C

            % Inicializa figura
            
            switch nVar
                case 0 
                    obj.fig = figure;
                case 1
                    obj.fig = figure;
                    view(varargin{1});    % muda o viewpoint
                case 2
                    obj.fig = varargin{1};
                    view(varargin{2});    % muda o viewpoint
            end
            set(obj.fig, 'Position', [403 246 560 550]);
            title('Object control simulation','Interpreter','latex','FontSize',12);
            xlabel('$x \,, cm$','Interpreter','latex','FontSize',12);
            ylabel('$-y \,, cm$','Interpreter','latex','FontSize',12);
            zlabel('$z \,, cm$','Interpreter','latex','FontSize',12);
            axis([-9 9 -9 9 -5 20]);
            hold on; grid on;
            rotate3d on;

            % Cilindro e esfera modelos
            [obj.x,obj.y,obj.z] = cylinder(0.5,20);
            [obj.x1, obj.y1, obj.z1] = sphere;
            
%             % Objetos gráficos principais
%             obj.cilindro = mesh(obj.x,obj.y,obj.z);
            obj.esfera = mesh(obj.x1,obj.y1,obj.z1);
            
            % Dedo A
            obj.f4A = mesh(obj.x,obj.y,obj.z);
            obj.f1A = mesh(obj.x,obj.y,obj.z);
            obj.f2A = mesh(obj.x,obj.y,obj.z);
            obj.f3A = mesh(obj.x,obj.y,obj.z);
            obj.pA = mesh(obj.x1,obj.y1,obj.z1);
            
            % Dedo B
            obj.f4B = mesh(obj.x,obj.y,obj.z);
            obj.f1B = mesh(obj.x,obj.y,obj.z);
            obj.f2B = mesh(obj.x,obj.y,obj.z);
            obj.f3B = mesh(obj.x,obj.y,obj.z);
            obj.pB = mesh(obj.x1,obj.y1,obj.z1);
            
            % Dedo C
            obj.f4C = mesh(obj.x,obj.y,obj.z);
            obj.f1C = mesh(obj.x,obj.y,obj.z);
            obj.f2C = mesh(obj.x,obj.y,obj.z);
            obj.f3C = mesh(obj.x,obj.y,obj.z);
            obj.pC = mesh(obj.x1,obj.y1,obj.z1);

            % Arrows
            xx = quiver3(0, 0, 0, 1, 1, 1,'Color','g','LineWidth',2);
            yy = quiver3(0, 0, 0, 1, 1, 1,'Color','g','LineWidth',2);
            zz = quiver3(0, 0, 0, 1, 1, 1,'Color','g','LineWidth',2);
            obj.xarrow = xx; obj.yarrow = yy; obj.zarrow = zz;
        end
        
        function Animate_Obj( obj, thetaHand, r, po, Rpo, varargin )
            
            if ~isempty(varargin)
                view(varargin{1});    % muda o viewpoint
            end

            % Conversão para cm
            po = po/10;
            r = r/10;
            
            % Definição dos ângulos
            o1a = -thetaHand(1);
            o2a = -thetaHand(2);
            o3a = -thetaHand(3);
            o4a = -thetaHand(4);
            o1b = thetaHand(5);
            o2b = thetaHand(6);
            o3b = thetaHand(7);
            o4b = -thetaHand(8);
            o1c = thetaHand(9);
            o2c = thetaHand(10);
            o3c = thetaHand(11);
            o4c = thetaHand(12);
            
            % Posições das falanges - dedo A
            initdAl4 = [obj.xa obj.ya obj.za];
            vecdAl4 = initdAl4 + [0 obj.l4a*sin(o4a) obj.l4a*cos(o4a)];
            initdAf1 = vecdAl4;
            vecdAf1 = initdAf1 + [-obj.l1*sin(o1a) obj.l1*cos(o1a)*sin(o4a) obj.l1*cos(o1a)*cos(o4a)];
            initdAf2 = vecdAf1;
            vecdAf2 = initdAf2 + [-obj.l2*sin(o1a+o2a) obj.l2*cos(o1a+o2a)*sin(o4a) obj.l2*cos(o1a+o2a)*cos(o4a)];
            initdAf3 = vecdAf2;
            vecdAf3 = initdAf3 + [-obj.l3*sin(o1a+o2a+o3a) obj.l3*cos(o1a+o2a+o3a)*sin(o4a) obj.l3*cos(o1a+o2a+o3a)*cos(o4a)];
            
            % Posições das falanges - dedo B
            initdBl4 = [obj.xb obj.yb obj.zb];
            vecdBl4 = initdBl4 + [0 obj.l4b*sin(o4b) obj.l4b*cos(o4b)];
            initdBf1 = vecdBl4;
            vecdBf1 = initdBf1 + [-obj.l1*sin(o1b) obj.l1*cos(o1b)*sin(o4b) obj.l1*cos(o1b)*cos(o4b)];
            initdBf2 = vecdBf1;
            vecdBf2 = initdBf2 + [-obj.l2*sin(o1b+o2b) obj.l2*cos(o1b+o2b)*sin(o4b) obj.l2*cos(o1b+o2b)*cos(o4b)];
            initdBf3 = vecdBf2;
            vecdBf3 = initdBf3 + [-obj.l3*sin(o1b+o2b+o3b) obj.l3*cos(o1b+o2b+o3b)*sin(o4b) obj.l3*cos(o1b+o2b+o3b)*cos(o4b)];

            % Posições das falanges - dedo B
            initdCl4 = [obj.xc obj.yc obj.zc];
            vecdCl4 = initdCl4 + [0 obj.l4c*sin(o4c) obj.l4c*cos(o4c)];
            initdCf1 = vecdCl4;
            vecdCf1 = initdCf1 + [-obj.l1*sin(o1c) obj.l1*cos(o1c)*sin(o4c) obj.l1*cos(o1c)*cos(o4c)];
            initdCf2 = vecdCf1;
            vecdCf2 = initdCf2 + [-obj.l2*sin(o1c+o2c) obj.l2*cos(o1c+o2c)*sin(o4c) obj.l2*cos(o1c+o2c)*cos(o4c)];
            initdCf3 = vecdCf2;
            vecdCf3 = initdCf3 + [-obj.l3*sin(o1c+o2c+o3c) obj.l3*cos(o1c+o2c+o3c)*sin(o4c) obj.l3*cos(o1c+o2c+o3c)*cos(o4c)];
            
            % Contact points
            p_p1 = Rpo*([5 0 0]'); p_p2 = Rpo*([0 5 0]'); p_p3 = Rpo*([0 0 5]');
            
            % Update arrows
            obj.xarrow.XData = po(2); obj.xarrow.YData = -po(1); obj.xarrow.ZData = po(3);
            obj.xarrow.UData = p_p1(2); obj.xarrow.VData = p_p1(1); obj.xarrow.WData = p_p1(3);
            
            obj.yarrow.XData = po(2); obj.yarrow.YData = -po(1); obj.yarrow.ZData = po(3);
            obj.yarrow.UData = p_p2(2); obj.yarrow.VData = p_p2(1); obj.yarrow.WData = p_p2(3);
            
            obj.zarrow.XData = po(2); obj.zarrow.YData = -po(1); obj.zarrow.ZData = po(3);
            obj.zarrow.UData = p_p3(2); obj.zarrow.VData = p_p3(1); obj.zarrow.WData = p_p3(3);
            
            % Update sphere
            obj.esfera.XData = obj.y1*r+po(2);
            obj.esfera.YData = -obj.x1*r-po(1);
            obj.esfera.ZData = obj.z1*r+po(3);
            
            % Update finger A
            set(obj.f4A,'xdata',obj.x + obj.xa,'ydata',obj.y + obj.ya,'zdata',obj.z*obj.l4a + obj.za)
            rotate(obj.f4A,[1 0 0],rad2deg(-o4a),[obj.xa obj.ya obj.za]);
            set(obj.f1A,'xdata',obj.x + vecdAl4(1),'ydata',obj.y + vecdAl4(2),'zdata',obj.z*obj.l1 + vecdAl4(3));
            rotate(obj.f1A,[1 0 0],rad2deg(-o4a),[vecdAl4(1) vecdAl4(2) vecdAl4(3)]);
            rotate(obj.f1A,[0 cos(-o4a) sin(-o4a)],rad2deg(-o1a),[vecdAl4(1) vecdAl4(2) vecdAl4(3)]);
            set(obj.f2A,'xdata',obj.x + vecdAf1(1),'ydata',obj.y + vecdAf1(2),'zdata',obj.z*obj.l2 + vecdAf1(3));
            rotate(obj.f2A,[1 0 0],rad2deg(-o4a),[vecdAf1(1) vecdAf1(2) vecdAf1(3)]);
            rotate(obj.f2A,[0 cos(-o4a) sin(-o4a)],rad2deg(-o1a-o2a),[vecdAf1(1) vecdAf1(2) vecdAf1(3)]);
            set(obj.f3A,'xdata',obj.x + vecdAf2(1),'ydata',obj.y + vecdAf2(2),'zdata',obj.z*obj.l3 + vecdAf2(3));
            rotate(obj.f3A,[1 0 0],rad2deg(-o4a),[vecdAf2(1) vecdAf2(2) vecdAf2(3)]);
            rotate(obj.f3A,[0 cos(-o4a) sin(-o4a)],rad2deg(-o1a-o2a-o3a),[vecdAf2(1) vecdAf2(2) vecdAf2(3)]);
            set(obj.pA,'xdata',obj.x1*0.5+vecdAf3(1),'ydata',obj.y1*0.5+vecdAf3(2),'zdata',obj.z1*0.5+vecdAf3(3));
            
            % Update finger B
            set(obj.f4B,'xdata',obj.x + obj.xb,'ydata',obj.y + obj.yb,'zdata',obj.z*obj.l4b + obj.zb);
            rotate(obj.f4B,[1 0 0],rad2deg(-o4b),[obj.xb obj.yb obj.zb]); 
            set(obj.f1B,'xdata',obj.x + vecdBl4(1),'ydata',obj.y + vecdBl4(2),'zdata',obj.z*obj.l1 + vecdBl4(3));
            rotate(obj.f1B,[1 0 0],rad2deg(-o4b),[vecdBl4(1) vecdBl4(2) vecdBl4(3)]);
            rotate(obj.f1B,[0 cos(-o4b) sin(-o4b)],rad2deg(-o1b),[vecdBl4(1) vecdBl4(2) vecdBl4(3)]);
            set(obj.f2B,'xdata',obj.x + vecdBf1(1),'ydata',obj.y + vecdBf1(2),'zdata',obj.z*obj.l2 + vecdBf1(3));
            rotate(obj.f2B,[1 0 0],rad2deg(-o4b),[vecdBf1(1) vecdBf1(2) vecdBf1(3)]);
            rotate(obj.f2B,[0 cos(-o4b) sin(-o4b)],rad2deg(-o1b-o2b),[vecdBf1(1) vecdBf1(2) vecdBf1(3)]);            
            set(obj.f3B,'xdata',obj.x + vecdBf2(1),'ydata',obj.y + vecdBf2(2),'zdata',obj.z*obj.l3 + vecdBf2(3));
            rotate(obj.f3B,[1 0 0],rad2deg(-o4b),[vecdBf2(1) vecdBf2(2) vecdBf2(3)]);
            rotate(obj.f3B,[0 cos(-o4b) sin(-o4b)],rad2deg(-o1b-o2b-o3b),[vecdBf2(1) vecdBf2(2) vecdBf2(3)]);            
            set(obj.pB,'xdata',obj.x1*0.5+vecdBf3(1),'ydata',obj.y1*0.5+vecdBf3(2),'zdata',obj.z1*0.5+vecdBf3(3));

            % Update finger C
            set(obj.f4C,'xdata',obj.x + obj.xc,'ydata',obj.y + obj.yc,'zdata',obj.z*obj.l4c + obj.zc);
            rotate(obj.f4C,[1 0 0],rad2deg(-o4c),[obj.xc obj.yc obj.zc]);
            set(obj.f1C,'xdata',obj.x + vecdCl4(1),'ydata',obj.y + vecdCl4(2),'zdata',obj.z*obj.l1 + vecdCl4(3));
            rotate(obj.f1C,[1 0 0],rad2deg(-o4c),[vecdCl4(1) vecdCl4(2) vecdCl4(3)]);
            rotate(obj.f1C,[0 cos(-o4c) sin(-o4c)],rad2deg(-o1c),[vecdCl4(1) vecdCl4(2) vecdCl4(3)]);
            set(obj.f2C,'xdata',obj.x + vecdCf1(1),'ydata',obj.y + vecdCf1(2),'zdata',obj.z*obj.l2 + vecdCf1(3));
            rotate(obj.f2C,[1 0 0],rad2deg(-o4c),[vecdCf1(1) vecdCf1(2) vecdCf1(3)]);
            rotate(obj.f2C,[0 cos(-o4c) sin(-o4c)],rad2deg(-o1c-o2c),[vecdCf1(1) vecdCf1(2) vecdCf1(3)]);
            set(obj.f3C,'xdata',obj.x + vecdCf2(1),'ydata',obj.y + vecdCf2(2),'zdata',obj.z*obj.l3 + vecdCf2(3));
            rotate(obj.f3C,[1 0 0],rad2deg(-o4c),[vecdCf2(1) vecdCf2(2) vecdCf2(3)]);
            rotate(obj.f3C,[0 cos(-o4c) sin(-o4c)],rad2deg(-o1c-o2c-o3c),[vecdCf2(1) vecdCf2(2) vecdCf2(3)]);
            set(obj.pC,'xdata',obj.x1*0.5+vecdCf3(1),'ydata',obj.y1*0.5+vecdCf3(2),'zdata',obj.z1*0.5+vecdCf3(3));
            
            pause(0.001);
            
        end
        
    end
    
end

