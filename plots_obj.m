% Plota gráficos dos experimentos de manipulação do objeto
close all

load ('obj_ctrl_reg_Ko20_Kr10_released');

TIT = 12; LBL = 12; LEG = 13; LWR = 1.5; LW = 1.5;

total_time = 60;

% Track
fig1 = figure(1);
set(fig1, 'Position', [403 246 560 1200]);
subplot(4,1,1);
    plot(time_vec,ref_vec(1,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(a) Object pos, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,state_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg1 = legend('$p_{od,y}$','$p_{o,y}$','Orientation','Horizontal');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -40 40]);
subplot(4,1,2);
    plot(time_vec,ref_vec(2,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(b) Rel pos fingers 1-2, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,state_vec(2,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg2 = legend('$p_{rd,y}$','$p_{r,y}$','Orientation','Horizontal');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -160 -60]);
subplot(4,1,3);
    plot(time_vec,ref_vec(3,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(c) Rel pos fingers 2-3, x-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,state_vec(3,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg3 = legend('$p_{rd,x}$','$p_{r,x}$','Orientation','Horizontal');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');  
    axis([0 total_time 20 150]);
subplot(4,1,4);
    plot(time_vec,ref_vec(4,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(d) Rel pos fingers 2-3, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,state_vec(4,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg4 = legend('$p_{rd,y}$','$p_{r,y}$','Orientation','Horizontal');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -5 5]);

% Errors
fig2 = figure(2);
subplot(2,2,1)
    plot(time_vec,error_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(a) Object pos. error, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('$e_{o,y}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -40 40]);
subplot(2,2,2)
    plot(time_vec,error_vec(2,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(b) Rel. pos. error, fingers 1-2, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('$e_{r,y}$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -10 90]);
subplot(2,2,3)
    plot(time_vec,error_vec(3,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(c) Rel pos error, fingers 2-3','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('$e_{r,x}$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -120 10]);
subplot(2,2,4)
    plot(time_vec,error_vec(4,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(d) Rel pos error, fingers 2-3','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg4 = legend('$e_{r,y}$');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -5 5]);

% Position Control Signals
fig3 = figure(3);
subplot(2,2,1)
    plot(time_vec,v_ctrl_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(a) Pos. ctrl. obj.','Interpreter','latex','FontSize',TIT);
    ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('$v_{o,y}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -750 750]);
subplot(2,2,2)
    plot(time_vec,v_ctrl_vec(2,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
    title('(b) Pos. ctrl. fingers 1-2, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('$v_{r,y}\,\,1\!-\!2$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -100 850]);
subplot(2,2,3)
    plot(time_vec,v_ctrl_vec(3,:),'-','Linewidth',LW,'Color',[1 0 0]); hold on; grid on;
    title('(c) Pos. ctrl. fingers 2-3, x-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('$v_{r,x}\,\,2\!-\!3$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -1200 200]);
subplot(2,2,4)
    plot(time_vec,v_ctrl_vec(4,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
    title('(d) Pos. ctrl. fingers 2-3, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg4 = legend('$v_{r,y}\,\,2\!-\!3$');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
    axis([0 total_time -50 50]);

fig4 = figure(4);    
subplot(2,2,1)
    plot(time_vec,dstate_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(a) Joint ctrl finger 1','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('$\dot{\theta}_{1a}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -10 10]);
subplot(2,2,2)
    plot(time_vec,dstate_vec(2,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
    title('(b) Joint ctrl finger 2','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('$\dot{\theta}_{2a}$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
    axis([0 total_time -10 10]);
subplot(2,2,3)
    plot(time_vec,dstate_vec(3,:),'-','Linewidth',LW,'Color',[1 0 0]); hold on; grid on;
    title('(c) Joint ctrl finger 3','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('$\dot{\theta}_{3a}$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
    axis([0 total_time -10 10]);
subplot(2,2,4)
    plot(time_vec,dstate_vec(4,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
    title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg4 = legend('$\dot{\theta}_{4a}$');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -5 5]);

% saveas(fig1,'reg_state_obj','fig')
% saveas(fig2,'reg_error_obj','fig')
% saveas(fig3,'reg_pos_ctrl_obj','fig')  
% saveas(fig4,'reg_joint_ctrl_obj','fig')

% saveas(fig1,'reg_state_obj','jpeg')
% saveas(fig2,'reg_error_obj','jpeg')
% saveas(fig3,'reg_pos_ctrl_obj','jpeg')
% saveas(fig4,'reg_joint_ctrl_obj','jpeg')

% saveas(fig1,'reg_state_obj','epsc')
% saveas(fig2,'reg_error_obj','epsc')
% saveas(fig3,'reg_pos_ctrl_obj','epsc')
% saveas(fig4,'reg_joint_ctrl_obj','epsc')
