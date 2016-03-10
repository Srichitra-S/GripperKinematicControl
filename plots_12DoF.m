% Plots for 12 DoF robot gripper.

load ('12DOF_exp')

TIT = 12; LBL = 12; LEG = 13; LWR = 1.5; LW = 1.0;

total_time = 60;
[~,s] = find(time_vec<total_time);
index_limit = max(s);

% Track
fig1 = figure(1);
% set(fig1, 'Position', [403 246 560 600]);
subplot(3,1,1);
    plot(time_vec,ref_vec(1,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(a) Object pos., x-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    
    plot(time_vec,state_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg1 = legend('$p_{od}$','$p_{o}$','Orientation','Vertical');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -40 40]);
subplot(3,1,2);
    plot(time_vec,ref_vec(2,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(b) Object pos., y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,state_vec(2,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg2 = legend('$p_{od}$','$p_{o}$','Orientation','Vertical');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -40 40]);
subplot(3,1,3);
    plot(time_vec,ref_vec(3,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(c) Object pos., z-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,state_vec(3,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg3 = legend('$p_{od}$','$p_{o}$','Orientation','Vertical');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');  
    axis([0 total_time -0 100]);


% % Relative states
% fig2 = figure(2);
% subplot(3,2,1)
%     plot(time_vec,ref_vec(4,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(a) Rel. pos. fingers 1-2, x-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     plot(time_vec,state_vec(4,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     leg1 = legend('$p_{rd}$','$p_{r}$','Orientation','Vertical');
%     set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -35 15]);
% subplot(3,2,2)
%     plot(time_vec,ref_vec(5,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(b) Rel. pos. fingers 1-2, y-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     plot(time_vec,state_vec(5,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     leg2 = legend('$p_{rd}$','$p_{r}$','Orientation','Vertical');
%     set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -80 0]);
% subplot(3,2,3)
%     plot(time_vec,ref_vec(6,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(c) Rel. pos. fingers 1-2, z-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
% %     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     plot(time_vec,state_vec(6,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     leg3 = legend('$p_{rd}$','$p_{r}$','Orientation','Vertical');
%     set(leg3,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
%     axis([0 total_time -40 40]);
% subplot(3,2,4)
%     plot(time_vec,ref_vec(7,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(d) Rel. pos. fingers 2-3, x-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
% %     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     plot(time_vec,state_vec(7,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     leg4 = legend('$p_{rd}$','$p_{r}$','Orientation','Vertical');
%     set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
% %     axis([0 total_time -5 5]);
% subplot(3,2,5)
%     plot(time_vec,ref_vec(8,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(e) Rel. pos. fingers 2-3, y-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     plot(time_vec,state_vec(8,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     leg5 = legend('$p_{rd}$','$p_{r}$','Orientation','Vertical');
%     set(leg5,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -5 5]);
% subplot(3,2,6)
%     plot(time_vec,ref_vec(9,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(f) Rel. pos. fingers 2-3, z-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     plot(time_vec,state_vec(9,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     leg6 = legend('$p_{rd}$','$p_{r}$','Orientation','Vertical');
%     set(leg6,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -5 5]);


% Object errors
fig3 = figure(3);
% set(fig3, 'Position', [403 246 560 600]);
subplot(3,1,1)
    plot(time_vec,error_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(a) Object error, x-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('${\tilde \xi}_{o}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -10 10]);
subplot(3,1,2)
    plot(time_vec,error_vec(2,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(b) Object error, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('${\tilde \xi}_{o}$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -10 10]);
subplot(3,1,3)
    plot(time_vec,error_vec(3,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(c) Object error, z-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('${\tilde \xi}_{o}$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -10 10]);


% % Relative errors
% fig4 = figure(4);
% subplot(3,2,1)
%     plot(time_vec,error_vec(4,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(d) Rel. error, fingers 1-2, x-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg4 = legend('$e_{r,x}\,\,1\!-\!2$');
%     set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -5 5]);
% subplot(3,2,2)
%     plot(time_vec,error_vec(5,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(d) Rel. error, fingers 1-2, y-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg5 = legend('$e_{r,y}\,\,1\!-\!2$');
%     set(leg5,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -5 5]);
% subplot(3,2,3)
%     plot(time_vec,error_vec(6,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(d) Rel. error, fingers 1-2, z-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg6 = legend('$e_{r,z}\,\,1\!-\!2$');
%     set(leg6,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -5 5]);
% subplot(3,2,4)
%     plot(time_vec,error_vec(7,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(d) Rel. error, fingers 2-3, x-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg7 = legend('$e_{r,x}\,\,2\!-\!3$');
%     set(leg7,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -5 5]);
% subplot(3,2,5)
%     plot(time_vec,error_vec(8,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(d) Rel. error, fingers 2-3, y-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg8 = legend('$e_{r,y}\,\,2\!-\!3$');
%     set(leg8,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -5 5]);
% subplot(3,2,6)
%     plot(time_vec,error_vec(9,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(d) Rel. error, fingers 2-3, z-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg9 = legend('$e_{r,z}\,\,2\!-\!3$');
%     set(leg9,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -5 5]);


% % Position Control Signals (object)
% fig5 = figure(5);
% subplot(3,1,1)
%     plot(time_vec,v_ctrl_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(a) Pos. ctrl. obj., x-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
% %     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg1 = legend('$v_{o,x}$');
%     set(leg1,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
%     axis([0 total_time -100 100]);
% subplot(3,1,2)
%     plot(time_vec,v_ctrl_vec(2,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(b) Pos. ctrl. obj., y-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
% %     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg2 = legend('$v_{o,y}$');
%     set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -100 100]);
% subplot(3,1,3)
%     plot(time_vec,v_ctrl_vec(3,:),'-','Linewidth',LW,'Color',[1 0 0]); hold on; grid on;
%     title('(c) Pos. ctrl. obj., z-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg3 = legend('$v_{o,z}$');
%     set(leg3,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
%     axis([0 total_time -100 100]);


% Position Control Signals (relative)
% fig6 = figure(6);
% subplot(3,2,1)
%     plot(time_vec,v_ctrl_vec(4,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
%     title('(d) Pos. ctrl. fingers 1-2, x-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg4 = legend('$v_{r,x}\,\,1\!-\!2$');
%     set(leg4,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
%     axis([0 total_time -10 10]);
% subplot(3,2,2)
%     plot(time_vec,v_ctrl_vec(5,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
%     title('(d) Pos. ctrl. fingers 1-2, y-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg5 = legend('$v_{r,y}\,\,1\!-\!2$');
%     set(leg5,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
%     axis([0 total_time -10 10]);
% subplot(3,2,3)
%     plot(time_vec,v_ctrl_vec(6,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
%     title('(d) Pos. ctrl. fingers 1-2, z-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg6 = legend('$v_{r,z}\,\,1\!-\!2$');
%     set(leg6,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
%     axis([0 total_time -10 10]);
% subplot(3,2,4)
%     plot(time_vec,v_ctrl_vec(7,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
%     title('(d) Pos. ctrl. fingers 2-3, x-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg7 = legend('$v_{r,x}\,\,2\!-\!3$');
%     set(leg7,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
%     axis([0 total_time -10 10]);
% subplot(3,2,5)
%     plot(time_vec,v_ctrl_vec(8,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
%     title('(d) Pos. ctrl. fingers 2-3, y-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg8 = legend('$v_{r,y}\,\,2\!-\!3$');
%     set(leg8,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
%     axis([0 total_time -10 10]);
% subplot(3,2,6)
%     plot(time_vec,v_ctrl_vec(9,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
%     title('(d) Pos. ctrl. fingers 2-3, z-axis','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg9 = legend('$v_{r,z}\,\,2\!-\!3$');
%     set(leg9,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
%     axis([0 total_time -10 10]);


% Joint control signals
fig7 = figure(7);
set(fig7, 'Position', [403 246 1000 600]);
% title('Joint ctrl signals','Interpreter','latex','FontSize',TIT);
subplot(3,4,1)
    plot(time_vec,dstate_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(a) Joint ctrl finger 1','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('$\dot{\theta}_{11}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
subplot(3,4,2)
    plot(time_vec,dstate_vec(2,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(b) Joint ctrl finger 1','Interpreter','latex','FontSize',TIT);
%     ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('$\dot{\theta}_{12}$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
subplot(3,4,3)
    plot(time_vec,dstate_vec(3,:),'-','Linewidth',LW,'Color',[1 0 0]); hold on; grid on;
%     title('(c) Joint ctrl finger 1','Interpreter','latex','FontSize',TIT);
%     ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('$\dot{\theta}_{13}$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
subplot(3,4,4)
    plot(time_vec,dstate_vec(4,:),'-','Linewidth',LW,'Color',[0 1 0]); hold on; grid on;
%     title('(c) Joint ctrl finger 1','Interpreter','latex','FontSize',TIT);
%     ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg4 = legend('$\dot{\theta}_{1s}$');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(3,4,5)
    plot(time_vec,dstate_vec(5,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('$\dot{\theta}_{21}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
subplot(3,4,6)
    plot(time_vec,dstate_vec(6,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
%     ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('$\dot{\theta}_{22}$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
subplot(3,4,7)
    plot(time_vec,dstate_vec(7,:),'-','Linewidth',LW,'Color',[1 0 0]); hold on; grid on;
%     title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
%     ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('$\dot{\theta}_{23}$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
subplot(3,4,8)
    plot(time_vec,dstate_vec(8,:),'-','Linewidth',LW,'Color',[0 1 0]); hold on; grid on;
%     title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
%     ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg4 = legend('$\dot{\theta}_{2s}$');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(3,4,9)
    plot(time_vec,dstate_vec(9,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
%     title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('$\dot{\theta}_{31}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
subplot(3,4,10)
    plot(time_vec,dstate_vec(10,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
%     title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
%     ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('$\dot{\theta}_{32}$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
subplot(3,4,11)
    plot(time_vec,dstate_vec(11,:),'-','Linewidth',LW,'Color',[1 0 0]); hold on; grid on;
%     title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
%     ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('$\dot{\theta}_{33}$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
subplot(3,4,12)
    plot(time_vec,dstate_vec(12,:),'-','Linewidth',LW,'Color',[0 1 0]); hold on; grid on;
%     title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
%     ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg4 = legend('$\dot{\theta}_{3s}$');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -3 3]);
    
fig10 = figure(10);
set(fig10, 'Position', [403 246 560 550]);
plot3(ref_vec(1,it_ctrl:index_limit),ref_vec(2,it_ctrl:index_limit),ref_vec(3,it_ctrl:index_limit),'--','Linewidth',LWR,'Color',[0 0.7 0]); 
hold on; grid on; view([50 36]);
title('Reference and object trajectory','Interpreter','latex','FontSize',TIT);
xlabel('$x \,, mm$','Interpreter','latex','FontSize',LBL);
ylabel('$y \,, mm$','Interpreter','latex','FontSize',LBL);
zlabel('$z \,, mm$','Interpreter','latex','FontSize',LBL);
plot3(state_vec(1,it_ctrl:index_limit),state_vec(2,it_ctrl:index_limit),state_vec(3,it_ctrl:index_limit),'-','Linewidth',LW,'Color',[0 0 1]); grid on;
leg10 = legend('$p_{od}(t)$','$p_{o}(t)$');
set(leg10,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
axis([-40 40 -40 40 0 100]);

% saveas(fig7,'12DOF_joint_ctrl','epsc')