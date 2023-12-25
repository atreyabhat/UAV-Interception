function ax = plotting(z,t)

ax = axes('NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
    'Xlim',[t(1), t(end)],...
    'TickLabelInterpreter','LaTeX','FontSize',14);
xlabel('t','Interpreter','LaTeX','FontSize',14);

plot(ax, t, z(:,1:3), 'LineWidth', 1.5);
legend(ax, {'$x_1$', '$x_2$', '$x_3$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax, '${\bf x}$','Interpreter','LaTeX','FontSize',14);

% for i=1:1
%     ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
%                 'Xlim',[t(1), t(end)],...
%                 'TickLabelInterpreter','LaTeX','FontSize',14);
%     xlabel('t','Interpreter','LaTeX','FontSize',14);        
% end
% 
% 
% plot(ax(1), t,z(:,1:3), 'LineWidth', 1.5);
% legend(ax(1), {'$x_1$', '$x_2$', '$x_3$'},... 
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(1), '${\bf x}$','Interpreter','LaTeX','FontSize',14);
% xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',14);

% plot(ax(3), t, z(:,4:6), 'LineWidth', 1.5);
% legend(ax(3), {'$\phi$', '$\theta$', '$\psi$'},...
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(3), '\boldmath$\alpha$','Interpreter','LaTeX','FontSize',14);
% 
% plot(ax(2), t, z(:,7:9), 'LineWidth', 1.5);
% legend(ax(2), {'$\dot{x}_1$', '$\dot{x}_2$', '$\dot{x}_3$'},...
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(2), '$\dot{\bf x}$','Interpreter','LaTeX','FontSize',14);
% 
% plot(ax(4), t, z(:,10:12), 'LineWidth', 1.5);
% legend(ax(4), {'$\omega_1$', '$\omega_2$', '$\omega_3$'},...
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(4), '\boldmath$\omega$','Interpreter','LaTeX','FontSize',14);


end