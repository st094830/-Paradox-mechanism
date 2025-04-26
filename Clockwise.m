clear; close all; clc;
% приведены относительные величины, AB=BM=BC=1
psi = deg2rad(43.669);
r = 0.557141;
d = 1.323724;
CO1 = 1.386699;
DO1 = 0.123191;
DM = 0.583647;

%особые точки
alpha_values_deg = [0.000, 63.491, 130.281, 180.000, 229.718, 296.510, 360.000];
alpha_values = deg2rad(alpha_values_deg);

% Считаем лямбда-механизм
xA = @(alpha) d + r * cos(alpha);
yA = @(alpha) r * sin(alpha);
CA = @(alpha) sqrt(r^2 + d^2 + 2*r*d*cos(alpha));
h = @(alpha) sqrt(1 - CA(alpha).^2/4);

xB = @(alpha) (0 + xA(alpha))/2 - h(alpha).*(yA(alpha) - 0)./CA(alpha);
yB = @(alpha) (0 + yA(alpha))/2 + h(alpha).*(xA(alpha) - 0)./CA(alpha);

xM = @(alpha) xA(alpha) + 2*(xB(alpha) - xA(alpha));
yM = @(alpha) yA(alpha) + 2*(yB(alpha) - yA(alpha));

% Параметризация маятника
MO1 = @(alpha) sqrt(xM(alpha).^2 + (yM(alpha) - CO1).^2);

w = @(alpha) atan2(xM(alpha), -(yM(alpha) - CO1));
theta1 = @(alpha) acos((MO1(alpha).^2 + DO1^2 - DM^2)./(2*MO1(alpha)*DO1));
theta2 = @(alpha) acos((DO1^2 + DM^2 - MO1(alpha).^2)./(2*DM*DO1));

alpha_deg_vec = 360:-0.5:0;
alpha_vec = deg2rad(alpha_deg_vec);

branch = ones(size(alpha_vec)); 

% Присвоение индекса стороны
for i = 1:length(alpha_values)
    [~, idx] = min(abs(alpha_vec - alpha_values(i)));
    if i < length(alpha_values)
        branch(idx:end) = mod(branch(idx:end), 2) + 1;
    end
end

figure('Name', 'Paradox Mechanism', 'Position', [100 100 800 600]);

rotation_counter=0;
defiler = 0;
for i = 1:length(alpha_vec)
    alpha = alpha_vec(i);
    current_branch = branch(i);
    xi_angle=2*pi;
    clf;
    hold on;
    axis equal;
    grid on;
    xlim([-1.5, 2]);
    ylim([-0.5, 2]);
    title(sprintf('α = %.1f°, Rotation %d', rad2deg(alpha), rotation_counter));
    xlabel('X');
    ylabel('Y');
    
    % Шарниры
    plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Point C 
    plot(d, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Point O
    plot(0, CO1, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', [0.8500 0.3250 0.0980]); % Point O1
    
    % Стержни
    plot([d, xA(alpha)], [0, yA(alpha)], 'k-', 'LineWidth', 2); % OA
    plot([xA(alpha), xB(alpha)], [yA(alpha), yB(alpha)], 'k-', 'LineWidth', 2); % AB
    plot([xB(alpha), 0], [yB(alpha), 0], 'k-', 'LineWidth', 2); % CB
    plot([xB(alpha), xM(alpha)], [yB(alpha), yM(alpha)], 'k-', 'LineWidth', 2); % BM
    
    % Расчёт углов с помощью индекса стороны
    if current_branch == 2
        xi_angle = w(alpha) + theta1(alpha);
    else
        xi_angle = w(alpha) - theta1(alpha);
    end
    eta_angle = xi_angle + pi + (current_branch*2-3)*theta2(alpha);

    
    xD = DO1 * sin(xi_angle);
    yD = CO1 - DO1 * cos(xi_angle);
    if (CO1>yD) && (xD>0) && (xD<0.05) && (defiler ==0)
        rotation_counter = rotation_counter+1;
        defiler = 1;
    end    
    if (CO1<yD)
        defiler = 0;
    end
    % Ещё стержни
    plot([0, xD], [CO1, yD], 'k-', 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); % O1D
    plot([xD, xM(alpha)], [yD, yM(alpha)], 'k-', 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); % DM
    
    % Ещё шарниры
    plot(xA(alpha), yA(alpha), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % A
    plot(xB(alpha), yB(alpha), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % B
    plot(xM(alpha), yM(alpha), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', [0.8500 0.3250 0.0980]); % M
    plot(xD, yD, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', [0.8500 0.3250 0.0980]); % D
    
    % Обозначаем окружностИ, в которые вписана траектория точки M (радиусы DM+DO1 и DM-DO1)
    viscircles([0, CO1], DM + DO1, 'Color', [0.5 0.5 0.5], 'LineStyle', '--');
    viscircles([0, CO1], DM - DO1, 'Color', [0.5 0.5 0.5], 'LineStyle', '--');
    hold off;
    drawnow;    
end
