function cart_pendulum_application
% Interactive nonlinear cart-pendulum simulation.
% x positive right; theta positive leaning left.
% Controller input and external disturbance are forces [N].

%% Physical parameters — replace with your group's values later
p.M = 0.5;       % Cart mass [kg]
p.m = 0.2;       % Pendulum mass [kg]
p.l = 0.3;       % Pivot-to-center-of-mass distance [m]
p.I = 0.006;     % Inertia about pendulum center of mass [kg*m^2]
p.b = 0.1;       % Cart viscous friction [N*s/m]
p.g = 9.81;
railLimit = 0.2; % Allowed cart-center displacement [m]
rodLength = 0.6; % Visual rod length only [m]

%% Upright linearization
J = p.I + p.m*p.l^2;
h = p.m*p.l;
D = (p.M+p.m)*J-h^2;

A = [0 1 0 0;
     0 -J*p.b/D h^2*p.g/D 0;
     0 0 0 1;
     0 -h*p.b/D (p.M+p.m)*h*p.g/D 0];

B = [0; J/D; 0; h/D];

assert(rank(ctrb(A,B)) == 4, 'Model is not controllable.');

%% Simulation state
z = zeros(4,1);
K = zeros(1,4);
t = 0;
pulseEnd = -inf;
pulseForce = 0;
running = false;
dt = 0.0025;       % Numerical integration step [s]
stepsPerTick = 20; % 0.05 simulated seconds per timer callback

%% Interface
fig = uifigure('Name','Cart-Pendulum LQR Lab', ...
    'Position',[100 80 1150 780]);

main = uigridlayout(fig,[1 2]);
main.ColumnWidth = {270,'1x'};

controls = uigridlayout(main,[18 2]);
controls.ColumnWidth = {'1x',100};
controls.RowHeight = [repmat({30},1,16),{65},{'1x'}];

qDefault = 1./[0.2 0.5 deg2rad(10) 1].^2;
qLabels = {'Q: cart position','Q: cart velocity', ...
           'Q: pendulum angle','Q: angular velocity'};
qFields = cell(1,4);

for n = 1:4
    qFields{n} = numberField(n,qLabels{n},qDefault(n),[0 Inf]);
end

rField = numberField(5,'R: force penalty',0.01,[1e-8 Inf]);
angleField = numberField(6,'Initial angle [deg]',10,[-180 180]);
forceField = numberField(7,'Motor limit [N]',10,[0.01 Inf]);
pushField = numberField(8,'Push force [N]',1,[-100 100]);
durationField = numberField(9,'Push duration [s]',0.2,[0.01 5]);

calcButton = wideButton(10,'Recalculate LQR',@recalculate);
startButton = wideButton(11,'Start',@startSimulation);
pauseButton = wideButton(12,'Pause',@pauseSimulation);
resetButton = wideButton(13,'Reset',@resetSimulation);
pushButton = wideButton(14,'Apply push',@applyPush);

gainLabel = uilabel(controls,'Text','');
gainLabel.Layout.Row = 15;
gainLabel.Layout.Column = [1 2];

poleLabel = uilabel(controls,'Text','');
poleLabel.Layout.Row = 16;
poleLabel.Layout.Column = [1 2];

status = uilabel(controls,'Text','Ready','WordWrap','on');
status.Layout.Row = 17;
status.Layout.Column = [1 2];

plots = uigridlayout(main,[4 1]);
plots.RowHeight = {'1.5x','1x','1x','1x'};

axAnimation = uiaxes(plots);
hold(axAnimation,'on');
axis(axAnimation,'equal');
axAnimation.XLim = [-0.4 0.4];
axAnimation.YLim = [-0.1 rodLength+0.1];
xlabel(axAnimation,'Position [m]');
ylabel(axAnimation,'Height [m]');
grid(axAnimation,'on');

plot(axAnimation,[-railLimit railLimit],[-0.05 -0.05], ...
    'k-','LineWidth',3);

cart = patch(axAnimation,[0 0 0 0],[0 0 0 0], ...
    [0.2 0.6 0.9]);
rod = plot(axAnimation,[0 0],[0 rodLength], ...
    'LineWidth',3,'Color',[0.9 0.4 0.1]);
tip = plot(axAnimation,0,rodLength,'o', ...
    'MarkerFaceColor',[0.9 0.4 0.1],'MarkerSize',8);

axAngle = uiaxes(plots);
axPosition = uiaxes(plots);
axForce = uiaxes(plots);
hold(axForce,'on');

angleLine = animatedline(axAngle,'Color',[0 0.45 0.75]);
positionLine = animatedline(axPosition,'Color',[0 0.45 0.75]);
motorLine = animatedline(axForce,'Color',[0 0.45 0.75]);
pushLine = animatedline(axForce,'Color',[0.9 0.4 0.1]);

ylabel(axAngle,'Angle [deg]');
ylabel(axPosition,'Position [m]');
ylabel(axForce,'Force [N]');
xlabel(axForce,'Time [s]');
legend(axForce,{'Motor','External push'},'Location','northeast');

for ax = [axAngle axPosition axForce]
    grid(ax,'on');
    ax.XLim = [0 10];
end

%% Timer keeps buttons responsive
clock = timer('ExecutionMode','fixedSpacing','Period',0.05, ...
    'TimerFcn',@tick,'ErrorFcn',@timerError);
fig.CloseRequestFcn = @closeApp;

recalculate([],[]);

%% UI helpers
    function field = numberField(row,label,value,limits)
        lab = uilabel(controls,'Text',label);
        lab.Layout.Row = row;
        lab.Layout.Column = 1;
        field = uieditfield(controls,'numeric', ...
            'Value',value,'Limits',limits);
        field.Layout.Row = row;
        field.Layout.Column = 2;
    end

    function button = wideButton(row,label,callback)
        button = uibutton(controls,'Text',label, ...
            'ButtonPushedFcn',callback);
        button.Layout.Row = row;
        button.Layout.Column = [1 2];
    end

%% Button callbacks
    function recalculate(~,~)
        pauseSimulation([],[]);
        try
            weights = cellfun(@(field) field.Value,qFields);
            Q = diag(weights);
            R = rField.Value;
            newK = lqr(A,B,Q,R);
            poles = eig(A-B*newK);

            if any(real(poles) >= 0)
                error('Calculated linear closed loop is not stable.');
            end

            K = newK;
            gainLabel.Text = sprintf('K: %.2f, %.2f, %.2f, %.2f',K);
            poleLabel.Text = sprintf('Largest pole real part: %.3f', ...
                max(real(poles)));
            resetSimulation([],[]);
            status.Text = 'LQR calculated. Reset complete. Press Start.';
        catch err
            status.Text = 'LQR calculation failed.';
            uialert(fig,err.message,'Calculation error');
        end
    end

    function startSimulation(~,~)
        if running
            return;
        end
        if abs(z(1)) >= railLimit || abs(z(3)) >= pi/2
            status.Text = 'Reset before restarting this failed trial.';
            return;
        end
        running = true;
        setEditing(false);
        status.Text = 'Running';
        start(clock);
    end

    function pauseSimulation(~,~)
        running = false;
        if strcmp(clock.Running,'on')
            stop(clock);
        end
        setEditing(true);
        status.Text = 'Paused';
    end

    function resetSimulation(~,~)
        pauseSimulation([],[]);
        t = 0;
        z = [0;0;deg2rad(angleField.Value);0];
        pulseEnd = -inf;
        pulseForce = 0;
        for line = [angleLine positionLine motorLine pushLine]
            clearpoints(line);
        end
        render();
        status.Text = 'Reset. Initial angle is used when you press Reset.';
    end

    function applyPush(~,~)
        if ~running
            status.Text = 'Press Start before applying a push.';
            return;
        end
        pulseForce = pushField.Value;
        pulseEnd = t + durationField.Value;
        status.Text = 'External force pulse applied.';
    end

    function setEditing(enabled)
        if enabled
            state = 'on';
        else
            state = 'off';
        end
        for n = 1:4
            qFields{n}.Enable = state;
        end
        rField.Enable = state;
        forceField.Enable = state;
        angleField.Enable = state;
        calcButton.Enable = state;
    end

%% Nonlinear plant
    function dz = dynamics(time,state)
        motorForce = max(-forceField.Value, ...
            min(forceField.Value,-K*state));

        externalForce = 0;
        if time < pulseEnd
            externalForce = pulseForce;
        end

        theta = state(3);
        omega = state(4);

        H = [p.M+p.m, -h*cos(theta);
             -h*cos(theta), J];

        rhs = [motorForce + externalForce - p.b*state(2) ...
               - h*sin(theta)*omega^2;
               h*p.g*sin(theta)];

        acc = H\rhs;
        dz = [state(2);acc(1);state(4);acc(2)];
    end

%% Advance simulation using fourth-order Runge-Kutta
    function tick(~,~)
        if ~running || ~isvalid(fig)
            return;
        end

        for n = 1:stepsPerTick
            % Split a step at the end of a pulse.
            step = dt;
            if t < pulseEnd && t+step > pulseEnd
                step = pulseEnd-t;
            end

            k1 = dynamics(t,z);
            k2 = dynamics(t+step/2,z+step*k1/2);
            k3 = dynamics(t+step/2,z+step*k2/2);
            k4 = dynamics(t+step,z+step*k3);
            z = z + step*(k1+2*k2+2*k3+k4)/6;
            t = t+step;

            if any(~isfinite(z)) || abs(z(1)) >= railLimit ...
                    || abs(z(3)) >= pi/2
                pauseSimulation([],[]);
                status.Text = ...
                    'Trial stopped: rail limit or 90-degree tilt reached.';
                break;
            end
        end
        render();
    end

%% Update animation and graphs
    function render()
        x = z(1);
        theta = z(3);
        tipX = x-rodLength*sin(theta);
        tipY = rodLength*cos(theta);

        cart.XData = x+[-0.06 0.06 0.06 -0.06];
        cart.YData = [-0.06 -0.06 0 0];
        rod.XData = [x tipX];
        rod.YData = [0 tipY];
        tip.XData = tipX;
        tip.YData = tipY;
        title(axAnimation,sprintf('t = %.2f s | angle = %.2f deg', ...
            t,rad2deg(theta)));

        motorForce = max(-forceField.Value, ...
            min(forceField.Value,-K*z));
        externalForce = 0;
        if t < pulseEnd
            externalForce = pulseForce;
        end

        addpoints(angleLine,t,rad2deg(theta));
        addpoints(positionLine,t,x);
        addpoints(motorLine,t,motorForce);
        addpoints(pushLine,t,externalForce);

        for ax = [axAngle axPosition axForce]
            ax.XLim = [max(0,t-10) max(10,t)];
        end
        drawnow limitrate;
    end

    function timerError(~,~)
        running = false;
        setEditing(true);
        status.Text = 'Simulation error. Check the Command Window.';
    end

    function closeApp(~,~)
        running = false;
        stop(clock);
        delete(clock);
        delete(fig);
    end
end