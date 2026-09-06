% ICARUS flight data logger / grapher
% v2 - auto column detection

%code breakdown
%reads the flight CSV off the SD card, finds the flight events and plots the lot into dashboards.
%column names come from the header row instead of being hardcoded, so it eats both the 18 column real flight log and the 20 column sim log.

clear; clc; close all;

%% config
CFG.file         = '';       %blank = grab the newest CSV in this folder
CFG.targetApogee = 530;      %target apogee in metres
CFG.servoStowed  = 126;      %SERVO_MAX // closed = least drag
CFG.servoOpen    = 0;        %SERVO_MIN // open = max drag
CFG.cdClean      = 0.4724;   %CDClean
CFG.cdMax        = 0.8776;   %CDMax
CFG.session      = 0;        %0 = newest logging session in the file
CFG.zoomToAscent = true;     %focus the time axis on boost and coast

HDRTAG  = 'Time(ms)';        %first column of the header row
MINCOL  = 18;                %the 18 columns both formats share, in the same order

%% file grabber
if isempty(CFG.file)
    d = dir('*.csv');
    if isempty(d)
        error('ICARUS:noCSV', 'No .csv found in %s', pwd);
    end
    [~, newest] = max([d.datenum]);
    CFG.file = d(newest).name;
    fprintf('No file specified - using newest CSV: %s\n', CFG.file);
end
if ~isfile(CFG.file)
    error('ICARUS:missing', 'Could not find "%s" in %s', CFG.file, pwd);
end

%% importing
fid = fopen(CFG.file, 'r');
if fid < 0, error('ICARUS:open', 'Could not open %s', CFG.file); end
raw = textscan(fid, '%s', 'Delimiter', '\n', 'WhiteSpace', '');
fclose(fid);

lines = string(raw{1});
lines = lines(strlength(strtrim(lines)) > 0);

hdrIdx = find(startsWith(lines, HDRTAG));
if isempty(hdrIdx)
    error('ICARUS:noHeader', ['No header row starting "%s" found in %s.\n' ...
        'Is this a flight log, or is it the old sim data?'], HDRTAG, CFG.file);
end

%the sketch appends, so one file can hold several power ups
bounds = [hdrIdx; numel(lines) + 1];
nSess  = numel(hdrIdx);
fprintf('Found %d logging session(s) in %s\n', nSess, CFG.file);

sess = CFG.session;
if sess == 0, sess = nSess; end
if sess < 1 || sess > nSess
    error('ICARUS:badSession', 'CFG.session = %d but the file holds %d session(s).', sess, nSess);
end
if nSess > 1
    fprintf('Analysing session %d of %d (set CFG.session to pick another)\n', sess, nSess);
end

%% header driven schema
hdrNames = strtrim(split(lines(bounds(sess)), ','));
NCOL     = numel(hdrNames);
if NCOL < MINCOL
    error('ICARUS:shortHeader', ['Header only declares %d columns, need at least %d.\n' ...
        'Header was: %s'], NCOL, MINCOL, lines(bounds(sess)));
end

COLS = matlab.lang.makeValidName(hdrNames);
COLS = matlab.lang.makeUniqueStrings(COLS);

%sim ground truth, only present in Xiao_sim_test logs
iTrueAlt = find(strcmpi(hdrNames, "TrueAlt"), 1);
iTrueVel = find(strcmpi(hdrNames, "TrueVel"), 1);
hasTruth = ~isempty(iTrueAlt) && ~isempty(iTrueVel);

if hasTruth
    fprintf('Schema: %d columns, SIM LOG (ground truth present)\n', NCOL);
else
    fprintf('Schema: %d columns, FLIGHT LOG (no ground truth)\n', NCOL);
end

%% row parse
body = lines(bounds(sess)+1 : bounds(sess+1)-1);
if isempty(body)
    error('ICARUS:emptySession', 'Session %d has a header but no data rows.', sess);
end

%bins any row that doesnt match the header width, catches half written lines from a power cut
nComma = count(body, ',');
keep   = nComma == (NCOL - 1);
nDropped = sum(~keep);
if nDropped > 0
    fprintf('Dropped %d malformed row(s) of %d.\n', nDropped, numel(body));
    if nDropped > 0.5 * numel(body)
        modeCommas = mode(nComma);
        fprintf(['  >> most rows carry %d commas (= %d columns) but the header ' ...
            'declares %d. Sketch and header disagree.\n'], ...
            modeCommas, modeCommas + 1, NCOL);
    end
end
if ~any(keep)
    error('ICARUS:noRows', ['No row matched the %d-column header. ' ...
        'Most common row width was %d columns.'], NCOL, mode(nComma) + 1);
end

M = str2double(split(body(keep), ','));
if size(M,2) ~= NCOL
    error('ICARUS:cols', 'Expected %d columns, parsed %d. Header/sketch mismatch?', NCOL, size(M,2));
end

bad = isnan(M(:,1));
if any(bad)
    fprintf('Dropped %d row(s) with an unreadable timestamp.\n', sum(bad));
    M = M(~bad, :);
end
if size(M,1) < 2
    error('ICARUS:tooShort', 'Only %d usable rows - nothing to plot.', size(M,1));
end

%% columns
D = array2table(M, 'VariableNames', COLS);

timeMs = M(:,1);
t      = timeMs / 1000;
t      = t - t(1);          %zero the clock on the first row
state  = M(:,2);            %0 IDLE // 1 ASCENT // 2 DESCENT // 3 LANDED
alt    = M(:,3);            %m AGL, already zeroed against groundAltitude
vel    = M(:,4);            %m/s, fusedVelocity
servo  = M(:,5);            %deg
tempC  = M(:,6);
pressPa = M(:,7);           %pascals not hPa
accel  = M(:, 8:10);        %m/s^2
gyro   = M(:,11:13);        %rad/s
mag    = M(:,14:16);        %microtesla
CD     = M(:,17);           %what the mpc asked for
predAp = M(:,18);           %what the winning candidate predicted

if hasTruth
    trueAlt = M(:,iTrueAlt);
    trueVel = M(:,iTrueVel);
end

accelMag = sqrt(sum(accel.^2, 2));
gyroDeg  = gyro * (180/pi);
rho      = pressPa ./ (287.05 * (tempC + 273.15)); %same ideal gas law as airDensity() in the sketch

%% flight events
iLaunch = find(state == 1, 1, 'first');
iApogee = find(state == 2, 1, 'first');
iLanded = find(state == 3, 1, 'first');

[apogeeAlt, iPeak] = max(alt);
[vMax, iVmax]      = max(vel);
[aMax, iAmax]      = max(accelMag);

dt     = diff(timeMs);
loopHz = 1000 / median(dt);

mpcLive = (state == 1) & ~isnan(predAp) & predAp > 0; %mpc only writes a prediction during ascent

%%                              flight summary                              %%
fprintf('\n================ FLIGHT SUMMARY ================\n');
fprintf('  samples            %d over %.2f s\n', height(D), t(end));
fprintf('  median loop rate   %.1f Hz  (dt %.1f ms, worst %.0f ms)\n', loopHz, median(dt), max(dt));
if ~isempty(iLaunch), fprintf('  launch detected    t = %.2f s\n', t(iLaunch)); end
if ~isempty(iApogee), fprintf('  apogee detected    t = %.2f s\n', t(iApogee)); end
if ~isempty(iLanded), fprintf('  landed             t = %.2f s\n', t(iLanded)); end
fprintf('  PEAK ALTITUDE      %.1f m  at t = %.2f s   (barometer)\n', apogeeAlt, t(iPeak));
fprintf('  target             %.0f m   ->  error %+.1f m (%+.2f%%)\n', ...
    CFG.targetApogee, apogeeAlt - CFG.targetApogee, ...
    100*(apogeeAlt - CFG.targetApogee)/CFG.targetApogee);
fprintf('  max velocity       %.1f m/s at t = %.2f s  (Mach %.3f)\n', vMax, t(iVmax), vMax/340);
fprintf('  max |accel|        %.1f m/s^2 (%.1f G) at t = %.2f s\n', aMax, aMax/9.81, t(iAmax));
fprintf('  servo travel       %.0f to %.0f deg\n', min(servo), max(servo));
fprintf('  CD range commanded %.4f to %.4f\n', min(CD), max(CD));

%scores the estimator against truth on a sim log, on a real flight the barometer peak is all there is
refApogee = apogeeAlt;
if hasTruth
    [trueApogee, iTruePeak] = max(trueAlt);
    refApogee = trueApogee;
    ascent    = state == 1;
    altErr    = alt - trueAlt;
    velErr    = vel - trueVel;
    fprintf('\n  ---- GROUND TRUTH (sim only) ----\n');
    fprintf('  TRUE APOGEE        %.1f m  at t = %.2f s\n', trueApogee, t(iTruePeak));
    fprintf('  true vs target     %+.1f m (%+.2f%%)\n', ...
        trueApogee - CFG.targetApogee, 100*(trueApogee - CFG.targetApogee)/CFG.targetApogee);
    fprintf('  baro peak error    %+.1f m vs truth\n', apogeeAlt - trueApogee);
    fprintf('  altitude estimate  RMS %.2f m, worst %+.2f m (ascent only)\n', ...
        sqrt(mean(altErr(ascent).^2)), altErr(find(abs(altErr.*ascent) == max(abs(altErr.*ascent)), 1)));
    fprintf('  velocity estimate  RMS %.2f m/s, worst %+.2f m/s (ascent only)\n', ...
        sqrt(mean(velErr(ascent).^2)), velErr(find(abs(velErr.*ascent) == max(abs(velErr.*ascent)), 1)));
    fprintf('  peak true velocity %.1f m/s  vs fused %.1f m/s\n', max(trueVel), vMax);
end

mpcOn = mpcLive;
if any(mpcOn)
    predErr = predAp(mpcOn) - refApogee;
    fprintf('\n  MPC prediction     mean %+.1f m, worst %+.1f m vs the apogee actually reached\n', ...
        mean(predErr), predErr(find(abs(predErr) == max(abs(predErr)), 1)));
end
fprintf('===============================================\n\n');

%%                      figure 1 - dynamics and control                      %%
f1 = figure('Name','ICARUS 1: Dynamics & Control','NumberTitle','off', ...
    'Position',[50 150 1200 800]);
L1 = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
title(L1, sprintf('Flight Dynamics & Control  -  %s', CFG.file), ...
    'FontSize',16,'FontWeight','bold','Interpreter','none');

%altitude
nexttile;
plot(t, alt, 'b-', 'LineWidth', 2); hold on;
if hasTruth
    plot(t, trueAlt, 'Color',[0.4 0.4 0.4], 'LineStyle','--', 'LineWidth', 1.5);
end
yline(CFG.targetApogee, 'r--', sprintf('Target (%d m)', CFG.targetApogee), ...
    'LineWidth', 1.5, 'LabelHorizontalAlignment','left');
plot(t(iPeak), apogeeAlt, 'ko', 'MarkerFaceColor','y', 'MarkerSize', 8);
text(t(iPeak), apogeeAlt, sprintf('  %.1f m', apogeeAlt), 'VerticalAlignment','bottom');
hold off; grid on;
title('Altitude AGL'); xlabel('Time (s)'); ylabel('Altitude (m)');
if hasTruth, legend('Barometer','Truth','Location','best'); end

%velocity
nexttile;
plot(t, vel, 'g-', 'LineWidth', 2); hold on;
if hasTruth
    plot(t, trueVel, 'Color',[0.4 0.4 0.4], 'LineStyle','--', 'LineWidth', 1.5);
end
yline(0, 'k-', 'Apogee (0 m/s)', 'LineWidth', 1);
hold off; grid on;
title('Fused Velocity'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
if hasTruth, legend('Fused','Truth','Location','best'); end

%phase plot
nexttile;
plot(alt, vel, 'm-', 'LineWidth', 2);
xline(CFG.targetApogee, 'r--', 'Target');
grid on; title('Phase Plot: Velocity vs Altitude');
xlabel('Altitude (m)'); ylabel('Velocity (m/s)');

%servo against flight state
nexttile;
yyaxis left;
plot(t, servo, 'c-', 'LineWidth', 2);
yline(CFG.servoStowed, 'k:', sprintf('Stowed (%d deg)', CFG.servoStowed));
yline(CFG.servoOpen,   'k:', sprintf('Full drag (%d deg)', CFG.servoOpen));
ylim([-10 CFG.servoStowed+10]); ylabel('Airbrake Angle (deg)');
yyaxis right;
stairs(t, state, 'k-', 'LineWidth', 1.5);
ylim([-0.5 3.5]); yticks(0:3);
yticklabels({'0 IDLE','1 ASCENT','2 DESCENT','3 LANDED'});
ylabel('Flight State');
grid on; title('Airbrake Output & State'); xlabel('Time (s)');

%%                       figure 2 - mpc performance                       %%
f2 = figure('Name','ICARUS 2: MPC Performance','NumberTitle','off', ...
    'Position',[80 120 1200 800]);
L2 = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
title(L2,'MPC Controller Performance','FontSize',16,'FontWeight','bold');

%predicted apogee
nexttile;
pa = predAp; pa(~mpcLive) = NaN;
plot(t, pa, 'b-', 'LineWidth', 2); hold on;
yline(CFG.targetApogee, 'r--', 'Target', 'LineWidth', 1.5);
yline(refApogee, 'k:', sprintf('Actual (%.1f m)', refApogee), 'LineWidth', 1.5);
hold off; grid on;
title('Predicted Apogee vs Target'); xlabel('Time (s)'); ylabel('Apogee (m)');
legend('MPC prediction','Location','best');

%works the servo angle back into the Cd the airframe actually saw
cdAchieved = CFG.cdClean + (CFG.servoStowed - servo) / ...
    (CFG.servoStowed - CFG.servoOpen) * (CFG.cdMax - CFG.cdClean);
cdCmd = CD; cdCmd(~mpcLive) = NaN;

nexttile;
plot(t, cdCmd, 'b-', 'LineWidth', 2); hold on;
plot(t, cdAchieved, 'r-', 'LineWidth', 1.5);
yline(CFG.cdClean,'k:','CDClean'); yline(CFG.cdMax,'k:','CDMax');
hold off; grid on;
title('Cd: commanded vs achieved'); xlabel('Time (s)'); ylabel('Cd');
legend('MPC commanded','Servo achieved','Location','best');

%slew rate and deadband show up in here as lag
nexttile;
trackErr = cdCmd - cdAchieved;
plot(t, trackErr, 'k-', 'LineWidth', 1.5);
yline(0,'r--');
grid on; title('Tracking error (commanded - achieved)');
xlabel('Time (s)'); ylabel('\DeltaCd');

%loop timing, should sit on 20ms
nexttile;
plot(t(2:end), dt, 'k-', 'LineWidth', 1);
yline(20, 'r--', '20 ms target', 'LineWidth', 1.5);
grid on; title(sprintf('Loop interval (median %.1f ms = %.1f Hz)', median(dt), loopHz));
xlabel('Time (s)'); ylabel('dt (ms)');

%%                         figure 3 - atmospherics                         %%
f3 = figure('Name','ICARUS 3: Atmospherics','NumberTitle','off', ...
    'Position',[110 90 900 800]);
L3 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
title(L3,'Atmospherics (BMP390)','FontSize',16,'FontWeight','bold');

nexttile;
plot(t, pressPa/100, 'k-', 'LineWidth', 2);
grid on; title('Barometric Pressure'); xlabel('Time (s)');
ylabel('Pressure (hPa)');
subtitle('logged in Pa, shown in hPa');

nexttile;
plot(t, tempC, 'r-', 'LineWidth', 2);
grid on; title('BMP390 Internal Temperature'); xlabel('Time (s)'); ylabel('Temp (\circC)');

nexttile;
plot(t, rho, 'b-', 'LineWidth', 2);
yline(1.225,'k:','ISA sea level');
grid on; title('Air Density (from P and T, same as airDensity())');
xlabel('Time (s)'); ylabel('\rho (kg/m^3)');

%%                          figure 4 - 9dof imu                          %%
f4 = figure('Name','ICARUS 4: 9-DOF IMU','NumberTitle','off', ...
    'Position',[140 60 1200 900]);
L4 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
title(L4,'9-DOF Kinematics (ICM-20948)','FontSize',16,'FontWeight','bold');

nexttile;
plot(t, accel(:,1), 'r-', 'LineWidth', 1.2); hold on;
plot(t, accel(:,2), 'g-', 'LineWidth', 1.2);
plot(t, accel(:,3), 'b-', 'LineWidth', 1.2);
plot(t, accelMag,   'k-', 'LineWidth', 1.8);
yline(9.81, 'k:', '1 G resting'); hold off;
grid on; title('Acceleration'); xlabel('Time (s)'); ylabel('Accel (m/s^2)');
legend('X','Y','Z','|a|','Location','best');

nexttile;
plot(t, gyroDeg(:,1), 'r-', 'LineWidth', 1.2); hold on;
plot(t, gyroDeg(:,2), 'g-', 'LineWidth', 1.2);
plot(t, gyroDeg(:,3), 'b-', 'LineWidth', 1.2); hold off;
grid on; title('Gyroscope'); xlabel('Time (s)'); ylabel('Rate (deg/s)');
subtitle('logged in rad/s, shown in deg/s');
legend('X','Y','Z','Location','best');

nexttile;
plot(t, mag(:,1), 'r-', 'LineWidth', 1.2); hold on;
plot(t, mag(:,2), 'g-', 'LineWidth', 1.2);
plot(t, mag(:,3), 'b-', 'LineWidth', 1.2); hold off;
grid on; title('Magnetometer'); xlabel('Time (s)'); ylabel('Field (\muT)');
legend('X','Y','Z','Location','best');

%%              figure 5 - estimator vs truth (sim logs only)              %%
f5 = [];
if hasTruth
    f5 = figure('Name','ICARUS 5: Estimator vs Truth','NumberTitle','off', ...
        'Position',[170 40 1200 800]);
    L5 = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    title(L5,'Estimator Validation against Sim Ground Truth', ...
        'FontSize',16,'FontWeight','bold');

    nexttile;
    plot(t, trueAlt, 'k-', 'LineWidth', 2); hold on;
    plot(t, alt, 'b--', 'LineWidth', 1.5);
    yline(CFG.targetApogee,'r--','Target');
    hold off; grid on;
    title('Altitude: truth vs barometer'); xlabel('Time (s)'); ylabel('Altitude (m)');
    legend('Truth','Barometer AGL','Location','best');

    nexttile;
    plot(t, trueVel, 'k-', 'LineWidth', 2); hold on;
    plot(t, vel, 'g--', 'LineWidth', 1.5);
    yline(0,'r--');
    hold off; grid on;
    title('Velocity: truth vs fused estimate'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
    legend('Truth','Fused','Location','best');

    nexttile;
    yyaxis left;
    plot(t, alt - trueAlt, 'b-', 'LineWidth', 1.4);
    ylabel('Altitude error (m)');
    yyaxis right;
    plot(t, vel - trueVel, 'g-', 'LineWidth', 1.4);
    ylabel('Velocity error (m/s)');
    yline(0,'k:');
    grid on; title('Estimator error (estimate - truth)'); xlabel('Time (s)');

    %error coloured by time, shows whether the fusion drifts as it speeds up
    nexttile;
    asc = state == 1;
    scatter(trueVel(asc), vel(asc) - trueVel(asc), 12, t(asc), 'filled');
    cb = colorbar; cb.Label.String = 'Time (s)';
    yline(0,'r--');
    grid on; title('Velocity error vs true speed (ascent only)');
    xlabel('True velocity (m/s)'); ylabel('Fused - true (m/s)');
end

%% zoom and tidy up
if CFG.zoomToAscent
    if ~isempty(iApogee), tEnd = t(iApogee) + 3;
    else,                 tEnd = t(iPeak) + 3;
    end
    xr = [0 min(tEnd, t(end))];
    fprintf('X axis focused on 0 to %.1f s (boost + coast). Zoom out to see recovery.\n', xr(2));
else
    xr = [0 t(end)];
end

%links every time axis so zooming one zooms the lot
timeAxes = [];
figList  = [f1 f2 f3 f4];
if ~isempty(f5), figList(end+1) = f5; end
for f = figList
    ax = findall(f, 'Type', 'axes');
    for a = ax(:)'
        if strcmp(get(get(a,'XLabel'),'String'), 'Time (s)')
            timeAxes(end+1) = a; %#ok<SAGROW>
        end
    end
end
if ~isempty(timeAxes)
    linkaxes(timeAxes, 'x');
    xlim(timeAxes(1), xr);
end

fprintf('%d dashboards generated. Time axes are linked - zoom one, they all follow.\n', numel(figList));

% end of code
