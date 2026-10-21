%% analyze_suntrack.m
% Astro 331 Lab 8 - KestrelSAT sun-tracking (reaction wheel) data analysis
%
% Reads every SunTrack_data*.csv file in this folder and, for each one,
% writes PDF plots to ./plots/<file name>/ plus a combined multi-page PDF
% (<file name>_all.pdf) and prints summary statistics to the command window.
%
% CSV columns (from KestrelSAT log):
%   mcu time(ms), mode, sun_plusX/plusY/minusX/minusY (count), sun_mag,
%   sun_direction(deg), error(deg), body_rate(deg/s), mag_X/mag_Y (uT),
%   torque_cmd(N*m), w_RW_cmd(RPM), w_RW_meas(RPM), saturated

clear; clc; close all;

%% User settings
dataDir      = fileparts(mfilename('fullpath'));
filePattern  = 'SunTrack_data*.csv';
errThreshDeg = 10;      % "on target" band for pointing-error statistics [deg]
modeOrder    = {'SPINUP','HOLD','TRACK'};
modeColors   = [0.85 0.33 0.10; 0.93 0.69 0.13; 0.47 0.67 0.19];
arrayAxisDeg = 180;     % solar array normal in body frame (-X face), SOLAR_ARRAY_AXIS_DEG
% Sun sensor calibration used by the flight code (sun_cal in main.cpp), order +X +Y -X -Y.
% Update this if you re-run command 10 so the diagnostic matches the firmware.
sunCal.dark   = [74.5  415.4  641.5  272.7];
sunCal.bright = [3249.1 4095.0 4036.5 4094.8];
adcMax       = 4095;    % 12-bit ADC full scale

files = dir(fullfile(dataDir, filePattern));
if isempty(files)
    error('No files matching %s found in %s', filePattern, dataDir);
end

for k = 1:numel(files)
    analyzeFile(fullfile(files(k).folder, files(k).name), ...
        errThreshDeg, modeOrder, modeColors, arrayAxisDeg, sunCal, adcMax);
end

%% ------------------------------------------------------------------------
function analyzeFile(csvPath, errThreshDeg, modeOrder, modeColors, ...
    arrayAxisDeg, sunCal, adcMax)

[~, name] = fileparts(csvPath);
outDir = fullfile(fileparts(csvPath), 'plots', name);
if ~exist(outDir, 'dir'), mkdir(outDir); end
combinedPdf = fullfile(outDir, [name '_all.pdf']);
if exist(combinedPdf, 'file'), delete(combinedPdf); end

fprintf('\n===== %s =====\n', name);

%% Load data
opts = detectImportOptions(csvPath, 'VariableNamingRule', 'preserve');
T = readtable(csvPath, opts);

t        = (T.("mcu time(ms)") - T.("mcu time(ms)")(1)) / 1000;   % [s]
mode     = string(T.mode);
sunPX    = T.("sun_plusX(count)");
sunPY    = T.("sun_plusY(count)");
sunMX    = T.("sun_minusX(count)");
sunMY    = T.("sun_minusY(count)");
sunMag   = T.sun_mag;
sunDir   = T.("sun_direction(deg)");
err      = T.("error(deg)");
rate     = T.("body_rate(deg/s)");
magX     = T.("mag_X(uT)");
magY     = T.("mag_Y(uT)");
torque   = T.("torque_cmd(N*m)");
rwCmd    = T.("w_RW_cmd(RPM)");
rwMeas   = T.("w_RW_meas(RPM)");
sat      = logical(T.saturated);

% Mode transition times (for shading/annotation)
segs = modeSegments(t, mode);

%% Figure 1 - Sun direction and pointing error
f = newFig('Pointing');
tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, breakWraps(sunDir), 'b-', 'LineWidth', 1.2);
ylabel('Sun direction (deg)'); ylim([0 360]); yticks(0:90:360);
title('Sun Direction in Body Frame'); grid on;
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, breakWraps(err), 'r-', 'LineWidth', 1.2);
yline([-errThreshDeg errThreshDeg], 'k--', 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
ylabel('Pointing error (deg)'); xlabel('Time (s)');
ylim([-180 180]); yticks(-180:45:180);
title('Pointing Error'); grid on;
modeLegend(ax, segs, modeOrder, modeColors);
saveFig(f, outDir, name, 'pointing', combinedPdf);

%% Figure 2 - Body rate
f = newFig('Body rate');
ax = axes(f); hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, rate, 'k-', 'LineWidth', 1.0);
yline(0, 'k:', 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Body rate (deg/s)');
title('Z-Axis Body Rate (Gyro)'); grid on;
modeLegend(ax, segs, modeOrder, modeColors);
saveFig(f, outDir, name, 'body_rate', combinedPdf);

%% Figure 3 - Reaction wheel speed, torque command, saturation
f = newFig('Reaction wheel');
tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, rwCmd, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Commanded');
plot(t, rwMeas, 'r-', 'LineWidth', 1.0, 'DisplayName', 'Measured');
ylabel('Wheel speed (RPM)'); title('Reaction Wheel Speed'); grid on;
legend(flipud(findobj(ax, 'Type', 'line')), 'Location', 'best');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, torque * 1e3, 'k-', 'LineWidth', 1.0, 'DisplayName', 'Torque cmd');
satT = t; satT(~sat) = NaN;
yl = ylim(ax);
plot(satT, yl(2) * ones(size(t)), 'm-', 'LineWidth', 4, 'DisplayName', 'Saturated');
ylim(ax, yl);
xlabel('Time (s)'); ylabel('Torque cmd (mN\cdotm)');
title('Commanded Torque (magenta = wheel saturated)'); grid on;
legend(flipud(findobj(ax, 'Type', 'line')), 'Location', 'best');
saveFig(f, outDir, name, 'reaction_wheel', combinedPdf);

%% Figure 4 - Sun sensor photodiodes
f = newFig('Sun sensors');
tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, sunPX, 'DisplayName', '+X');
plot(t, sunPY, 'DisplayName', '+Y');
plot(t, sunMX, 'DisplayName', '-X');
plot(t, sunMY, 'DisplayName', '-Y');
yline(4095, 'k--', 'ADC max', 'HandleVisibility', 'off');
ylabel('ADC counts'); title('Sun Sensor Photodiode Readings'); grid on;
legend(flipud(findobj(ax, 'Type', 'line')), 'Location', 'best');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, sunMag, 'k-', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('sun\_mag'); title('Sun Vector Magnitude'); grid on;
saveFig(f, outDir, name, 'sun_sensors', combinedPdf);

%% Figure 5 - Magnetometer
magHeading = mod(atan2d(magY, magX), 360);
f = newFig('Magnetometer');
tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
nexttile; hold on;
plot(t, magX, 'DisplayName', 'mag_X');
plot(t, magY, 'DisplayName', 'mag_Y');
plot(t, hypot(magX, magY), 'k--', 'DisplayName', '|B_{xy}|');
xlabel('Time (s)'); ylabel('Field (\muT)'); title('Magnetometer'); grid on;
xlim([t(1) t(end)]);
legend('Location', 'best', 'Interpreter', 'tex');
nexttile;
scatter(magX, magY, 8, t, 'filled'); axis equal; grid on;
cb = colorbar; cb.Label.String = 'Time (s)';
xlabel('mag_X (\muT)'); ylabel('mag_Y (\muT)');
title('Magnetometer X-Y (hard-iron check)');
saveFig(f, outDir, name, 'magnetometer', combinedPdf);

%% Figure 6 - Sun vs magnetic heading
f = newFig('Headings');
ax = axes(f); hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, sunDir, 'b.', 'MarkerSize', 4, 'DisplayName', 'Sun direction');
plot(t, magHeading, 'r.', 'MarkerSize', 4, 'DisplayName', 'Mag heading atan2(Y,X)');
xlabel('Time (s)'); ylabel('Angle (deg)'); ylim([0 360]); yticks(0:90:360);
title('Sun Direction vs. Magnetometer Heading'); grid on;
legend(flipud(findobj(ax, 'Type', 'line')), 'Location', 'best');
saveFig(f, outDir, name, 'headings', combinedPdf);

%% Figure 7 - Solar array (-X face) vs. sun location in a room-fixed frame
% Heading psi = gyro-integrated rotation about +Z (0 = body +X direction at t = 0).
% (The magnetometer isn't used here: it has a large uncorrected hard-iron offset.)
% -X face direction = psi + array axis; sun direction = psi + measured body-frame sun angle.
% A fixed light source should give a flat sun line; tracking works when the two lines overlap.
psi      = cumtrapz(t, rate);
% Each line is wrapped around its own median so neither is chopped at 0/360.
faceAz   = wrapNear(psi + arrayAxisDeg);
sunAz    = wrapNear(psi + sunDir);
f = newFig('Array vs sun');
tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, breakWraps(faceAz), 'k-', 'LineWidth', 1.8, 'DisplayName', '-X face (solar array)');
plot(t, breakWraps(sunAz), 'Color', [0.9 0.6 0], 'LineWidth', 1.2, 'DisplayName', 'Sun (from sun sensor)');
ylabel('Room-frame angle (deg)'); yticks(-360:45:720);
title('Solar Array Direction vs. Measured Sun Direction (0 = body +X at start)'); grid on;
legend(flipud(findobj(ax, 'Type', 'line')), 'Location', 'best');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, psi, 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\psi (deg)');
title('KestrelSAT Rotation (integrated gyro)'); grid on;
saveFig(f, outDir, name, 'array_vs_sun', combinedPdf);

%% Figure 8 - Sun sensor diagnostic (why does the sun direction flip?)
% Recompute the firmware's calibrated channels (compute_sun in sun_tracker.cpp)
raw  = [sunPX sunPY sunMX sunMY];
span = max(sunCal.bright - sunCal.dark, 1);
nrm  = min(max((raw - sunCal.dark) ./ span, 0), 1.2);
sunX = nrm(:, 1) - nrm(:, 3);
sunY = nrm(:, 2) - nrm(:, 4);
dirCalc = mod(atan2d(sunY, sunX), 360);
railed  = raw >= adcMax;
% A "flip": the error jumps between about +180 and -180 because the sun is
% reported behind the array and sun_Y crossed zero, so the turn direction reverses.
flips   = [false; sign(err(2:end)) ~= sign(err(1:end-1)) & abs(err(2:end)) > 90];
chName  = {'+X', '+Y', '-X', '-Y'};

f = newFig('Sun sensor diagnostic');
f.Position(4) = 9;
tiledlayout(f, 3, 1, 'TileSpacing', 'compact');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
for c = 1:4
    plot(t, nrm(:, c), 'LineWidth', 1.0, 'DisplayName', ...
        sprintf('%s (%.0f%% at ADC max)', chName{c}, 100 * mean(railed(:, c))));
end
yline(1.2, 'k--', 'cap 1.2', 'HandleVisibility', 'off');
ylabel('Calibrated level'); ylim([0 1.3]); title('Calibrated Sun Sensor Channels (1 = calibration "bright")');
grid on; legend(flipud(findobj(ax, 'Type', 'line')), 'Location', 'eastoutside');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
plot(t, sunX, 'r-', 'LineWidth', 1.2, 'DisplayName', 'sun\_X = +X - (-X)');
plot(t, sunY, 'b-', 'LineWidth', 1.2, 'DisplayName', 'sun\_Y = +Y - (-Y)');
yline(0, 'k-', 'HandleVisibility', 'off');
ylabel('Component'); grid on;
title('Sun Vector Components (direction = atan2(sun\_Y, sun\_X); sun\_X < 0 needed to see 180\circ)');
legend(flipud(findobj(ax, 'Type', 'line')), 'Location', 'eastoutside');
ax = nexttile; hold on;
shadeModes(ax, segs, modeOrder, modeColors);
sunDir180 = mod(sunDir + 180, 360) - 180;
plot(t, breakWraps(sunDir180), 'b-', 'LineWidth', 1.0, 'DisplayName', 'Sun direction (body)');
plot(t, breakWraps(err), 'r-', 'LineWidth', 1.0, 'DisplayName', 'Pointing error');
plot(t(flips), err(flips), 'mv', 'MarkerFaceColor', 'm', 'MarkerSize', 5, ...
    'DisplayName', sprintf('Error flips +/-180 (%d)', nnz(flips)));
yline(wrapNear(arrayAxisDeg, 0) * [1 -1], 'k--', 'array axis', 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Angle (deg)'); ylim([-180 180]); yticks(-180:45:180);
title('Sun Direction and Pointing Error, with Turn-Direction Flips Marked'); grid on;
legend(flipud(findobj(ax, 'Type', 'line')), 'Location', 'eastoutside');
saveFig(f, outDir, name, 'sun_sensor_diagnostic', combinedPdf);

%% Figure 9 - Sun vector X-Y plane: which directions can the sensor even report?
f = newFig('Sun vector plane');
ax = axes(f); hold on;
scatter(sunX, sunY, 8, t, 'filled');
th = linspace(0, 2*pi, 200); plot(cos(th), sin(th), 'k:');
plot([0 cosd(arrayAxisDeg)], [0 sind(arrayAxisDeg)], 'k-', 'LineWidth', 2);
text(cosd(arrayAxisDeg), sind(arrayAxisDeg), '  array axis', 'HorizontalAlignment', 'left');
xline(0, 'k-'); yline(0, 'k-');
axis equal; xlim([-1.3 1.3]); ylim([-1.3 1.3]); grid on;
cb = colorbar; cb.Label.String = 'Time (s)';
xlabel('sun\_X'); ylabel('sun\_Y');
title('Measured Sun Vector (points must reach the left half to face the array)');
saveFig(f, outDir, name, 'sun_vector_plane', combinedPdf);

%% Figure 10 - Phase plane (error vs rate), TRACK mode only
trk = mode == "TRACK";
f = newFig('Phase plane');
ax = axes(f); hold on;
scatter(err(trk), rate(trk), 8, t(trk), 'filled');
plot(err(find(trk, 1)), rate(find(trk, 1)), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(err(find(trk, 1, 'last')), rate(find(trk, 1, 'last')), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xline(0, 'k:'); yline(0, 'k:');
cb = colorbar; cb.Label.String = 'Time (s)';
xlabel('Pointing error (deg)'); ylabel('Body rate (deg/s)');
title('Phase Plane During TRACK (o = start, x = end)'); grid on;
xlim([-180 180]);
saveFig(f, outDir, name, 'phase_plane', combinedPdf);

%% Figure 11 - Pointing error histogram in TRACK
f = newFig('Error histogram');
histogram(err(trk), -180:5:180, 'Normalization', 'probability');
xline([-errThreshDeg errThreshDeg], 'r--');
xlabel('Pointing error (deg)'); ylabel('Fraction of TRACK samples');
title(sprintf('Pointing Error Distribution in TRACK (\\pm%d^\\circ band in red)', errThreshDeg));
grid on;
saveFig(f, outDir, name, 'error_histogram', combinedPdf);

%% Summary statistics
dt = median(diff(t));
fprintf('Duration: %.1f s, %d samples, dt = %.0f ms\n', t(end), numel(t), dt * 1e3);
for m = 1:numel(segs)
    fprintf('  %-6s  %7.2f - %7.2f s\n', segs(m).mode, segs(m).t0, segs(m).t1);
end
if any(trk)
    e = err(trk);
    inBand = abs(e) <= errThreshDeg;
    fprintf('TRACK pointing error: mean %.1f, RMS %.1f, max |err| %.1f deg\n', ...
        mean(e), rms(e), max(abs(e)));
    fprintf('TRACK time within +/-%d deg: %.1f %%\n', errThreshDeg, 100 * mean(inBand));
    % Settling: first time after which error stays within the band
    tt = t(trk);
    lastOut = find(~inBand, 1, 'last');
    if isempty(lastOut)
        fprintf('Settling time: on target for all of TRACK\n');
    elseif lastOut == numel(e)
        fprintf('Settling time: never settled within +/-%d deg\n', errThreshDeg);
    else
        fprintf('Settling time: %.2f s after TRACK start\n', tt(lastOut + 1) - tt(1));
    end
    fprintf('TRACK body rate: RMS %.2f deg/s\n', rms(rate(trk)));
end
fprintf('Wheel saturated: %.1f %% of samples\n', 100 * mean(sat));
fprintf('Sun sensor: samples at ADC max  +X %.0f%%  +Y %.0f%%  -X %.0f%%  -Y %.0f%%\n', 100 * mean(railed));
fprintf('Sun sensor: sun_X range %.2f to %.2f (must go negative to see the array axis)\n', min(sunX), max(sunX));
fprintf('Sun sensor: %d pointing-error flips across +/-180 deg (turn direction reversed)\n', nnz(flips));
fprintf('Recomputed vs logged direction: max diff %.2f deg (check sunCal matches firmware)\n', ...
    max(abs(mod(dirCalc - sunDir + 180, 360) - 180)));
fprintf('Wheel speed tracking error (cmd - meas): RMS %.1f RPM\n', rms(rwCmd - rwMeas));
fprintf('Plots saved to %s\n', outDir);
end

%% ------------------------------------------------------------------------
function segs = modeSegments(t, mode)
% Contiguous blocks of identical mode -> struct array (mode, t0, t1)
idx = [1; find(mode(2:end) ~= mode(1:end-1)) + 1; numel(mode) + 1];
segs = struct('mode', {}, 't0', {}, 't1', {});
for i = 1:numel(idx) - 1
    segs(i).mode = mode(idx(i));
    segs(i).t0   = t(idx(i));
    segs(i).t1   = t(min(idx(i + 1), numel(t)));
end
end

function shadeModes(ax, segs, modeOrder, modeColors)
% Light background patches showing which flight mode is active
yl = [-1e6 1e6];
for i = 1:numel(segs)
    c = modeColors(strcmp(modeOrder, segs(i).mode), :);
    if isempty(c), c = [0.7 0.7 0.7]; end
    patch(ax, [segs(i).t0 segs(i).t1 segs(i).t1 segs(i).t0], yl([1 1 2 2]), c, ...
        'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off', ...
        'YLimInclude', 'off');   % don't let the shading drive the y-limits
end
set(ax, 'Layer', 'top');
xlim(ax, [segs(1).t0 segs(end).t1]);
end

function a = wrapNear(a, center)
% Wrap angles (deg) into [center-180, center+180); center defaults to the circular median
if nargin < 2
    center = atan2d(median(sind(a)), median(cosd(a)));
end
a = mod(a - center + 180, 360) - 180 + center;
end

function y = breakWraps(y)
% Insert NaN at +/-180 deg wrap jumps so plots don't draw vertical lines
y(find(abs(diff(y)) > 180) + 1) = NaN;
end

function modeLegend(ax, segs, modeOrder, modeColors)
% Legend showing mode shading colors
present = unique([segs.mode], 'stable');
h = gobjects(numel(present), 1);
for i = 1:numel(present)
    c = modeColors(strcmp(modeOrder, present(i)), :);
    h(i) = patch(ax, NaN, NaN, c, 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
        'DisplayName', char(present(i)));
end
legend(ax, h, 'Location', 'best');
end

function f = newFig(label)
f = figure('Name', label, 'Color', 'w', 'Units', 'inches', ...
    'Position', [1 1 10 6.5], 'Visible', 'off');
end

function saveFig(f, outDir, name, tag, combinedPdf)
pdfPath = fullfile(outDir, sprintf('%s_%s.pdf', name, tag));
exportgraphics(f, pdfPath, 'ContentType', 'vector');
exportgraphics(f, combinedPdf, 'ContentType', 'vector', 'Append', true);
close(f);
end
