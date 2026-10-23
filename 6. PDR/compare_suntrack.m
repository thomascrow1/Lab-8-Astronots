%% compare_suntrack.m
% Astro 331 Lab 8 - overlay two (or more) KestrelSAT sun-tracking runs
%
% Plots the chosen runs on the same axes so the differences between tests
% are easy to see. Writes PDF plots to ./<SunTrack_AAA_vs_BBB>/ next to this
% script, plus a combined multi-page PDF, and prints a side-by-side summary.
% CSV files are found anywhere under the 5. DATA folder.

clear; clc; close all;

%% User settings
runNumbers   = [51 52];   % runs to overlay (first one is the baseline)
scriptDir    = fileparts(mfilename('fullpath'));
dataDir      = fileparts(scriptDir);           % 5. DATA
errThreshDeg = 10;        % "on target" band [deg]
arrayAxisDeg = 180;       % solar array normal in body frame (-X face)
adcMax       = 4095;      % 12-bit ADC full scale
runColors    = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19; 0.49 0.18 0.56];

%% Load runs
R = struct('num', {}, 'label', {}, 't', {}, 'mode', {}, 'raw', {}, 'sunMag', {}, ...
    'sunDir', {}, 'err', {}, 'rate', {}, 'torque', {}, 'rwCmd', {}, 'rwMeas', {}, ...
    'sat', {}, 'tTrack', {});
for k = 1:numel(runNumbers)
    name = sprintf('SunTrack_data%03d.csv', runNumbers(k));
    hit  = dir(fullfile(dataDir, '**', name));
    if isempty(hit), error('Could not find %s under %s', name, dataDir); end
    T = readtable(fullfile(hit(1).folder, hit(1).name), ...
        detectImportOptions(fullfile(hit(1).folder, hit(1).name), 'VariableNamingRule', 'preserve'));
    r.num    = runNumbers(k);
    r.label  = sprintf('Run %03d', runNumbers(k));
    r.t      = (T.("mcu time(ms)") - T.("mcu time(ms)")(1)) / 1000;
    r.mode   = string(T.mode);
    r.raw    = [T.("sun_plusX(count)") T.("sun_plusY(count)") T.("sun_minusX(count)") T.("sun_minusY(count)")];
    r.sunMag = T.sun_mag;
    r.sunDir = T.("sun_direction(deg)");
    r.err    = mod(T.("error(deg)") + 180, 360) - 180;   % signed, 0 = on target
    r.rate   = T.("body_rate(deg/s)");
    r.torque = T.("torque_cmd(N*m)");
    r.rwCmd  = T.("w_RW_cmd(RPM)");
    r.rwMeas = T.("w_RW_meas(RPM)");
    r.sat    = logical(T.saturated);
    r.tTrack = r.t(find(r.mode == "TRACK", 1));
    R(k) = r;
end

tag    = ['SunTrack_' strjoin(arrayfun(@(n) sprintf('%03d', n), runNumbers, 'UniformOutput', false), '_vs_')];
outDir = fullfile(scriptDir, tag);
if ~exist(outDir, 'dir'), mkdir(outDir); end
combinedPdf = fullfile(outDir, [tag '_all.pdf']);
if exist(combinedPdf, 'file'), delete(combinedPdf); end
tMax = max(arrayfun(@(r) r.t(end), R));

%% Figure 1 - Pointing error and sun direction
f = newFig('Pointing');
tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
nexttile; hold on;
for k = 1:numel(R)
    plotWrap(R(k).t, R(k).err, [-180 180], 'Color', runColors(k, :), 'LineWidth', 1.1, ...
        'DisplayName', R(k).label);
end
yline([-errThreshDeg errThreshDeg], 'k--', 'HandleVisibility', 'off');
trackLines(R, runColors);
ylabel('Pointing error (deg)'); ylim([-180 180]); yticks(-180:45:180); xlim([0 tMax]);
title(sprintf('Pointing Error (0 = on target, dashed = \\pm%d^\\circ)', errThreshDeg)); grid on;
legend('Location', 'eastoutside');
nexttile; hold on;
for k = 1:numel(R)
    plotWrap(R(k).t, R(k).sunDir, [0 360], 'Color', runColors(k, :), 'LineWidth', 1.1, ...
        'DisplayName', R(k).label);
end
yline(arrayAxisDeg, 'k--', 'array axis', 'HandleVisibility', 'off');
trackLines(R, runColors);
xlabel('Time (s)'); ylabel('Sun direction (deg)'); ylim([0 360]); yticks(0:90:360); xlim([0 tMax]);
title('Sun Direction in Body Frame (target = 180, the -X array face)'); grid on;
legend('Location', 'eastoutside');
saveFig(f, outDir, tag, 'pointing', combinedPdf);

%% Figure 2 - Body rotation and rate
f = newFig('Rotation');
tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
nexttile; hold on;
for k = 1:numel(R)
    plot(R(k).t, cumtrapz(R(k).t, R(k).rate), 'Color', runColors(k, :), 'LineWidth', 1.2, ...
        'DisplayName', R(k).label);
end
trackLines(R, runColors);
ylabel('\psi (deg)'); xlim([0 tMax]); grid on;
title('KestrelSAT Rotation (integrated gyro, 0 = start heading)');
legend('Location', 'eastoutside');
nexttile; hold on;
for k = 1:numel(R)
    plot(R(k).t, R(k).rate, 'Color', runColors(k, :), 'LineWidth', 1.0, 'DisplayName', R(k).label);
end
yline(0, 'k:', 'HandleVisibility', 'off');
trackLines(R, runColors);
xlabel('Time (s)'); ylabel('Body rate (deg/s)'); xlim([0 tMax]); grid on;
title('Z-Axis Body Rate (Gyro)');
legend('Location', 'eastoutside');
saveFig(f, outDir, tag, 'rotation', combinedPdf);

%% Figure 3 - Reaction wheel
f = newFig('Reaction wheel');
tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
nexttile; hold on;
for k = 1:numel(R)
    plot(R(k).t, R(k).rwMeas, '-', 'Color', runColors(k, :), 'LineWidth', 1.1, ...
        'DisplayName', [R(k).label ' measured']);
    plot(R(k).t, R(k).rwCmd, ':', 'Color', runColors(k, :), 'LineWidth', 1.1, ...
        'DisplayName', [R(k).label ' commanded']);
end
trackLines(R, runColors);
ylabel('Wheel speed (RPM)'); xlim([0 tMax]); grid on;
title('Reaction Wheel Speed (solid = measured, dotted = commanded)');
legend('Location', 'eastoutside');
nexttile; hold on;
for k = 1:numel(R)
    plot(R(k).t, R(k).torque * 1e3, 'Color', runColors(k, :), 'LineWidth', 1.0, ...
        'DisplayName', sprintf('%s (saturated %.0f%%)', R(k).label, 100 * mean(R(k).sat)));
end
trackLines(R, runColors);
xlabel('Time (s)'); ylabel('Torque cmd (mN\cdotm)'); xlim([0 tMax]); grid on;
title('Commanded Torque');
legend('Location', 'eastoutside');
saveFig(f, outDir, tag, 'reaction_wheel', combinedPdf);

%% Figure 4 - Sun sensor photodiodes, one panel per channel
chName = {'+X', '+Y', '-X', '-Y'};
f = newFig('Sun sensors');
f.Position(4) = 8;
tiledlayout(f, 2, 2, 'TileSpacing', 'compact');
for c = 1:4
    nexttile; hold on;
    for k = 1:numel(R)
        plot(R(k).t, R(k).raw(:, c), 'Color', runColors(k, :), 'LineWidth', 0.9, ...
            'DisplayName', sprintf('%s (%.0f%% at max)', R(k).label, 100 * mean(R(k).raw(:, c) >= adcMax)));
    end
    yline(adcMax, 'k--', 'ADC max', 'HandleVisibility', 'off');
    ylim([0 4400]); xlim([0 tMax]); grid on;
    title(sprintf('%s photodiode', chName{c}));
    if c > 2, xlabel('Time (s)'); end
    if mod(c, 2) == 1, ylabel('ADC counts'); end
    legend('Location', 'south');
end
saveFig(f, outDir, tag, 'sun_sensors', combinedPdf);

%% Figure 5 - Sun vector plane (logged sun direction and magnitude)
f = newFig('Sun vector plane');
ax = axes(f); hold on;
for k = 1:numel(R)
    scatter(R(k).sunMag .* cosd(R(k).sunDir), R(k).sunMag .* sind(R(k).sunDir), 6, ...
        runColors(k, :), 'filled', 'MarkerFaceAlpha', 0.35, 'DisplayName', R(k).label);
end
lim = 1.1 * max(arrayfun(@(r) max(r.sunMag), R));
plot([0 lim * cosd(arrayAxisDeg)], [0 lim * sind(arrayAxisDeg)], 'k-', 'LineWidth', 2, ...
    'DisplayName', 'Array axis (target)');
xline(0, 'k-', 'HandleVisibility', 'off'); yline(0, 'k-', 'HandleVisibility', 'off');
axis(ax, 'equal'); xlim([-lim lim]); ylim([-lim lim]); grid on;
xlabel('sun\_mag \cdot cos(direction)'); ylabel('sun\_mag \cdot sin(direction)');
title('Measured Sun Vector in Body Frame (on target = along the array axis)');
legend('Location', 'eastoutside');
saveFig(f, outDir, tag, 'sun_vector_plane', combinedPdf);

%% Figure 6 - Pointing error histogram in TRACK
f = newFig('Error histogram');
ax = axes(f); hold on;
for k = 1:numel(R)
    e = R(k).err(R(k).mode == "TRACK");
    histogram(e, -180:5:180, 'Normalization', 'probability', 'FaceColor', runColors(k, :), ...
        'FaceAlpha', 0.45, 'EdgeColor', 'none', ...
        'DisplayName', sprintf('%s (%.0f%% within \\pm%d^\\circ)', R(k).label, ...
        100 * mean(abs(e) <= errThreshDeg), errThreshDeg));
end
xline([-errThreshDeg errThreshDeg], 'k--', 'HandleVisibility', 'off');
xlim([-180 180]); xticks(-180:45:180); grid on;
xlabel('Pointing error (deg)'); ylabel('Fraction of TRACK samples');
title('Pointing Error Distribution in TRACK');
legend('Location', 'northeast');
saveFig(f, outDir, tag, 'error_histogram', combinedPdf);

%% Side-by-side summary
fprintf('%-34s', '');
fprintf('%14s', R.label); fprintf('\n');
row('Duration (s)',                     arrayfun(@(r) r.t(end), R), '%14.1f');
row('TRACK start (s)',                  [R.tTrack], '%14.1f');
row(sprintf('TRACK within +/-%d deg (%%)', errThreshDeg), ...
    arrayfun(@(r) 100 * mean(abs(r.err(r.mode == "TRACK")) <= errThreshDeg), R), '%14.1f');
row('TRACK error mean (deg)',           arrayfun(@(r) mean(r.err(r.mode == "TRACK")), R), '%14.1f');
row('TRACK error RMS (deg)',            arrayfun(@(r) rms(r.err(r.mode == "TRACK")), R), '%14.1f');
row('TRACK body rate RMS (deg/s)',      arrayfun(@(r) rms(r.rate(r.mode == "TRACK")), R), '%14.2f');
row('Turn reversals (error crosses 180)', arrayfun(@(r) nnz(abs(diff(r.err)) > 180), R), '%14d');
row('Wheel saturated (%)',              arrayfun(@(r) 100 * mean(r.sat), R), '%14.1f');
for c = 1:4
    row(sprintf('%s sensor at ADC max (%%)', chName{c}), ...
        arrayfun(@(r) 100 * mean(r.raw(:, c) >= adcMax), R), '%14.1f');
end
fprintf('Plots saved to %s\n', outDir);

%% ------------------------------------------------------------------------
function row(label, vals, fmt)
fprintf('%-34s', label); fprintf(fmt, vals); fprintf('\n');
end

function trackLines(R, runColors)
% Dashed vertical line where each run enters TRACK mode
for k = 1:numel(R)
    if ~isempty(R(k).tTrack)
        xline(R(k).tTrack, ':', 'Color', runColors(k, :), 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
    end
end
end

function h = plotWrap(t, y, lims, varargin)
% Plot an angle that wraps between lims(1) and lims(2). At each wrap the
% line runs to the edge of the axis and continues from the opposite edge,
% instead of drawing a line straight across the plot.
t = t(:); y = y(:);
span = lims(2) - lims(1);
w = find(abs(diff(y)) > span / 2);
tt = t; yy = y;
for i = flipud(w)'
    if y(i + 1) < y(i)                    % crossed the top going up
        yA = lims(2); yB = lims(1); d = y(i + 1) + span - y(i);
    else                                  % crossed the bottom going down
        yA = lims(1); yB = lims(2); d = y(i + 1) - span - y(i);
    end
    tc = t(i) + (t(i + 1) - t(i)) * (yA - y(i)) / d;
    tt = [tt(1:i); tc; NaN; tc; tt(i + 1:end)];
    yy = [yy(1:i); yA; NaN; yB; yy(i + 1:end)];
end
h = plot(tt, yy, varargin{:});
end

function f = newFig(label)
f = figure('Name', label, 'Color', 'w', 'Units', 'inches', ...
    'Position', [1 1 11 6.5], 'Visible', 'off');
end

function saveFig(f, outDir, tag, plotTag, combinedPdf)
pdfPath = fullfile(outDir, sprintf('%s_%s.pdf', tag, plotTag));
exportgraphics(f, pdfPath, 'ContentType', 'vector');
exportgraphics(f, combinedPdf, 'ContentType', 'vector', 'Append', true);
close(f);
end
