%% Experiment 1: path-planning performance in three obstacle scenes
% Read only the 15 Experiment 1 records (5 trials per scene) from
% 实验数据.xlsx. Bars show means; error bars show sample SD (n - 1).

clear;
clc;

excelFile = "实验数据.xlsx";
sheetName = "Sheet1";
trialsPerScene = 5;

if ~isfile(excelFile)
    error("Data file not found: %s", excelFile);
end

%% Read and isolate Experiment 1
raw = readcell(excelFile, "Sheet", sheetName);
headerRow = find(strcmp(strtrim(string(raw(:, 1))), "场景"), 1, "first");
if isempty(headerRow)
    error("Could not find the Experiment 1 header '场景' in %s.", excelFile);
end

data = raw(headerRow + 1:end, :);
sceneId = cellToDoubleVector(data(:, 1));
trialId = cellToDoubleVector(data(:, 2));
minObstacleDistance = cellToDoubleVector(data(:, 4));
actualTrajectoryLength = cellToDoubleVector(data(:, 8));
averageFps = cellToDoubleVector(data(:, 10));

% Scene identifiers are stored once per merged five-row block in Excel.
for rowIdx = 2:numel(sceneId)
    if isnan(sceneId(rowIdx))
        sceneId(rowIdx) = sceneId(rowIdx - 1);
    end
end

% Exclude blank rows and the PID comparison table below Experiment 1.
isExperiment1Row = ismember(trialId, 1:trialsPerScene) ...
    & isfinite(sceneId) ...
    & isfinite(minObstacleDistance) ...
    & isfinite(actualTrajectoryLength) ...
    & isfinite(averageFps);

sceneId = sceneId(isExperiment1Row);
trialId = trialId(isExperiment1Row);
minObstacleDistance = minObstacleDistance(isExperiment1Row);
actualTrajectoryLength = actualTrajectoryLength(isExperiment1Row);
averageFps = averageFps(isExperiment1Row);

sceneList = unique(sceneId, "stable");
numScenes = numel(sceneList);
if numScenes ~= 3
    error("Expected 3 scenes, but found %d.", numScenes);
end

for sceneIdx = 1:numScenes
    currentTrials = trialId(sceneId == sceneList(sceneIdx));
    if numel(currentTrials) ~= trialsPerScene ...
            || ~isequal(sort(currentTrials(:))', 1:trialsPerScene)
        error("Scene %d must contain trials 1-%d exactly once.", ...
            sceneList(sceneIdx), trialsPerScene);
    end
end

%% Calculate mean and sample standard deviation
metricValues = {
    actualTrajectoryLength, ...
    minObstacleDistance, ...
    averageFps
};

metricMeans = zeros(numScenes, numel(metricValues));
metricStds = zeros(numScenes, numel(metricValues));

for metricIdx = 1:numel(metricValues)
    values = metricValues{metricIdx};
    for sceneIdx = 1:numScenes
        sceneValues = values(sceneId == sceneList(sceneIdx));
        metricMeans(sceneIdx, metricIdx) = mean(sceneValues);
        metricStds(sceneIdx, metricIdx) = std(sceneValues, 0);
    end
end

summaryTable = table( ...
    sceneList(:), ...
    metricMeans(:, 1), metricStds(:, 1), ...
    metricMeans(:, 2), metricStds(:, 2), ...
    metricMeans(:, 3), metricStds(:, 3), ...
    'VariableNames', { ...
        'Scene', ...
        'TrajectoryMean_mm', 'TrajectorySD_mm', ...
        'MinDistanceMean_mm', 'MinDistanceSD_mm', ...
        'FPSMean', 'FPSSD'});
disp(summaryTable);

%% Draw a single horizontal three-panel figure
fontName = "Times New Roman";
fontSize = 15;
errorColor = [0.10, 0.10, 0.10];
xPosition = 1:numScenes;
xLabels = compose("Scene %d", sceneList);

titles = [
    "Actual trajectory length", ...
    "Minimum obstacle distance", ...
    "Average FPS"
];
yLabels = [
    "Actual trajectory length (mm)", ...
    "Minimum obstacle distance (mm)", ...
    "Average FPS"
];
barColors = [
    0.61, 0.89, 0.95; ...
    0.30, 0.55, 0.85; ...
    0.72, 0.95, 0.74
];
yLimits = [
    0, 4000; ...
    0, 200; ...
    0, 3.5
];

fig = figure( ...
    "Color", "w", ...
    "Units", "pixels", ...
    "Position", [100, 100, 1500, 460]);
layout = tiledlayout(fig, 1, 3, ...
    "TileSpacing", "compact", ...
    "Padding", "compact");

for metricIdx = 1:numel(metricValues)
    ax = nexttile(layout, metricIdx);

    bar(ax, xPosition, metricMeans(:, metricIdx), 0.62, ...
        "FaceColor", barColors(metricIdx, :), ...
        "EdgeColor", "none");
    hold(ax, "on");

    errorbar(ax, xPosition, metricMeans(:, metricIdx), ...
        metricStds(:, metricIdx), ...
        "LineStyle", "none", ...
        "Color", errorColor, ...
        "LineWidth", 1.6, ...
        "CapSize", 10);

    title(ax, titles(metricIdx), "FontWeight", "normal");
    ylabel(ax, yLabels(metricIdx));
    xlim(ax, [0.45, numScenes + 0.55]);
    ylim(ax, yLimits(metricIdx, :));
    xticks(ax, xPosition);
    xticklabels(ax, xLabels);

    grid(ax, "on");
    box(ax, "on");
    ax.XGrid = "on";
    ax.YGrid = "on";
    ax.GridAlpha = 0.20;
    ax.FontName = fontName;
    ax.FontSize = fontSize;
    ax.LineWidth = 1.0;
    ax.Layer = "top";

    hold(ax, "off");
end

set(findall(fig, "-property", "FontName"), "FontName", fontName);
exportgraphics(fig, "experiment1_performance_comparison.png", ...
    "Resolution", 600);

function values = cellToDoubleVector(cells)
%CELLTODOUBLEVECTOR Convert a mixed readcell column to a numeric vector.
    values = nan(size(cells));
    for idx = 1:numel(cells)
        value = cells{idx};
        if isnumeric(value) && isscalar(value)
            values(idx) = value;
        elseif isstring(value) || ischar(value)
            values(idx) = str2double(string(value));
        end
    end
end
