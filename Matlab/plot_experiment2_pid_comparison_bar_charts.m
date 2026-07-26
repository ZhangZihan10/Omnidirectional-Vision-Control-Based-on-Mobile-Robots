%% 北京会议，实验2绘制统计图 Plot Experiment 2 PID comparison grouped bar charts
% This script reads the PID comparison table from the Excel file in the
% current folder and plots three separate grouped bar charts:
% 1) Wheel RMSE
% 2) Max Error
% 3) Final Error
%
% The x-axis is test1, test2, and test3. Each group compares three methods:
% Feedforward only, Fixed PID, and Fuzzy adaptive PID.

clear; clc;

excel_files = dir("*.xlsx");
excel_files = excel_files(~startsWith({excel_files.name}, "~$"));
if isempty(excel_files)
    error("No Excel file (*.xlsx) was found in the current folder.");
end

[~, newest_file_idx] = max([excel_files.datenum]);
excel_file = string(excel_files(newest_file_idx).name);
sheet_name = "Sheet1";

% ===== Global figure style =====
font_name = "Times New Roman";
font_size = 20;
bar_colors = [0.20, 0.45, 0.75;
              0.85, 0.35, 0.15;
              0.95, 0.65, 0.10];

% ===== Read Experiment 2 table =====
raw = readcell(excel_file, "Sheet", sheet_name);

header_row = findHeaderRow(raw, "Control mode:");
if isnan(header_row)
    error("Could not find the Experiment 2 header row containing 'Control mode:'.");
end

headers = string(raw(header_row, :));
method_col = find(headers == "Control mode:", 1);
test_col = method_col - 1;
rmse_col = find(headers == "Wheel RMSE:mm/s", 1);
max_error_col = find(headers == "Max Error:mm/s", 1);
final_error_col = find(headers == "Final Error:mm", 1);

if isempty(method_col) || isempty(rmse_col) || isempty(max_error_col) || isempty(final_error_col)
    error("Experiment 2 columns were not found. Please check the Excel header names.");
end

if test_col < 1
    error("Could not determine the test label column before 'Control mode:'.");
end

data_start = header_row + 1;
data_end = findExperiment2DataEnd(raw, data_start, method_col);
data = raw(data_start:data_end, :);

test_raw = string(data(:, test_col));
test_raw = strip(test_raw);
test_labels = fillDownTestLabels(test_raw);

method_raw = string(data(:, method_col));
method_raw = strip(method_raw);

rmse_values = cellToDoubleVector(data(:, rmse_col));
max_error_values = cellToDoubleVector(data(:, max_error_col));
final_error_values = cellToDoubleVector(data(:, final_error_col));

method_ids = ["FEEDFORWARD_ONLY", "Fixed PID", "Fuzzy adaptive PID"];
method_labels = ["Feedforward only", "Fixed PID", "Fuzzy adaptive PID"];
test_ids = unique(test_labels(test_labels ~= ""), "stable");

metrics = {
    rmse_values, "Wheel RMSE", "Wheel RMSE / mm/s";
    max_error_values, "Max Error", "Max Error / mm/s";
    final_error_values, "Final Error", "Final Error / mm"
};

for k = 1:size(metrics, 1)
    values = metrics{k, 1};
    fig_title = metrics{k, 2};
    y_label = metrics{k, 3};

    plot_values = nan(numel(test_ids), numel(method_ids));
    for test_i = 1:numel(test_ids)
        for method_i = 1:numel(method_ids)
            idx = test_labels == test_ids(test_i) & method_raw == method_ids(method_i);
            if any(idx)
                plot_values(test_i, method_i) = values(find(idx, 1));
            end
        end
    end

    figure("Color", "w");
    b = bar(plot_values, "grouped");
    for method_i = 1:numel(method_ids)
        b(method_i).FaceColor = bar_colors(method_i, :);
        b(method_i).EdgeColor = "none";
    end

    grid on;
    box on;
    xlabel("Test", "FontName", font_name, "FontSize", font_size);
    ylabel(y_label, "FontName", font_name, "FontSize", font_size);
    title(fig_title, "FontName", font_name, "FontSize", font_size);
    legend(method_labels, "Location", "best");

    ax = gca;
    ax.FontName = font_name;
    ax.FontSize = font_size;
    ax.LineWidth = 1.5;
    ax.XTick = 1:numel(test_ids);
    ax.XTickLabel = test_ids;
    xlim([0.4, numel(test_ids) + 0.6]);
end

result_table = table(test_labels, method_raw, rmse_values, max_error_values, final_error_values, ...
    'VariableNames', {'Test', 'Method', 'Wheel_RMSE', 'Max_Error', 'Final_Error'});
disp(result_table);

function header_row = findHeaderRow(raw, header_text)
%FINDHEADERROW Find the row containing a target header text.
    header_row = NaN;
    for rr = 1:size(raw, 1)
        row_values = string(raw(rr, :));
        if any(row_values == header_text)
            header_row = rr;
            return;
        end
    end
end

function data_end = findExperiment2DataEnd(raw, data_start, method_col)
%FINDEXPERIMENT2DATAEND Find the last row of the Experiment 2 data table.
    data_end = data_start - 1;
    for rr = data_start:size(raw, 1)
        method_value = string(raw{rr, method_col});
        if strlength(strip(method_value)) == 0
            break;
        end
        data_end = rr;
    end
end

function labels = fillDownTestLabels(test_raw)
%FILLDOWNTESTLABELS Fill merged/blank test labels from the previous row.
    labels = test_raw;
    last_label = "";
    for ii = 1:numel(labels)
        if strlength(labels(ii)) > 0
            last_label = labels(ii);
        else
            labels(ii) = last_label;
        end
    end
end

function values = cellToDoubleVector(cells)
%CELLTODOUBLEVECTOR Convert readcell output to numeric values.
    values = nan(size(cells));
    for ii = 1:numel(cells)
        x = cells{ii};
        if isnumeric(x)
            values(ii) = x;
        elseif isstring(x) || ischar(x)
            token = regexp(string(x), "[-+]?\d*\.?\d+", "match", "once");
            if ~isempty(token)
                values(ii) = str2double(token);
            end
        end
    end
end
