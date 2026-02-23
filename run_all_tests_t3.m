clear;
clc;

SCRIPT_DIR = fileparts(mfilename('fullpath'));
DB_ROOT = fullfile(SCRIPT_DIR, 'DB_T3');
WINDOW = 10;
TARGET_DIST = 10;
dirs = {'S','W','E'};

fprintf('=== RUN ALL TESTS T (PERSONAL vs GLOBAL) ===\n');
fprintf('Window=%d, Target=%.1f\n\n', WINDOW, TARGET_DIST);

rows = {};
for i = 1:numel(dirs)
    d = dirs{i};
    test_file = fullfile(SCRIPT_DIR, sprintf('test_generated_t3_%s.csv', d));
    db_personal = fullfile(DB_ROOT, sprintf('veh_%s', d));

    if ~exist(test_file, 'file')
        fprintf('[%s] missing test file: %s\n', d, test_file);
        continue;
    end
    if ~exist(db_personal, 'dir')
        fprintf('[%s] missing personal DB: %s\n', d, db_personal);
        continue;
    end

    fprintf('--- Direction %s ---\n', d);
    fprintf('Personal DB:\n');
    m_personal = run_predict_capture(test_file, WINDOW, TARGET_DIST, db_personal);
    fprintf('Global DB:\n');
    m_global = run_predict_capture(test_file, WINDOW, TARGET_DIST, DB_ROOT);

    rows{end+1} = struct('dir', d, ...
        'pred_p', m_personal.total, 'pred_g', m_global.total, ...
        'acc_p', m_personal.acc, 'acc_g', m_global.acc, ...
        'w1_p', m_personal.within1, 'w1_g', m_global.within1); %#ok<AGROW>
    fprintf('\n');
end

fprintf('=== SUMMARY (T) ===\n');
fprintf('Dir | Pred(P/G) | Acc%% (P/G) | <=1%% (P/G)\n');
for i = 1:numel(rows)
    r = rows{i};
    fprintf('%3s | %4d/%-4d | %6.2f/%-6.2f | %6.2f/%-6.2f\n', ...
        r.dir, r.pred_p, r.pred_g, r.acc_p, r.acc_g, r.w1_p, r.w1_g);
end

function m = run_predict_capture(test_file, window, target_dist, db_folder)
txt = evalc(call_predict(test_file, window, target_dist, db_folder));
m = parse_metrics(txt);
end

function cmd = call_predict(test_file, window, target_dist, db_folder)
cmd = sprintf('mygrid3(''predict'',''%s'',0,%d,%g,''%s'');', ...
    escape_path(test_file), window, target_dist, escape_path(db_folder));
end

function p = escape_path(p)
p = strrep(p, '''', '''''');
end

function m = parse_metrics(txt)
m = struct('total', 0, 'acc', NaN, 'within1', NaN);
tok_total = regexp(txt, 'Total Predictions:\s*(\d+)', 'tokens', 'once');
tok_acc = regexp(txt, 'Accuracy \(exact\):\s*([0-9.]+)%', 'tokens', 'once');
tok_w1 = regexp(txt, 'Within 1 Grid:\s*([0-9.]+)%', 'tokens', 'once');
if ~isempty(tok_total), m.total = str2double(tok_total{1}); end
if ~isempty(tok_acc), m.acc = str2double(tok_acc{1}); end
if ~isempty(tok_w1), m.within1 = str2double(tok_w1{1}); end
end
