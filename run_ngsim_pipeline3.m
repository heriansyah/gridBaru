clear;
clc;

disp('=== STEP 1: GENERATING SYNTHETIC DATA (SMARTGRID3) ===');
run(fullfile(fileparts(mfilename('fullpath')), 'generate_custom_data3.m'));

disp('=== STEP 2: TRAINING MODEL (SMARTGRID3) ===');
run(fullfile(fileparts(mfilename('fullpath')), 'train_generated3.m'));

disp('=== STEP 3: TESTING (MYGRID3) ===');
mygrid3('predict', fullfile(fileparts(mfilename('fullpath')), 'test_generated3.csv'), 0, 15, 10);

% Optional:
% disp('=== STEP 4: VISUALIZATION (SMARTGRID3) ===');
% run(fullfile(fileparts(mfilename('fullpath')), 'visualize_grid3.m'));


