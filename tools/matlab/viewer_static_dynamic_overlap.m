clear;

% result_dir= '/home/user/removert/results/';
result_dir = '/home/user/Desktop/data/riverside01/removert/results/';

%%
pc_original = pcread(fullfile(result_dir, 'OriginalNoisyMapLocal.pcd'));

pc_static_scanside = pcread(fullfile(result_dir, 'map_static', 'StaticMapScansideMapLocal.pcd'));
pc_dynamic_scanside = pcread(fullfile(result_dir, 'map_dynamic', 'DynamicMapScansideMapLocal.pcd'));

%%
static_pt_size = 2;
dynamic_pt_size = 100;
color_axis = [0, 3]; % meter
% ylim_for_better_view = [-60, 40];
% xlim_for_better_view = [-50, 250];

figure(1); clf;
pcshow(pc_original.Location, 'MarkerSize', static_pt_size ); 
colormap jet; caxis(color_axis); 
% xlim(xlim_for_better_view);
% ylim(ylim_for_better_view);
view(0, 90);
title("original map");

figure(2); clf;
pcshow(pc_static_scanside.Location, [0.98, 0.98, 0.98], 'MarkerSize', static_pt_size ); hold on;
pcshow(pc_dynamic_scanside.Location, 'r', 'MarkerSize', dynamic_pt_size ); 
% xlim(xlim_for_better_view);
% ylim(ylim_for_better_view);
view(0, 90);
title("static map from scan-side removal");

