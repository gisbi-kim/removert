
result_dir= '/home/user/removert/results/';

%%
pc_original = pcread(fullfile(result_dir, 'OriginalNoisyMapLocal.pcd'));

pc_static_mapside = pcread(fullfile(result_dir, 'map_static', 'StaticMapMapsideLocalResX1.500000.pcd'));
pc_dynamic_mapside = pcread(fullfile(result_dir, 'map_dynamic', 'DynamicMapMapsideLocalResX1.500000.pcd'));

pc_static_scanside = pcread(fullfile(result_dir, 'map_static', 'StaticMapScansideMapLocal.pcd'));
pc_dynamic_scanside = pcread(fullfile(result_dir, 'map_dynamic', 'DynamicMapScansideMapLocal.pcd'));

%%
pt_size = 2;
color_axis = [0, 3]; % meter
ylim_for_better_view = [-60, 40];
xlim_for_better_view = [-50, 250];

figure(1); clf;
pcshow(pc_original.Location, 'MarkerSize', pt_size ); 
colormap jet; caxis(color_axis); 
xlim(xlim_for_better_view);
ylim(ylim_for_better_view);
view(0, 90);
title("original map");

figure(2); clf;
pcshow(pc_static_mapside.Location, 'MarkerSize', pt_size ); 
colormap jet; caxis(color_axis); 
xlim(xlim_for_better_view);
ylim(ylim_for_better_view);
view(0, 90);
title("static map from map-side removal");

figure(3); clf;
pcshow(pc_static_scanside.Location, 'MarkerSize', pt_size ); 
colormap jet; caxis(color_axis); 
xlim(xlim_for_better_view);
ylim(ylim_for_better_view);
view(0, 90);
title("static map from scan-side removal");

