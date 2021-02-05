


mesh_base_dir = fullfile(get_this_dir(), 'meshes');
[v_base, f_base, n_base, ~] = ...
                 stlReadBinary(fullfile(mesh_base_dir, 'base.stl'));

body_color = [0.2000    0.4000    1.0000];
g_base = hgtransform;     
             
name = '';
figure;
stlPlot(v_base, f_base, name, 'FaceColor', body_color,'EdgeColor', [0,0,0], 'Parent', hgtransform);



camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
view([-135 35]);
grid on;
title(name);