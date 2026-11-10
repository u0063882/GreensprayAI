function viewRegisteredContributions(folder)
%VIEWREGISTEREDCONTRIBUTIONS Display the exported PLYs in the toolbox row frame.
% Extract separate_registered_scans.zip, then supply that folder.
arguments
    folder (1,1) string
end
files=dir(fullfile(folder,'scan_*_registered.ply'));
assert(~isempty(files),'No separate registered scan PLY files found.');
% Proper +90-degree rotation, not an axis reflection:
% MATLAB X_across = 1.9875 - Python Y_across
% MATLAB Y_along  = Python X_along
R=[0 -1 0;1 0 0;0 0 1]; t=[1.9875 0 0];
figure('Color','w');ax=axes;hold(ax,'on');
for k=1:numel(files)
    pc=pcread(fullfile(files(k).folder,files(k).name));
    pc=pctransform(pc,rigidtform3d(R,t));
    pcshow(pc,'Parent',ax,'MarkerSize',2);
end
axis(ax,'equal');grid(ax,'on');view(ax,3);
xlabel(ax,'Across tree row X [m]');ylabel(ax,'Along tree row Y [m]');zlabel(ax,'Up Z [m]');
title(ax,'Registered contributions; one stored RGB colour per recording');
end
