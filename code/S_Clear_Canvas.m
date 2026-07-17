% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : N/A                                                %
%   Output : Initialize visualization conditions                %
%                                                               %
% -------------------------- Content -------------------------- %

set(0, 'DefaultFigureRenderer', 'painters')
set(0, 'defaultfigurecolor', [1 1 1]);
set(0, 'DefaultAxesXGrid', 'on');
set(groot, 'DefaultLineLineWidth', 2);
set(0, 'DefaultTextInterpreter', 'latex');   % For all text objects (titles, labels, etc.)
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');  % For axis tick labels
set(0, 'DefaultLegendInterpreter', 'latex'); % For legends

fig_loc = [2500 680 750 700];
Fig = @(fig_loc) figure('rend', 'painters', 'pos', fig_loc);
