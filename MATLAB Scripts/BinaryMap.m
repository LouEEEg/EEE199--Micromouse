% Hard coded Binary occupancy map. This code is resused in Navigation.m

w = 13;     % width of the walls   (mm)
x = 1150;   % length of the x-axis (mm)
y = 570;    % length of the y-axis (mm)

total_wall = 7*w;
cell_size = (1150-total_wall)/6;

a = zeros(y,x);         % Map size

a(1:w,:)=1;             % Top wall
a((y-w):y,:)=1;         % Bottom wall 
a(:,1:w)=1;             % Left wall
a(:,(x-w):x)=1;         % Right wall

home = 2*w + 2*cell_size;
a(floor(w+cell_size):y, home:floor(home+w))=1;
a(floor(w+cell_size):floor(2*w+cell_size),floor(home-cell_size):floor(home+w)) = 1;
a(1:floor(w+cell_size),floor(3*(w+cell_size)):floor(3*(w+cell_size))+w)=1;
a(floor(2*(cell_size+w)):y,floor(3*(w+cell_size)):floor(3*(w+cell_size))+w)=1;
a(1:floor(2*(w+cell_size)),floor(4*(w+cell_size)):floor(4*(w+cell_size))+w)=1;
a(floor(w+cell_size):floor(2*(w+cell_size))+w,floor(5*(w+cell_size)):floor(5*(w+cell_size))+w)=1;
a(floor(2*(w+cell_size)):floor(2*(w+cell_size))+w, floor(6*w+5*cell_size):x)=1;

map = occupancyMap(a,'Resolution', 1000);
show(map);