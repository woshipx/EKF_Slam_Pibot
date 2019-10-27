function h = draw_error_ellipse(state,Sigma)
%Draw error ellipse and return objects handle
%   mu = [x,y]
%   Sigma = [xx xy; xy xx]

p = 0.2;
s = -2 * log(1 - p);
 
[V, D] = eig(Sigma * s);
t = linspace(0, 2 * pi);
a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

hold on;
h = plot(a(1, :) + state(1), a(2, :) + state(2),'r');
hold off;

end

