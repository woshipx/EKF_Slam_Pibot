function draw_map()

    hold on;
    
    plot([0,2],[0,0],'r','linewidth',2);
    
    x0=2;
    y0=0.5;
    r=0.5;
    theta=3*pi/2:pi/158:2*pi;
    x=x0+r*cos(theta);
    y=y0+r*sin(theta);
    plot(x,y,'r','linewidth',2);

    plot([2,2],[-0.05,0.05],'b','linewidth',2);

    x0=3;
    y0=0.5;
    r=0.5;
    theta=pi/2:pi/158:pi;
    x=x0+r*cos(theta);
    y=y0+r*sin(theta);
    plot(x,y,'r','linewidth',2);

    x0=3;
    y0=1.5;
    r=0.5;
    theta=pi/2*3:pi/158:pi/2*5;
    x=x0+r*cos(theta);
    y=y0+r*sin(theta);
    plot(x,y,'r','linewidth',2);

    plot([3,3],[0.95,1.05],'b','linewidth',2);
    plot([0,0],[-0.05,0.05],'b','linewidth',2);
    plot([2,3],[2,2],'r','linewidth',2);
    plot([3,3],[1.95,2.05],'b','linewidth',2);

    x0=2;
    y0=1.5;
    r=0.5;
    theta=pi/2:pi/158:pi;
    x=x0+r*cos(theta);
    y=y0+r*sin(theta);
    plot(x,y,'r','linewidth',2);

    plot([2,2],[1.95,2.05],'b','linewidth',2);

    x0=1;
    y0=1.5;
    r=0.5;
    theta=3*pi/2:pi/158:2*pi;
    x=x0+r*cos(theta);
    y=y0+r*sin(theta);
    plot(x,y,'r','linewidth',2);

    x0=1;
    y0=0.5;
    r=0.5;
    theta=pi/2:pi/158:pi;
    x=x0+r*cos(theta);
    y=y0+r*sin(theta);
    plot(x,y,'r','linewidth',2);

    plot([0.45,0.55],[0.5,0.5],'b','linewidth',2);

    axis('equal');
    xlabel('x(m)');
    ylabel('y(m)');
    grid on;
    
%     xlim([-1 6]);
%     ylim([-1 6]);
    
    hold off;

end

