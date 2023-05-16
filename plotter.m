%Function to plot the optimal path
function plotter(c1, c2, Pos, ts, tx, es, ef)

r = 5; %radius
for i = 1:(length(ts))

    %initial circle
    viscircles(c1(i, :), r, 'color', 'blue', 'linewidth', 1);
    hold on
    axis equal
    
    
    %Straight line
    plot([ts(i, 1) tx(i, 1)], [ts(i, 2) tx(i, 2)], 'g');
    
    %end circle
    viscircles(c2(i, :), r, 'color', 'red', 'linewidth', 1);
    title("Dubins path by Shanthinath Mallinathan");
    xlabel("X (m)");
    ylabel("Y (m)")
end


    