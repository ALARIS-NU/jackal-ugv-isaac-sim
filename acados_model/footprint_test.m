footprint = [[-0.215, -0.2540]; [-0.215, 0.2540]; [0.215, 0.2540]; [0.215, -0.2540]];
scatter(footprint(:,1),footprint(:,2))
r_base = 0.28;
viscircles([0.0, -0.105;0.0, 0.105],[r_base,r_base]);
