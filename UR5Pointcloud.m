function UR5Pointcloud(ur5)
%UR5Pointcloud creates a point cloud showing the UR5 workspace area and volume 

% Defines how refined the point cloud is (smaller degrees = more detail)
stepRads = deg2rad(30);
qlim = ur5.model.qlim;

pointCloudeSize = prod(floor((qlim(1:6,2)-qlim(1:6,1))/stepRads + 1)); % prod is like sum but does multiplication (product) of all values
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic

% Creating a point cloud along the limits of all joints
for q1 = qlim(1,1):stepRads:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            for q4 = qlim(4,1):stepRads:qlim(4,2)
                for q5 = qlim(5,1):stepRads:qlim(5,2)
                     for q6 = qlim(6,1):stepRads:qlim(6,2)
                         % Don't need to worry about joint 7, just assume it=0. As moving it does not effect x,y,z position
                         q7 = 0;
                        q = [q1,q2,q3,q4,q5,q6,q7];
                        tr = ur5.model.fkine(q);                        
                        pointCloud(counter,:) = tr(1:3,4)';
                        counter = counter + 1; 
                        if mod(counter/pointCloudeSize * 100,1) == 0
                            % Displays time passes and percetage of point cloud loading completion
                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                        end
                     end
                end
            end
        end
    end
end

% Create a 3D model showing where the end effector can be over all these samples.  
plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

disp('UR5 point cloud complete');

% Calculation to display working radius of the UR3
xRadius = (max(pointCloud(:,1))-min(pointCloud(:,1)))/2;
yRadius = (max(pointCloud(:,2))-min(pointCloud(:,2)))/2;
zMax = (max(pointCloud(:,3))-ur5.model.base(3,4));
zMin = (min(pointCloud(:,3))-ur5.model.base(3,4));

disp(['UR5 reach radius: x = ',num2str(xRadius),' y = ',num2str(yRadius),' zMax = ',num2str(zMax),' zMin = ',num2str(zMin)]);