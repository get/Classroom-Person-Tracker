function lucasKanadeCentroids(indexMat)

    function startDragFcn(varargin)
        set (fig, 'WindowButtonMotionFcn', @mouseMove);
    end
    function mouseMove (object, eventdata)
        dragCoordinates = get (fig, 'CurrentPoint');
        dragCoordinates(2) = fig.Position(4) - dragCoordinates(2);
        dragCoordinates(1) = dragCoordinates(1);
    end

    function displayTrackingResults(f, CC)
        
        u = -1 * u;
        v = -1 * v;
        
        % Filter out small vectors
        for m = size(CC,1):-1:1
            if (sqrt(u(m)^2 + v(m)^2) < 1)
                CC(m, :) = [];
                u(m) = [];
                v(m) = [];
            end
        end
        
        % Track boxes to ignore
        ignoreList = zeros(size(CC, 1), 1);
        vectorWeights = ones(size(CC, 1), 1);
        frameCount = zeros(size(CC, 1), 1);
        vectorDistanceThreshold = 75;
        
        if (size(CC, 1) > 1)
            for i = 1:size(CC, 1)
                % Find points
                for j = i+1:size(CC, 1)
                    distanceIJ = sqrt(abs(CC(i,1) - CC(j,1))^2 + ...
                        abs(CC(i,2) - CC(j,2))^2);
                    % Can merge nearby vectors by local flow constancy assumption
                    % Use weighted average to find vector centroid
                    if (distanceIJ < vectorDistanceThreshold && ignoreList(j) == 0)
                        CC(i,1) = (CC(i,1)*vectorWeights(i) + ...
                            CC(j,1)*vectorWeights(j))/ ...
                            ((vectorWeights(i) + vectorWeights(j)));
                        CC(i,2) = (CC(i,2)*vectorWeights(i) + ...
                            CC(j,2)*vectorWeights(j))/ ...
                            ((vectorWeights(i) + vectorWeights(j)));
                        frameCount(i) = 1;
                        % Increment ith vector weight
                        vectorWeights(i) = vectorWeights(i) + 1;
                        % Ignore jth vector
                        ignoreList(j) = 1;
                    end
                end
            end
        end
        
        % Aggregate bounding box components
        boxes = [CC(:,1) CC(:,2) u v frameCount];
        % Filter out ignored vectors
        boxes = boxes(ignoreList(:, 1) == 0, :);
        
        imshow(f,'Border','tight');
        hold on;
        quiver(CC(:,1), CC(:,2), u, v, 1, 'r');
        for i = 1:size(boxes,1)
            % Shift rectangle if vector component is negative
            
            for bx = size(boxes,1):-1:1
                % Scale the dimensions of the box
                rectangle('Position', [boxes(i,1), boxes(i,2), 3, 3], ...
                    'EdgeColor', 'g', 'LineWidth', 3);
            end
            for pt = size(clickPoints,1):-1:1
                point = cell2mat(clickPoints(pt));
                rectangle('Position', [point(1) point(2) 3 3],'EdgeColor','r','FaceColor','r','LineWidth',3);
            end
            boxCounts = [boxCounts size(boxes,1)];
            F = getframe;
            %step(videoFWriter, F.cdata);
            
            %click = ginput(1);
            click = dragCoordinates;
            closestBoxDistance = Inf;
            for bx = size(boxes,1):-1:1
                box = boxes(bx,:);
                xx = box(1);
                yy = box(2);
                d = pdist2([xx yy], [click(1) click(2)],'euclidean');
                if(d < closestBoxDistance)
                    closestBoxDistance = d;
                end
            end
            disparities = [disparities closestBoxDistance];
            
            clickPoints = [clickPoints; click];
        end
        
        hold off;
        pause(1/1000);
        f = getframe;
    end
%% Create System Objects
% Create System objects used for reading the video frames, detecting
% foreground objects, and displaying results.

    function obj = setupSystemObjects()
        % Initialize Video I/O
        % Create objects for reading a video from a file, drawing the tracked
        % objects in each frame, and playing the video.
        
        % Create a video file reader.
        obj.reader = vision.VideoFileReader('multiple_people.m4v');
        obj.writer = vision.VideoFileWriter('dp_output.avi', 'FrameRate',obj.reader.info.VideoFrameRate);
        
        % Create two video players, one to display the video,
        % and one to display the foreground mask.
        obj.videoPlayer = vision.VideoPlayer('Position', [20, 400, 700, 400]);
        obj.maskPlayer = vision.VideoPlayer('Position', [740, 400, 700, 400]);
        
        % Create System objects for foreground detection and blob analysis
        
        % The foreground detector is used to segment moving objects from
        % the background. It outputs a binary mask, where the pixel value
        % of 1 corresponds to the foreground and the value of 0 corresponds
        % to the background.
    end

%% Read a Video Frame
% Read the next video frame from the video file.
    function frame = readFrame()
        frame = obj.reader.step();
    end

obj = setupSystemObjects();
boxCounts = {};
allBoxes = {};
clickPoints = {};
disparities = {};
fig = figure;
set(fig, 'Position', [440   378   560   420]);
dragCoordinates = [0 0];
% Detect moving objects, and track them across video frames.
prevframe = readFrame();
% boxDetections = zeros(1, 5);
while ~isDone(obj.reader)
    frame = readFrame();
    
    im1 = im2double(rgb2gray(frame));
    im2 = im2double(rgb2gray(prevframe));
    
    % Define the window size for Lucas-Kanade method
    ww = 5;
    w = round(ww/2);
    
    % Reduce the size of the image
    sc = 2;
    im2c = imresize(im2, 1/sc);
    C1 = corner(im2c);
    C1 = C1*sc;
    
    % Discard coners near the margin of the image
    k = 1;
    for i = 1:size(C1,1)
        x_i = C1(i, 2);
        y_i = C1(i, 1);
        if x_i-w>=1 && y_i-w>=1 && x_i+w<=size(im1,1)-1 && y_i+w<=size(im1,2)-1
            C(k,:) = C1(i,:);
            k = k+1;
        end
    end
    
    % Lucas Kanade Method
    % For each point, calculate I_x, I_y, I_t
    Ix_m = conv2(im1,[-1 1; -1 1], 'valid');
    Iy_m = conv2(im1, [-1 -1; 1 1], 'valid');
    It_m = conv2(im1, ones(2), 'valid') + conv2(im2, -ones(2), 'valid');
    u = zeros(length(C),1);
    v = zeros(length(C),1);
    
    % Within window ww * ww
    for k = 1:length(C(:,2))
        i = C(k,2);
        j = C(k,1);
        Ix = Ix_m(i-w:i+w, j-w:j+w);
        Iy = Iy_m(i-w:i+w, j-w:j+w);
        It = It_m(i-w:i+w, j-w:j+w);
        
        Ix = Ix(:);
        Iy = Iy(:);
        b = -It(:); % get b here
        
        A = [Ix Iy]; % get A here
        nu = pinv(A)*b;
        
        u(k)=nu(1);
        v(k)=nu(2);
    end;
    
    set(fig,'WindowButtonDownFcn',@startDragFcn);
    
    displayTrackingResults(prevframe, C);
    prevframe = frame;
end
disparities = cell2mat(disparities);
figure;
plot(disparities);
save(sprintf('multiple_people%d.mat',indexMat),'disparities');
end