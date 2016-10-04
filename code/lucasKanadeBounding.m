function lucasKanadeBounding(indexMat)
    function startDragFcn(varargin)
        set (fig, 'WindowButtonMotionFcn', @mouseMove);
    end
    function mouseMove (object, eventdata)
        dragCoordinates = get (fig, 'CurrentPoint');
        dragCoordinates(2) = fig.Position(4) - dragCoordinates(2);
        dragCoordinates(1) = dragCoordinates(1);
    end

    function box = mergeBoxes(box1, box2)
        % Input: Two overlapping rectangles box1 and box2. Each one is a
        % vector of [x, y, width, height] format.
        % Output: The function returns a bounding box that
        % contains both rectangles box1 and box2 entirely by drawing a
        % rectangle from the top left-most corner to the bottom right-most
        % corner of their united shape.
        topLeftBox1 = [box1(1), box1(2)];
        topLeftBox2 = [box2(1), box2(2)];
        bottomRightBox1 = [box1(1) + box1(3), box1(2) + box1(4)];
        bottomRightBox2 = [box2(1) + box2(3), box2(2) + box2(4)];
        minX = min([topLeftBox1(1) topLeftBox2(1) bottomRightBox1(1) bottomRightBox2(1)]);
        minY = min([topLeftBox1(2) topLeftBox2(2) bottomRightBox1(2) bottomRightBox2(2)]);
        maxX = max([topLeftBox1(1) topLeftBox2(1) bottomRightBox1(1) bottomRightBox2(1)]);
        maxY = max([topLeftBox1(2) topLeftBox2(2) bottomRightBox1(2) bottomRightBox2(2)]);
        box = [minX, minY, maxX-minX, maxY-minY];
    end

    function boxes = mergeOverlappingBoxes(boxes)
        i1 = 1;
        while(i1 <= size(boxes,1))
            i2 = 1;
            while(i2 <= size(boxes,1))
                if(i1 ~= i2)
                    box1 = cell2mat(boxes(i1,:));
                    box2 = cell2mat(boxes(i2,:));
                    overlapRatio = bboxOverlapRatio(box1, box2);
                    if(overlapRatio > 0)
                        box = mergeBoxes(box1, box2);
                        % Array rescales after deletion, so make sure we delete
                        % the box with the higher index first.
                        if(i1 > i2)
                            boxes(i1,:) = [];
                            boxes(i2,:) = [];
                        else
                            boxes(i2,:) = [];
                            boxes(i1,:) = [];
                        end
                        boxes = [boxes; box];
                        i1 = 1;
                        i2 = 1;
                    end
                end
                i2 = i2+1;
            end
            i1 = i1+ 1;
        end
    end

    function displayTrackingResults(f, CC)
        
        % Remove vectors whose sizes are too small.
        for m = size(CC,1):-1:1
            if(sqrt(u(m)^2 + v(m)^2) < 1)
                CC(m, :) = [];
                u(m) = [];
                v(m) = [];
            end
        end
        
        % Calculate boxes for each of the optical flow vectors, store them
        % inside "boxes".
        boxes = {};
        for vec = size(CC,1):-1:1
            x = CC(vec,1);
            y = CC(vec,2);
            width = u(vec);
            height = v(vec);
            if(abs(width) > 0 || abs(height) > 0)
                if(width > 0 && height > 0)
                    box = [x y abs(width) abs(height)];
                elseif(width > 0 && height < 0)
                    box = [x y+height abs(width) abs(height)];
                elseif(width < 0 && height > 0)
                    box = [x+width y abs(width) abs(height)];
                else
                    box = [x+width y+height abs(width) abs(height)];
                end
                % Eliminate boxes that are disproportionately wide or tall
                if(box(4) > box(3))
                    if(box(4) / box(3) > 5)
                        continue;
                    end
                else
                    if(box(3) / box(4) > 5)
                        continue;
                    end
                end
                % Scale the dimensions of the box
                box(3) = box(3) * 60;
                box(4) = box(4) * 60;
                box(1) = box(1) - box(3) * 0.5;
                box(2) = box(2) - box(4) * 0.5;
                boxes = [boxes; box];
            end
        end
        
        boxes = mergeOverlappingBoxes(boxes);
        boxesBeforeAll = boxes;
        boxes = [allBoxes; boxes];
        allBoxes = boxes;
        imshow(f,'Border','tight');
        hold on;
        %quiver(CC(:,1), CC(:,2), u.*10,v.*10,'r','Autoscale','off');
        
        for bx = size(boxes,1):-1:1
            % Scale the dimensions of the box
            box = boxes{bx};
            box(3) = box(3) * 0.5;
            box(4) = box(4) * 0.5;
            box(1) = box(1) + box(3) * 0.5;
            box(2) = box(2) + box(4) * 0.5;
            xx = box(1) + box(3)/2;
            yy = box(2) + box(4)/2;
            rectangle('Position', [xx yy 3 3],'EdgeColor','g','FaceColor','g','LineWidth',3);
            %rectangle('Position', box,'EdgeColor','g','LineWidth',3);
        end
        for pt = size(clickPoints,1):-1:1
            point = cell2mat(clickPoints(pt));
            rectangle('Position', [point(1) point(2) 3 3],'EdgeColor','r','FaceColor','r','LineWidth',3);
        end
        boxCounts = [boxCounts size(boxes,1)];
        
        %click = ginput(1);
        click = dragCoordinates;
        closestBoxDistance = Inf;
        for bx = size(boxesBeforeAll,1):-1:1
            box = boxesBeforeAll{bx};
            xx = box(1) + box(3)/2;
            yy = box(2) + box(4)/2;
            d = pdist2([xx yy], [click(1) click(2)],'euclidean');
            if(d < closestBoxDistance)
                closestBoxDistance = d;
            end
        end
            disparities = [disparities closestBoxDistance];
        
        clickPoints = [clickPoints; click];
        
        hold off;
        pause(1/1000);
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
        obj.writer = vision.VideoFileWriter('tf_output.avi', 'FrameRate',obj.reader.info.VideoFrameRate);
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

videoFWriter = vision.VideoFileWriter('myFile.avi');

% Detect moving objects, and track them across video frames.
prevframe = readFrame();
boxCounts = {};
allBoxes = {};
clickPoints = {};
disparities = {};
fig = figure;
dragCoordinates = [0 0];
while ~isDone(obj.reader)
    frame = readFrame();
    
    im1 = im2double(rgb2gray(frame));
    im2 = im2double(rgb2gray(prevframe));
    
    
    %% Find corners
    % Define the window size for Lucas-Kanade method
    ww = 25;
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
    
    %% Implementing Lucas Kanade Method
    % for each point, calculate I_x, I_y, I_t
    Ix_m = conv2(im1,[-1 1; -1 1], 'valid'); % partial on x
    Iy_m = conv2(im1, [-1 -1; 1 1], 'valid'); % partial on y
    It_m = conv2(im1, ones(2), 'valid') + conv2(im2, -ones(2), 'valid'); % partial on t
    u = zeros(length(C),1);
    v = zeros(length(C),1);
    
    % within window ww * ww
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
mean(disparities)
figure;
plot(disparities);
save(sprintf('multiple_people%d.mat',indexMat),'disparities');
release(videoFWriter);

end