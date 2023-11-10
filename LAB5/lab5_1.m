close all

% Path to your images.
yourImagesDir = 'images/wall'; % Replace with the path to your images
scene = imageDatastore(yourImagesDir);

% Display images to be stitched.
montage(scene.Files)

% Read the first image from the image set and resize it.
I = readimage(scene, 1);
scaleFactor = 0.2; % Change this factor as needed to fit in your memory
I = imresize(I, scaleFactor); % Resize image

% Initialize features for I(1)
grayImage = im2gray(I);
[y, x, m] = harris(grayImage, 1000, 'tile', [2 2], 'disp');
points = [x, y];
[features, points] = extractFeatures(grayImage, points);

% Initialize all the transforms to the identity matrix.
numImages = numel(scene.Files);
tforms(numImages) = projective2d(eye(3));

% Initialize variable to hold image sizes.
imageSize = zeros(numImages, 2);
imageSize(1, :) = size(grayImage); % Save the size of the first image

% Iterate over remaining image pairs
for n = 2:numImages
    
    % Store points and features for I(n-1).
    pointsPrevious = points;
    featuresPrevious = features;
        
    % Read I(n), convert to grayscale, and resize.
    I = readimage(scene, n);
    I = imresize(I, scaleFactor);
    grayImage = im2gray(I);    
    
    % Save image size.
    imageSize(n, :) = size(grayImage);
    
    % Detect and extract Harris features.
    [y, x, m] = harris(grayImage, 1000, 'tile', [2 2], 'disp');
    points = [x, y];
    [features, points] = extractFeatures(grayImage, points);
  
    % Find correspondences between I(n) and I(n-1).
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true);
       
    matchedPoints = points(indexPairs(:, 1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:, 2), :);        
    
    % Estimate the transformation between I(n) and I(n-1).
    tforms(n) = estimateGeometricTransform2D(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);
    
    % Compute T(n) * T(n-1) * ... * T(1)
    tforms(n).T = tforms(n).T * tforms(n-1).T; 
end

% Compute the output limits for each transform and find the center image.
for i = 1:numel(tforms)           
    [xlim(i, :), ylim(i, :)] = outputLimits(tforms(i), [1 imageSize(i, 2)], [1 imageSize(i, 1)]);    
end

avgXLim = mean(xlim, 2);
[~, idx] = sort(avgXLim);
centerIdx = floor((numel(tforms) + 1) / 2);
centerImageIdx = idx(centerIdx);

% Adjust each transform to center on the center image.
Tinv = invert(tforms(centerImageIdx));
for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end

% Recompute the output limits after centering.
for i = 1:numel(tforms)           
    [xlim(i, :), ylim(i, :)] = outputLimits(tforms(i), [1 imageSize(i, 2)], [1 imageSize(i, 1)]);
end

% Find the minimum and maximum output limits.
maxImageSize = max(imageSize);
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);
yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width 3], 'like', I);

% Create a blender and spatial reference object for the panorama.
blender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');  
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Create the panorama.
for i = 1:numImages
    
    % Read and resize the image.
    I = readimage(scene, i);   
    I = imresize(I, scaleFactor);
   
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask and overlay the warpedImage onto the panorama.
    mask = imwarp(true(size(I, 1), size(I, 2)), tforms(i), 'OutputView', panoramaView);
    panorama = step(blender, panorama, warpedImage, mask);
end

% Display the panorama.
figure
imshow(panorama)
