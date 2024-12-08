function img = getImage(filename)
% This function accepts the filename of an image and returns the image

% read an image in the workspace
img = imread(filename);

end 

function [features, valid_points] = getFeatures(img)
%{
    This function takes an image and finds the 200 strongest features
    using SIFT. Using those points, each points feature vector and location
    are extracted and returned.
%}

% detecting all SIFT features
points = detectSURFFeatures(rgb2gray(img));
% selecting >= 200 features from the points
points = points.selectStrongest(200);
% extracting the feature vectors and locations from strongest points
[features, valid_points] = extractFeatures(rgb2gray(img), points);
end

function [match1, match2] = getMatchingPoints(features1, points1, features2, points2)
%{
    This function takes the feature vectors and locations from the
    strongest SIFT features from two images and finds matching features 
    between the images. It returns two matrices corresponding to the 
    homogeneous coordinates of the matching points in a two vector. 
%}

% finding matching features between both images and storing their indexes
index_pairs = matchFeatures(features1, features2);

% extracting matching points
match1 = points1(index_pairs(:, 1), :);
match2 = points2(index_pairs(:, 2), :);

% making each point homogenous
match1 = [match1.Location, ones(size((match1.Location), 1), 1)];
match2 = [match2.Location, ones(size((match2.Location), 1), 1)];
end

function correspondences = constuctCorrespondenceMatrix(x, x_prime)
%{
    This function will construct the matrix of corresponding coordinates
    by calculating x' x Hx and dividing out H for >= 100 point 
    correspondences. 
%}

% initializing correspondence matrix
correspondences = single.empty;

% looping through point correspondences
for i = 1:size(x, 1)
    % calculating 3x9 submatrix for each point and concatenating points
    correspondences = [correspondences; zeros(1, 3), -x_prime(i, 3)*x(i, :), x_prime(i, 2)*x(i, :)
                        x_prime(i, 3)*x(i, :), zeros(1, 3), -x_prime(i, 1)*x(i, :)];
                        %-x_prime(i, 2)*x(i, :), x_prime(i, 1)*x(i, :), zeros(1, 3)]; 
end

end

function homography = calculateHomography(correspondence_matrix)
%{
    This function takes in a correspondence matrix representing the A
    matrix in the equation Ah = 0. To find the nullspace of A, SVD will be 
    performed on matrix A and the last column of the V matrix is the
    solution and will be returned. 
%}

% performing SVD on A matrix
[U, D, V] = svd(correspondence_matrix);
% extracting last column of V matrix 
% constructing 3x3 matrix
homography = [V(1:3, 9)'; V(4:6, 9)'; V(7:9, 9)'];
end

function homography = estimateHomography(match1, match2)
%{
    This function estimates the best homography for a set of matching
    points by iteratively finding inliers between the two matching point
    sets and extracting the coordinates for the inliers. Inliers are
    extracted until there are only 4 inliers left, or the number of inliers
    doesn't change between iterations. Once complete, the set of points is
    used to calculate the homography at infinity between the images which is returned. 
%}

num_points = size(match1, 1); % current total number of inliers
prev_points = -1; % previous number of inliers

% select inliers until the amount of inliers is 4 or the number of inliers
% doesn't change
while (num_points > 4) && (prev_points ~= num_points)
    % finding inliers
    [~, inlier_idx] = estgeotform2d(match1(:, 1:2), match2(:, 1:2), "projective");
    % extracting the indexes of inliers
    indexes = find(inlier_idx == 1);
    % extracting the coordinates of the inliers
    match1 = match1(indexes, : );
    match2 = match2(indexes, : );
    % adjusting stop critera variables
    prev_points = num_points;
    num_points = size(match1, 1);
end

% constructing homography from inliers
correspondence = constuctCorrespondenceMatrix(match1, match2);
homography = calculateHomography(correspondence);
end

% initializing matrix for image sizes 
image_sizes = zeros(5, 2);

% importing images and saving their sizes 
% *** CHANGE IMAGE PATHS FOR DIFFERENT DATASETS ***
img1 = getImage('Assignment3/mov2/mov2b_7.jpg');
image_sizes(1, :) = size(rgb2gray(img1));
img2 = getImage('Assignment3/mov2/mov2b_11.jpg');
image_sizes(2, :) = size(rgb2gray(img2));
img3 = getImage('Assignment3/mov2/mov2b_15.jpg');
image_sizes(3, :) = size(rgb2gray(img3));
img4 = getImage('Assignment3/mov2/mov2b_19.jpg');
image_sizes(4, :) = size(rgb2gray(img4));
img5 = getImage('Assignment3/mov2/mov2b_26.jpg');
image_sizes(5, :) = size(rgb2gray(img5));

% getting image features
[features1, points1] = getFeatures(img1);
[features2, points2] = getFeatures(img2);
[features3, points3] = getFeatures(img3); % center image
[features4, points4] = getFeatures(img4);
[features5, points5] = getFeatures(img5);

% initializing transformation matrix for all homographies
transformations = zeros(3, 3, 5);   % 5 identity matrices
transformations(:, :, 3) = eye(3);  % center transformation

% calculating homographies using DLT and RANSAC
% image 1 to image 2
[match1, match2] = getMatchingPoints(features1, points1, features2, points2);
transformations(:, :, 1) = estimateHomography(match1, match2);

% imgae 2 to image 3
[match2, match3] = getMatchingPoints(features2, points2, features3, points3);
transformations(:, :, 2) = estimateHomography(match2, match3);
% multiplying first homography with the second to adjust for the second transformation
transformations(:, :, 1) = transformations(:, :, 2)*transformations(:,:, 1);

% image 4 to image 3
[match3, match4] = getMatchingPoints(features3, points3, features4, points4);
transformations(:, :, 4) = estimateHomography(match4, match3);

% imgage 5 to image 4
[match4, match5] = getMatchingPoints(features4, points4, features5, points5);
transformations(:, :, 5) = estimateHomography(match5, match4);
% multiplying fifth homography with the fourth to adjust for the fourth transformation
transformations(:, :, 5) = transformations(:, :, 4)*transformations(:,:, 5);

% calculating panorama dimensions
T(5) = projtform2d;
for i = 1:5
    T(i) = projtform2d(transformations(:, :, i));
    [x_limits_out(i, :), y_limits_out(i, :)] = outputLimits(T(i), [1 image_sizes(i, 2)], [1 image_sizes(i, 1)]);
end

% calculating max and min size of images
max_size = max(image_sizes);
min_size = min(image_sizes);

% calculating max and min x and y values of all images
x_min = min(x_limits_out(:));
y_min = min(y_limits_out(:));
x_max = max([max_size(2); x_limits_out(:)]);
y_max = max([max_size(1); y_limits_out(:)]);

% calculating dimensions of panoramic
width = round(x_max - x_min);
height = round(y_max - y_min);
x_limits = [x_min, x_max];
y_limits = [y_min, y_max];

% initializing panorama and image pane
panoramic = zeros([height, width, 3], "like", img1);
outim = imref2d([height, width], x_limits, y_limits);

% creating panorama
% image 1
outim1 = imwarp(img1,T(1),OutputView=outim);                 
mask = imwarp(true(size(img1,1), size(img1,2)), T(1), OutputView=outim);
panoramic = imblend(outim1,panoramic, mask, foregroundopacity=1);

% imgae 2
outim2 = imwarp(img2,T(2),OutputView=outim);                 
mask = imwarp(true(size(img2,1),size(img2,2)),T(2),OutputView=outim);
panoramic = imblend(outim2,panoramic, mask, foregroundopacity=1);

% imgae 3
outim3 = imwarp(img3,T(3),OutputView=outim);                 
mask = imwarp(true(size(img3,1),size(img3,2)),T(3),OutputView=outim);
panoramic = imblend(outim3,panoramic, mask, foregroundopacity=1);

% imgae 4
outim4 = imwarp(img4,T(4),OutputView=outim);                 
mask = imwarp(true(size(img4,1),size(img4,2)),T(4),OutputView=outim);
panoramic = imblend(outim4,panoramic, mask, foregroundopacity=1);

% imgage 5
outim5 = imwarp(img5,T(5),OutputView=outim);                 
mask = imwarp(true(size(img5,1),size(img5,2)),T(5),OutputView=outim);
panoramic = imblend(outim5,panoramic, mask, foregroundopacity=1);

% displaying panorama
imshow(panoramic)

