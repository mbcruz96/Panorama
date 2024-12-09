# Panorama
- Implementation of creating image mosaics of sequences of images obtained such that the camera projection center does not change between images and only rotation occurs
- Feature point correspondences are found for each image in the sequence using SIFT and the strong 200 points are selected
- Matching image points are found using the feature vectors of each image and the image directly before or after the image in the sequence (depending on the position of the image in the sequence in reference to the center image)
- A modified version of RANSAC is used to find inliers in each set of matching image points to reduce the number of points in efforts not to have an over-determined solution
- A point correspondence matrix is constructed from the inlier matching points of each pair adjacent images
- For each pair of adjacent images in the sequence, the infinite homography between the two images is computed by finding the nullspace of the point correspondence matrix
- Each image is warped using the infinite homography and then stitched into a single panoramic image
- Results can be found in the experiment report of the results folder
