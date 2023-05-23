import cv2
import numpy as np

# Load image
image = cv2.imread('pista4.jpg')

# Apply a Gaussian Blur filter
blurred = cv2.GaussianBlur(image, (9, 9), 0)

# Apply adaptive threshold to obtain a binary image
gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 4)

# Apply Canny Edge Detector
edges = cv2.Canny(gray, 100, 100)

# #Show image
# cv2.imshow('Edges', edges)

# Create a black mask with the same dimensions as the edges image
mask = np.zeros_like(edges)

# Define the coordinates of the area of interest (example: a rectangular area)
x1, y1 = 600, 300  # Top-left corner
x2, y2 = 900, 900  # Bottom-right corner

# Set the area inside the specified coordinates as white in the mask
mask[y1:y2, x1:x2] = 255

# Apply the mask to the edge image
masked_edges = cv2.bitwise_and(edges, mask)

# Draw area of interest
rectangle = cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

# Show area of interest
# cv2.imshow('Area of interest', rectangle)

# Find the contours
contours, _ = cv2.findContours(masked_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find contours that are black lines
black_lines = []
widest_line_width = 0
highest_line_height = 0
widest_contours = []

#Find the widthest and longest contour
for contour in contours:
    x, y, w, h = cv2.boundingRect(contour)

    # Calculate line width and height
    line_width = w
    line_height = h

    # Check if the contour meets the criteria
    if line_width > widest_line_width and line_height > highest_line_height:
        widest_line_width = line_width
        highest_line_height = line_height

#Find the contours similar to the biggest contourn
for contour in contours:
    x, y, w, h = cv2.boundingRect(contour)

    # Calculate line width and height
    line_width = w
    line_height = h
    
    # Collect all contours that meet the criteria
    if line_width >= widest_line_width * 0.04 and line_height >= highest_line_height * 0.1:
        # print(str(line_width) + " " + str(line_height))
        widest_contours.append(contour)

# Draw all widest line contours
cv2.drawContours(image, widest_contours, -1, (0, 255, 0), 2)

# Show the image with the enclosed line
cv2.imshow('Enclosed Line', image)

cv2.waitKey(0)
cv2.destroyAllWindows()