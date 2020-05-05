import cv2 as cv
import numpy as np
import numbers


def remap_range(val, old_min, old_max, new_min, new_max, saturate=False):
    """
    Remaps a value from one range to another range.

    Args:
        val: (number) A number form the old range to be rescaled.
        old_min: (number) The inclusive 'lower' bound of the old range.
        old_max: (number) The inclusive 'upper' bound of the old range.
        new_min: (number) The inclusive 'lower' bound of the new range.
        new_max: (number) The inclusive 'upper' bound of the new range.
        saturate: (bool) If true, the new_min and new_max limits are enforced.

    Note:
        min need not be less than max; flipping the direction will cause the sign of
        the mapping to flip.  val does not have to be between old_min and old_max.

    Example:
        # a will be set to 25
        a = remap_range(5, 0, 10, 0, 50)

        # b will be set to 975
        b = remap_range(5, 0, 20, 1000, 900)

        # c will be set to 30
        c = remap_range(2, 0, 1, -10, 10)

        # d will be set to 20
        d = remap_range(2, 0, 1, -10, 10, True)
    """
    assert isinstance(val, numbers.Number), "val must be a number"
    assert isinstance(old_min, numbers.Number), "old_min must be a number"
    assert isinstance(old_max, numbers.Number), "old_max must be a number"
    assert isinstance(new_min, numbers.Number), "new_min must be a number"
    assert isinstance(new_max, numbers.Number), "new_max must be a number"

    old_span = old_max - old_min
    new_span = new_max - new_min
    new_val = new_min + new_span * (float(val - old_min) / float(old_span))

    # If saturate is true, enforce the new_min and new_max limits
    if saturate:
        new_val = max(new_min, min(new_max, new_val))

    return new_val


def crop(image, top_left_inclusive, bottom_right_exclusive):
    """
    Crops an image to a rectangle based on the specified pixel points.

    Args:
        image: (depth image or color image) The image to crop.
        top_left_inclusive: ((int, int)) The (row, column) of the top left pixel
            of the crop rectangle.
        bottom_right_exclusive: ((int, int)) The (row, column) of the pixel one
            past the bottom right corner of the crop rectangle.

    Returns:
        (depth image or color image) A cropped version of the image.

    Note:
        The top_left_inclusive pixel is included in the crop rectangle, but
        the bottom_right_exclusive pixel is not.  This is similar to how the how
        range(1, 4) returns [1, 2, 3] in Python.

    Example:
        image = rc.camera.get_image()

        # Crop the image to only keep the top half
        cropped_image = rc_utils.crop(
            image, (0, 0), (rc.camera.get_height() / 2, rc.camera.get_width())
        )
    """
    assert (len(image.shape) == 3 and image.shape[2] == 3) or (
        len(image.shape) == 2 and isinstance(image[0][0], numbers.Number)
    ), "image must be a color (2D array of pixels) or depth (2D array of depths) image"

    # Extract the minimum and maximum pixel rows and columns from the parameters
    r_min, c_min = top_left_inclusive
    r_max, c_max = bottom_right_exclusive

    # Shorten the array to the specified row and column ranges
    return image[r_min:r_max, c_min:c_max]


def find_contours(image, hsv_lower, hsv_upper):
    """
    Finds all contours of the specified color range in the provided image.

    Args:
        image: (2D numpy array of triples) The image in which to find contours,
            with pixels represented in the bgr (blue-green-red) format.
        hsv_lower: ((int, int, int)) The lower bound for the hue, saturation,
            and value of colors to contour.
        hsv_upper: ((int, int, int)) The upper bound for the hue, saturation,
            and value of the colors to contour.

    Returns:
        ([contours]) A list of contours around the specified color ranges
        found in the provided image.

    Note:
        Each channel in hsv_lower and hsv_upper ranges from 0 to 255.

    Example:
        # Define the lower and upper hsv ranges for the color blue
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)

        # Extract contours around all blue portions of the current image
        contours = rc_utils.find_contours(
            rc.camera.get_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
    """
    assert (
        len(image.shape) == 3 and image.shape[2] == 3
    ), "image must be a 2D numpy array of pixels, with each pixel stored as a triple"
    assert (
        len(hsv_lower) == 3 and len(filter(hsv_lower, lambda x: 0 <= x <= 255)) == 3
    ), "hsv_lower must be a triple of numbers ranging from 0 to 255 inclusive"
    assert (
        len(hsv_upper) == 3 and len(filter(hsv_upper, lambda x: 0 <= x <= 255)) == 3
    ), "hsv_upper must be a triple of numbers ranging from 0 to 255 inclusive"

    # Convert the image from a blue-green-red pixel representation to a
    # hue-saturation-value representation
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # Create a mask based on the pixels in the image with hsv values that
    # fall between HSV_lower and HSV_upper
    mask = cv.inRange(hsv_image, hsv_lower, hsv_upper)

    # Find and return a list of all contours of this mask
    return cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]


def get_largest_contour(contours, min_contour_size=30):
    """
    Finds the largest contour with size greater than min_contour_size.

    Args:
        contours: ([contour]) A list of contours found in an image.
        min_contour_size: (int) The smallest contour to consider

    Returns:
        (contour or None) The largest contour from the list, or None if no
        contour was larger than min_contour_size.

    Example:
        # Extract the blue contours
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )

        # Find the largest contour
        largest_contour = rc_utils.get_largest_contour(contours)
    """
    # Check that the list contains at least one contour
    if len(contours) == 0:
        return None

    # Find and return the largest contour if it is larger than MIN_CONTOUR_SIZE
    greatest_contour = max(contours, key=cv.contourArea)
    if cv.contourArea(greatest_contour) < min_contour_size:
        return None

    return greatest_contour


def draw_contour(image, contour, color=(0, 255, 0)):
    """
    Draws a contour on a copy of the provided image.

    Example:
        image = rc.camera.get_image()

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(image, BLUE_HSV_MIN, BLUE_HSV_MAX)
        largest_contour = rc_utils.get_largest_contour(contours)

        # Draw this contour onto image
        if (largest_contour is not None):
            image_labeled = draw_contour(image, largest_contour)

    Args:
        image: (2D numpy array of triples) The image on which to draw the contour.
        contour: (contour) A contour to draw on the image.
        color: ((int, int, int)) The color to draw the contour, specified as
            blue-green-red channels each ranging from 0 to 255.

    Returns:
        (2D numpy array of triples) A copy of image with the contour drawn on it.
    """
    assert (
        len(image.shape) == 3 and image.shape[2] == 3
    ), "image must be a 2D numpy array of pixels, with each pixel stored as a triple"
    assert (
        len(color) == 3 and len(filter(color, lambda x: 0 <= x <= 255)) == 3
    ), "color must be a triple of numbers ranging from 0 to 255 inclusive"

    return cv.drawContours(np.copy(image), [contour], 0, color, 3)


def get_center(contour):
    """
    Finds the center of a contour from an image.

    Example:
        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
        largest_contour = rc_utils.get_largest_contour(contours)

        # Find the center of this contour if it exists
        if (largest_contour is not None):
            center = rc_utils.get_center(largest_contour)

    Args:
        contour: (contour) The contour of which to find the center.

    Returns:
        ((int, int)) The (row, column) of the pixel at the center of the contour

    Note:
        Returns a None if the contour parameter is None or contains no pixels.
    """
    # Verify that we were passed a valid contour
    if contour is None:
        return None

    M = cv.moments(contour)

    # Check that the contour is not empty
    # (M["m00"] is the number of pixels in the contour)
    if M["m00"] <= 0:
        return None

    # Compute and return the center of mass of the contour
    return (M["m01"] / M["m00"], M["m10"] / M["m00"])


def get_area(contour):
    """
    Finds the area of a contour from an image.

    Args:
        contour: (contour) The contour of which to measure the area.

    Returns:
        (float) The number of pixels contained within the contour, or 0 if
        an invalid contour is provided

    Example:
        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
        largest_contour = rc_utils.get_largest_contour(contours)

        # Find the area of this contour (will evaluate to 0 if no contour was found)
        area = rc_utils.get_contour_area(contour)
    """
    # Verify that we were passed a valid contour
    if contour is None:
        return 0

    return cv.contourArea(contour)


def get_center_distance(depth_image, kernel_size=7):
    """
    Finds the distance of the center object in a depth image.

    Args:
        depth_image: (2D numpy array of depth values) The depth image to process.
        kernel_size: (int) The size of the area to average around the center.

    Returns:
        (float) The distance in millimeters of the object in the center of the image.

    Warning:
        kernel_size must be an odd integer.

    Note:
        The larger the kernel_size, the more that the center is averaged
        with the depth of the surrounding pixels.  This helps reduce noise but
        also reduces accuracy if the center object is not flat.

    Example:
        depth_image = rc.camera.get_depth_image()

        # Find the distance of the object (in mm) the center of depth_image
        center_distance = rc_utils.get_center_distance(depth_image)
    """
    assert len(depth_image.shape) == 2 and isinstance(
        depth_image[0][0], numbers.Number
    ), "depth_image must be a 2D numpy array of depth values (in mm)"
    assert (
        isinstance(kernel_size, numbers.Integral) and kernel_size % 2 == 1
    ), "kernel_size must be an odd integer"

    # Calculate the center pixel
    center_row = depth_image.shape[0] // 2
    center_col = depth_image.shape[1] // 2

    # Use get_average_distance to average the distance around this center pixel
    return get_average_distance(depth_image, center_row, center_col, kernel_size)


def get_average_distance(depth_image, pix_row, pix_col, kernel_size=7):
    """
    Finds the distance of a pixel averaged with its neighbors in a depth image.

    Args:
        depth_image: (2D numpy array of depth values) The depth image to process.
        pix_row: (int) The row of the pixel to measure.
        pix_col: (int) The column of the pixel to measure.
        kernel_size: (int) The size of the area to average around the pixel.

    Returns:
        (float) The distance in millimeters of the object at the provided pixel.

    Warning:
        kernel_size must be an odd integer.

    Note:
        The larger the kernel_size, the more that the requested pixel is averaged
        with the distances of the surrounding pixels.  This reduces noise at the cost of
        reduced accuracy.

    Example:
        depth_image = rc.camera.get_depth_image()

        # Find the distance of the object (in mm) at the pixel (100, 20) of depth_image
        average_distance = rc_utils.get_average_distance(depth_image, 100, 20)
    """
    assert len(depth_image.shape) == 2 and isinstance(
        depth_image[0][0], numbers.Number
    ), "depth_image must be a 2D numpy array of depth values (in mm)"
    assert (
        isinstance(pix_row, numbers.Integral) and 0 <= pix_row < depth_image.shape[0]
    ), "pix_row must be a row index within depth_image"
    assert (
        isinstance(pix_col, numbers.Integral) and 0 <= pix_col < depth_image.shape[1]
    ), "pix_col must be a column index within depth_image"
    assert (
        isinstance(kernel_size, numbers.Integral)
        and kernel_size > 0
        and kernel_size % 2 == 1
    ), "kernel_size must be a positive odd integer"

    # Crop out out a kernel around the requested pixel
    cropped_center = crop(
        depth_image,
        (pix_row - kernel_size // 2, pix_col - kernel_size // 2),
        (pix_row + kernel_size // 2, pix_col + kernel_size // 2),
    )

    # Apply a Gaussian blur to the cropped depth image to average the surrounding
    # pixel depths
    blurred_center = cv.GaussianBlur(cropped_center, (kernel_size, kernel_size), 0)

    # Return the depth of the center pixel
    return blurred_center[kernel_size // 2, kernel_size // 2]


def get_closest_pixel(depth_image, kernel_size=5):
    """
    Finds the closest pixel in a depth image.

    Args:
        depth_image: (2D numpy array of depth values) The depth image to process.
        kernel_size: (int) The size of the area to average around each pixel.

    Returns:
        ((int,int)) The (row, column) position of the pixel which is closest
        to the car.

    Warning:
        kernel_size must be an odd integer.

    Note:
        The larger the kernel_size, the more that the depth of each pixel is
        averaged with the depth of surrounding pixels.  This helps reduce noise
        but also reduces accuracy.

    Example:
        depth_image = rc.camera.get_depth_image()

        # Find the closest pixel
        closest_pixel = rc_utils.get_closest_pixel(depth_image)
    """
    assert len(depth_image.shape) == 2 and isinstance(
        depth_image[0][0], numbers.Number
    ), "depth_image must be a 2D numpy array of depth values (in mm)"
    assert (
        isinstance(kernel_size, numbers.Integral)
        and kernel_size > 0
        and kernel_size % 2 == 1
    ), "kernel_size must be a positive odd integer"

    # Apply a Gaussian blur to the depth portion of the image to reduce noise
    blurred_depth = cv.GaussianBlur(depth_image, (kernel_size, kernel_size), 0)

    # Find the pixel location of the minimum depth
    (_, _, minLoc, _) = cv.minMaxLoc(blurred_depth)

    return minLoc


def get_closest_point(scan):
    """
    Finds the closest point from a lidar scan.

    Args:
        scan: (tuple of float distance values) The current lidar scan.

    Returns:
        ((float, float)) The (angle, distance) of the location of the point which
        is closest to the car.

    Warning:
        Not counting 0.0 as a valid closest point.

    Note:
        In areas with glass, mirrors, or large open spaces, there is a high
        likelihood of distance error.

    Example:
        scan = rc.lidar.get_ranges()

        # Find the angle and distance of the closest point
        (angle, distance) = rc_utils.get_closest_point(scan)
    """

    # Remove 0.0 values from scan
    clean_scan = [elem for elem in scan if elem > 0.0]
    closest_point = min(clean_scan)

    position = scan.index(closest_point)

    # Convert the index of the list to degree
    degree = position // 2

    return (degree, closest_point)


def distance_forward(scan):
    """
    Finds the average distance to obstacles in front of the car.

    Args:
        scan: (tuple of float distance values) The current lidar scan.

    Returns:
        (float) The average (distance) of the points ahead of the car

    Warning:
        Not counting points that are 0.0.

    Example:
        scan = rc.lidar.get_ranges()

        #Find forward distance
        forward_distance = rc_utils.distance_forward()
    """

    forward_scan = scan[:20]
    forward_scan += scan[710:719]

    forward_clean_scan = [elem for elem in forward_scan if elem > 0.0]

    # If the forward distance is blocked
    if len(forward_clean_scan) == 0:
        return 0.0

    else:
        average_distance = sum(forward_clean_scan) / len(forward_clean_scan)
        return average_distance


def distance_cardinal_directions(scan):
    """
    Finds the average distance to obstacles in the four cardinal directions.

    Args:
        scan: (tuple of float distance values) The current lidar scan.

    Returns:
        ((float, float, float, float)): The (front, back, left, right) distances.

    Warning:
        Not counting points that are 0.0.

    Example:
        scan = rc.lidar.get_ranges()

        # Find cardinal direction distances
        (front, back, right, left) = rc_utils.distance_cardinal_direction()
    """
    average_front = distance_forward(scan)

    back_scan = scan[350:370]
    back_clean_scan = [elem for elem in back_scan if elem > 0.0]

    if len(back_clean_scan) == 0:
        average_back = 0.0

    else:
        average_back = sum(back_clean_scan) / len(back_clean_scan)

    left_scan = scan[170:190]
    left_clean_scan = [elem for elem in left_scan if elem > 0.0]

    if len(left_clean_scan) == 0:
        average_left = 0.0

    else:
        average_left = sum(left_clean_scan) / len(left_clean_scan)

    right_scan = scan[530:550]
    right_clean_scan = [elem for elem in right_scan if elem > 0.0]

    if len(right_clean_scan) == 0:
        average_right = 0.0

    else:
        average_right = sum(right_clean_scan) / len(right_clean_scan)

    return (average_front, average_back, average_right, average_left)


def color_depth_image(depth_image, min_depth=1, max_depth=5000):
    """
    Converts a depth image into a color image.

    Args:
        depth_image: (2D numpy array of depth values) The depth image to convert.
        min_depth: (float) The depth to represent as the reddest color.
        max_depth: (float) The depth to represent as the bluest color.

    Returns:
        (2D numpy array of triples) A color image of bgr pixels, in which the depth
        of each pixel has been mapped to the red-blue color range.

    Example:
        depth_image = rc.camera.get_depth_image()

        # Colorize and show the depth image
        colorized_image = rc_utils.color_depth_image(depth_image)
        rc.display.show_image(colorized_image)
    """
    assert len(depth_image.shape) == 2 and isinstance(
        depth_image[0][0], numbers.Number
    ), "depth_image must be a 2D numpy array of depth values (in mm)"
    assert (
        isinstance(min_depth, numbers.Number) and min_depth >= 0
    ), "min_depth must be a positive number"
    assert (
        isinstance(max_depth, numbers.Number) and max_depth >= 0
    ), "max_depth must be a positive number"

    # Use map to apply __convert_depth_to_color to each pixel
    return map(depth_image, lambda row: map(row, __convert_depth_to_color))


def __convert_depth_to_color(depth, min_depth, max_depth):
    """
    A helper function for show_depth_image which convents a depth into a bgr
    color on the red-blue range (red is closest, blue is farthest)
    """
    # Scale depth to the range (0, 255)
    scaled = max(0, min(255, (depth - min_depth) / (max_depth - min_depth)))

    # Map depth to a red-blue scale (red is closest, blue is farthest)
    return (255 - scaled, 0, scaled)
