import cv2 as cv
import numpy as np
from typing import *
from nptyping import NDArray
from enum import Enum


class ColorBGR(Enum):
    """
    Common colors defined in the blue-green-red (BGR) format, with each channel
    ranging from 0 to 255 inclusive.
    """

    blue = (255, 0, 0)
    light_blue = (255, 255, 0)
    green = (0, 255, 0)
    dark_green = (0, 127, 0)
    yellow = (0, 255, 255)
    orange = (0, 127, 255)
    red = (0, 0, 255)
    pink = (255, 0, 255)
    purple = (255, 0, 127)
    black = (0, 0, 0)
    dark_gray = (63, 63, 63)
    gray = (127, 127, 127)
    light_gray = (191, 191, 191)
    white = (255, 255, 255)
    brown = (0, 63, 127)


def clamp(min: float, max: float, value: float) -> float:
    """
    Clamps a value between a minimum and maximum value.

    Args:
        min: The minimum allowed value.
        max: The maximum allowed value.
        value: The input to clamp.

    Returns:
        The value saturated between min and max.

    Example:
        # a will be set to 3
        a = rc_utils.clamp(0, 10, 3)

        # b will be set to 0
        b = rc_utils.remap_range(0, 10, -2)

        # c will be set to 10
        c = rc_utils.remap_range(0, 10, 11)
    """
    return min if value < min else max if value > max else value


def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
    saturate: bool = False,
) -> float:
    """
    Remaps a value from one range to another range.

    Args:
        val: A number form the old range to be rescaled.
        old_min: The inclusive 'lower' bound of the old range.
        old_max: The inclusive 'upper' bound of the old range.
        new_min: The inclusive 'lower' bound of the new range.
        new_max: The inclusive 'upper' bound of the new range.
        saturate: If true, the new_min and new_max limits are enforced.

    Note:
        min need not be less than max; flipping the direction will cause the sign of
        the mapping to flip.  val does not have to be between old_min and old_max.

    Example:
        # a will be set to 25
        a = rc_utils.remap_range(5, 0, 10, 0, 50)

        # b will be set to 975
        b = rc_utils.remap_range(5, 0, 20, 1000, 900)

        # c will be set to 30
        c = rc_utils.remap_range(2, 0, 1, -10, 10)

        # d will be set to 10
        d = rc_utils.remap_range(2, 0, 1, -10, 10, True)
    """
    old_span: float = old_max - old_min
    new_span: float = new_max - new_min
    new_val: float = new_min + new_span * (float(val - old_min) / float(old_span))

    # If saturate is true, enforce the new_min and new_max limits
    if saturate:
        return clamp(new_min, new_max, new_val)

    return new_val


def crop(
    image: NDArray[(Any, ...), Any],
    top_left_inclusive: Tuple[float, float],
    bottom_right_exclusive: Tuple[float, float],
):
    """
    Crops an image to a rectangle based on the specified pixel points.

    Args:
        image: The color or depth image to crop.
        top_left_inclusive: The (row, column) of the top left pixel
            of the crop rectangle.
        bottom_right_exclusive: The (row, column) of the pixel one
            past the bottom right corner of the crop rectangle.

    Returns:
        (depth image or color image) A cropped version of the image.

    Note:
        The top_left_inclusive pixel is included in the crop rectangle, but the
        bottom_right_exclusive pixel is not.
        If bottom_right_exclusive exceeds the bottom or right edge of the image, the
        full image is included along that axis.

    Example:
        image = rc.camera.get_color_image()

        # Crop the image to only keep the top half
        cropped_image = rc_utils.crop(
            image, (0, 0), (rc.camera.get_height() // 2, rc.camera.get_width())
        )
    """
    assert (
        0 <= top_left_inclusive[0] < image.shape[0]
    ), "top_left_inclusive[0] ({}) must be a pixel row index in color_image.".format(
        top_left_inclusive[0]
    )
    assert (
        0 <= top_left_inclusive[1] < image.shape[1]
    ), "top_left_inclusive[1] ({}) must be a pixel column index in color_image.".format(
        top_left_inclusive[1]
    )
    assert (
        bottom_right_exclusive[0] > 0 and bottom_right_exclusive[1] > 0
    ), "The row and column in bottom_right_exclusive ({}) must be positive.".format(
        bottom_right_exclusive
    )

    # Extract the minimum and maximum pixel rows and columns from the parameters
    r_min, c_min = top_left_inclusive
    r_max, c_max = bottom_right_exclusive

    # Shorten the array to the specified row and column ranges
    return image[r_min:r_max, c_min:c_max]


def find_contours(
    color_image: NDArray[(Any, Any, 3), np.uint8],
    hsv_lower: Tuple[int, int, int],
    hsv_upper: Tuple[int, int, int],
) -> List[NDArray]:
    """
    Finds all contours of the specified color range in the provided image.

    Args:
        color_image: The color image in which to find contours,
            with pixels represented in the bgr (blue-green-red) format.
        hsv_lower: The lower bound for the hue, saturation, and value of colors
            to contour.
        hsv_upper: The upper bound for the hue, saturation, and value of the colors
            to contour.

    Returns:
        A list of contours around the specified color ranges found in color_image.

    Note:
        Each channel in hsv_lower and hsv_upper ranges from 0 to 255.

    Example:
        # Define the lower and upper hsv ranges for the color blue
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)

        # Extract contours around all blue portions of the current image
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
    """
    assert (
        0 <= hsv_lower[0] <= 179 and 0 <= hsv_upper[0] <= 179
    ), "The hue of hsv_lower ({}) and hsv_upper ({}) must be in the range 0 to 179 inclusive.".format(
        hsv_lower, hsv_upper
    )
    assert (
        0 <= hsv_lower[1] <= 255 and 0 <= hsv_upper[1] <= 255
    ), "The saturation of hsv_lower ({}) and hsv_upper ({}) must be in the range 0 to 255 inclusive.".format(
        hsv_lower, hsv_upper
    )
    assert (
        0 <= hsv_lower[0] <= 255 and 0 <= hsv_upper[0] <= 255
    ), "The value of hsv_lower ({}) and hsv_upper ({}) must be in the range 0 to 255 inclusive.".format(
        hsv_lower, hsv_upper
    )
    assert (
        hsv_lower[1] <= hsv_upper[1]
    ), "The saturation channel of hsv_lower ({}) must be less than that of hsv_upper ({}).".format(
        hsv_lower, hsv_upper
    )
    assert (
        hsv_lower[2] <= hsv_upper[2]
    ), "The value channel of hsv_lower ({}) must be less than that of of hsv_upper ({}).".format(
        hsv_lower, hsv_upper
    )

    # Convert the image from a blue-green-red pixel representation to a
    # hue-saturation-value representation
    hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

    # Create a mask containing the pixels in the image with hsv values between
    # hsv_lower and hsv_upper.
    mask: NDArray
    if hsv_lower[0] <= hsv_upper[0]:
        mask = cv.inRange(hsv_image, hsv_lower, hsv_upper)

    # If the color range passes the 255-0 boundary, we must create two masks
    # and merge them
    else:
        mask1 = cv.inRange(hsv_image, hsv_lower, (255, hsv_upper[1], hsv_upper[2]))
        mask2 = cv.inRange(hsv_image, (0, hsv_lower[1], hsv_lower[2]), hsv_upper)
        mask = cv.bitwise_or(mask1, mask2)

    # Find and return a list of all contours of this mask
    return cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]


def get_largest_contour(
    contours: List[NDArray], min_contour_size: int = 30
) -> Optional[NDArray]:
    """
    Finds the largest contour with size greater than min_contour_size.

    Args:
        contours: A list of contours found in an image.
        min_contour_size: The smallest contour to consider (in number of pixels)

    Returns:
        The largest contour from the list, or None if no
        contour was larger than min_contour_size.

    Example:
        # Extract the blue contours
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
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


def draw_contour(
    color_image: NDArray[(Any, Any, 3), np.uint8],
    contour: NDArray,
    color: Tuple[int, int, int] = ColorBGR.green.value,
) -> None:
    """
    Draws a contour on the provided image.

    Example:
        image = rc.camera.get_color_image()

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(image, BLUE_HSV_MIN, BLUE_HSV_MAX)
        largest_contour = rc_utils.get_largest_contour(contours)

        # Draw this contour onto image
        if (largest_contour is not None):
            draw_contour(image, largest_contour)

    Args:
        color_image: The color image on which to draw the contour.
        contour: The contour to draw on the image.
        color: The color to draw the contour, specified as
            blue-green-red channels each ranging from 0 to 255 inclusive.
    """
    for channel in color:
        assert (
            0 <= channel <= 255
        ), "Each channel in color ({}) must be in the range 0 to 255 inclusive.".format(
            color
        )

    cv.drawContours(color_image, [contour], 0, color, 3)


def draw_circle(
    color_image: NDArray[(Any, Any, 3), np.uint8],
    center: Tuple[int, int],
    color: Tuple[int, int, int] = ColorBGR.yellow.value,
    radius: int = 6,
) -> None:
    """
    Draws a circle on the provided image.

    Example:
        image = rc.camera.get_color_image()

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(image, BLUE_HSV_MIN, BLUE_HSV_MAX)
        largest_contour = rc_utils.get_largest_contour(contours)

        # Draw a dot at the center of this contour in red
        if (largest_contour is not None):
            center = get_contour_center(contour)
            draw_circle(image, center, rc_utils.ColorBGR.red.value)

    Args:
        color_image: The color image on which to draw the contour.
        center: The pixel (row, column) of the center of the image.
        color: The color to draw the circle, specified as
            blue-green-red channels each ranging from 0 to 255 inclusive.
        radius: The radius of the circle in pixels
    """
    for channel in color:
        assert (
            0 <= channel <= 255
        ), "Each channel in color ({}) must be in the range 0 to 255 inclusive.".format(
            color
        )
    assert (
        0 <= center[0] < color_image.shape[0]
    ), "center[0] ({}) must be a pixel row index in color_image.".format(center[0])
    assert (
        0 <= center[1] < color_image.shape[1]
    ), "center[1] ({}) must be a pixel column index in color_image.".format(center[1])
    assert radius > 0, "radius ({}) must be a positive integer.".format(radius)

    # cv.circle expects the center in (column, row) format
    cv.circle(color_image, (center[1], center[0]), radius, color, -1)


def get_contour_center(contour: NDArray) -> Optional[Tuple[int, int]]:
    """
    Finds the center of a contour from an image.

    Example:
        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
        largest_contour = rc_utils.get_largest_contour(contours)

        # Find the center of this contour if it exists
        if (largest_contour is not None):
            center = rc_utils.get_contour_center(largest_contour)

    Args:
        contour: The contour of which to find the center.

    Returns:
        The (row, column) of the pixel at the center of the contour.

    Note:
        Returns None if the contour contains no pixels.
    """
    M = cv.moments(contour)

    # Check that the contour is not empty
    # (M["m00"] is the number of pixels in the contour)
    if M["m00"] <= 0:
        return None

    # Compute and return the center of mass of the contour
    center_row = round(M["m01"] / M["m00"])
    center_column = round(M["m10"] / M["m00"])
    return (center_row, center_column)


def get_contour_area(contour: NDArray) -> float:
    """
    Finds the area of a contour from an image.

    Args:
        contour: The contour of which to measure the area.

    Returns:
        The number of pixels contained within the contour

    Example:
        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
        largest_contour = rc_utils.get_largest_contour(contours)

        # Find the area of this contour (will evaluate to 0 if no contour was found)
        area = rc_utils.get_contour_area(contour)
    """
    return cv.contourArea(contour)


def get_depth_image_center_distance(
    depth_image: NDArray[(Any, Any), np.float32], kernel_size: int = 5
) -> float:
    """
    Finds the distance of the center object in a depth image.

    Args:
        depth_image: The depth image to process.
        kernel_size: The size of the area to average around the center.

    Returns:
        The distance in cm of the object in the center of the image.

    Warning:
        kernel_size must be positive and odd.

    Note:
        The larger the kernel_size, the more that the center is averaged
        with the depth of the surrounding pixels.  This helps reduce noise at the cost
        of reduced accuracy.  If kernel_size = 1, no averaging is done.

    Example:
        depth_image = rc.camera.get_depth_image()

        # Find the distance of the object (in cm) the center of depth_image
        center_distance = rc_utils.get_depth_image_center_distance(depth_image)
    """
    assert (
        kernel_size > 0 and kernel_size % 2 == 1
    ), "kernel_size ({}) must positive and odd.".format(kernel_size)

    # Calculate the center pixel
    center_coords = (depth_image.shape[0] // 2, depth_image.shape[1] // 2)

    # Use get_average_distance to average the distance around this center pixel
    return get_pixel_average_distance(depth_image, center_coords, kernel_size)


def get_pixel_average_distance(
    depth_image: NDArray[(Any, Any), np.float32],
    pix_coord: Tuple[int, int],
    kernel_size: int = 5,
) -> float:
    """
    Finds the distance of a pixel averaged with its neighbors in a depth image.

    Args:
        depth_image: The depth image to process.
        pix_coord: The (row, column) of the pixel to measure.
        kernel_size: The size of the area to average around the pixel.

    Returns:
        The distance in cm of the object at the provided pixel.

    Warning:
        kernel_size must be positive and odd.

    Note:
        The larger the kernel_size, the more that the requested pixel is averaged
        with the distances of the surrounding pixels.  This helps reduce noise at the
        cost of reduced accuracy.

    Example:
        depth_image = rc.camera.get_depth_image()

        # Find the distance of the object (in cm) at the pixel (100, 20) of depth_image
        average_distance = rc_utils.get_average_distance(depth_image, 100, 20)
    """
    (pix_row, pix_col) = pix_coord
    assert (
        0 <= pix_row < depth_image.shape[0]
    ), "pix_coord[0] ({}) must be a pixel row index within depth_image.".format(
        pix_coord[0]
    )
    assert (
        0 <= pix_col < depth_image.shape[1]
    ), "pix_coord[1] ({}) must be a pixel column index within depth_image.".format(
        pix_coord[1]
    )
    assert (
        kernel_size > 0 and kernel_size % 2 == 1
    ), "kernel_size ({}) must positive and odd.".format(kernel_size)

    # Crop out out a kernel around the requested pixel
    cropped_center = crop(
        depth_image,
        (pix_row - kernel_size // 2, pix_col - kernel_size // 2),
        (pix_row + kernel_size // 2 + 1, pix_col + kernel_size // 2 + 1),
    )

    # Apply a Gaussian blur to the cropped depth image to average the surrounding
    # pixel depths
    blurred_center = cv.GaussianBlur(cropped_center, (kernel_size, kernel_size), 0)

    # Return the depth of the center pixel
    return blurred_center[kernel_size // 2, kernel_size // 2]


def get_closest_pixel(
    depth_image: NDArray[(Any, Any), np.float32], kernel_size: int = 5
) -> Tuple[int, int]:
    """
    Finds the closest pixel in a depth image.

    Args:
        depth_image: The depth image to process.
        kernel_size: The size of the area to average around each pixel.

    Returns:
        The (row, column) of the pixel which is closest to the car.

    Warning:
        kernel_size be positive and odd.
        It is highly recommended that you crop off the bottom of the image, or else
        this function will likely return the ground directly in front of the car.

    Note:
        The larger the kernel_size, the more that the depth of each pixel is averaged
        with the distances of the surrounding pixels.  This helps reduce noise at the
        cost of reduced accuracy.

    Example:
        depth_image = rc.camera.get_depth_image()

        # Crop off the ground directly in front of the car
        cropped_image = rc_utils.crop(
            image, (0, 0), (int(rc.camera.get_height() * 0.66), rc.camera.get_width())
        )

        # Find the closest pixel
        closest_pixel = rc_utils.get_closest_pixel(depth_image)
    """
    assert (
        kernel_size > 0 and kernel_size % 2 == 1
    ), "kernel_size ({}) must positive and odd.".format(kernel_size)

    # Shift 0.0 values to 10,000 so they are not considered for the closest pixel
    depth_image = (depth_image - 1) % 10000

    # Apply a Gaussian blur to to reduce noise
    if kernel_size > 1:
        blurred_depth = cv.GaussianBlur(depth_image, (kernel_size, kernel_size), 0)

    # Find the pixel location of the minimum depth
    (_, _, minLoc, _) = cv.minMaxLoc(blurred_depth)

    # minLoc is formatted as (column, row), so we flip the order
    return (minLoc[1], minLoc[0])


def get_lidar_closest_point(scan: NDArray[Any, np.float32]) -> Tuple[float, float]:
    """
    Finds the closest point from a LIDAR scan.

    Args:
        scan: The samples from a LIDAR scan.

    Returns:
        The (angle, distance) of the point closest to the car. Angle is in degrees,
        starting at 0 directly in front of the car and increasing clockwise.
        Distance is in cm.

    Warning:
        In areas with glass, mirrors, or large open spaces, there is a high
        likelihood of distance error.

    Note:
        Ignores any samples with a value of 0.0 (no data).

    Example:
        scan = rc.lidar.get_samples()

        # Find the angle and distance of the closest point
        angle, distance = rc_utils.get_lidar_closest_point(scan)
    """
    # Find the minimum value in scan which is not 0.0 (no data)
    clean_scan = [elem for elem in scan if elem > 0.0]
    closest_distance = min(clean_scan)

    # Use the index of this value to find its angle
    index = np.where(scan == closest_distance)[0][0]
    degree = index * 360 / (scan.shape[0])

    return (degree, closest_distance)


def get_lidar_average_distance(
    scan: NDArray[Any, np.float32], angle: float, window_angle: float = 4
) -> float:
    """
    Finds the average distance to obstacles in front of the car.

    Args:
        scan: The samples from a LIDAR scan
        angle: The angle (in degrees) at which to measure distance, starting at 0
            directly in front of the car and increasing clockwise.
        window_angle: The number of degrees to consider around angle.

    Returns:
        The average distance of the points at angle in cm.

    Note:
        Ignores any samples with a value of 0.0 (no data).
        Increasing window_angle reduces noise at the cost of reduced accuracy.

    Example:
        scan = rc.lidar.get_samples()

        # Find the distance directly behind the car (6:00 position)
        back_distance = rc_utils.get_lidar_average_distance(scan, 180)

        # Find the distance to the forward and right of the car (1:30 position)
        forward_right_distance = rc_utils.get_lidar_average_distance(scan, 45)
    """
    assert 0 <= angle < 360, "angle ({}) must be in the range 0 to 360.".format(angle)
    assert (
        0 <= window_angle < 360
    ), "window_angle ({}) must be in the range 0 to 360, and reasonably should not exceed 20.".format(
        window_angle
    )

    # Calculate the indices at the edges of the requested window
    center_index: int = int(angle * scan.shape[0] / 360)
    num_side_samples: int = int(window_angle / 2 * scan.shape[0] / 360)
    left_index: int = (center_index - num_side_samples) % len(scan)
    right_index: int = (center_index + num_side_samples) % len(scan)

    # Select samples in the window, handling if we cross the edge of the array
    samples: List[float]
    if right_index < left_index:
        samples = scan[left_index:].tolist() + scan[0 : right_index + 1].tolist()
    else:
        samples = scan[left_index : right_index + 1].tolist()

    # Remove samples with no data (0.0)
    samples = [elem for elem in samples if elem > 0]

    # If no valid samples remain, return 0.0
    if len(samples) == 0:
        return 0.0

    return sum(samples) / len(samples)
