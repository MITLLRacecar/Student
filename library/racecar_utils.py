import cv2 as cv
import numpy as np
from typing import *
from nptyping import NDArray


def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
    saturate: bool = False,
):
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
        a = remap_range(5, 0, 10, 0, 50)

        # b will be set to 975
        b = remap_range(5, 0, 20, 1000, 900)

        # c will be set to 30
        c = remap_range(2, 0, 1, -10, 10)

        # d will be set to 20
        d = remap_range(2, 0, 1, -10, 10, True)
    """
    old_span: float = old_max - old_min
    new_span: float = new_max - new_min
    new_val: float = new_min + new_span * (float(val - old_min) / float(old_span))

    # If saturate is true, enforce the new_min and new_max limits
    if saturate:
        return max(new_min, min(new_max, new_val))

    return new_val


def crop(
    image: NDArray[(Any, Any,), Any],
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
        The top_left_inclusive pixel is included in the crop rectangle, but
        the bottom_right_exclusive pixel is not.  This is similar to how the how
        range(1, 4) returns [1, 2, 3].

    Example:
        image = rc.camera.get_image()

        # Crop the image to only keep the top half
        cropped_image = rc_utils.crop(
            image, (0, 0), (rc.camera.get_height() // 2, rc.camera.get_width())
        )
    """
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
            rc.camera.get_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
    """
    assert (
        len(filter(hsv_lower, lambda x: 0 <= x <= 255)) == 3
    ), "Each channel in hsv_lower must be in the range 0 to 255 inclusive"
    assert (
        len(filter(hsv_upper, lambda x: 0 <= x <= 255)) == 3
    ), "Each channel in hsv_upper must be in the range 0 to 255 inclusive"

    # Convert the image from a blue-green-red pixel representation to a
    # hue-saturation-value representation
    hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

    # Create a mask based on the pixels in the image with hsv values that
    # fall between HSV_lower and HSV_upper
    mask = cv.inRange(hsv_image, hsv_lower, hsv_upper)

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


def draw_contour(
    color_image: NDArray[(Any, Any, 3), np.uint8],
    contour: NDArray,
    color: Tuple[int, int, int] = (0, 255, 0),
) -> NDArray[(Any, Any, 3), np.uint8]:
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
        color_image: The color image on which to draw the contour.
        contour: The contour to draw on the image.
        color: The color to draw the contour, specified as
            blue-green-red channels each ranging from 0 to 255 inclusive.

    Returns:
        A copy of image with the contour drawn on it.
    """
    assert (
        len(filter(color, lambda x: 0 <= x <= 255)) == 3
    ), "Each channel in color must be in the range 0 to 255 inclusive"

    return cv.drawContours(np.copy(color_image), [contour], 0, color, 3)


def get_contour_center(contour: Optional[NDArray]) -> Optional[Tuple[int, int]]:
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
        contour: The contour of which to find the center.

    Returns:
        The (row, column) of the pixel at the center of the contour.

    Note:
        Returns None if the contour parameter is None or contains no pixels.
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


def get_contour_area(contour: Optional[NDArray]) -> float:
    """
    Finds the area of a contour from an image.

    Args:
        contour: The contour of which to measure the area.

    Returns:
        The number of pixels contained within the contour, or 0 if
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


def get_depth_image_center_distance(
    depth_image: NDArray[(Any, Any), np.float32], kernel_size: int = 7
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
        of reduced accuracy.

    Example:
        depth_image = rc.camera.get_depth_image()

        # Find the distance of the object (in cm) the center of depth_image
        center_distance = rc_utils.get_center_distance(depth_image)
    """
    assert kernel_size > 0 and kernel_size % 2 == 1, "kernel_size must positive and odd"

    # Calculate the center pixel
    center_row = depth_image.shape[0] // 2
    center_col = depth_image.shape[1] // 2

    # Use get_average_distance to average the distance around this center pixel
    return get_pixel_average_distance(depth_image, center_row, center_col, kernel_size)


def get_pixel_average_distance(
    depth_image: NDArray[(Any, Any), np.float32],
    pix_row: int,
    pix_col: int,
    kernel_size: int = 7,
) -> float:
    """
    Finds the distance of a pixel averaged with its neighbors in a depth image.

    Args:
        depth_image: The depth image to process.
        pix_row: The row of the pixel to measure.
        pix_col: The column of the pixel to measure.
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
    assert (
        0 <= pix_row < depth_image.shape[0]
    ), "pix_row must be a row index within depth_image"
    assert (
        0 <= pix_col < depth_image.shape[1]
    ), "pix_col must be a column index within depth_image"
    assert kernel_size > 0 and kernel_size % 2 == 1, "kernel_size must positive and odd"

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
    assert kernel_size > 0 and kernel_size % 2 == 1, "kernel_size must positive and odd"

    # Apply a Gaussian blur to to reduce noise
    blurred_depth = cv.GaussianBlur(depth_image, (kernel_size, kernel_size), 0)

    # Find the pixel location of the minimum depth
    (_, _, minLoc, _) = cv.minMaxLoc(blurred_depth)

    return minLoc


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
    # Remove 0.0 values from scan
    clean_scan = [elem for elem in scan if elem > 0.0]

    closest_point = min(clean_scan)
    index = scan.index(closest_point)

    # Convert sample index to degree
    degree = index * 360 / (scan.shape[0])

    return (degree, closest_point)


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
    assert (
        window_angle < 360
    ), "window_angle cannot exceed 360, and reasonably should not exceed 20."

    # Calculate the indices at the edges of the requested window
    center_index: int = int(angle * scan.shape[0] / 360)
    num_side_samples: int = int(window_angle / 2 * scan.shape[0] / 360)
    left_index: int = (center_index - num_side_samples) % len(scan)
    right_index: int = (center_index + num_side_samples) % len(scan)

    # Select samples in the window, handling if we cross the edge of the array
    samples: List[float]
    if right_index < left_index:
        samples = scan[left_index:].tolist() + scan[0:right_index + 1].tolist()
    else:
        samples = scan[left_index:right_index + 1].tolist()

    # Remove samples with no data (0.0)
    samples = [elem for elem in samples if elem > 0]

    # If no valid samples remain, return 0.0
    if len(samples) == 0:
        return 0.0

    return sum(samples) / len(samples)
