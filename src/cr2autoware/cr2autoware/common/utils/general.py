import pathlib

# ROS imports
from ament_index_python import get_package_share_directory  # type: ignore
import rclpy.logging as ros_logging

# get logger
logger = ros_logging.get_logger(__name__)


def get_absolute_path_from_package(
    relative_path: str, package_name: str = "cr2autoware"
) -> pathlib.Path:
    """Find the absolute path of the configuration file.

    :param relative_path: path to the config file; may be an absolute path
        or a path relative **to the packages src directory**.
        (e.g. "autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/cr2autoware" for the cr2autoware node)
    :param package_name: the package name to which paths are relative to
    :return: absolute path to the config file
    """
    if pathlib.Path(relative_path).is_absolute():
        logger.debug(f"Found absolute path to config file: {relative_path}")
        return pathlib.Path(relative_path)

    base_path = get_package_share_directory(package_name)
    logger.debug(
        f"Found relative path to config file: {relative_path}. "
        f"Joining with base path: {base_path}"
    )
    return pathlib.Path(base_path).joinpath(relative_path)