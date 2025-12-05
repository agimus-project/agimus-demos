import os
import sys
import xml

from xacro import filestack

try:  # python 2
    _basestr = basestring
    encoding = {"encoding": "utf-8"}
except NameError:  # python 3
    _basestr = str
    unicode = str
    encoding = {}


def process_xacro(*args):
    import xacro

    opts, input_file_name = xacro.process_args(args)

    # If AMENT_PREFIX_PATH is defined (ROS 2), pass the content to retrieve resources
    ament_prefix_path = os.getenv("AMENT_PREFIX_PATH")
    if ament_prefix_path is not None:
        dirs = list(map(lambda s: s + "/share", ament_prefix_path.split(":")))
    # open and process file
    doc = xacro.process_file(retrieve_resource(input_file_name, dirs), **vars(opts))

    # write output
    return doc.toprettyxml(indent="  ", **encoding)


def retrieve_resource(path, dirs=None, env_var="ROS_PACKAGE_PATH"):
    """
    Retrieve resource of the form "package://", resolving the package in the list of
    dirs.
    If the list of dirs is None, it is initialized with
    the content of the environnement variable env_var.
    """
    if path.startswith("package://"):
        relpath = path[len("package://") :]
        import os
        if dirs is None:
            dirs = os.environ[env_var].split(":")
        for dir in dirs:
            abspath = os.path.join(dir, relpath)
            if os.path.isfile(abspath):
                return abspath
        raise IOError(f'"{path}" not found. Searched in {dirs}')
    return path
