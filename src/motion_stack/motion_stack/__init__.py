"""Python core of the motion stack.

There is no ROS2 code in here, only python "nodes". Those the are skeltons to be run by any runtime, either python, ROS2 or else.

This allows for:

.. hlist::
    :columns: 1

    - untangling from ROS2
    - testing through pytest
    - multiple ROS versions and runtime

Authors: 
    .. hlist::
        :columns: 1

        * Elian NEPPEL
        * Shamistan KARIMOV

"""

import numpy as np
# import matplotlib
# import scipy

# matplotlib.use("Agg")  # fix for when there is no display
# scipy.randn = np.random

# rtb patch is applied lazily when robot_parsing is first imported,
# not here, to avoid pulling in ~130 MB of roboticstoolbox on every
# import of motion_stack (e.g. lvl1 processes that only need joint info).
