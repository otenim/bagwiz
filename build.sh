#!/bin/bash

colcon build --symlink-install --packages-up-to bagwiz --cmake-args -DCMAKE_BUILD_TYPE=Release
