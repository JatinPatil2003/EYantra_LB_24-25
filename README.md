# EYantra_LB_24-25

## Setup Instructions

Run the following command to clone repo with all submodules

```bash
git clone https://github.com/JatinPatil2003/EYantra_LB_24-25.git --recursive
```

## Usage Instructions

- `eyrc-24-25-logistic-cobot` submodule is only added for getting latest release

- For development use folders for perticular tasks


## Instructions to Build Specific Task

```bash
# Make sure your are in project root directory
cd Task<task-no>
colcon build --symlink-install 

# and for sourcing workspace, Make sure to be in Task directory
source install/setup.bash

# You can make Linux alias for sourcing workspace easily in few clicks
```
