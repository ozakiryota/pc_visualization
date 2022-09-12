# pc_visualization

## Demo
![demo](https://user-images.githubusercontent.com/37431972/158047203-0c343126-d70b-4611-a68d-e1d4658c838d.png)

## Installation 
### Build locally
Requirements:
* ROS
* PCL

```bash
cd ~/catkin_ws/src
git clone https://github.com/ozakiryota/pc_visualization.git
cd ..
catkin_make
```

### Build with Docker
```bash
git clone https://github.com/ozakiryota/pc_visualization.git
cd pc_visualization/docker
./build.sh
```

## Parameters
* m_per_cell
* grid_dim
* height_diff_threshold

![pc_visualization](https://user-images.githubusercontent.com/37431972/159700061-9f1c7368-821c-42a8-aac9-3dc36f23eb29.png)

## Usage
1. Edit the launch file
1. Run
```bash
roslaunch pc_visualization pc_visualization.launch
```
