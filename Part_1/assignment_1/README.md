# Euclidean Clustering Object Detection

## Academic Year

2025/2026

## Build the project

### Requirements

- Linux-based OS or VM (Ubuntu 18–24 is fine)
- PCL library
- CMake

Suggested install commands:

```bash
sudo apt-get install pcl-tools
sudo apt install libpcl-dev
sudo apt-get install cmake
```

### Build and run

```bash
cd build
cmake ..
make
./cluster_extraction
```

## Goals

Find and segment cars and pedestrians lying on the road.

### Main tasks

1. Implement Euclidean clustering using PCL, following the instructions in `cluster_extraction.cpp`.
2. Optional: implement your own Euclidean clustering algorithm (proximity function + pseudo-code), plot cluster distances, color nearby front vehicles, and stress-test on dataset 2.
3. Make the solution work on dataset 2. If not possible, provide a short report explaining why.

## Evaluation metrics (15 points)

- Code compiles and runs (2 points)
- Good cluster detection with low false positives and high frame rate (5 points)
- Optional custom clustering implementation (1 point)
- Works on dataset 2, or report explaining limitations/fixes (1 point)
- Additional useful functionality (4 points), for example:
  - plot distance of each cluster with respect to ego vehicle
  - color in red vehicles that are in front and within 5 meters
  - propose and implement a different clustering strategy beyond Euclidean heuristic

## Important

You must update the dataset path in code:

```cpp
std::vector<boost::filesystem::path> stream(
    boost::filesystem::directory_iterator{"/media/nacho/DATA/Sensors/pcl/dataset_1/"},
    boost::filesystem::directory_iterator{});
```

Replace it with your local dataset path, or modify the program to accept the dataset path as a command-line argument.
