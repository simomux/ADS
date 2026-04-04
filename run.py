#!/usr/bin/env python3
"""ADS Assignment Runner"""

import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
CONDA_ROS2_ENV = "ros2"


def _find_libpython(env_name):
    """Find libpython in the conda environment."""
    result = subprocess.run(
        ["conda", "run", "-n", env_name, "python", "-c",
         "import sysconfig, glob, os; "
         "d = sysconfig.get_config_var('LIBDIR'); "
         "libs = glob.glob(os.path.join(d, 'libpython*.dylib')); "
         "print(libs[0] if libs else '')"],
        capture_output=True, text=True
    )
    path = result.stdout.strip()
    if path and os.path.isfile(path):
        return path
    return None


LIBPYTHON = _find_libpython(CONDA_ROS2_ENV)
if not LIBPYTHON:
    print(f"[WARN] libpython not found in env '{CONDA_ROS2_ENV}'. "
          "Part_1/Assignment_3 may not work.")


def run(cmd, cwd=None, env=None):
    print(f"\n$ {cmd}\n")
    subprocess.run(cmd, shell=True, cwd=cwd, env=env)


def build_cmake(assignment_dir, executable, jobs=None, run_args=""):
    build_dir = os.path.join(assignment_dir, "build")
    exe_path = os.path.join(build_dir, executable)

    needs_build = True
    if os.path.isfile(exe_path):
        ans = input("Already built. Rebuild? [y/N] ").strip().lower()
        needs_build = ans == "y"

    if needs_build:
        os.makedirs(build_dir, exist_ok=True)
        if run("cmake -DCMAKE_PREFIX_PATH=$(brew --prefix) ..", cwd=build_dir) is not None:
            return
        j = f"-j{jobs}" if jobs else ""
        if subprocess.run(f"make {j}".strip(), shell=True, cwd=build_dir).returncode != 0:
            print("make failed.")
            return

    run(f"./{executable} {run_args}".strip(), cwd=build_dir)


def open_terminal_macos(cmd):
    """Open a new Terminal window on macOS and run cmd."""
    safe = cmd.replace('"', '\\"')
    subprocess.run(["osascript", "-e", f'tell app "Terminal" to do script "{safe}"'])


def part1_assign1():
    data_dir = os.path.join(ROOT, "Part_1", "assignment_1", "data")
    datasets = sorted([d for d in os.listdir(data_dir)
                       if os.path.isdir(os.path.join(data_dir, d))])
    if not datasets:
        print("No datasets found in data/")
        return
    print("\nAvailable datasets:")
    for i, d in enumerate(datasets, 1):
        print(f"  {i}) {d}")
    choice = input("Select dataset [1]: ").strip()
    idx = (int(choice) - 1) if choice.isdigit() and 1 <= int(choice) <= len(datasets) else 0
    dataset_path = os.path.join(data_dir, datasets[idx])
    build_cmake(os.path.join(ROOT, "Part_1", "assignment_1"), "cluster_extraction",
                run_args=dataset_path)


def part1_assign2():
    build_cmake(os.path.join(ROOT, "Part_1", "assignment_2"), "main", jobs=os.cpu_count())


def part1_assign3(extra_args=""):
    assign_dir = os.path.join(ROOT, "Part_1", "assignment_3")
    src_dir    = os.path.join(assign_dir, "src")
    setup      = os.path.join(assign_dir, "install", "setup.zsh")
    pf_node    = os.path.join(assign_dir, "install", "pf", "lib", "pf", "pf_node")
    bag_dir    = os.path.join(assign_dir, "data")

    # Split --plot-only from args forwarded to plotter.py
    args_list   = extra_args.split()
    plot_only   = "--plot-only" in args_list
    plotter_args = [a for a in args_list if a != "--plot-only"]

    if plot_only:
        print("\nRunning plotter...")
        subprocess.run([sys.executable, "plotter.py"] + plotter_args, cwd=assign_dir)
        return

    # Build if needed
    needs_build = not os.path.isfile(pf_node)
    if not needs_build:
        ans = input("Already built. Rebuild? [y/N] ").strip().lower()
        needs_build = ans == "y"
    if needs_build:
        particle_src = os.path.join(src_dir, "particle")
        build_cmd = (
            f"conda run -n {CONDA_ROS2_ENV} --no-capture-output "
            f"colcon build --symlink-install --base-paths {particle_src}"
        )
        if subprocess.run(build_cmd, shell=True, cwd=assign_dir).returncode != 0:
            print("Build failed.")
            return

    node_cmd = f"source {setup} && DYLD_INSERT_LIBRARIES={LIBPYTHON} {pf_node}"
    bag_cmd  = f"source {setup} && ros2 bag play {bag_dir}"

    # Start node in background (must load map before bag starts publishing)
    print(f"\n[node] $ {node_cmd}\n")
    node_proc = subprocess.Popen(["zsh", "-c", node_cmd], cwd=assign_dir)

    # Wait for node to load the map and initialize the viewer
    import time
    print("Waiting for node initialization (4s)...")
    time.sleep(4)

    # Run bag in foreground — blocks until playback is complete
    print(f"\n[bag]  $ {bag_cmd}\n")
    subprocess.run(["zsh", "-c", bag_cmd], cwd=assign_dir)

    # Bag finished → shut down the node
    print("\nBag finished. Shutting down node...")
    node_proc.terminate()
    try:
        node_proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        node_proc.kill()

    # Launch plotter automatically
    print("\nRunning plotter...")
    subprocess.run([sys.executable, "plotter.py"] + plotter_args, cwd=assign_dir)



def part2():
    d = os.path.join(ROOT, "Part_2")
    if not os.path.isfile(os.path.join(d, "ba.ipynb")):
        print("Notebook not found.")
        return
    run("jupyter notebook ba.ipynb", cwd=d)


def part3(n, extra_args=""):
    d = os.path.join(ROOT, "Part_3", f"Assignment_{n}")
    if not os.path.isfile(os.path.join(d, "main.py")):
        print(f"main.py not found in {d}")
        return
    run(f"python main.py {extra_args}".strip(), cwd=d)


MENU = """
╔══════════════════════════════════════════════╗
║           ADS Assignment Runner              ║
╠══════════════════════════════════════════════╣
║  PART 1 — Perception & Tracking (C++)        ║
║    1) Assignment 1 — Euclidean Clustering    ║
║    2) Assignment 2 — Kalman Filter Tracking  ║
║    3) Assignment 3 — Particle Filter (ROS2)  ║
╠══════════════════════════════════════════════╣
║  PART 2 — Visual Odometry (Jupyter)          ║
║    4) Bundle Adjustment (KITTI)              ║
╠══════════════════════════════════════════════╣
║  PART 3 — Vehicle Control & Planning         ║
║    5) Assignment 1 — Vehicle Modeling        ║
║    6) Assignment 2 — Control (PID/PP/MPC)    ║
║    7) Assignment 3 — Frenet Planner          ║
╠══════════════════════════════════════════════╣
║    q) Quit                                   ║
╚══════════════════════════════════════════════╝
"""

def _make_actions(extra_args=""):
    return {
        "1": part1_assign1,
        "2": part1_assign2,
        "3": lambda: part1_assign3(extra_args),
        "4": part2,
        "5": lambda: part3(1, extra_args),
        "6": lambda: part3(2, extra_args),
        "7": lambda: part3(3, extra_args),
    }


if __name__ == "__main__":
    # Usage: python run.py [number] [extra args for the script]
    # e.g.:  python run.py 6 --speed 25
    if len(sys.argv) > 1:
        choice = sys.argv[1]
        extra = " ".join(sys.argv[2:])
        action = _make_actions(extra).get(choice)
        if action:
            action()
        else:
            print(f"Invalid choice: {choice}. Use a number from 1 to 7.")
        sys.exit(0)

    while True:
        print(MENU)
        choice = input("Select: ").strip().lower()
        if choice in ("q", "quit", "exit"):
            break
        action = _make_actions().get(choice)
        if action:
            action()
            input("\n[Press Enter to return to menu]")
        else:
            print("Invalid choice.")