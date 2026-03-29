#!/usr/bin/env python3
"""ADS Assignment Runner"""

import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
CONDA_ROS2_ENV = "ros2"


def _find_libpython(env_name):
    """Trova libpython nell'environment conda tramite `conda info`."""
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
    print(f"[WARN] libpython non trovata nell'env '{CONDA_ROS2_ENV}'. "
          "Part_1/Assignment_3 potrebbe non funzionare.")


def run(cmd, cwd=None, env=None):
    print(f"\n$ {cmd}\n")
    subprocess.run(cmd, shell=True, cwd=cwd, env=env)


def build_cmake(assignment_dir, executable, jobs=None, run_args=""):
    build_dir = os.path.join(assignment_dir, "build")
    exe_path = os.path.join(build_dir, executable)

    needs_build = True
    if os.path.isfile(exe_path):
        ans = input("Già compilato. Ricompilare? [y/N] ").strip().lower()
        needs_build = ans == "y"

    if needs_build:
        os.makedirs(build_dir, exist_ok=True)
        if run("cmake -DCMAKE_PREFIX_PATH=$(brew --prefix) ..", cwd=build_dir) is not None:
            return
        j = f"-j{jobs}" if jobs else ""
        if subprocess.run(f"make {j}".strip(), shell=True, cwd=build_dir).returncode != 0:
            print("make fallito.")
            return

    run(f"./{executable} {run_args}".strip(), cwd=build_dir)


def open_terminal_macos(cmd):
    """Apre una nuova finestra Terminal su macOS ed esegue cmd."""
    safe = cmd.replace('"', '\\"')
    subprocess.run(["osascript", "-e", f'tell app "Terminal" to do script "{safe}"'])


def part1_assign1():
    data_dir = os.path.join(ROOT, "Part_1", "assignment_1", "data")
    datasets = sorted([d for d in os.listdir(data_dir)
                       if os.path.isdir(os.path.join(data_dir, d))])
    if not datasets:
        print("Nessun dataset trovato in data/")
        return
    print("\nDataset disponibili:")
    for i, d in enumerate(datasets, 1):
        print(f"  {i}) {d}")
    choice = input("Seleziona dataset [1]: ").strip()
    idx = (int(choice) - 1) if choice.isdigit() and 1 <= int(choice) <= len(datasets) else 0
    dataset_path = os.path.join(data_dir, datasets[idx])
    build_cmake(os.path.join(ROOT, "Part_1", "assignment_1"), "cluster_extraction",
                run_args=dataset_path)


def part1_assign2():
    build_cmake(os.path.join(ROOT, "Part_1", "assignment_2"), "main", jobs=os.cpu_count())


def part1_assign3():
    assign_dir = os.path.join(ROOT, "Part_1", "assignment_3")
    src_dir    = os.path.join(assign_dir, "src")
    setup      = os.path.join(assign_dir, "install", "setup.zsh")
    pf_node    = os.path.join(assign_dir, "install", "pf", "lib", "pf", "pf_node")
    bag_dir    = os.path.join(assign_dir, "data")

    # Build se necessario
    if not os.path.isfile(pf_node):
        print("pf_node non trovato, eseguo build...")
        particle_src = os.path.join(src_dir, "particle")
        build_cmd = (
            f"conda run -n {CONDA_ROS2_ENV} --no-capture-output "
            f"colcon build --symlink-install --base-paths {particle_src}"
        )
        if subprocess.run(build_cmd, shell=True, cwd=assign_dir).returncode != 0:
            print("Build fallita.")
            return

    # Terminale 2: bag player
    bag_cmd = (
        f"conda activate {CONDA_ROS2_ENV} && "
        f"source {setup} && "
        f"ros2 bag play {bag_dir}"
    )
    print("\nApertura secondo terminale per la bag...")
    open_terminal_macos(bag_cmd)

    # Terminale 1: nodo (con libpython precaricata)
    node_cmd = f"source {setup} && DYLD_INSERT_LIBRARIES={LIBPYTHON} {pf_node}"
    print(f"\n$ {node_cmd}\n")
    subprocess.run(["zsh", "-c", node_cmd], cwd=src_dir)


# import_name -> pip_name (coincidono se il valore è None)
DEPS = {
    "part2":         {"cv2": "opencv-python", "numpy": None, "matplotlib": None,
                      "gtsam": None, "open3d": None, "tqdm": None, "jupyter": None},
    "part3_assign1": {"numpy": None, "matplotlib": None},
    "part3_assign2": {"numpy": None, "matplotlib": None, "casadi": None},
    "part3_assign3": {"numpy": None, "matplotlib": None},
}


def _ensure_deps(key):
    missing = []
    for import_name in DEPS[key]:
        try:
            __import__(import_name)
        except ImportError:
            missing.append(import_name)

    if not missing:
        return True

    pip_names = [DEPS[key][m] or m for m in missing]
    print(f"Moduli mancanti: {', '.join(pip_names)}")
    ans = input("Installare ora? [Y/n] ").strip().lower()
    if ans in ("", "y"):
        ret = subprocess.run([sys.executable, "-m", "pip", "install"] + pip_names)
        return ret.returncode == 0
    return False


def part2():
    d = os.path.join(ROOT, "Part_2")
    if not os.path.isfile(os.path.join(d, "ba.ipynb")):
        print("Notebook non trovato.")
        return
    _ensure_deps("part2")
    run("jupyter notebook ba.ipynb", cwd=d)


def part3(n, extra_args=""):
    d = os.path.join(ROOT, "Part_3", f"Assignment_{n}")
    if not os.path.isfile(os.path.join(d, "main.py")):
        print(f"main.py non trovato in {d}")
        return
    _ensure_deps(f"part3_assign{n}")
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
║    q) Esci                                   ║
╚══════════════════════════════════════════════╝
"""

def _make_actions(extra_args=""):
    return {
        "1": part1_assign1,
        "2": part1_assign2,
        "3": part1_assign3,
        "4": part2,
        "5": lambda: part3(1, extra_args),
        "6": lambda: part3(2, extra_args),
        "7": lambda: part3(3, extra_args),
    }


if __name__ == "__main__":
    # Uso: python run.py [numero] [args extra per lo script]
    # Es:  python run.py 6 --speed 25
    if len(sys.argv) > 1:
        choice = sys.argv[1]
        extra = " ".join(sys.argv[2:])
        action = _make_actions(extra).get(choice)
        if action:
            action()
        else:
            print(f"Scelta non valida: {choice}. Usa un numero da 1 a 7.")
        sys.exit(0)

    while True:
        print(MENU)
        choice = input("Seleziona: ").strip().lower()
        if choice in ("q", "quit", "exit"):
            break
        action = _make_actions().get(choice)
        if action:
            action()
            input("\n[Invio per tornare al menu]")
        else:
            print("Scelta non valida.")