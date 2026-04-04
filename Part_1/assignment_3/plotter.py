import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--overlap', action='store_true',
                    help='Show overlaid trajectories')
args = parser.parse_args()

x, y, time = [], [], []
xgt, ygt = [], []

#myfile<< best_particle.x<< " "<< best_particle.y<< " " <<gt_x << " "<<gt_y<<" "<<RMSE(0)<<" "<< RMSE(1)<<" " <<duration.count()<<'\n';
with open('res.txt','r') as file:
    # reading each line    
    for line in file:
        # reading each word        
        word = line.split()
        x.append(float(word[0]))
        y.append(float(word[1]))  
        time.append(float(word[2]))

with open('pf_slam.txt','r') as file:
    # reading each line    
    for line in file:
        # reading each word        
        word = line.split()
        xgt.append(float(word[1]))
        ygt.append(float(word[2])) 


x, y = np.array(x), np.array(y)
xgt, ygt = np.array(xgt), np.array(ygt)

if args.overlap:
    # Align by truncating to the shorter sequence
    n = min(len(x), len(xgt))
    x, y = x[:n], y[:n]
    xgt, ygt = xgt[:n], ygt[:n]

    dist = np.sqrt((x - xgt)**2 + (y - ygt)**2)
    rmse = np.sqrt(np.mean(dist**2))
    mean_err = np.mean(dist)
    max_err = np.max(dist)

    fig, (ax2, ax3) = plt.subplots(2, 1, figsize=(8, 8))
    fig.suptitle(
        f"Particle Filter vs Reference  |  RMSE: {rmse:.3f} m  |"
        f"  Mean: {mean_err:.3f} m  |  Max: {max_err:.3f} m",
        fontsize=11
    )

    ax2.scatter(y, -x, color='royalblue', s=4, label='PF estimate')
    ax2.scatter(ygt, -xgt, color='forestgreen', s=4, label='Hipert reference')
    ax2.set_xlabel('y [m]')
    ax2.set_ylabel('-x [m]')
    ax2.set_title('Trajectory comparison')
    ax2.legend(markerscale=3)
    ax2.set_aspect('equal')
    t = np.arange(n)
    ax3.plot(t, dist, color='tomato', linewidth=0.8, label='Frame error [m]')
    ax3.axhline(mean_err, color='orange', linestyle='--', linewidth=1.2,
                label=f'Mean {mean_err:.3f} m')
    ax3.axhline(rmse, color='red', linestyle='--', linewidth=1.2,
                label=f'RMSE {rmse:.3f} m')
    ax3.set_xlabel('Frame')
    ax3.set_ylabel('Distance [m]')
    ax3.set_title('Per-frame error vs Hipert reference')
    ax3.legend()

else:
    fig, (ax1, ax2, ax3) = plt.subplots(3)

    ax1.scatter(x, y, color='blue', s=5)
    ax2.scatter(xgt, ygt, color='green', s=5)

    t = [i for i in range(len(time))]
    ax3.plot(t, time)

plt.tight_layout()

plt.show()
