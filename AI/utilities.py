import tf_transformations
from scipy.stats import norm
import numpy as np

DELTA_DIST = 0.5
DELTA_ANGLE = np.deg2rad(180.0)
OFFSETs_IDX = [1,2,3,4,5,8,10]
CUT_FRACTION = 0.1

def quaternion_to_yaw(q):
    _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def get_pose_for_scan(scan_time, poses):
    left, right = 0, len(poses) - 1
    best_idx = 0
    min_diff = abs(scan_time - poses[0][0])
    
    while left <= right:
        mid = (left + right) // 2
        diff = abs(scan_time - poses[mid][0])
        if diff < min_diff:
            min_diff = diff
            best_idx = mid
        if poses[mid][0] < scan_time:
            left = mid + 1
        else:
            right = mid - 1
    
    return poses[best_idx][1]


def plot_error_with_gaussian(plt, errors, title, xlabel):
    errors = np.array(errors, dtype=np.float64)
    errors = np.nan_to_num(errors, nan=0.0, posinf=0.0, neginf=0.0)
    counts, bins, _ = plt.hist(errors, bins=50, density=False, alpha=0.6, color='skyblue', edgecolor='black')
    
    mu, std = norm.fit(errors)
    
    x = np.linspace(min(errors), max(errors), 100)
    p = norm.pdf(x, mu, std)
    
    scale = max(counts) / max(p)
    p_scaled = p * scale
    
    plt.plot(x, p_scaled, 'r', linewidth=2, label=f'Gauss: μ={mu:.4f}, σ={std:.4f}')
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("Liczba próbek")
    plt.legend()

def is_data_near(transformation1, transformation2):
    x1 = transformation1[0]
    y1 = transformation1[1]
    yaw1 = transformation1[2]
    x2 = transformation2[0]
    y2 = transformation2[1]
    yaw2 = transformation2[2]

    dist = np.sqrt((x1-x2)**2 + (y1-y2)**2)
    delta_yaw = np.arctan2(np.sin(yaw2-yaw1),np.cos(yaw2-yaw1))
    dx = x2 - x1
    dy = y2 - y1

    data = {
            "dx": dx,
            "dy": dy,
            "dyaw": delta_yaw,
            "is_near": False
        }

    if dist < DELTA_DIST and delta_yaw < DELTA_ANGLE:
        data["is_near"] = True
    
    return data

def make_gaussian_noise(scan_transformation_pairs, noise=0.1):
    noisy_pairs = []
    for scans, d_trans in scan_transformation_pairs:
        scans_arr = np.array(scans)
        noise_std = np.abs(scans_arr) * noise
        noise_array = np.random.normal(0, noise_std)
        noisy_scans = scans_arr + noise_array
        noisy_pairs.append((tuple(noisy_scans), d_trans))
    return noisy_pairs

def cut_data_from_scan_pair(scan_pair, max_points_to_cut=20):
    points = np.random.randint(1,max_points_to_cut)
    idx = np.random.randint(0,360)
    scan_pair_arr = np.array(scan_pair)
    idxs = [(idx+i)%360 for i in range(points)]
    scan_pair_arr[:, idxs] = 0
    return scan_pair_arr

def cut_data_from_scans(scan_transformation_pairs, max_points_to_cut=20):
    res = []
    for scan_pair, d_trans in scan_transformation_pairs:
        scan_pair_cut = cut_data_from_scan_pair(scan_pair)
        res.append( (scan_pair_cut, d_trans) )
    return res    