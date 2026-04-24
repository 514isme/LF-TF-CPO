# LF-TF-CPO: A Dynamic Fault-Tolerant Trajectory Planning Framework for UAV Swarms

This repository contains the source code for the proposed **LF-TF-CPO** algorithm. It provides a robust, Min-Max bottleneck-oriented cooperative coverage planning framework with sub-second dynamic fault tolerance.

## ⚠️ Important Notice on Data Availability
Due to national geographic data security regulations and the sensitive nature of the high-resolution Digital Elevation Model (DEM) data (Yunling Mountains, China) used in the original study, the authentic `.mat` elevation matrix cannot be publicly released. 

To ensure **full transparency and reproducibility** in accordance with open-science standards, this repository provides:
1. **`代码.zip` (Original Archive):** Contains the exact pipeline used in the paper. Researchers with their own valid DEM data can use this archive.
2. **Standard Source Tree (`.m` files):** We have exposed the fully reproducible pipeline directly in the repository root. This pipeline uses a **Synthetic Mountain DEM** that perfectly mimics the spatial complexity (3 km × 3 km, 10 m resolution) of the original study, allowing readers to verify the algorithm's logic and performance without needing restricted data.
https://github.com/514isme/LF-TF-CPO/blob/main/README.md
---

## 📂 Repository Structure (Source Tree)

* **`Generate_Synthetic_Terrain.m`**: Generates a synthetic 3D fractal-noise DEM (`Mountain_Env_3km.mat`) to bypass data restrictions while preserving physical mountain features (slopes, roughness).
* **`Experiment_1_Comparison.m`**: Compares LF-TF-CPO against 5 state-of-the-art baselines (e.g., AE, QEDGIX).
* **`Experiment_2_Objective_Ablation.m`**: Analysis comparing Min-Sum vs. Min-Max objectives.
* **`Experiment_3_Fault_Tolerance_Single.m`**: Dynamic re-planning under a single UAV crash scenario.
* **`Experiment_4_Fault_Tolerance_Double.m`**: Extreme stress test with simultaneous double-crash at 80% mission progress.
* **`Experiment_5_Fault_Tolerance_Cascading.m`**: Cascading failure scenario (crashes at 60% and 90%).

---

## 🛠️ Software & Environment Requirements
* **Platform:** MATLAB R2022a or later (recommended).
* **Toolboxes:** Standard MATLAB installation (No deep learning or specialized commercial toolboxes are required, ensuring maximum accessibility).
* **OS:** Windows 10/11, macOS, or Linux.

---

## 🚀 Executable Run Instructions
To verify the algorithm's logic and replicate the experiments, please follow these steps:

**Step 1: Generate the Synthetic Environment**
Open MATLAB, navigate to this repository's folder, and run:
```matlab
Generate_Synthetic_Terrain
(Note: A pre-generated Mountain_Env_3km.mat is already included in the repo for your convenience, but you can re-run this script to see how the synthetic terrain is built.)


**Step 2: Run the Experiments
Simply open any of the Experiment_X_*.m scripts and click Run. For example, to evaluate the algorithm's performance against baselines, execute:

Matlab
Experiment_1_Comparison
To test the dynamic fault-tolerance under the cascading failure scenario, execute:

Matlab
Experiment_5_Fault_Tolerance_Cascading
⚙️ Parameters & Random Seeds
To rigorously eliminate stochastic variance ("butterfly effects") and guarantee exact reproducibility across different machines:

Random Seeds: A fixed global random seed (rng(2026)) is applied across all comparative and ablation scripts to ensure all heuristic algorithms start from the exact same initial spatial configuration.

Parameter Files: All hyperparameters (e.g., swarm size num_uavs = 5, iteration limit MaxIter = 80) are transparently hardcoded at the top of each executable script under the %% Global Parameters section.

📊 Expected Output (Test Case Example)
If you run Experiment_1_Comparison.m using the provided synthetic environment, the algorithm will complete the optimization seamlessly. The console will output a comprehensive energy audit table.

Expected Console Output Snippet:

Plaintext
======================================================== 终极工程指标对抗结果 ========================================================
Algorithm                    | Max_E(kJ)    | E_Std(kJ)    | Total_E(kJ)   | Turns    | Path(km)   | Mission_Time(min)  | Comp_Time(s)
-------------------------------------------------------------------------------------------------------------------------------------
3D Boustrophedon (Offline)   | 1695.02      | 125.61*      | 7833.76       | 600      | 201.16     | 86.08*             | 0.01*       
Standard PSO                 | 1764.14      | 206.30       | 7454.93       | 350      | 196.48     | 92.55              | 8.02        
Alpha Evolution (AE, 2024)   | 1801.29      | 185.89       | 7436.22       | 336*     | 197.13     | 90.27              | 7.96        
MADDPG                       | 1793.38      | 203.64       | 7401.19       | 342      | 195.04*    | 91.11              | 5.94        
QEDGIX (GNN+QMIX, 2024)      | 1797.27      | 218.77       | 7389.32*      | 352      | 195.76     | 90.28              | 6.01        
Proposed LF-TF-CPO           | 1684.17*     | 138.36       | 7514.62       | 372      | 197.10     | 91.39              | 8.80        
=====================================================================================================================================
(The script will also automatically pop up a High-Fidelity 3D trajectory plot and a Min-Max Convergence Curve figure).
