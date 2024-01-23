# Control of the Cart-Pole System

Implementation of control algorithms for the cart-pole system, including Nonlinear Model Predictive Control (NMPC), Linear Quadratic Control (LQR). 

(add the gif for the cart-pole system)

## LQR/NMPC Controllers

// TODO: Results of (gif) of the two controllers. Table Form

## Dependencies

Follow the instructions (link) to install Ipopt.
1. Request and download `Coin-HSL Archive` (free for personal use). 
2. Folllow install instruction on `ThirdParty-HSL` to install the solver library. 
3. Follow install instruction on `Ipopt`.

Install dependencies for python3
```
pip3 install -r requirements.txt
```

The cart-pole env in gymnasium was used for visualization. Test if visualization is set up correctly. 
```
python3 ./vis/examples/test_vis.py
```

Test if the cart-pole plant (physical model) can run smoothly:
```
python3 ./system/plant.py --enable_vis
```

## Build

build

```
mkdir build && cd build
cmake ..
make -j4
```

## Simulation

How to run the simulation

```
python3 run.py --save_gif
```

## Settings

Settings of the controller type and parameter values are in the json file, `config/control_param.json`.  

```
"type": "NMPC",     // Available Types: "LQR", "NMPC"

"nmpc_cfg": {       // Control parameters for NMPC, active when "type" is "NMPC"
    "mpc_dt": 0.02,     // [s], NMPC time step
    "hp": 50,           // Prediction Horizon
    "hc": 50,           // Control Horizon
    "Qx": 5000.0,       // Weight for tracking error of x
    "Qtheta": 100.0,    // Weight for tracking error of theta
    "R_u": 100.0,       // Weight for control effort (force)
    "R_du": 0.0         // Weight for control rate (force rate)
}

"lqr_cfg": {        // Control parameters for LQR, active when "type" is "LQR"
    "Qx": 5000.0,       // Weight for tracking error of x
    "Qtheta": 100.0,    // Weight for tracking error of theta
    "R_u": 1.0,         // Weight for control effort (force)
    "R_du": 0.1,        // Weight for control rate (force rate)
    "use_finite_lqr": true,  // If true, use finite horizon LQR solver; Otherwise, use infinite horizon LQR solver
    "hp": 50,           // Horizon, valid only if `is_finite` is true
    "lqr_dt": 0.02      // DARE step size, valid only if `is_finite` is true; Generally, `lqr_dt` = `ctrl_time_step`
}

```

