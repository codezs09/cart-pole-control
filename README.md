# Nonlinear Model Predictive Control for the Cart-Pole System

(add the gif for the cart-pole system)

## Dependencies

Follow the instructions of Ipopt(link) to install.
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
mkdir build
cd build
cmake ..
make -j4
```

## Simulation

How to run the simulation

```
python3 run.py --save_gif
```

## System Models


## NMPC Controller



