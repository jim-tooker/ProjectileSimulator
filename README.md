# Projectile Simulator
(Package `projectilesim`)

This package simulates projectile motion using VPython for visualization. It calculates and displays the trajectory of a projectile in different environments and for different projectile types.

Here are the runtime options: `python projectile_sim.py -h`:
```
usage: projectile_sim.py [-h] [--test] [--no_gui]

Projectile Simulator

options:
  -h, --help  show this help message and exit
  --test      Run with pre-defined test cases
  --no_gui    Run without GUI
```

## Modules:
* `projectile_sim`: The main module that runs the system. It manages the entire simulation, physics calculations, and plotting.
* `environment`: Represents the environment that the projectile will be flying through.  Includes gravity and air density.
* `projectile`: Represents the projectile.  Includes mass, radius, and surface area.

## Documentation
For detailed API documentation, see [API Documentation](https://jim-tooker.github.io/projectilesim/docs/projectile_sim.html).

## To Use
1. Clone this repository.
2. Run the setup script: `setup.sh`.  *(This will install needed dependencies)*
3. Run `python projectile_sim.py`.  *(With or without command line options)*

## Sample Screenshot
![projectilesim-screen](https://github.com/user-attachments/assets/9afad9a7-7216-4db0-9f0b-a66f79789f41)
