# Robot-Charging-in-Basement-Grid-map-Using-GAMA

This project involves a simulation using GAMA to model and analyze robot charging strategies within a basement grid map environment. The primary goal is to optimize the movement and charging behavior of robots to ensure efficient energy usage and task completion.

## Features

- **Simulation Environment**: The simulation is set up in a basement grid map environment.
- **Algorithms**: Multiple pathfinding algorithms are implemented, including A*, Dijkstra, JPS, BF, and a custom algorithm named Thinh.
- **Robot Behavior**: Robots are programmed to navigate the grid, avoiding obstacles and seeking charging stations when necessary.
- **Energy Management**: The simulation monitors robot energy levels and initiates charging when energy falls below a certain threshold.
- **Statistics Collection**: The simulation collects data on path lengths and charging events to analyze performance.

## Parameters

- **Scenario**: Choose between "random map" and "basement map".
- **Algorithm**: Select from A*, Dijkstra, JPS, BF, and Thinh.
- **Neighborhood Type**: Specify the type of neighborhood (4 or 8).
- **Obstacle Rate**: Set the rate of obstacles in the grid (0.0 to 0.4).
- **Grid Size**: Define the height and width of the grid (default 100x100).
- **Energy Limit**: Set the maximum energy limit for robots (0 to 2500).

## Setup

1. **Clone the Repository**:
   ```sh
   git clone https://github.com/Thinhvip9999/Robot-Charging-in-Basement-Grid-map-Using-GAMA.git
   cd Robot-Charging-in-Basement-Grid-map-Using-GAMA
   ```

2. **Open the Project in GAMA**:
   - Open GAMA and load the project files.

3. **Run the Simulation**:
   - Configure the parameters as needed.
   - Start the simulation to see the robots navigate and charge within the grid.

## Files

- `README.md`: Project documentation.
- `Algorithms_On_Map.gaml`: The main simulation model file.
- `includes/images/robot.png`: Image file for robot representation.
- `includes/images/charger.png`: Image file for charging station representation.
- `includes/images/electric-car.png`: Additional image file used in the simulation.

## Contributions

Contributions are welcome! Please fork the repository and submit pull requests for any improvements or new features.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

---

For more information about GAMA, visit [GAMA Platform](https://gama-platform.org/).
```

You can customize this README file further based on specific details or additional information you want to include about your project.
