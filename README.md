# path-planning

## Installation

* install CoppeliaSim 
* clone the repository
    ```
    git clone https://github.com/SelkieAnna/path-planning.git
    ```
* find the remoteApi file inside the CoppeliaSim directory
    * it should be located in the `programming/remoteApiBindings` folder (either in `lib` or in `lua` depending on the running mode)
    * the file extension also depends on your OS
* install the project requirements
    ```
    pip3 install -r requirements.txt
    ```

## Running

* open CoppeliaSim
* load `scane.ttt` scene from this repository
* start the simulation
* finally, run `main.py`
    ```
    python3 main.py
    ```
    * before the simulation in CoppeliaSim will be displayed you will have to close the plot
    * to adjust the target point you should modify `target_x` and `target_y` variables in main.py