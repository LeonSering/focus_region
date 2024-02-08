# Focus Map Tool

This tool was created by Leon Sering during the research project on "focus regions" at the university of Würzburg, supervised by Jan-Henrik Haunert.

The result are published in Transactions on Visualization and Computer Graphics and can be found [here](https://doi.org/10.1109/TVCG.2011.191).

The tool allows the user to choose a focus region within a larger area on a road network, together with a zoom factor.  By solving a convex quadratic program a new visualization of the road map is created, where all elements in the focus region are scaled by the zoom factor, all elements outside the area untouched and the elements within the area but outside the focus region are replaced such that the distortions are minimized.

The result is a new map, where the focus region is strongly enlarged, but still the map looks topologically very similar to the original map.

A typical use case is the visualization of a long route from a dense city to another dense city on a GPS unit in a car. You can enlarge the area close to the origin and destination, where typically a lot of small roads are important for the driver. The route on the highway between the cities is usually easier to navigate, so it could be shrunk.

The result is a visualization of the whole route within a road map that still looks quite similar to the original one, but the two focus regions are enlarged such that each turn is clearly visible.

# Install (Linux)

#### Install CPLEX

1. go to [this IBM page](https://www.ibm.com/academic/topic/data-science)

2. choose "Software" -> "ILOG CPLEX Optimization Studio"

3. You need to register with an academic email address (or buy a license).

4. choose "HTTP"

5. search for "IBM ILOG CPLEX Optimization Studio V22.1.1 for Linux x86-64" and select it (on the left)

6. "I agree" -> "Download now"

7. install CPLEX Optimization Studio via:
   
   ```bash
   cd Downloads
   chmod +x cplex_studio2211.linux_x86_64.bin
   ./cplex_studio2211.linux_x86_64.bin
   ```

8. choose as installation path: ```/home/<your_username>/opt/CPLEX``` (otherwise, you have to edit the path in run.sh)

#### Create JAR-file

1. If you have a different CPLEX version than 22.1.1 then you have to copy the ```cplex.jar``` from ```/home/<your_username>/opt/CPLEX/cplex/lib/``` to ```focusmap/lib/``` and override the existing file.

2. Create the JAR-file with the ```build.sh```-bash-script:
   
   ```bash
   ./build.sh
   ```

# Usage

1. To start the program, run:
   
   ```bash
   ./run.sh
   ```

2. Open an SHP-file (you can find some in GIS_data).

3. Click and drag a cyclic focus region onto the map.

4. Choose a zoom factor. Everything within the circle is enlarged by this factor.

5. Choose an area size. Everything outside of the area is fixed. So distortions only happen in the area without the focus region.

6. Click on "Compute" to start the "focus map"-computation with CPLEX.

#### More options

- You can add more focus regions by selecting "Add Envelopes".

- Unselect "Circle" to draw a rectangular focus region instead.

- Zoom with the mouse wheel

- Select "Zoom & Pen" to move the visible canvas with the left mouse button.

- Click "Project" to generate a classical fish-eye-projection.

- use "Save to Shapefile" to save generated maps.
