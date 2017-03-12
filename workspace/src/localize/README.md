# localize
package name: localize

## Contents

### Nodes

#### location.py
Publishes the estimated pose of the vehicle relative to a reference global position.

Subscribes to:

pose: Pose2D message 

speed_straight: Float64  
sas: Float64  
speed_lateral: Float64  
dot_Psi: Float64  
leftLane: LaneMeasure  
rightLane: LaneMeasure  

Publishes to:

location: Pose2D  
runtime: Int32  

### Message Types:
Pose2D: data.x, data.y, data.theta  
LaneMeasure: data.a0, data.a1, data.a2, data.a3, data.quality  


### Launch files
launch file naming convention: (active sensors)_(motion model)_(GPS acceptance interval)_(GPS deny interval)_(GPS reset flag).launch

launches location.py node and sets all tuning parameters and arguments
