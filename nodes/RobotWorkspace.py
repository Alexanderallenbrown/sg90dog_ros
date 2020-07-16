from numpy import *
from matplotlib.pyplot import *


# Initialize allowable angle ranges for each joint
hipJRangeMin = 65
hipJRangeMax = 92
femurJRangeMin = 0
femurJRangeMax = 150
tibiaJRangeMin = 0
tibiaJRangeMax = 160

# Initialize allowable servo angle ranges for each joint
hipSRangeMin = 0
hipSRangeMax = 180
femurSRangeMin = 0+135-90
femurSRangeMax = 180+135-90
tibiaSRangeMin = 0+76.8-90
tibiaSRangeMax = 180+76.8-90

# Find the overlap in these two ranges by identifying the smallest range for each joint
hipRangeMin = max([hipJRangeMin, hipSRangeMin])
hipRangeMax = min([hipJRangeMax, hipSRangeMax])
femurRangeMin = max([femurJRangeMin, femurSRangeMin])
femurRangeMax = min([femurJRangeMax, femurSRangeMax])
tibiaRangeMin = max([tibiaJRangeMin, tibiaSRangeMin])
tibiaRangeMax = min([tibiaJRangeMax, tibiaSRangeMax])

hipRange = range(hipRangeMin, hipRangeMax+1)
femurRange = range(femurRangeMin, femurRangeMax+1)
tibiaRange = range(tibiaRangeMin, tibiaRangeMax+1)

# For each combination of angles, use forward kinematics to find the foot position
for h in hipRange:
    # Calculate location of femur joint
    # position of femur joint relative to hip in neutral position

    # Relative change due to angle (won't affect x)

    for f in femurRange:
        # Calculate location of tibia joint
        # length of femur

        # x, y, z position (look at ME240 notes?)

        for t in tibiaRange:
            # Calculate location of foot
            # length of tibia (approx or actual?)
