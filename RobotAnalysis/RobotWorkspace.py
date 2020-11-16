from numpy import *
from matplotlib.pyplot import *
rcParams['agg.path.chunksize'] = 10000

## TODO: Add more points to y-z plane to fill out workspace area

# Limb dimensions
# From inside face of hip near hip servo to inside face near femur servo (inch)
hipY = 1.35;
hipZ = 0.568 + 0.25;
hipX = 0.36

# From inside face of hip near femur servo to inside face of tibia near tibia servo
femurLength = 1.715; # inch

# From inside face of tibia near tibia servo to foot
tibiaLength = 2.786; # inch

# Initialize possible foot position arrays
all_xp = []
all_yp = []
all_zp = []

# Initialize allowable angle ranges for each joint
hipJRangeMin = 20 #65
hipJRangeMax = 140 #92
femurJRangeMin = 10
femurJRangeMax = 180
tibiaJRangeMin = 10
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

hipRange = arange(hipRangeMin, hipRangeMax+1,0.25)
femurRange = arange(femurRangeMin, femurRangeMax+1, 0.75)
tibiaRange = arange(tibiaRangeMin, tibiaRangeMax+1, 0.75)

# For each combination of angles, use forward kinematics to find the foot position
for h in hipRange:
    # Calculate location of femur joint
    # position of femur joint relative to hip in neutral position

    # Relative change due to angle (won't affect x)
    x2 = - hipX
    y2 = hipY * sin(deg2rad(h)) + hipZ * sin(deg2rad(90-h))
    z2 = hipY * cos(deg2rad(h)) - hipZ * cos(deg2rad(90-h))

    for f in femurRange:
        # Calculate location of tibia joint
        # length of femur

        # x, y, z position
        x3 = x2 + femurLength * cos(-f)
        # y3 = y2 + femurLength * sin(h)
        z3_prime = femurLength * sin(-f)

        for t in tibiaRange:
            # Calculate location of foot
            # length of tibia (approx or actual?)

            xp_prime = tibiaLength * cos(t)
            z4_prime = tibiaLength * sin(t)

            xp = x3 + xp_prime *cos(-f) - z4_prime * sin(-f)
            z4 = z3_prime + xp_prime*sin(-f) + z4_prime*cos(-f)

            zp = z2 + z4*cos(90-h)
            yp = y2 - z4 * sin(90-h)

            all_xp.append(xp)
            all_yp.append(yp)
            all_zp.append(zp)


#figure()
#plot(all_xp, all_zp,'k.')
#xlabel('X Positions (in)')
#ylabel('Z Positions (in)')

figure()
plot(all_yp, all_zp,'b.')
xlabel('Y Positions (in)')
ylabel('Z Positions (in)')
plot([-4, 4], [0, 0], 'r')
plot([-4, -4], [0, 2], 'r')
plot([4, 4], [0,2],'r')
plot([-4, 4], [2, 2], 'r')

#figure()
#plot(all_zp, all_yp)
#xlabel('Z Positions')
#ylabel('Y Positions')
show()
