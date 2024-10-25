#!/bin/bash

# Define two opposite corners of the rectangle (x, y)
x1=14.11
y1=76.89
x2=1.19
y2=62.68

# Helper function to generate a random float between two numbers
rand_float() {
    awk -v min="$1" -v max="$2" 'BEGIN { srand(); print min + rand() * (max - min) }'
}

# Ensure min and max values are assigned correctly
min_value() {
    echo "$1 $2" | awk '{if ($1 < $2) print $1; else print $2}'
}

max_value() {
    echo "$1 $2" | awk '{if ($1 > $2) print $1; else print $2}'
}

# Calculate min and max for x and y coordinates
min_x=$(min_value $x1 $x2)
max_x=$(max_value $x1 $x2)
min_y=$(min_value $y1 $y2)
max_y=$(max_value $y1 $y2)

# Infinite loop to publish a random goal every 10 seconds
while true; do
    # Generate random (x, y) within the rectangle
    random_x=$(rand_float $min_x $max_x)
    random_y=$(rand_float $min_y $max_y)

    # Publish the random goal using rostopic
    rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped \
    "{
        header: {
            seq: 0,
            stamp: {secs: 0, nsecs: 0},
            frame_id: ''
        },
        pose: {
            position: {x: $random_x, y: $random_y, z: 0},
            orientation: {x: 0, y: 0, z: 0, w: 1.0}
        }
    }"
    # Wait for 15 seconds before publishing the next goal
    sleep 15
done
