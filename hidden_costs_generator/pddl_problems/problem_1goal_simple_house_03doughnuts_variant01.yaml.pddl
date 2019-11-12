(define (problem task)
(:domain turtlebot_demo)
(:objects
    kenny - robot
    order_0 - order
)
(:init
    (robot_at kenny wp0)


    (= (order_complete order_0) 0)
    (part_of dougnut_1 order_0)
    (part_of dougnut_2 order_0)
    (part_of dougnut_3 order_0)

    (not_inspected dougnut_1)
    (not_inspected dougnut_2)
    (not_inspected dougnut_3)
)
(:goal (and

    (>= (order_complete order_0) 3)
))
)
