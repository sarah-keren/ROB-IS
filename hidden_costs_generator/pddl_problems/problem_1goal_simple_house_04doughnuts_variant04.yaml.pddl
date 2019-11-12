(define (problem task)
(:domain turtlebot_demo)
(:objects
    kenny - robot
    order_0 order_1 order_2 order_3 - order
)
(:init
    (robot_at kenny wp0)


    (= (order_complete order_0) 0)
    (part_of dougnut_1 order_0)

    (= (order_complete order_1) 0)
    (part_of dougnut_2 order_1)

    (= (order_complete order_2) 0)
    (part_of dougnut_3 order_2)

    (= (order_complete order_3) 0)
    (part_of dougnut_4 order_3)

    (not_inspected dougnut_1)
    (not_inspected dougnut_2)
    (not_inspected dougnut_3)
    (not_inspected dougnut_4)
)
(:goal (and

    (>= (order_complete order_0) 1)
))
)
