(define (problem task)
(:domain turtlebot_demo)
(:objects
    kenny - robot
    order_0 - order
)
(:init



    (= (order_complete order_0) 0)
    (part_of dougnut_1 order_0)
    (part_of dougnut_2 order_0)

    (not_inspected dougnut_1)
    (not_inspected dougnut_2)
)
(:goal (and

    (>= (order_complete order_0) 2)
))
)
