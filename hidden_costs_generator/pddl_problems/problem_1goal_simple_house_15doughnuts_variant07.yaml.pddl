(define (problem task)
(:domain turtlebot_demo)
(:objects
    kenny - robot
    order_0 order_1 order_2 order_3 - order
)
(:init



    (= (order_complete order_0) 0)
    (part_of dougnut_1 order_0)
    (part_of dougnut_2 order_0)
    (part_of dougnut_3 order_0)

    (= (order_complete order_1) 0)
    (part_of dougnut_4 order_1)
    (part_of dougnut_5 order_1)
    (part_of dougnut_6 order_1)

    (= (order_complete order_2) 0)
    (part_of dougnut_7 order_2)
    (part_of dougnut_8 order_2)
    (part_of dougnut_9 order_2)

    (= (order_complete order_3) 0)
    (part_of dougnut_10 order_3)
    (part_of dougnut_11 order_3)
    (part_of dougnut_12 order_3)

    (not_inspected dougnut_1)
    (not_inspected dougnut_2)
    (not_inspected dougnut_3)
    (not_inspected dougnut_4)
    (not_inspected dougnut_5)
    (not_inspected dougnut_6)
    (not_inspected dougnut_7)
    (not_inspected dougnut_8)
    (not_inspected dougnut_9)
    (not_inspected dougnut_10)
    (not_inspected dougnut_11)
    (not_inspected dougnut_12)
    (not_inspected dougnut_13)
    (not_inspected dougnut_14)
    (not_inspected dougnut_15)
)
(:goal (and

    (>= (order_complete order_0) 3)
))
)
