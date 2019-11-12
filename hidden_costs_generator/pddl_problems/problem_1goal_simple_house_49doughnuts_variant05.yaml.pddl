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
    (part_of dougnut_2 order_0)
    (part_of dougnut_3 order_0)
    (part_of dougnut_4 order_0)
    (part_of dougnut_5 order_0)
    (part_of dougnut_6 order_0)
    (part_of dougnut_7 order_0)
    (part_of dougnut_8 order_0)
    (part_of dougnut_9 order_0)
    (part_of dougnut_10 order_0)
    (part_of dougnut_11 order_0)
    (part_of dougnut_12 order_0)

    (= (order_complete order_1) 0)
    (part_of dougnut_13 order_1)
    (part_of dougnut_14 order_1)
    (part_of dougnut_15 order_1)
    (part_of dougnut_16 order_1)
    (part_of dougnut_17 order_1)
    (part_of dougnut_18 order_1)
    (part_of dougnut_19 order_1)
    (part_of dougnut_20 order_1)
    (part_of dougnut_21 order_1)
    (part_of dougnut_22 order_1)
    (part_of dougnut_23 order_1)
    (part_of dougnut_24 order_1)

    (= (order_complete order_2) 0)
    (part_of dougnut_25 order_2)
    (part_of dougnut_26 order_2)
    (part_of dougnut_27 order_2)
    (part_of dougnut_28 order_2)
    (part_of dougnut_29 order_2)
    (part_of dougnut_30 order_2)
    (part_of dougnut_31 order_2)
    (part_of dougnut_32 order_2)
    (part_of dougnut_33 order_2)
    (part_of dougnut_34 order_2)
    (part_of dougnut_35 order_2)
    (part_of dougnut_36 order_2)

    (= (order_complete order_3) 0)
    (part_of dougnut_37 order_3)
    (part_of dougnut_38 order_3)
    (part_of dougnut_39 order_3)
    (part_of dougnut_40 order_3)
    (part_of dougnut_41 order_3)
    (part_of dougnut_42 order_3)
    (part_of dougnut_43 order_3)
    (part_of dougnut_44 order_3)
    (part_of dougnut_45 order_3)
    (part_of dougnut_46 order_3)
    (part_of dougnut_47 order_3)
    (part_of dougnut_48 order_3)

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
    (not_inspected dougnut_16)
    (not_inspected dougnut_17)
    (not_inspected dougnut_18)
    (not_inspected dougnut_19)
    (not_inspected dougnut_20)
    (not_inspected dougnut_21)
    (not_inspected dougnut_22)
    (not_inspected dougnut_23)
    (not_inspected dougnut_24)
    (not_inspected dougnut_25)
    (not_inspected dougnut_26)
    (not_inspected dougnut_27)
    (not_inspected dougnut_28)
    (not_inspected dougnut_29)
    (not_inspected dougnut_30)
    (not_inspected dougnut_31)
    (not_inspected dougnut_32)
    (not_inspected dougnut_33)
    (not_inspected dougnut_34)
    (not_inspected dougnut_35)
    (not_inspected dougnut_36)
    (not_inspected dougnut_37)
    (not_inspected dougnut_38)
    (not_inspected dougnut_39)
    (not_inspected dougnut_40)
    (not_inspected dougnut_41)
    (not_inspected dougnut_42)
    (not_inspected dougnut_43)
    (not_inspected dougnut_44)
    (not_inspected dougnut_45)
    (not_inspected dougnut_46)
    (not_inspected dougnut_47)
    (not_inspected dougnut_48)
    (not_inspected dougnut_49)
)
(:goal (and

    (>= (order_complete order_0) 12)
))
)
