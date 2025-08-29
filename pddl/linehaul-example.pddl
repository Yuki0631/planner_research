(define (problem linehaul-example)
    (:domain linehaul_without_costs)

    (:objects
        ADoubleRef - refrigerated_truck
        BDouble - truck
        depot GV E BW - location
        n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 
        n11 n12 n13 n14 n15 n16 n17 n18 n19 
        n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 
        n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 
        n40 - quantity
    )

    (:init
        (at ADoubleRef depot)
        (at BDouble depot)
        (free_capacity ADoubleRef n40)
        (free_capacity BDouble n34)
        (demand_chilled_goods GV n18)
        (demand_ambient_goods GV n12)
        (demand_chilled_goods E n7)
        (demand_ambient_goods E n2)
        (demand_chilled_goods BW n3)
        (demand_ambient_goods BW n0)
        (plus1 n0 n1)
        (plus1 n1 n2)
        (plus1 n2 n3)
        (plus1 n3 n4)
        (plus1 n4 n5)
        (plus1 n5 n6)
        (plus1 n6 n7)
        (plus1 n7 n8)
        (plus1 n8 n9)
        (plus1 n9 n10)
        (plus1 n10 n11)
        (plus1 n11 n12)
        (plus1 n12 n13)
        (plus1 n13 n14)
        (plus1 n14 n15)
        (plus1 n15 n16)
        (plus1 n16 n17)
        (plus1 n17 n18)
        (plus1 n18 n19)
        (plus1 n19 n20)
        (plus1 n20 n21)
        (plus1 n21 n22)
        (plus1 n22 n23)
        (plus1 n23 n24)
        (plus1 n24 n25)
        (plus1 n25 n26)
        (plus1 n26 n27)
        (plus1 n27 n28)
        (plus1 n28 n29)
        (plus1 n29 n30)
        (plus1 n30 n31)
        (plus1 n31 n32)
        (plus1 n32 n33)
        (plus1 n33 n34)
        (plus1 n34 n35)
        (plus1 n35 n36)
        (plus1 n36 n37)
        (plus1 n37 n38)
        (plus1 n38 n39)
        (plus1 n39 n40)

        ;; specify distances between locations:
        (= (distance depot GV) 573)
        (= (distance depot E) 896)
        (= (distance depot BW) 876)
        (= (distance GV depot) 573)
        (= (distance GV E) 372)
        (= (distance GV BW) 296)
        (= (distance E depot) 896)
        (= (distance E GV) 372)
        (= (distance E BW) 79)
        (= (distance BW depot) 876)
        (= (distance BW GV) 296)
        (= (distance BW E) 79)

        ;; specify per-kilometer cost for the trucks:
        (= (per_km_cost ADoubleRef) 3.04)
        (= (per_km_cost BDouble) 2.59)

        ;; initialize total cost to zero
        (= (total-cost) 0)
    )

    (:goal (and
        (demand_chilled_goods GV n0)
        (demand_ambient_goods GV n0)
        (demand_chilled_goods E n0)
        (demand_ambient_goods E n0)
        (demand_chilled_goods BW n0)
        (demand_ambient_goods BW n0)
        (at ADoubleRef depot)
        (at BDouble depot)
    )
    )

    (:metric minimize (total-cost))
)