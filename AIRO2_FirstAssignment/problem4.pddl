;; Problem Description: [1]
;  There are 2 customers at table 2: they ordered 2 cold drinks. Tables 3 and 4 need to be cleaned. 

(define (problem caffe_bar_problem_4)

    ;; Let the planner know about the domain 

	(:domain coffee-shop)
        
    ;; Introduce objects

    (:objects 
        b - barista
        w1 w2 - waiter ; Waiter
        bar - bar
        t1 t2 t3 t4 - table ; Tables
        d1 d2 d3 d4 d5 d6 d7 d8 - drink ; Drinks
        u1 u2 u3 u4 - biscuit
        c1 c2 c3 c4 c5 c6 c7 c8 - customer
    )

    ;; Assert the initial state

    (:init

        ; General initial state of the coffee shop: same for all problems.

        ; Objects definition
        (barista b)
        (waiter w1) (waiter w2)
        (bar bar) (table t1) (table t2) (table t3) (table t4)
        ; Predicates definition
        (free-bar b)
        (is-static w1) (is-static w2)
        (empty-hand w1) (empty-hand w2)
        (duty-waiter w1 t1) (duty-waiter w1 t3)
        (duty-waiter w2 t2) (duty-waiter w2 t4)
        (at w1 bar) (at w2 t1)
        (free-location t2) (free-location t3) (free-location t4)
        ; Functions initialization
        (= (velocity w1) 2)
        (= (velocity w2) 2)
        (= (drinks-on-tray w1) 0)
        (= (drinks-on-tray w2) 0)
        (= (drinks-on-tray-left w1) 0)
        (= (drinks-on-tray-left w1) 0)
        (= (biscuits-on-tray  w1) 0)
        (= (biscuits-on-tray  w2) 0)
        (= (biscuits-on-tray-left w1) 0)
        (= (biscuits-on-tray-left w1) 0)
        (= (width t1) 1)
        (= (width t2) 1)
        (= (width t3) 2)
        (= (width t4) 1)
        (= (dist bar t1) 2) (= (dist bar t2) 2) (= (dist bar t3) 3) (= (dist bar t4) 3)
        (= (dist t1 bar) 2) (= (dist t2 bar) 2) (= (dist t3 bar) 3) (= (dist t4 bar) 3)
        (= (dist t1 t2) 1) (= (dist t2 t1) 1)
        (= (dist t1 t3) 1) (= (dist t3 t1) 1) (= (dist t2 t3) 1) (= (dist t3 t2) 1)
        (= (dist t1 t4) 1) (= (dist t4 t1) 1) (= (dist t2 t4) 1) (= (dist t4 t2) 1) (= (dist t3 t4) 1) (= (dist t4 t3) 1)
        (= (number-of-customers-left t1) 0)        
        (= (number-of-customers-left t2) 0)
        (= (number-of-customers-left t3) 0)
        (= (number-of-customers-left t4) 0)

        ; Particular initial state of the coffee shop: different for each problem.

        ; Drinks, biscuit and customers definition
        (icy d1) (icy d2) (hot d3) (hot d4) (hot d5) (hot d6) (icy d7) (icy d8)
        (biscuit u1) (biscuit u2) (biscuit u3) (biscuit u4) 
        (customer c1) (customer c2) (customer c3) (customer c4) (customer c5) (customer c6) (customer c7) (customer c8)

        (different d1 d2) (different d1 d7) (different d1 d8) 
        (different d2 d7) (different d2 d8)
        (different d7 d8)

        (different d3 d4) (different d3 d5) (different d3 d6) 
        (different d4 d5) (different d4 d6)
        (different d5 d6)

        (different-b u1 u2) (different-b u1 u3) (different-b u1 u4)
        (different-b u2 u3) (different-b u2 u4)
        (different-b u3 u4)
        ; Order information
        (drink-info c1 d1 t1) (drink-info c2 d2 t1) (drink-info c3 d3 t3) (drink-info c4 d4 t3)
        (drink-info c5 d5 t3) (drink-info c6 d6 t3) (drink-info c7 d7 t4) (drink-info c8 d8 t4)
        (biscuit-info c1 u1 t1) (biscuit-info c2 u2 t1) (biscuit-info c7 u3 t4) (biscuit-info c8 u4 t4)
        (biscuit-info c1 u1 bar) (biscuit-info c2 u2 bar) (biscuit-info c7 u3 bar) (biscuit-info c8 u4 bar)
        ; Number of customer at each table
        (= (number-of-customers t1) 2)
        (= (number-of-customers t2) 0)
        (= (number-of-customers t3) 4)
        (= (number-of-customers t4) 2)
        ; we supposed table 2 has to be cleaned
    )

    ;; Assert the goal state
    (:goal (and (clean t1) (clean t2) (clean t3) (clean t4)))

    (:metric minimize (total-time))
)