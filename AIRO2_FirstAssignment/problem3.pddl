;; Problem Description: [1]
;  There are 2 customers at table 2: they ordered 2 cold drinks. Tables 3 and 4 need to be cleaned. 

(define (problem caffe_bar_problem_3)

    ;; Let the planner know about the domain 

	(:domain coffee-shop)
        
    ;; Introduce objects

    (:objects 
        b - barista
        w1 w2 - waiter ; Waiter
        bar - bar
        t1 t2 t3 t4 - table ; Tables
        d1 d2 d3 d4 - drink ; Drinks
        b1 b2 - biscuit
        c1 c2 c3 c4 - customer
    )

    ;; Assert the initial state

    (:init

        ; General initial state of the coffee shop: same for all problems

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

        ; Particular initial state of the coffee shop: different for each problem

        ; Drinks, biscuits and customers definition (and distingtion between drinks and biscuits)

        (hot d1) (hot d2) (hot d3) (hot d4)
        (customer c1) (customer c2) (customer c3) (customer c4)
        (different d1 d2) (different d1 d3) (different d1 d4) 
        (different d2 d3) (different d2 d4)
        (different d3 d4)
        ; Order info definition:
        (drink-info c1 d1 t1) (drink-info c2 d2 t1) (drink-info c3 d3 t4) (drink-info c4 d4 t4)
        ; Number of customer at each table:
        (= (number-of-customers t1) 2)
        (= (number-of-customers t2) 0)
        (= (number-of-customers t3) 0)
        (= (number-of-customers t4) 2)
        ; Table status
        (clean t2)
    )

    ;; Assert the goal state
    (:goal (and (clean t1) (clean t2) (clean t3) (clean t4)))

    (:metric minimize (total-time))
)