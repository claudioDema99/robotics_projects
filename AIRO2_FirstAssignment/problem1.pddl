;; Problem Description: [1]
;  There are 2 customers at table 2: they ordered 2 cold drinks. Tables 3 and 4 need to be cleaned. 

(define (problem caffe_bar_problem_1)

    ;; Let the planner know about the domain 

	(:domain coffee-shop)
        
    ;; Introduce objects

    (:objects 
        b - barista
        w1 w2 - waiter ; Waiter
        o1 o2 - order 
        bar - bar
        t1 t2 t3 t4 - table ; Tables
        d1 d2 - drink ; Drinks
        u1 u2 - biscuit
        c1 c2 - customer
    )

    ;; Assert the initial state

    (:init

        ; General initial state of the coffee shop: same for each problem.

        ; Objects definition
        (barista b)
        (waiter w1) (waiter w2)
        (bar bar) (table t1) (table t2) (table t3) (table t4)
        ; Predicates initialization
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

        ; Particular initial state of the world: different for each problems.

        ; Drinks, biscuits and customers definition (and distinction between drinks and biscuits):
        (icy d1) (icy d2)
        (biscuit u1) (biscuit u2) 
        (customer c1) (customer c2)
        (different d1 d2)
        (different-b u1 u2)
        ; Order info definition:
        (drink-info c1 d1 t2) (drink-info c2 d2 t2)
        (biscuit-info c1 u1 t2) (biscuit-info c2 u2 t2)
        (biscuit-info c1 u1 bar) (biscuit-info c2 u2 bar)
        ; Number of customers at each table:
        (= (number-of-customers t1) 0)
        (= (number-of-customers t2) 2)
        (= (number-of-customers t3) 0)
        (= (number-of-customers t4) 0)
        ; Tables already cleaned:
        (clean t1)

    )

    ;; Assert the goal state
    (:goal (and (clean t2) (clean t3) (clean t4))) 

    (:metric minimize (total-time))
)