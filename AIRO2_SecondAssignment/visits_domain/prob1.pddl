(define (problem prob1)
     (:domain localization)
     (:objects
          r0 r1 r2 r3 r4 r5 - region
          R2D2 - robot
     )

     (:init
          (robot_in R2D2 r0)
  
          (= (act-cost) 0)
          (= (dummy) 0)

          (= (reports_in r1) 1)
          (= (reports_in r2) 1)
          (= (reports_in r3) 1)
          (= (reports_in r4) 1)

          (= (to_grab) 2)
     )

     (:goal (and
          (= (to_grab) 0)
          ;(= (reports_in r1) 0) ; Here you can change the region to be explored (r1, r2, r3, r4)
          ;(= (reports_in r4) 0) ; Here you can change the region to be explored 
          (robot_in R2D2 r5)

     ))

     (:metric minimize (act-cost))
)



