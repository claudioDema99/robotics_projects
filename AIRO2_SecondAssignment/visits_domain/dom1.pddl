(define (domain localization)

	(:requirements
		:typing
		:durative-actions
		:numeric-fluents
		:negative-preconditions
		:action-costs
		:conditional-effects
		:equality
		:fluents
	)

	(:types 	
		robot 
		region 
	)

	(:predicates
		(robot_in ?v - robot ?r - region) 
		(grabbed ?r - region )
		(visited ?r - region)
	)

	(:functions
		(act-cost)
		(triggered ?from ?to - region)
		(dummy)
		(reports_in ?r - region)
		(to_grab)
	)

	(:durative-action goto_region
		:parameters (?r - robot ?from ?to - region)
		:duration (= ?duration 100)
		:condition (and (at start (robot_in ?r ?from)))
		:effect (and
			(at start (not (robot_in ?r ?from))) 
			(at start (increase (triggered ?from ?to) 1))
			(at end (robot_in ?r ?to)) 
			(at end (assign (triggered ?from ?to) 0)) 
			(at end (visited ?to)) 	
			(at end (increase (act-cost) (dummy))))
	)

	(:action grab
		:parameters (?r - robot ?rg - region)
		:precondition (and
			(visited ?rg)
			(robot_in ?r ?rg)
			(= (reports_in ?rg) 1)
		)
			:effect (and
			(grabbed ?rg)
			(decrease (reports_in ?rg) 1)
			(decrease (to_grab) 1)
		)
	)	
)


