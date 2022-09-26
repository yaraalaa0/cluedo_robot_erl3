(define (problem cluedo_prob)
(:domain cluedo_dom)
(:objects 
    wp0 wp1 wp2 wp3 wp4 wp5 wp6 - waypoint
)

(:init
    (base_at wp0)
    (center wp0)
    (unexplored wp1)
    (unexplored wp2)
    (unexplored wp3)
    (unexplored wp4)
    (unexplored wp5)
    (unexplored wp6)
    (= (hints) 0.0)

)

(:goal (and
    (correct_hyp)
    )
)

)
